/* The copyright in this software is being made available under the BSD
* License, included below. This software may be subject to other third party
* and contributor rights, including patent rights, and no such rights are
* granted under this license.
*
* Copyright (c) 2010-2021, ITU/ISO/IEC
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
*    be used to endorse or promote products derived from this software without
*    specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*/

/** \file     MatrixIntraPrediction.cpp
\brief    matrix-based intra prediction class
*/

#include "MatrixIntraPrediction.h"
#include "dtrace_next.h"

#include "UnitTools.h"
#include "MipData.h"


MatrixIntraPrediction::MatrixIntraPrediction():
  m_component(MAX_NUM_COMPONENT),
  m_reducedBoundary          (MIP_MAX_INPUT_SIZE),
  m_reducedBoundaryTransposed(MIP_MAX_INPUT_SIZE),
  m_inputOffset      ( 0 ),
  m_inputOffsetTransp( 0 ),
  m_refSamplesTop (MIP_MAX_WIDTH),
  m_refSamplesLeft(MIP_MAX_HEIGHT),
  m_blockSize( 0, 0 ),
  m_sizeId( 0 ),
  m_reducedBdrySize( 0 ),
  m_reducedPredSize( 0 ),
  m_upsmpFactorHor( 0 ),
  m_upsmpFactorVer( 0 )
{
}

void MatrixIntraPrediction::prepareInputForPred(const CPelBuf &pSrc, const Area &block, const int bitDepth,
                                                const ComponentID compId)
{
  m_component = compId;

  // Step 1: Save block size and calculate dependent values
  // 保存块大小并计算MIP相关参数
  initPredBlockParams(block);

  // Step 2: Get the input data (left and top reference samples)
  // 获取输入像素(左和上参考像素)
  // 获取上方参考像素
  m_refSamplesTop.resize(block.width);
  for (int x = 0; x < block.width; x++)
  {
    m_refSamplesTop[x] = pSrc.at(x + 1, 0);
  }
  // 左边参考像素
  m_refSamplesLeft.resize(block.height);
  for (int y = 0; y < block.height; y++)
  {
    m_refSamplesLeft[y] = pSrc.at(y + 1, 1);
  }

  // Step 3: Compute the reduced boundary via Haar-downsampling (input for the prediction)
  // Step 3: 通过Haar下采样计算缩减边界（预测输入）
  // 下采样后输入向量的尺寸为4或者8(4x4是4，其余的是8)
  const int inputSize = 2 * m_reducedBdrySize;
  // 不需要转置时，下采样像素的顺序：先上后左
  m_reducedBoundary          .resize( inputSize );
  // 转置时，下采样像素的顺序：先左后上
  m_reducedBoundaryTransposed.resize( inputSize );

  // 下采样
  int* const topReduced = m_reducedBoundary.data();
  boundaryDownsampling1D( topReduced, m_refSamplesTop.data(), block.width, m_reducedBdrySize );

  int* const leftReduced = m_reducedBoundary.data() + m_reducedBdrySize;
  boundaryDownsampling1D( leftReduced, m_refSamplesLeft.data(), block.height, m_reducedBdrySize );

  // 转置就是上+左的顺序变成左+上的顺序，根据模式号进行调整
  int* const leftReducedTransposed = m_reducedBoundaryTransposed.data();
  int* const topReducedTransposed  = m_reducedBoundaryTransposed.data() + m_reducedBdrySize;

  // 左和上下采样并转置
  for( int x = 0; x < m_reducedBdrySize; x++ )
  {
    topReducedTransposed[x] = topReduced[x];
  }
  for( int y = 0; y < m_reducedBdrySize; y++ )
  {
    leftReducedTransposed[y] = leftReduced[y];
  }

  // Step 4: Rebase the reduced boundary
  // 推导矩阵乘法输入向量p，mipSizeId=0/1和mipSizeId=2的推导方法不一样
  m_inputOffset       = m_reducedBoundary[0];
  m_inputOffsetTransp = m_reducedBoundaryTransposed[0];

  const bool hasFirstCol = (m_sizeId < 2);
  m_reducedBoundary          [0] = hasFirstCol ? ((1 << (bitDepth - 1)) - m_inputOffset      ) : 0; // first column of matrix not needed for large blocks
  m_reducedBoundaryTransposed[0] = hasFirstCol ? ((1 << (bitDepth - 1)) - m_inputOffsetTransp) : 0;
  for (int i = 1; i < inputSize; i++)
  {
    m_reducedBoundary          [i] -= m_inputOffset;
    m_reducedBoundaryTransposed[i] -= m_inputOffsetTransp;
  }
}

void MatrixIntraPrediction::predBlock(int *const result, const int modeIdx, const bool transpose, const int bitDepth,
                                      const ComponentID compId)
{
  CHECK(m_component != compId, "Boundary has not been prepared for this component.");

  // 是否需要上采样
  const bool needUpsampling = ( m_upsmpFactorHor > 1 ) || ( m_upsmpFactorVer > 1 );

  // 根据mipSizeId(CU的大小)选择MIP矩阵
  // 4x4:   [16, 16, 4] 
  // 8x8:   [8, 16, 8]
  // 16x16: [6, 64, 7]
  const uint8_t* matrix = getMatrixData(modeIdx);

  // 存储下采样的预测像素
  static_vector<int, MIP_MAX_REDUCED_OUTPUT_SAMPLES> bufReducedPred( m_reducedPredSize * m_reducedPredSize );

  int* const       reducedPred     = needUpsampling ? bufReducedPred.data() : result;
  // 根据是否转置获得缩减边界像素向量
  const int* const reducedBoundary = transpose ? m_reducedBoundaryTransposed.data() : m_reducedBoundary.data();
  // 进行矩阵乘法计算缩减预测像素
  computeReducedPred(reducedPred, reducedBoundary, matrix, transpose, bitDepth);
  // 如果需要上采样
  if( needUpsampling )
  {
    // 上采样函数，利用缩减预测像素获得整个块的预测像素
    predictionUpsampling( result, reducedPred );
  }
}


void MatrixIntraPrediction::initPredBlockParams(const Size& block)
{
  m_blockSize = block;
  // init size index
  m_sizeId = getMipSizeId( m_blockSize );

  // init reduced boundary size

  // 初始缩减边界尺寸
  // 对于4x4的块宽度和高度分别下采样为2个像素
  // 对于其余尺寸的块宽度和高度分别缩减为4个像素
  
  m_reducedBdrySize = (m_sizeId == 0) ? 2 : 4;

  // init reduced prediction size
  // 初始化缩减预测后的尺寸
  // 对于mipSizeId = 0、1的块(4x4, 4xN, Nx4, 8x8)，MIP预测后输出4x4的块
  // 对于mipSizeId = 2的块，MIP预测后输出8x8的块
  m_reducedPredSize = ( m_sizeId < 2 ) ? 4 : 8;



  // init upsampling factors
  // 上采样因子，即一个输出像素生成几个最终的预测像素
  m_upsmpFactorHor = m_blockSize.width  / m_reducedPredSize;
  m_upsmpFactorVer = m_blockSize.height / m_reducedPredSize;
  // 初始上采样因子
  CHECKD( (m_upsmpFactorHor < 1) || ((m_upsmpFactorHor & (m_upsmpFactorHor - 1)) != 0), "Need power of two horizontal upsampling factor." );
  CHECKD( (m_upsmpFactorVer < 1) || ((m_upsmpFactorVer & (m_upsmpFactorVer - 1)) != 0), "Need power of two vertical upsampling factor." );
}



void MatrixIntraPrediction::boundaryDownsampling1D(int* reducedDst, const int* const fullSrc, const SizeType srcLen, const SizeType dstLen)
{
  // 下采样尺寸小于实际尺寸才进行，否则直接复制就可以。下采样相当于求平均
  if (dstLen < srcLen)
  {
    // Create reduced boundary by downsampling
    const SizeType downsmpFactor = srcLen / dstLen;
    const int log2DownsmpFactor = floorLog2(downsmpFactor);
    const int roundingOffset = (1 << (log2DownsmpFactor - 1));

    SizeType srcIdx = 0;
    for( SizeType dstIdx = 0; dstIdx < dstLen; dstIdx++ )
    {
      int sum = 0;
      for( int k = 0; k < downsmpFactor; k++ )
      {
        sum += fullSrc[srcIdx++];
      }
      reducedDst[dstIdx] = (sum + roundingOffset) >> log2DownsmpFactor;
    }
  }
  else
  {
    // Copy boundary if no downsampling is needed
    for (SizeType i = 0; i < dstLen; ++i) // 不需要下采样直接复制边界
    {
      reducedDst[i] = fullSrc[i];
    }
  }
}


void MatrixIntraPrediction::predictionUpsampling1D(int* const dst, const int* const src, const int* const bndry,
                                                   const SizeType srcSizeUpsmpDim, const SizeType srcSizeOrthDim,
                                                   const SizeType srcStep, const SizeType srcStride,
                                                   const SizeType dstStep, const SizeType dstStride,
                                                   const SizeType bndryStep,
                                                   const unsigned int upsmpFactor )
{
  const int log2UpsmpFactor = floorLog2( upsmpFactor );
  CHECKD( upsmpFactor <= 1, "Upsampling factor must be at least 2." );
  const int roundingOffset = 1 << (log2UpsmpFactor - 1);

  SizeType idxOrthDim = 0;
  const int* srcLine = src;//矩阵乘法输出或水平插值结果
  int* dstLine = dst;
  const int* bndryLine = bndry + bndryStep - 1;//边界参考像素
  while( idxOrthDim < srcSizeOrthDim )
  {
    SizeType idxUpsmpDim = 0;
    const int* before = bndryLine;//前一个参考像素
    const int* behind = srcLine;//后一个参考像素
    int* currDst = dstLine;
    while( idxUpsmpDim < srcSizeUpsmpDim )
    {
      SizeType pos = 1;//控制当前插值的位置，将插值结果和矩阵乘法结果放到各自相应的位置上
      int scaledBefore = ( *before ) << log2UpsmpFactor;
      int scaledBehind = 0;
      while( pos <= upsmpFactor )
      {
        // 通过+-操作可以控制插值时参考像素的权重
        scaledBefore -= *before;
        scaledBehind += *behind;
        *currDst = (scaledBefore + scaledBehind + roundingOffset) >> log2UpsmpFactor;

        pos++;
        currDst += dstStep;
      }

      idxUpsmpDim++;
      before = behind;//移动前一个参考像素
      behind += srcStep;//移动后一个参考像素
    }

    idxOrthDim++;
    srcLine += srcStride;
    dstLine += dstStride;
    bndryLine += bndryStep;
  }
}


void MatrixIntraPrediction::predictionUpsampling( int* const dst, const int* const src ) const
{
  const int* verSrc     = src;
  SizeType   verSrcStep = m_blockSize.width;
  // 插值过程固定，先水平后垂直
  if( m_upsmpFactorHor > 1 ) // 如果需要插值
  {
    int* const horDst = dst + (m_upsmpFactorVer - 1) * m_blockSize.width;
    verSrc = horDst;
    verSrcStep *= m_upsmpFactorVer;

    predictionUpsampling1D( horDst, src, m_refSamplesLeft.data(),
                            m_reducedPredSize, m_reducedPredSize,
                            1, m_reducedPredSize, 1, verSrcStep,
                            m_upsmpFactorVer, m_upsmpFactorHor );
  }

  if( m_upsmpFactorVer > 1 )
  {
    predictionUpsampling1D( dst, verSrc, m_refSamplesTop.data(),
                            m_reducedPredSize, m_blockSize.width,
                            verSrcStep, 1, m_blockSize.width, 1,
                            1, m_upsmpFactorVer );
  }
}

const uint8_t* MatrixIntraPrediction::getMatrixData(const int modeIdx) const
{
  switch( m_sizeId )
  {
  case 0: return &mipMatrix4x4[modeIdx][0][0];

  case 1: return &mipMatrix8x8[modeIdx][0][0];

  case 2: return &mipMatrix16x16[modeIdx][0][0];

  default: THROW( "Invalid mipSizeId" );
  }
}

void MatrixIntraPrediction::computeReducedPred( int*const result, const int* const input,
                                                const uint8_t* matrix,
                                                const bool transpose, const int bitDepth )
{
  // 输出的长度 4或8
  const int inputSize = 2 * m_reducedBdrySize;

  // use local buffer for transposed result
  static_vector<int, MIP_MAX_REDUCED_OUTPUT_SAMPLES> resBufTransposed( m_reducedPredSize * m_reducedPredSize );

  int*const resPtr = (transpose) ? resBufTransposed.data() : result;

  int sum = 0;
  for( int i = 0; i < inputSize; i++ ) { sum += input[i]; }
  // MIP_SHIFT_MATRIX 移位因子sW固定为6
  // MIP_OFFSET_MATRIX 偏移因子fO固定为32
  // 计算偏移量Bias
  const int offset = (1 << (MIP_SHIFT_MATRIX - 1)) - MIP_OFFSET_MATRIX * sum;
  CHECK( inputSize != 4 * (inputSize >> 2), "Error, input size not divisible by four" );

  const uint8_t *weight = matrix;// 权重矩阵
  const int   inputOffset = transpose ? m_inputOffsetTransp : m_inputOffset;

  const bool redSize = (m_sizeId == 2);
  int posRes = 0;
  for( int y = 0; y < m_reducedPredSize; y++ )
  {
    for( int x = 0; x < m_reducedPredSize; x++ )
    {
      if( redSize ) weight -= 1;
      int tmp0 = redSize ? 0 : (input[0] * weight[0]);
      int tmp1 = input[1] * weight[1];
      int tmp2 = input[2] * weight[2];
      int tmp3 = input[3] * weight[3];
      for (int i = 4; i < inputSize; i += 4)
      {
        tmp0 += input[i]     * weight[i];
        tmp1 += input[i + 1] * weight[i + 1];
        tmp2 += input[i + 2] * weight[i + 2];
        tmp3 += input[i + 3] * weight[i + 3];
      }
      // 对矩阵乘法输出采样钳位
      resPtr[posRes++] = ClipBD<int>(((tmp0 + tmp1 + tmp2 + tmp3 + offset) >> MIP_SHIFT_MATRIX) + inputOffset, bitDepth);

      weight += inputSize;
    }
  }

  if( transpose )
  {// 将矩阵乘法结果进行转置
    for( int y = 0; y < m_reducedPredSize; y++ )
    {
      for( int x = 0; x < m_reducedPredSize; x++ )
      {
        result[ y * m_reducedPredSize + x ] = resPtr[ x * m_reducedPredSize + y ];
      }
    }
  }
}
