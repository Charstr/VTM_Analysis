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
  // ������С������MIP��ز���
  initPredBlockParams(block);

  // Step 2: Get the input data (left and top reference samples)
  // ��ȡ��������(����ϲο�����)
  // ��ȡ�Ϸ��ο�����
  m_refSamplesTop.resize(block.width);
  for (int x = 0; x < block.width; x++)
  {
    m_refSamplesTop[x] = pSrc.at(x + 1, 0);
  }
  // ��߲ο�����
  m_refSamplesLeft.resize(block.height);
  for (int y = 0; y < block.height; y++)
  {
    m_refSamplesLeft[y] = pSrc.at(y + 1, 1);
  }

  // Step 3: Compute the reduced boundary via Haar-downsampling (input for the prediction)
  // Step 3: ͨ��Haar�²������������߽磨Ԥ�����룩
  // �²��������������ĳߴ�Ϊ4����8(4x4��4���������8)
  const int inputSize = 2 * m_reducedBdrySize;
  // ����Ҫת��ʱ���²������ص�˳�����Ϻ���
  m_reducedBoundary          .resize( inputSize );
  // ת��ʱ���²������ص�˳���������
  m_reducedBoundaryTransposed.resize( inputSize );

  // �²���
  int* const topReduced = m_reducedBoundary.data();
  boundaryDownsampling1D( topReduced, m_refSamplesTop.data(), block.width, m_reducedBdrySize );

  int* const leftReduced = m_reducedBoundary.data() + m_reducedBdrySize;
  boundaryDownsampling1D( leftReduced, m_refSamplesLeft.data(), block.height, m_reducedBdrySize );

  // ת�þ�����+���˳������+�ϵ�˳�򣬸���ģʽ�Ž��е���
  int* const leftReducedTransposed = m_reducedBoundaryTransposed.data();
  int* const topReducedTransposed  = m_reducedBoundaryTransposed.data() + m_reducedBdrySize;

  // ������²�����ת��
  for( int x = 0; x < m_reducedBdrySize; x++ )
  {
    topReducedTransposed[x] = topReduced[x];
  }
  for( int y = 0; y < m_reducedBdrySize; y++ )
  {
    leftReducedTransposed[y] = leftReduced[y];
  }

  // Step 4: Rebase the reduced boundary
  // �Ƶ�����˷���������p��mipSizeId=0/1��mipSizeId=2���Ƶ�������һ��
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

  // �Ƿ���Ҫ�ϲ���
  const bool needUpsampling = ( m_upsmpFactorHor > 1 ) || ( m_upsmpFactorVer > 1 );

  // ����mipSizeId(CU�Ĵ�С)ѡ��MIP����
  // 4x4:   [16, 16, 4] 
  // 8x8:   [8, 16, 8]
  // 16x16: [6, 64, 7]
  const uint8_t* matrix = getMatrixData(modeIdx);

  // �洢�²�����Ԥ������
  static_vector<int, MIP_MAX_REDUCED_OUTPUT_SAMPLES> bufReducedPred( m_reducedPredSize * m_reducedPredSize );

  int* const       reducedPred     = needUpsampling ? bufReducedPred.data() : result;
  // �����Ƿ�ת�û�������߽���������
  const int* const reducedBoundary = transpose ? m_reducedBoundaryTransposed.data() : m_reducedBoundary.data();
  // ���о���˷���������Ԥ������
  computeReducedPred(reducedPred, reducedBoundary, matrix, transpose, bitDepth);
  // �����Ҫ�ϲ���
  if( needUpsampling )
  {
    // �ϲ�����������������Ԥ�����ػ���������Ԥ������
    predictionUpsampling( result, reducedPred );
  }
}


void MatrixIntraPrediction::initPredBlockParams(const Size& block)
{
  m_blockSize = block;
  // init size index
  m_sizeId = getMipSizeId( m_blockSize );

  // init reduced boundary size

  // ��ʼ�����߽�ߴ�
  // ����4x4�Ŀ��Ⱥ͸߶ȷֱ��²���Ϊ2������
  // ��������ߴ�Ŀ��Ⱥ͸߶ȷֱ�����Ϊ4������
  
  m_reducedBdrySize = (m_sizeId == 0) ? 2 : 4;

  // init reduced prediction size
  // ��ʼ������Ԥ���ĳߴ�
  // ����mipSizeId = 0��1�Ŀ�(4x4, 4xN, Nx4, 8x8)��MIPԤ������4x4�Ŀ�
  // ����mipSizeId = 2�Ŀ飬MIPԤ������8x8�Ŀ�
  m_reducedPredSize = ( m_sizeId < 2 ) ? 4 : 8;



  // init upsampling factors
  // �ϲ������ӣ���һ������������ɼ������յ�Ԥ������
  m_upsmpFactorHor = m_blockSize.width  / m_reducedPredSize;
  m_upsmpFactorVer = m_blockSize.height / m_reducedPredSize;
  // ��ʼ�ϲ�������
  CHECKD( (m_upsmpFactorHor < 1) || ((m_upsmpFactorHor & (m_upsmpFactorHor - 1)) != 0), "Need power of two horizontal upsampling factor." );
  CHECKD( (m_upsmpFactorVer < 1) || ((m_upsmpFactorVer & (m_upsmpFactorVer - 1)) != 0), "Need power of two vertical upsampling factor." );
}



void MatrixIntraPrediction::boundaryDownsampling1D(int* reducedDst, const int* const fullSrc, const SizeType srcLen, const SizeType dstLen)
{
  // �²����ߴ�С��ʵ�ʳߴ�Ž��У�����ֱ�Ӹ��ƾͿ��ԡ��²����൱����ƽ��
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
    for (SizeType i = 0; i < dstLen; ++i) // ����Ҫ�²���ֱ�Ӹ��Ʊ߽�
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
  const int* srcLine = src;//����˷������ˮƽ��ֵ���
  int* dstLine = dst;
  const int* bndryLine = bndry + bndryStep - 1;//�߽�ο�����
  while( idxOrthDim < srcSizeOrthDim )
  {
    SizeType idxUpsmpDim = 0;
    const int* before = bndryLine;//ǰһ���ο�����
    const int* behind = srcLine;//��һ���ο�����
    int* currDst = dstLine;
    while( idxUpsmpDim < srcSizeUpsmpDim )
    {
      SizeType pos = 1;//���Ƶ�ǰ��ֵ��λ�ã�����ֵ����;���˷�����ŵ�������Ӧ��λ����
      int scaledBefore = ( *before ) << log2UpsmpFactor;
      int scaledBehind = 0;
      while( pos <= upsmpFactor )
      {
        // ͨ��+-�������Կ��Ʋ�ֵʱ�ο����ص�Ȩ��
        scaledBefore -= *before;
        scaledBehind += *behind;
        *currDst = (scaledBefore + scaledBehind + roundingOffset) >> log2UpsmpFactor;

        pos++;
        currDst += dstStep;
      }

      idxUpsmpDim++;
      before = behind;//�ƶ�ǰһ���ο�����
      behind += srcStep;//�ƶ���һ���ο�����
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
  // ��ֵ���̶̹�����ˮƽ��ֱ
  if( m_upsmpFactorHor > 1 ) // �����Ҫ��ֵ
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
  // ����ĳ��� 4��8
  const int inputSize = 2 * m_reducedBdrySize;

  // use local buffer for transposed result
  static_vector<int, MIP_MAX_REDUCED_OUTPUT_SAMPLES> resBufTransposed( m_reducedPredSize * m_reducedPredSize );

  int*const resPtr = (transpose) ? resBufTransposed.data() : result;

  int sum = 0;
  for( int i = 0; i < inputSize; i++ ) { sum += input[i]; }
  // MIP_SHIFT_MATRIX ��λ����sW�̶�Ϊ6
  // MIP_OFFSET_MATRIX ƫ������fO�̶�Ϊ32
  // ����ƫ����Bias
  const int offset = (1 << (MIP_SHIFT_MATRIX - 1)) - MIP_OFFSET_MATRIX * sum;
  CHECK( inputSize != 4 * (inputSize >> 2), "Error, input size not divisible by four" );

  const uint8_t *weight = matrix;// Ȩ�ؾ���
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
      // �Ծ���˷��������ǯλ
      resPtr[posRes++] = ClipBD<int>(((tmp0 + tmp1 + tmp2 + tmp3 + offset) >> MIP_SHIFT_MATRIX) + inputOffset, bitDepth);

      weight += inputSize;
    }
  }

  if( transpose )
  {// ������˷��������ת��
    for( int y = 0; y < m_reducedPredSize; y++ )
    {
      for( int x = 0; x < m_reducedPredSize; x++ )
      {
        result[ y * m_reducedPredSize + x ] = resPtr[ x * m_reducedPredSize + y ];
      }
    }
  }
}
