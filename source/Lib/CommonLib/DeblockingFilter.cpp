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

/** \file     DeblockingFilter.cpp
    \brief    deblocking filter
*/

#include "DeblockingFilter.h"
#include "Slice.h"
#include "Mv.h"
#include "Unit.h"
#include "UnitTools.h"
#include "UnitPartitioner.h"
#include "dtrace_codingstruct.h"
#include "dtrace_buffer.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define DEBLOCK_SMALLEST_BLOCK  8
#define DEFAULT_INTRA_TC_OFFSET 2 ///< Default intra TC offset

// ====================================================================================================================
// Tables
// ====================================================================================================================

const uint16_t DeblockingFilter::sm_tcTable[MAX_QP + 1 + DEFAULT_INTRA_TC_OFFSET] = {
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   3,   4,   4,   4,
  4,  5,  5,  5,  5,  7,  7,  8,  9,  10,  10,  11,  13,  14,  15,  17,  19,  21,  24,  25,  29,  33,
  36, 41, 45, 51, 57, 64, 71, 80, 89, 100, 112, 125, 141, 157, 177, 198, 222, 250, 280, 314, 352, 395
};

const uint8_t DeblockingFilter::sm_betaTable[MAX_QP + 1] = { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                                                       6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 22, 24,
                                                       26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56,
                                                       58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88 };

inline static uint32_t getRasterIdx(const Position& pos, const PreCalcValues& pcv)
{
  return ( ( pos.x & pcv.maxCUWidthMask ) >> pcv.minCUWidthLog2 ) + ( ( pos.y & pcv.maxCUHeightMask ) >> pcv.minCUHeightLog2 ) * pcv.partsInCtuWidth;
}

// ====================================================================================================================
// utility functions
// ====================================================================================================================
static bool isAvailableLeft( const CodingUnit& cu, const CodingUnit& cu2, const bool bEnforceSliceRestriction, const bool bEnforceTileRestriction, const bool bEnforceSubPicRestriction)
{
  return ((!bEnforceSliceRestriction || CU::isSameSlice(cu, cu2)) && (!bEnforceTileRestriction || CU::isSameTile(cu, cu2)) && (!bEnforceSubPicRestriction || CU::isSameSubPic(cu, cu2)));
}

static bool isAvailableAbove( const CodingUnit& cu, const CodingUnit& cu2, const bool bEnforceSliceRestriction, const bool bEnforceTileRestriction, const bool bEnforceSubPicRestriction)
{
  return ( !bEnforceSliceRestriction || CU::isSameSlice( cu, cu2 ) ) && ( !bEnforceTileRestriction || CU::isSameTile( cu, cu2 ) ) && (!bEnforceSubPicRestriction || CU::isSameSubPic(cu, cu2));
}


// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

DeblockingFilter::DeblockingFilter()
{
}

DeblockingFilter::~DeblockingFilter()
{
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================
void DeblockingFilter::create( const unsigned uiMaxCUDepth )
{
  destroy();
  const unsigned numPartitions = 1 << ( uiMaxCUDepth << 1 );
  for( int edgeDir = 0; edgeDir < NUM_EDGE_DIR; edgeDir++ )
  {
    m_aapucBS       [edgeDir].resize( numPartitions );
    m_aapbEdgeFilter[edgeDir].resize( numPartitions );
  }
  m_enc = false;
}

void DeblockingFilter::initEncPicYuvBuffer(ChromaFormat chromaFormat, const Size &size, const unsigned maxCUSize)
{
  const Area a = Area(Position(), size);
  m_encPicYuvBuffer.destroy();
  m_encPicYuvBuffer.create(chromaFormat, a, maxCUSize, 7);
}

void DeblockingFilter::destroy()
{
  for( int edgeDir = 0; edgeDir < NUM_EDGE_DIR; edgeDir++ )
  {
    m_aapucBS       [edgeDir].clear();
    m_aapbEdgeFilter[edgeDir].clear();
  }
  m_encPicYuvBuffer.destroy();
}

/**
 - call deblocking function for every CU
 对每一个CU进行去方块
 \param  pcPic   picture class (Pic) pointer
 */
void DeblockingFilter::deblockingFilterPic( CodingStructure& cs
                                )
{
  const PreCalcValues& pcv = *cs.pcv;
  m_shiftHor = ::getComponentScaleX( COMPONENT_Cb, cs.pcv->chrFormat );
  m_shiftVer = ::getComponentScaleY( COMPONENT_Cb, cs.pcv->chrFormat );

  DTRACE_UPDATE( g_trace_ctx, ( std::make_pair( "poc", cs.slice->getPOC() ) ) );
#if ENABLE_TRACING
  for( int y = 0; y < pcv.heightInCtus; y++ )
  {
    for( int x = 0; x < pcv.widthInCtus; x++ )
    {
      const UnitArea ctuArea( pcv.chrFormat, Area( x << pcv.maxCUWidthLog2, y << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth ) );
      DTRACE    ( g_trace_ctx, D_CRC, "CTU %d %d", ctuArea.Y().x, ctuArea.Y().y );
      DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.picture->getRecoBuf( clipArea( ctuArea, *cs.picture ) ), &ctuArea.Y() );
    }
  }
#endif

  for( int y = 0; y < pcv.heightInCtus; y++ )
  {
    for( int x = 0; x < pcv.widthInCtus; x++ )
    {
      memset( m_aapucBS       [EDGE_VER].data(), 0,     m_aapucBS       [EDGE_VER].byte_size() );
      memset( m_aapbEdgeFilter[EDGE_VER].data(), false, m_aapbEdgeFilter[EDGE_VER].byte_size() );
      memset( m_maxFilterLengthP, 0, sizeof(m_maxFilterLengthP) );
      memset( m_maxFilterLengthQ, 0, sizeof(m_maxFilterLengthQ) );
      memset( m_transformEdge, false, sizeof(m_transformEdge) );
      m_ctuXLumaSamples = x << pcv.maxCUWidthLog2;
      m_ctuYLumaSamples = y << pcv.maxCUHeightLog2;

      const UnitArea ctuArea( pcv.chrFormat, Area( x << pcv.maxCUWidthLog2, y << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth ) );
      CodingUnit* firstCU = cs.getCU( ctuArea.lumaPos(), CH_L);
      cs.slice = firstCU->slice;

      // CU-based deblocking

      // CU-based deblocking 基于CU的去块滤波，垂直边界滤波
      for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, CH_L ), CH_L ) )
      {
        xDeblockCU( currCU, EDGE_VER );
      }

      if( CS::isDualITree( cs ) )//对于双树，亮度色度单独划分，所以针对色度需要单独滤波
      {
        memset( m_aapucBS       [EDGE_VER].data(), 0,     m_aapucBS       [EDGE_VER].byte_size() );
        memset( m_aapbEdgeFilter[EDGE_VER].data(), false, m_aapbEdgeFilter[EDGE_VER].byte_size() );
        memset( m_maxFilterLengthP, 0, sizeof(m_maxFilterLengthP) );
        memset( m_maxFilterLengthQ, 0, sizeof(m_maxFilterLengthQ) );
        memset( m_transformEdge, false, sizeof(m_transformEdge) );

        for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, CH_C ), CH_C ) )
        {
          xDeblockCU( currCU, EDGE_VER );
        }
      }
    }
  }

  // Vertical filtering
  // CU-based deblocking 基于CU的去块滤波，水平边界滤波
  for( int y = 0; y < pcv.heightInCtus; y++ )
  {
    for( int x = 0; x < pcv.widthInCtus; x++ )
    {
      memset( m_aapucBS       [EDGE_HOR].data(), 0,     m_aapucBS       [EDGE_HOR].byte_size() );
      memset( m_aapbEdgeFilter[EDGE_HOR].data(), false, m_aapbEdgeFilter[EDGE_HOR].byte_size() );
      memset( m_maxFilterLengthP, 0, sizeof(m_maxFilterLengthP) );
      memset( m_maxFilterLengthQ, 0, sizeof(m_maxFilterLengthQ) );
      memset( m_transformEdge, false, sizeof(m_transformEdge) );
      m_ctuXLumaSamples = x << pcv.maxCUWidthLog2;
      m_ctuYLumaSamples = y << pcv.maxCUHeightLog2;

      const UnitArea ctuArea( pcv.chrFormat, Area( x << pcv.maxCUWidthLog2, y << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth ) );
      CodingUnit* firstCU = cs.getCU( ctuArea.lumaPos(), CH_L);
      cs.slice = firstCU->slice;

      // CU-based deblocking
      for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, CH_L ), CH_L ) )
      {
        xDeblockCU( currCU, EDGE_HOR );
      }

      if( CS::isDualITree( cs ) )
      {
        memset( m_aapucBS       [EDGE_HOR].data(), 0,     m_aapucBS       [EDGE_HOR].byte_size() );
        memset( m_aapbEdgeFilter[EDGE_HOR].data(), false, m_aapbEdgeFilter[EDGE_HOR].byte_size() );
        memset( m_maxFilterLengthP, 0, sizeof(m_maxFilterLengthP) );
        memset( m_maxFilterLengthQ, 0, sizeof(m_maxFilterLengthQ) );
        memset( m_transformEdge, false, sizeof(m_transformEdge) );

        for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, CH_C ), CH_C ) )
        {
          xDeblockCU( currCU, EDGE_HOR );
        }
      }
    }
  }

  DTRACE_PIC_COMP(D_REC_CB_LUMA_LF,   cs, cs.getRecoBuf(), COMPONENT_Y);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_LF, cs, cs.getRecoBuf(), COMPONENT_Cb);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_LF, cs, cs.getRecoBuf(), COMPONENT_Cr);

  DTRACE    ( g_trace_ctx, D_CRC, "DeblockingFilter" );
  DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.getRecoBuf() );
}

void DeblockingFilter::resetFilterLengths()
{
  memset(m_aapucBS[EDGE_VER].data(), 0, m_aapucBS[EDGE_VER].byte_size());
  memset(m_aapbEdgeFilter[EDGE_VER].data(), false, m_aapbEdgeFilter[EDGE_VER].byte_size());
  memset(m_aapucBS[EDGE_HOR].data(), 0, m_aapucBS[EDGE_HOR].byte_size());
  memset(m_aapbEdgeFilter[EDGE_HOR].data(), false, m_aapbEdgeFilter[EDGE_HOR].byte_size());
  memset(m_maxFilterLengthP, 0, sizeof(m_maxFilterLengthP));
  memset(m_maxFilterLengthQ, 0, sizeof(m_maxFilterLengthQ));
  memset(m_transformEdge, false, sizeof(m_transformEdge));
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
 Deblocking filter process in CU-based (the same function as conventional's)

 \param cu               the CU to be deblocked
 \param edgeDir          the direction of the edge in block boundary (horizontal/vertical), which is added newly
*/
/*
xDeblockCU函数是以CU为单元进行去块滤波的，其第一个参数是需要进行滤波的CU，第二个参数是滤波方向（垂直或水平）。该函数的过程如下：

调用xSetLoopfilterParam函数设置边界是否可用去块滤波（内部子块边界internalEdge，左侧边界leftEdge和上侧边界topEdge）
遍历该CU中的全部子TU，设置子TU的垂直和水平边界（xSetEdgefilterMultiple函数，是否使用滤波），并根据滤波TU的边界长度确定需要进行滤波的像素数（xSetMaxFilterLengthPQFromTransformSizes函数）
并将相应的边界加入边界的数组中（CU边界也在该步骤中设置）
遍历该CU中的全部子PU，进行和第二步类似的过程。
遍历所有的4x4单元，以4x4区域为基本单元设置边界强度（xGetBoundaryStrengthSingle函数）
对所有需要进行滤波的边界进行排序，然后遍历所有需要进行滤波的边界，对每一个需要滤波的边界调用xEdgeFilterLuma和xEdgeFilterChroma进行滤波。
注意：在VVC中，是不再区分CU、PU、TU的，上述的子TU边界滤波是由SBT和ISP模式引入的，子PU边界是由SbTMVP和仿射模式引入的。

*/
void DeblockingFilter::xDeblockCU( CodingUnit& cu, const DeblockEdgeDir edgeDir )
{
  const PreCalcValues& pcv = *cu.cs->pcv;
  const Area area          = cu.Y().valid() ? cu.Y() : Area( recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].pos() ), recalcSize( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].size() ) );

  bool horEdgeFilter = false, verEdgeFilter = false;
  int  numHorVirBndry = 0, numVerVirBndry = 0;
  int  horVirBndryPos[] = { 0, 0, 0 };
  int  verVirBndryPos[] = { 0, 0, 0 };

  bool isCuCrossedByVirtualBoundaries = isCrossedByVirtualBoundaries( area.x, area.y, area.width, area.height, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, cu.cs->picHeader );

  xSetDeblockingFilterParam( cu );//设置滤波边界是否可用（左、上、中间）
  static_vector<int, 2*MAX_CU_SIZE> edgeIdx;
  edgeIdx.clear();

  if (m_enc)
  {
    m_shiftHor = ::getComponentScaleX(COMPONENT_Cb, cu.chromaFormat);
    m_shiftVer = ::getComponentScaleY(COMPONENT_Cb, cu.chromaFormat);
    int x, y;//CU的x，y坐标
    if (cu.Y().valid())
    {
      x = cu.block(COMPONENT_Y).x;
      y = cu.block(COMPONENT_Y).y;
    }
    else
    {
      x = cu.block(COMPONENT_Cb).x << m_shiftHor;
      y = cu.block(COMPONENT_Cb).y << m_shiftVer;
    }
    m_ctuXLumaSamples = x & ~(cu.slice->getSPS()->getMaxCUWidth()  - 1);
    m_ctuYLumaSamples = y & ~(cu.slice->getSPS()->getMaxCUHeight() - 1);
  }

  for( auto &currTU : CU::traverseTUs( cu ) )//遍历所有的TU
  {
    const Area& areaTu = cu.Y().valid() ? currTU.block( COMPONENT_Y ) : Area( recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, currTU.blocks[cu.chType].pos() ), recalcSize( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, currTU.blocks[cu.chType].size() ) );
    // 垂直和水平边界滤波设置为CU的中间边界
    verEdgeFilter = m_stLFCUParam.internalEdge;
    horEdgeFilter = m_stLFCUParam.internalEdge;

    if ((edgeDir == EDGE_HOR && areaTu.y % 4 != 0) || (edgeDir == EDGE_VER && areaTu.x % 4 != 0))
    {
      if (cu.chromaFormat != CHROMA_400 && currTU.block(COMPONENT_Cb).valid())
      {
        if ((edgeDir == EDGE_HOR && currTU.block(COMPONENT_Cb).y % 4 == 0)
            || (edgeDir == EDGE_VER && currTU.block(COMPONENT_Cb).x % 4 == 0))
        {
          // Set max filter length for chroma in narrow/short CUs that use ISP mode
          xSetMaxFilterLengthPQFromTransformSizes(edgeDir, cu, currTU, COMPONENT_Cb);
        }
      }
      continue; // 跳过不是4的整数倍的边界
    }

    if( isCuCrossedByVirtualBoundaries )//虚拟边界
    {
      xDeriveEdgefilterParam( areaTu.x, areaTu.y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos, verEdgeFilter, horEdgeFilter );
    }
    xSetEdgefilterMultiple( cu, EDGE_VER, areaTu, verEdgeFilter );//设置TU的垂直边界
    xSetEdgefilterMultiple( cu, EDGE_HOR, areaTu, horEdgeFilter );//设置TU的水平边界
    xSetMaxFilterLengthPQFromTransformSizes(edgeDir, cu, currTU, COMPONENT_Y);//设置根据TU边界长度确定参与滤波的像素数
    if (cu.Y().valid())                                     //向边界索引中添加该边界的信息
    {
      edgeIdx.push_back( ( edgeDir == EDGE_HOR ) ? ( currTU.blocks[cu.chType].y - cu.blocks[cu.chType].y ) / 4 : ( currTU.blocks[cu.chType].x - cu.blocks[cu.chType].x ) / 4 );
    }
    else
    {
      edgeIdx.push_back( ( edgeDir == EDGE_HOR ) ? (( currTU.blocks[cu.chType].y - cu.blocks[cu.chType].y ) << ::getComponentScaleY(COMPONENT_Cb, cu.chromaFormat))  / 4 : (( currTU.blocks[cu.chType].x - cu.blocks[cu.chType].x ) << ::getComponentScaleX(COMPONENT_Cb, cu.chromaFormat)) / 4 );
    }
  }

  bool mvSubBlocks = false;
  int subBlockSize = 8;
  for( auto &currPU : CU::traversePUs( cu ) )//遍历PU
  {
    const Area& areaPu = cu.Y().valid() ? currPU.block( COMPONENT_Y ) : area;//当前PU的左上角坐标和CU的左上角的坐标是否一致
    const bool xOff    = currPU.blocks[cu.chType].x != cu.blocks[cu.chType].x;
    const bool yOff    = currPU.blocks[cu.chType].y != cu.blocks[cu.chType].y;

    verEdgeFilter = (xOff ? m_stLFCUParam.internalEdge : m_stLFCUParam.leftEdge);
    horEdgeFilter = (yOff ? m_stLFCUParam.internalEdge : m_stLFCUParam.topEdge);
    if( isCuCrossedByVirtualBoundaries )
    {
      xDeriveEdgefilterParam( areaPu.x, areaPu.y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos, verEdgeFilter, horEdgeFilter );
    }

    xSetEdgefilterMultiple( cu, EDGE_VER, areaPu, verEdgeFilter, xOff );
    xSetEdgefilterMultiple( cu, EDGE_HOR, areaPu, horEdgeFilter, yOff );
    edgeIdx.push_back( ( edgeDir == EDGE_HOR ) ? ( currPU.blocks[cu.chType].y - cu.blocks[cu.chType].y ) / 4 : ( currPU.blocks[cu.chType].x - cu.blocks[cu.chType].x ) / 4 );

    if ((currPU.mergeFlag && (currPU.mergeType == MRG_TYPE_SUBPU_ATMVP)) || cu.affine)//如果是SBTMVP或affine
    {
      mvSubBlocks = true;
      if (edgeDir == EDGE_HOR)
      {
        for (uint32_t off = subBlockSize; off < areaPu.height; off += subBlockSize)
        {
          const Area mvBlockH(cu.Y().x, cu.Y().y + off, cu.Y().width, pcv.minCUHeight);
          horEdgeFilter = m_stLFCUParam.internalEdge;
          if( isCuCrossedByVirtualBoundaries )
          {
            xDeriveEdgefilterParam( mvBlockH.x, mvBlockH.y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos, verEdgeFilter, horEdgeFilter );
          }

          xSetEdgefilterMultiple(cu, EDGE_HOR, mvBlockH, horEdgeFilter, 1);
          edgeIdx.push_back( ( currPU.blocks[cu.chType].y + off - cu.blocks[cu.chType].y ) / 4 );
        }
      }
      else
      {
        for (uint32_t off = subBlockSize; off < areaPu.width; off += subBlockSize)
        {
          const Area mvBlockV(cu.Y().x + off, cu.Y().y, pcv.minCUWidth, cu.Y().height);
          verEdgeFilter = m_stLFCUParam.internalEdge;
          if( isCuCrossedByVirtualBoundaries )
          {
            xDeriveEdgefilterParam( mvBlockV.x, mvBlockV.y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos, verEdgeFilter, horEdgeFilter );
          }

          xSetEdgefilterMultiple(cu, EDGE_VER, mvBlockV, verEdgeFilter, 1);
          edgeIdx.push_back( ( currPU.blocks[cu.chType].x + off - cu.blocks[cu.chType].x ) / 4 );
        }
      }
    }

    xSetMaxFilterLengthPQForCodingSubBlocks( edgeDir, cu, currPU, mvSubBlocks, subBlockSize, areaPu );
  }

  const unsigned uiPelsInPart = pcv.minCUWidth;// 4x4单元
  /*******************遍历所有的以4x4为基本单元的边界，获得每个4x4单元的边界强度*********************/
  for( int y = 0; y < area.height; y += uiPelsInPart )
  {
    for( int x = 0; x < area.width; x += uiPelsInPart )
    {
      unsigned uiBSCheck = 1;
      const Position localPos  { area.x + x, area.y + y };
      const unsigned rasterIdx = getRasterIdx( localPos, pcv );//当前4x4单元的光栅扫描顺序索引
      // 参数m_aapbEdgeFilter在xSetEdgefilterMultiple中设置
      if( m_aapbEdgeFilter[edgeDir][rasterIdx] && uiBSCheck )
      {
        char bS = 0;   // 获取边界强度
        if(cu.treeType != TREE_C)
        {
          bS |= xGetBoundaryStrengthSingle( cu, edgeDir, localPos, CHANNEL_TYPE_LUMA );
        }
        if(cu.treeType != TREE_L && cu.chromaFormat != CHROMA_400 && cu.blocks[COMPONENT_Cb].valid())
        {
          bS |= xGetBoundaryStrengthSingle( cu, edgeDir, localPos, CHANNEL_TYPE_CHROMA );
        }
        m_aapucBS[edgeDir][rasterIdx] = bS;
      }
    }
  }

   /************************* 进行最终滤波 ***********************/
  std::sort( edgeIdx.begin(), edgeIdx.end() );
  int prevEdgeIdx = -1;
  for ( const int& edge : edgeIdx )//遍历所有的边界
  {
    if ( edge == prevEdgeIdx ) // skip duplicate edgeIdx marked by both transform and coding subblock processes
    {
      continue;//跳过由变换和编码子块过程标记的重复edgeIdx
    }
    prevEdgeIdx = edge;

    if ( cu.blocks[COMPONENT_Y].valid() )
    {
      xEdgeFilterLuma( cu, edgeDir, edge );
    }

    if ( pcv.chrFormat != CHROMA_400 && cu.blocks[COMPONENT_Cb].valid() )
    {
      if ( !cu.ispMode || edge == 0 )
      {
        xEdgeFilterChroma( cu, edgeDir, edge );
      }
    }
  }
}

inline bool DeblockingFilter::isCrossedByVirtualBoundaries(const int xPos, const int yPos, const int width, const int height, int& numHorVirBndry, int& numVerVirBndry, int horVirBndryPos[], int verVirBndryPos[], const PicHeader* picHeader )
{
  numHorVirBndry = 0; numVerVirBndry = 0;
  if( picHeader->getVirtualBoundariesPresentFlag() )
  {
    for (int i = 0; i < picHeader->getNumHorVirtualBoundaries(); i++)
    {
      if (yPos <= picHeader->getVirtualBoundariesPosY(i) && picHeader->getVirtualBoundariesPosY(i) < yPos + height)
      {
        horVirBndryPos[numHorVirBndry++] = picHeader->getVirtualBoundariesPosY(i);
      }
    }
    for (int i = 0; i < picHeader->getNumVerVirtualBoundaries(); i++)
    {
      if (xPos <= picHeader->getVirtualBoundariesPosX(i) && picHeader->getVirtualBoundariesPosX(i) < xPos + width)
      {
        verVirBndryPos[numVerVirBndry++] = picHeader->getVirtualBoundariesPosX(i);
      }
    }
  }
  return numHorVirBndry > 0 || numVerVirBndry > 0;
}

inline void DeblockingFilter::xDeriveEdgefilterParam( const int xPos, const int yPos, const int numVerVirBndry, const int numHorVirBndry, const int verVirBndryPos[], const int horVirBndryPos[], bool &verEdgeFilter, bool &horEdgeFilter )
{
  for (int i = 0; i < numVerVirBndry; i++)
  {
    if (verVirBndryPos[i] == xPos)
    {
      verEdgeFilter = false;
      break;
    }
  }

  for (int i = 0; i < numHorVirBndry; i++)
  {
    if (horVirBndryPos[i] == yPos)
    {
      horEdgeFilter = false;
      break;
    }
  }
}
/*
根据边界处相邻块的长度判断需要进行滤波的最大像素数，记边界两侧的块分别为P块和Q块

对于垂直边界，需要判断P块和Q块的宽度（width）；对于水平边界，需要判断P块和Q块的高度（Height）

对于亮度分量，分为以下三种情况：

小块（小于等于4），最多滤波1个像素
大块（大于等于32），最多滤波7个像素
否则，最多滤波3个像素
对于色度分量，分为以下两种情况：

大块（大于等于8），最多滤波3个像素
否则，最多滤波1个像素
这里只是计算了每个边界在进行滤波时的最大像素数，其真正进行滤波的像素数还需要根据滤波强度决策来决定。

*/
void DeblockingFilter::xSetMaxFilterLengthPQFromTransformSizes(const DeblockEdgeDir edgeDir, const CodingUnit &cu,
                                                         const TransformUnit &currTU, const int firstComponent)
{
  const TransformUnit& tuQ = currTU;// 将Q设置为当前TU

  if ( edgeDir == EDGE_HOR )//水平边界
  {// 遍历所有的颜色分量
    for (int cIdx = firstComponent; cIdx < ::getNumberValidComponents(tuQ.chromaFormat); cIdx++)   // per component
    {
      const ComponentID comp = ComponentID(cIdx);
      const ChannelType ch   = toChannelType(comp);
      const int shiftHor     = ( ( ch == CH_L ) ? 0 : m_shiftHor );
      const int shiftVer     = ( ( ch == CH_L ) ? 0 : m_shiftVer );
      const int ctuXOff      = currTU.block(comp).x - ( m_ctuXLumaSamples >> shiftHor ); // x offset from left edge of CTU in respective channel sample units
      const int ctuYOff      = currTU.block(comp).y - ( m_ctuYLumaSamples >> shiftVer ); // y offset from top edge of CTU in respective channel sample units
      const int minCUWidth   = cu.cs->pcv->minCUWidth >> shiftHor;
      if ( currTU.block(comp).valid() && ( ( currTU.block(comp).y == cu.block(comp).y ) ? m_stLFCUParam.topEdge : m_stLFCUParam.internalEdge ) ) // Edge deblocking needs to be recomputed since ISP contains whole CU chroma transforms in last TU of the CU
      {   //边缘去块需要重新计算，因为ISP在CU的最后一个TU中包含整个CU色度变换
        for (int x = 0; x < currTU.blocks[cIdx].width; x += minCUWidth)   //以4x4为单位遍历边界
        {
          const Position  posQ     = Position( currTU.blocks[ch].x + x, currTU.blocks[ch].y );
          const Position  posP     = posQ.offset( 0, -1 );
          const int sizeQSide      = tuQ.block(comp).height;//Q像素长度
          const TransformUnit& tuP = *cu.cs->getTU( posP, ch );
          const int sizePSide      = tuP.block(comp).height;//P像素长度
          m_transformEdge[cIdx][ctuXOff+x][ctuYOff] = true;

          if ( comp == COMPONENT_Y )
          {
            bool smallBlock = (sizePSide <= 4) || (sizeQSide <= 4);
            if (smallBlock)//对于亮度的小块（小于等于4x4），仅滤波1个像素
            {
              m_maxFilterLengthQ[cIdx][ctuXOff + x][ctuYOff] = 1;
              m_maxFilterLengthP[cIdx][ctuXOff + x][ctuYOff] = 1;
            }
            else
            {//对于大块（大于等于32），滤波7个像素，否则滤波3个像素
              m_maxFilterLengthQ[cIdx][ctuXOff + x][ctuYOff] = (sizeQSide >= 32) ? 7 : 3;
              m_maxFilterLengthP[cIdx][ctuXOff + x][ctuYOff] = (sizePSide >= 32) ? 7 : 3;
            }
          }
          else
          {   //对于色度大块（大于等于8），滤波3个像素，否则滤波1个像素
            m_maxFilterLengthQ[cIdx][ctuXOff+x][ctuYOff] = ( sizeQSide >= 8 && sizePSide >= 8 ) ? 3 : 1;
            m_maxFilterLengthP[cIdx][ctuXOff+x][ctuYOff] = ( sizeQSide >= 8 && sizePSide >= 8 ) ? 3 : 1;
          }
        }
      }
    }
  }
  if ( edgeDir == EDGE_VER )//垂直边界,重复上述流程
  {
    for ( int cIdx = firstComponent; cIdx < ::getNumberValidComponents(tuQ.chromaFormat); cIdx++ ) // per component
    {
      const ComponentID comp = ComponentID(cIdx);
      const ChannelType ch   = toChannelType(comp);
      const int shiftHor     = ( ( ch == CH_L ) ? 0 : m_shiftHor );
      const int shiftVer     = ( ( ch == CH_L ) ? 0 : m_shiftVer );
      const int ctuXOff      = currTU.block(comp).x - ( m_ctuXLumaSamples >> shiftHor ); // x offset from left edge of CTU in respective channel sample units
      const int ctuYOff      = currTU.block(comp).y - ( m_ctuYLumaSamples >> shiftVer ); // y offset from top edge of CTU in respective channel sample units
      const int minCUHeight  = cu.cs->pcv->minCUHeight >> shiftVer;
      if ( currTU.block(comp).valid() && ( ( currTU.block(comp).x == cu.block(comp).x ) ? m_stLFCUParam.leftEdge : m_stLFCUParam.internalEdge ) ) // Edge deblocking needs to be recomputed since ISP contains whole CU chroma transforms in last TU of the CU
      {
        for ( int y = 0; y < currTU.blocks[cIdx].height; y += minCUHeight )
        {
          const Position  posQ     = Position( currTU.blocks[ch].x, currTU.blocks[ch].y + y );
          const Position  posP     = posQ.offset( -1, 0 );
          const int sizeQSide      = tuQ.block(comp).width;
          const TransformUnit& tuP = *cu.cs->getTU( posP, ch );
          const int sizePSide      = tuP.block(comp).width;
          m_transformEdge[cIdx][ctuXOff][ctuYOff+y] = true;

          if ( comp == COMPONENT_Y )
          {
            bool smallBlock = (sizePSide <= 4) || (sizeQSide <= 4);
            if (smallBlock)
            {
              m_maxFilterLengthQ[cIdx][ctuXOff][ctuYOff + y] = 1;
              m_maxFilterLengthP[cIdx][ctuXOff][ctuYOff + y] = 1;
            }
            else
            {
              m_maxFilterLengthQ[cIdx][ctuXOff][ctuYOff + y] = (sizeQSide >= 32) ? 7 : 3;
              m_maxFilterLengthP[cIdx][ctuXOff][ctuYOff + y] = (sizePSide >= 32) ? 7 : 3;
            }
          }
          else
          {
            m_maxFilterLengthQ[cIdx][ctuXOff][ctuYOff+y] = ( sizeQSide >= 8 && sizePSide >= 8 ) ? 3 : 1;
            m_maxFilterLengthP[cIdx][ctuXOff][ctuYOff+y] = ( sizeQSide >= 8 && sizePSide >= 8 ) ? 3 : 1;
          }
        }
      }
    }
  }
}

void DeblockingFilter::xSetMaxFilterLengthPQForCodingSubBlocks( const DeblockEdgeDir edgeDir, const CodingUnit& cu, const PredictionUnit& currPU, const bool& mvSubBlocks, const int& subBlockSize, const Area& areaPu )
{
  if ( mvSubBlocks && currPU.Y().valid() )
  {
    const int cIdx         = 0;
    const ComponentID comp = ComponentID(cIdx);
    const int ctuYOff      = currPU.block(comp).y - m_ctuYLumaSamples; // y offset from top edge of CTU in luma samples
    const int ctuXOff      = currPU.block(comp).x - m_ctuXLumaSamples; // x offset from left edge of CTU in luma samples
    const int minCUWidth   = cu.cs->pcv->minCUWidth;
    const int minCUHeight  = cu.cs->pcv->minCUHeight;
    if ( edgeDir == EDGE_HOR )
    {
      for ( int y = 0; y < areaPu.height; y += subBlockSize )
      {
        for ( int x = 0; x < areaPu.width; x += minCUWidth )
        {
          if ( m_transformEdge[cIdx][ctuXOff+x][ctuYOff+y] )
          {
            m_maxFilterLengthQ[cIdx][ctuXOff+x][ctuYOff+y] = std::min<int>(m_maxFilterLengthQ[cIdx][ctuXOff+x][ctuYOff+y], 5);
            if ( y > 0 )
            {
              m_maxFilterLengthP[cIdx][ctuXOff+x][ctuYOff+y] = std::min<int>(m_maxFilterLengthP[cIdx][ctuXOff+x][ctuYOff+y], 5);
            }
          }
          else if (y > 0 && (m_transformEdge[cIdx][ctuXOff + x][ctuYOff + y - 4] || ((y + 4) >= areaPu.height) || m_transformEdge[cIdx][ctuXOff + x][ctuYOff + y + 4])) // adjacent to transform edge  +/- 4
          {
            m_maxFilterLengthQ[cIdx][ctuXOff + x][ctuYOff + y] = 1;
            m_maxFilterLengthP[cIdx][ctuXOff + x][ctuYOff + y] = 1;
          }
          else if (y > 0 && ( ( y == 8 ) || m_transformEdge[cIdx][ctuXOff+x][ctuYOff+y-8] || (( y + 8 ) >= areaPu.height) || m_transformEdge[cIdx][ctuXOff+x][ctuYOff+y+8] )) // adjacent to transform edge on 8x8 grid
          {
            m_maxFilterLengthQ[cIdx][ctuXOff+x][ctuYOff+y] = 2;
            m_maxFilterLengthP[cIdx][ctuXOff+x][ctuYOff+y] = 2;
          }
          else
          {
            m_maxFilterLengthQ[cIdx][ctuXOff+x][ctuYOff+y] = 3;
            m_maxFilterLengthP[cIdx][ctuXOff+x][ctuYOff+y] = 3;
          }
        }
      }
    }
    else // edgeDir == EDGE_VER
    {
      for ( int x = 0; x < areaPu.width; x += subBlockSize )
      {
        for ( int y = 0; y < areaPu.height; y += minCUHeight )
        {
          if ( m_transformEdge[cIdx][ctuXOff+x][ctuYOff+y] )
          {
            m_maxFilterLengthQ[cIdx][ctuXOff+x][ctuYOff+y] = std::min<int>(m_maxFilterLengthQ[cIdx][ctuXOff+x][ctuYOff+y], 5);
            if ( x > 0 )
            {
              m_maxFilterLengthP[cIdx][ctuXOff+x][ctuYOff+y] = std::min<int>(m_maxFilterLengthP[cIdx][ctuXOff+x][ctuYOff+y], 5);
            }
          }
          else if (x > 0 && (m_transformEdge[cIdx][ctuXOff + x - 4][ctuYOff + y] || ((x + 4) >= areaPu.width) || m_transformEdge[cIdx][ctuXOff + x + 4][ctuYOff + y])) // adjacent to transform edge +/- 4
          {
            m_maxFilterLengthQ[cIdx][ctuXOff + x][ctuYOff + y] = 1;
            m_maxFilterLengthP[cIdx][ctuXOff + x][ctuYOff + y] = 1;
          }
          else if ( x > 0 && ( ( x == 8 ) || m_transformEdge[cIdx][ctuXOff+x-8][ctuYOff+y] || ( (x + 8) >= areaPu.width ) || m_transformEdge[cIdx][ctuXOff+x+8][ctuYOff+y] ) ) // adjacent to transform edge on 8x8 grid
          {
            m_maxFilterLengthQ[cIdx][ctuXOff+x][ctuYOff+y] = 2;
            m_maxFilterLengthP[cIdx][ctuXOff+x][ctuYOff+y] = 2;
          }
          else
          {
            m_maxFilterLengthQ[cIdx][ctuXOff+x][ctuYOff+y] = 3;
            m_maxFilterLengthP[cIdx][ctuXOff+x][ctuYOff+y] = 3;
          }
        }
      }
    }
  }
}
//遍历全部的边界（垂直或者水平），设置其是否使用滤波
void DeblockingFilter::xSetEdgefilterMultiple( const CodingUnit&    cu,
                                         const DeblockEdgeDir edgeDir,
                                         const Area&          area,
                                         const bool           bValue,
                                         const bool           EdgeIdx )
{
  const PreCalcValues& pcv = *cu.cs->pcv;

  const unsigned uiAdd     = ( edgeDir == EDGE_VER ) ? pcv.partsInCtuWidth : 1;//垂直滤波往下走1个，水平滤波往右走1个
  const unsigned uiNumElem = ( edgeDir == EDGE_VER ) ? ( area.height / pcv.minCUHeight ) : ( area.width / pcv.minCUWidth );
  unsigned uiBsIdx         = getRasterIdx( area, pcv );//第一个边界光栅扫描的索引值

  for( int ui = 0; ui < uiNumElem; ui++ )//遍历所有的边界（垂直或者水平）
  {
    m_aapbEdgeFilter[edgeDir][uiBsIdx] = bValue;//设置是否使用滤波器
    if ( m_aapucBS[edgeDir][uiBsIdx] && bValue )
    {
      m_aapucBS[edgeDir][uiBsIdx] = 3;  // both the TU and PU edge   表示既是TU边界也是PU边界
    }
    else
    {
      if( ! EdgeIdx )
      {
        m_aapucBS[edgeDir][uiBsIdx] = bValue;
      }
    }
    uiBsIdx += uiAdd;
  }
}

void DeblockingFilter::xSetDeblockingFilterParam( const CodingUnit& cu )
{
  const Slice& slice = *cu.slice;
  const PPS&   pps   = *cu.cs->pps;

  if( slice.getDeblockingFilterDisable() )
  {
    m_stLFCUParam.leftEdge = m_stLFCUParam.topEdge = m_stLFCUParam.internalEdge = false;
    return;
  }

  const Position& pos = cu.blocks[cu.chType].pos();

  m_stLFCUParam.internalEdge = true;

  m_stLFCUParam.leftEdge = (0 < pos.x) && isAvailableLeft(cu, *cu.cs->getCU(pos.offset(-1, 0), cu.chType), !pps.getLoopFilterAcrossSlicesEnabledFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag(),
    !( pps.getSubPicFromCU(cu).getloopFilterAcrossEnabledFlag() && pps.getSubPicFromCU(*cu.cs->getCU(pos.offset(-1, 0), cu.chType)).getloopFilterAcrossEnabledFlag()));
  m_stLFCUParam.topEdge = (0 < pos.y) && isAvailableAbove(cu, *cu.cs->getCU(pos.offset(0, -1), cu.chType), !pps.getLoopFilterAcrossSlicesEnabledFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag(),
    !( pps.getSubPicFromCU(cu).getloopFilterAcrossEnabledFlag() && pps.getSubPicFromCU(*cu.cs->getCU(pos.offset(0, -1), cu.chType)).getloopFilterAcrossEnabledFlag()));
}
//确定滤波强度

unsigned DeblockingFilter::xGetBoundaryStrengthSingle ( const CodingUnit& cu, const DeblockEdgeDir edgeDir, const Position& localPos, const ChannelType chType ) const
{
  // The boundary strength that is output by the function xGetBoundaryStrengthSingle is a multi component boundary strength that contains boundary strength for luma (bits 0 to 1), cb (bits 2 to 3) and cr (bits 4 to 5).
  //函数xGetBoundaryStrengthSingle输出的边界强度是多分量边界强度，它包含luma（位0到1）、cb（位2到3）和cr（位4到5）的边界强度。
  const Slice& sliceQ = *cu.slice;

  int shiftHor = cu.Y().valid() ? 0 : ::getComponentScaleX(COMPONENT_Cb, cu.firstPU->chromaFormat);
  int shiftVer = cu.Y().valid() ? 0 : ::getComponentScaleY(COMPONENT_Cb, cu.firstPU->chromaFormat);
  const Position& posQ = Position{ localPos.x >> shiftHor,  localPos.y >> shiftVer };
  const Position  posP  = ( edgeDir == EDGE_VER ) ? posQ.offset( -1, 0 ) : posQ.offset( 0, -1 );

  const CodingUnit& cuQ = cu;
  const CodingUnit& cuP = (chType == CHANNEL_TYPE_CHROMA && cuQ.chType == CHANNEL_TYPE_LUMA) ?
                          *cu.cs->getCU(recalcPosition( cu.chromaFormat, CHANNEL_TYPE_LUMA, CHANNEL_TYPE_CHROMA, posP), CHANNEL_TYPE_CHROMA) :
                          *cu.cs->getCU( posP, cu.chType );

  //-- Set BS for Intra MB : BS = 4 or 3 相邻块中的至少一个用intra或CIIP模式编码
  if( ( MODE_INTRA == cuP.predMode ) || ( MODE_INTRA == cuQ.predMode ) ) //首先处理I帧（包括CIIP）
  {
    if( chType == CHANNEL_TYPE_LUMA )
    {
      int bsY = (MODE_INTRA == cuP.predMode && cuP.bdpcmMode) && (MODE_INTRA == cuQ.predMode && cuQ.bdpcmMode) ? 0 : 2;
      return BsSet(bsY, COMPONENT_Y);
    }
    else
    {
      int bsC = (MODE_INTRA == cuP.predMode && cuP.bdpcmModeChroma) && (MODE_INTRA == cuQ.predMode && cuQ.bdpcmModeChroma) ? 0 : 2;
      return (BsSet(bsC, COMPONENT_Cb) + BsSet(bsC, COMPONENT_Cr));
    }
  }

  const TransformUnit& tuQ = *cuQ.cs->getTU(posQ, cuQ.chType);
  const TransformUnit& tuP = (cuP.chType == CHANNEL_TYPE_CHROMA && cuQ.chType == CHANNEL_TYPE_LUMA) ?
                             *cuP.cs->getTU(recalcPosition( cu.chromaFormat, CHANNEL_TYPE_LUMA, CHANNEL_TYPE_CHROMA, posP), CHANNEL_TYPE_CHROMA) :
                             *cuP.cs->getTU(posP, cuQ.chType);

  const PreCalcValues& pcv = *cu.cs->pcv;
  const unsigned rasterIdx = getRasterIdx( Position{ localPos.x,  localPos.y }, pcv );
  if (m_aapucBS[edgeDir][rasterIdx] && (cuP.firstPU->ciipFlag || cuQ.firstPU->ciipFlag))
  {
    if(chType == CHANNEL_TYPE_LUMA)
    {
      return BsSet(2, COMPONENT_Y);
    }
    else
    {
      return BsSet(2, COMPONENT_Cb) + BsSet(2, COMPONENT_Cr);
    }
  }

  unsigned tmpBs = 0;
  //-- Set BS for not Intra MB : BS = 2 or 1 or 0 相邻块中的至少一个具有非零变换系数
  if(chType == CHANNEL_TYPE_LUMA)
  {
    // Y
    if (m_aapucBS[edgeDir][rasterIdx] && (TU::getCbf(tuQ, COMPONENT_Y) || TU::getCbf(tuP, COMPONENT_Y)))
    {
      tmpBs += BsSet(1, COMPONENT_Y);
    }
  }
  else
  {
    if (pcv.chrFormat != CHROMA_400)
    {
      // U
      if (m_aapucBS[edgeDir][rasterIdx]
          && (TU::getCbf(tuQ, COMPONENT_Cb) || TU::getCbf(tuP, COMPONENT_Cb) || tuQ.jointCbCr || tuP.jointCbCr))
      {
        tmpBs += BsSet(1, COMPONENT_Cb);
      }
      // V
      if (m_aapucBS[edgeDir][rasterIdx]
          && (TU::getCbf(tuQ, COMPONENT_Cr) || TU::getCbf(tuP, COMPONENT_Cr) || tuQ.jointCbCr || tuP.jointCbCr))
      {
        tmpBs += BsSet(1, COMPONENT_Cr);
      }
    }
  }
  if (BsGet(tmpBs, COMPONENT_Y) == 1)
  {
    return tmpBs;
  }

  if ( !cu.Y().valid() )
  {
    return tmpBs;
  }

  // and now the pred
  if (m_aapucBS[edgeDir][rasterIdx] != 0 && m_aapucBS[edgeDir][rasterIdx] != 3)
  {
    return tmpBs;
  }
  if( chType == CHANNEL_TYPE_CHROMA )
  {
    return tmpBs;
  }
  if( cuP.predMode != cuQ.predMode && chType == CHANNEL_TYPE_LUMA )
  {
    return BsSet(1, COMPONENT_Y);
  }
  const Position& lumaPosQ  = Position{ localPos.x,  localPos.y };
  const Position  lumaPosP  = ( edgeDir == EDGE_VER ) ? lumaPosQ.offset( -1, 0 ) : lumaPosQ.offset( 0, -1 );
  const MotionInfo&     miQ = cuQ.cs->getMotionInfo( lumaPosQ );
  const MotionInfo&     miP = cuP.cs->getMotionInfo( lumaPosP );
  const Slice&       sliceP = *cuP.slice;

  if (sliceQ.isInterB() || sliceP.isInterB()) //帧间模式 帧间B帧 
  {
    const Picture *piRefP0 = (CU::isIBC(cuP) ? sliceP.getPic() : ((0 > miP.refIdx[0]) ? NULL : sliceP.getRefPic(REF_PIC_LIST_0, miP.refIdx[0])));
    const Picture *piRefP1 = (CU::isIBC(cuP) ? NULL            : ((0 > miP.refIdx[1]) ? NULL : sliceP.getRefPic(REF_PIC_LIST_1, miP.refIdx[1])));
    const Picture *piRefQ0 = (CU::isIBC(cuQ) ? sliceQ.getPic() : ((0 > miQ.refIdx[0]) ? NULL : sliceQ.getRefPic(REF_PIC_LIST_0, miQ.refIdx[0])));
    const Picture *piRefQ1 = (CU::isIBC(cuQ) ? NULL            : ((0 > miQ.refIdx[1]) ? NULL : sliceQ.getRefPic(REF_PIC_LIST_1, miQ.refIdx[1])));
    Mv mvP0, mvP1, mvQ0, mvQ1;

    if (0 <= miP.refIdx[0])
    {
      mvP0 = miP.mv[0];
    }
    if (0 <= miP.refIdx[1])
    {
      mvP1 = miP.mv[1];
    }
    if (0 <= miQ.refIdx[0])
    {
      mvQ0 = miQ.mv[0];
    }
    if (0 <= miQ.refIdx[1])
    {
      mvQ1 = miQ.mv[1];
    }

    int nThreshold = (1 << MV_FRACTIONAL_BITS_INTERNAL) >> 1;
    unsigned uiBs = 0;

    //th can be optimized
    if ( ((piRefP0==piRefQ0)&&(piRefP1==piRefQ1)) || ((piRefP0==piRefQ1)&&(piRefP1==piRefQ0)) )
    {
      if ( piRefP0 != piRefP1 )   // Different L0 & L1
      {
        if ( piRefP0 == piRefQ0 )
        {
          uiBs  = ((abs(mvQ0.getHor() - mvP0.getHor()) >= nThreshold) || (abs(mvQ0.getVer() - mvP0.getVer()) >= nThreshold) ||
                   (abs(mvQ1.getHor() - mvP1.getHor()) >= nThreshold) || (abs(mvQ1.getVer() - mvP1.getVer()) >= nThreshold))
                  ? 1 : 0;
        }
        else
        {
          uiBs  = ((abs(mvQ1.getHor() - mvP0.getHor()) >= nThreshold) || (abs(mvQ1.getVer() - mvP0.getVer()) >= nThreshold) ||
                   (abs(mvQ0.getHor() - mvP1.getHor()) >= nThreshold) || (abs(mvQ0.getVer() - mvP1.getVer()) >= nThreshold))
                  ? 1 : 0;
        }
      }
      else    // Same L0 & L1
      {
        uiBs  = ((abs(mvQ0.getHor() - mvP0.getHor()) >= nThreshold) || (abs(mvQ0.getVer() - mvP0.getVer()) >= nThreshold) ||
                 (abs(mvQ1.getHor() - mvP1.getHor()) >= nThreshold) || (abs(mvQ1.getVer() - mvP1.getVer()) >= nThreshold))
              &&
                ((abs(mvQ1.getHor() - mvP0.getHor()) >= nThreshold) || (abs(mvQ1.getVer() - mvP0.getVer()) >= nThreshold) ||
                 (abs(mvQ0.getHor() - mvP1.getHor()) >= nThreshold) || (abs(mvQ0.getVer() - mvP1.getVer()) >= nThreshold))
              ? 1 : 0;
      }
    }
    else // for all different Ref_Idx
    {
      uiBs = 1;
    }
    return uiBs + tmpBs;
  }


  // pcSlice->isInterP()  帧间P帧：如果MVD大于等于半像素精度 
  CHECK(CU::isInter(cuP) && 0 > miP.refIdx[0], "Invalid reference picture list index");
  CHECK(CU::isInter(cuP) && 0 > miQ.refIdx[0], "Invalid reference picture list index");
  //相邻块中的一个以IBC预测模式编码，另一个以帧间预测模式编码
  const Picture *piRefP0 = (CU::isIBC(cuP) ? sliceP.getPic() : sliceP.getRefPic(REF_PIC_LIST_0, miP.refIdx[0]));
  const Picture *piRefQ0 = (CU::isIBC(cuQ) ? sliceQ.getPic() : sliceQ.getRefPic(REF_PIC_LIST_0, miQ.refIdx[0]));
  if (piRefP0 != piRefQ0)
  {
    return tmpBs + 1;
  }

  Mv mvP0 = miP.mv[0];
  Mv mvQ0 = miQ.mv[0];

  int nThreshold = (1 << MV_FRACTIONAL_BITS_INTERNAL) >> 1;
  return ( ( abs( mvQ0.getHor() - mvP0.getHor() ) >= nThreshold ) || ( abs( mvQ0.getVer() - mvP0.getVer() ) >= nThreshold ) ) ? (tmpBs + 1) : tmpBs;
}

#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
void DeblockingFilter::deriveLADFShift( const Pel* src, const int stride, int& shift, const DeblockEdgeDir edgeDir, const SPS sps )
{
  uint32_t lumaLevel = 0;
  shift = sps.getLadfQpOffset(0);

  if (edgeDir == EDGE_VER)
  {
    lumaLevel = (src[0] + src[3*stride] + src[-1] + src[3*stride - 1]) >> 2;
  }
  else // (edgeDir == EDGE_HOR)
  {
    lumaLevel = (src[0] + src[3] + src[-stride] + src[-stride + 3]) >> 2;
  }

  for ( int k = 1; k < sps.getLadfNumIntervals(); k++ )
  {
    const int th = sps.getLadfIntervalLowerBound( k );
    if ( lumaLevel > th )
    {
      shift = sps.getLadfQpOffset( k );
    }
    else
    {
      break;
    }
  }
}
#endif
//对亮度像素的一边界进行去块滤波
/*
对于每一个长度为4的边界，进行如下过程：

获取滤波开关和滤波强度阈值参数：根据P块和Q块两侧的Qp值计算索引值，根据索引查表获得参数Tc和β
滤波开关决策，计算边界两侧像素的变化率dL（调用deriveLADFShift函数推导重建像素的平均亮度级）
如果dL小于β，表明该边界需要进行滤波。
对于需要滤波的边界，进行滤波强度决策，通过xUseStrongFiltering函数判断是否使用强滤波
调用xPelFilterLuma函数进行滤波过程

*/
void DeblockingFilter::xEdgeFilterLuma( const CodingUnit& cu, const DeblockEdgeDir edgeDir, const int iEdge )
{
  const CompArea&  lumaArea = cu.block(COMPONENT_Y);
  const PreCalcValues& pcv = *cu.cs->pcv;

  PelBuf        picYuvRec = m_enc ? m_encPicYuvBuffer.getBuf( lumaArea ) : cu.cs->getRecoBuf( lumaArea );
  Pel           *piSrc    = picYuvRec.buf;
  const int     iStride   = picYuvRec.stride;
  Pel           *piTmpSrc = piSrc;
  const PPS     &pps      = *(cu.cs->pps);
  const SPS     &sps      = *(cu.cs->sps);
  const Slice   &slice    = *(cu.slice);
  const bool    spsPaletteEnabledFlag          = sps.getPLTMode();
  const int     bitDepthLuma                   = sps.getBitDepth(CHANNEL_TYPE_LUMA);
  const ClpRng& clpRng( cu.cs->slice->clpRng(COMPONENT_Y) );

  int          iQP          = 0;
  unsigned     uiNumParts   = ( ( ( edgeDir == EDGE_VER ) ? lumaArea.height / pcv.minCUHeight : lumaArea.width / pcv.minCUWidth ) );
  int          pelsInPart   = pcv.minCUWidth;
  unsigned     uiBsAbsIdx   = 0, uiBs = 0;
  int          iOffset, iSrcStep;

  bool  bPartPNoFilter  = false;
  bool  bPartQNoFilter  = false;
  int   betaOffsetDiv2  = slice.getDeblockingFilterBetaOffsetDiv2();
  int   tcOffsetDiv2    = slice.getDeblockingFilterTcOffsetDiv2();
  int   xoffset, yoffset;

  Position pos;

  if (edgeDir == EDGE_VER)//垂直滤波所需信息
  {
    xoffset   = 0;
    yoffset   = pelsInPart;
    iOffset   = 1;
    iSrcStep  = iStride;
    piTmpSrc += iEdge * pelsInPart;
    pos       = Position{ lumaArea.x + iEdge * pelsInPart, lumaArea.y - yoffset };
  }
  else  // (edgeDir == EDGE_HOR)
  {
    xoffset   = pelsInPart;
    yoffset   = 0;
    iOffset   = iStride;
    iSrcStep  = 1;
    piTmpSrc += iEdge*pelsInPart*iStride;
    pos       = Position{ lumaArea.x - xoffset, lumaArea.y + iEdge * pelsInPart };
  }

  const int iBitdepthScale = 1 << (bitDepthLuma - 8); //处理高位深需要对滤波器进行缩放

  // dec pos since within the loop we first calc the pos 遍历所有的minCU
  // 对于垂直边界，遍历该垂直边界中的所有长度为4的区域，因此iSrcStep=iStride；
  // 垂直边界滤波处理的是水平方向的像素，因此iOffset=1
  // 对于水平边界，遍历该水平边界中的所有长度为4的区域，因此iSrcStep=1
  // 水平边界滤波处理的是垂直方向的像素，因此iOffset=iStride

  for( int iIdx = 0; iIdx < uiNumParts; iIdx++ )
  {
    pos.x += xoffset;
    pos.y += yoffset;

    // Deblock luma boundaries on 4x4 grid only 只以亮度4x4网格作为边界，否则不滤波
    if (edgeDir == EDGE_HOR && (pos.y % 4) != 0)
    {
      continue;
    }
    if (edgeDir == EDGE_VER && (pos.x % 4) != 0)
    {
      continue;
    }
    uiBsAbsIdx = getRasterIdx( pos, pcv );//获取网格索引
    uiBs = BsGet(m_aapucBS[edgeDir][uiBsAbsIdx], COMPONENT_Y);//获取对应边界的边界强度

    if( uiBs )
    {
      const CodingUnit &cuQ = cu;   //当前CU
      const CodingUnit& cuP = *cu.cs->getCU(pos.offset(xoffset - pelsInPart, yoffset - pelsInPart), cu.chType);// 相邻CU
      // Derive neighboring PU index
      if (edgeDir == EDGE_VER)
      {
        if (!isAvailableLeft(cu, cuP, !pps.getLoopFilterAcrossSlicesEnabledFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag(),
          !( pps.getSubPicFromCU(cu).getloopFilterAcrossEnabledFlag() && pps.getSubPicFromCU(cuP).getloopFilterAcrossEnabledFlag())))
        {
          m_aapucBS[edgeDir][uiBsAbsIdx] = uiBs = 0;
          continue;
        }
      }
      else  // (iDir == EDGE_HOR)
      {
        if (!isAvailableAbove(cu, cuP, !pps.getLoopFilterAcrossSlicesEnabledFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag(),
          !( pps.getSubPicFromCU(cu).getloopFilterAcrossEnabledFlag() && pps.getSubPicFromCU(cuP).getloopFilterAcrossEnabledFlag())))
        {
          m_aapucBS[edgeDir][uiBsAbsIdx] = uiBs = 0;
          continue;
        }
      }

      iQP = (cuP.qp + cuQ.qp + 1) >> 1;//获取QP作为第一维索引，后面还要加一个shift

#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
      if ( sps.getLadfEnabled() )
      {
        int iShift = 0;
        deriveLADFShift( piTmpSrc + iSrcStep * (iIdx*pelsInPart), iStride, iShift, edgeDir, sps );
        iQP += iShift;
      }
#endif

      bool sidePisLarge   = false;
      bool sideQisLarge   = false;
      // P和Q的最大滤波长度
      int maxFilterLengthP = m_maxFilterLengthP[COMPONENT_Y][pos.x-m_ctuXLumaSamples][pos.y-m_ctuYLumaSamples];
      int maxFilterLengthQ = m_maxFilterLengthQ[COMPONENT_Y][pos.x-m_ctuXLumaSamples][pos.y-m_ctuYLumaSamples];
      if (maxFilterLengthP > 3)
      {
        sidePisLarge = true;
        if ( maxFilterLengthP > 5 )
        {
          // restrict filter length if sub-blocks are used (e.g affine or ATMVP)
          // 如果使用子块，则限制过滤器长度（例如，affine或ATMVP）   
          if (cuP.affine)
          {
            maxFilterLengthP = std::min(maxFilterLengthP, 5);
          }
        }
      }
      if (maxFilterLengthQ > 3)
      {
        sideQisLarge = true;
      }

      if (edgeDir == EDGE_HOR && pos.y % slice.getSPS()->getCTUSize() == 0)
      {
        sidePisLarge = false;
      }
      /**************** 获取查表得到的tc，beta数值 ****************/
      const int iIndexTC  = Clip3(0, MAX_QP + DEFAULT_INTRA_TC_OFFSET, int(iQP + DEFAULT_INTRA_TC_OFFSET*(uiBs - 1) + (tcOffsetDiv2 << 1)));
      const int iIndexB   = Clip3(0, MAX_QP, iQP + (betaOffsetDiv2 << 1));

      const int iTc = bitDepthLuma < 10 ? ((sm_tcTable[iIndexTC] + (1 << (9 - bitDepthLuma))) >> (10 - bitDepthLuma)) : ((sm_tcTable[iIndexTC]) << (bitDepthLuma - 10));
      const int iBeta     = sm_betaTable[iIndexB ] * iBitdepthScale;
      const int iSideThreshold = ( iBeta + ( iBeta >> 1 ) ) >> 3;
      const int iThrCut   = iTc * 10;

      const unsigned uiBlocksInPart = pelsInPart / 4 ? pelsInPart / 4 : 1;

      for( int iBlkIdx = 0; iBlkIdx < uiBlocksInPart; iBlkIdx++ )//遍历每个minCU中的4x4的块
      { /****************** 1.2 根据像素的变化率，来进行一次滤波开关决策，与QP找到的beta有关 ******************/
        //对于垂直边界，计算水平方向的变化率；对于水平边界，计算垂直方向的变化率

        const int dp0 = xCalcDP(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 0), iOffset);
        const int dq0 = xCalcDQ(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 0), iOffset);
        const int dp3 = xCalcDP(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 3), iOffset);
        const int dq3 = xCalcDQ(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 3), iOffset);
        int dp0L = dp0;//对大块变化率的初始化
        int dq0L = dq0;
        int dp3L = dp3;
        int dq3L = dq3;

        if (sidePisLarge)//如果P是大块
        {
          dp0L = (dp0L + xCalcDP(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 0) - 3 * iOffset, iOffset) + 1) >> 1;
          dp3L = (dp3L + xCalcDP(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 3) - 3 * iOffset, iOffset) + 1) >> 1;
        }
        if (sideQisLarge)//如果Q是大块
        {
          dq0L = (dq0L + xCalcDQ(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 0) + 3 * iOffset, iOffset) + 1) >> 1;
          dq3L = (dq3L + xCalcDQ(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 3) + 3 * iOffset, iOffset) + 1) >> 1;
        }
        /****************** 大块：双线性强滤波 ******************/
        bool useLongtapFilter = false;
        if (sidePisLarge || sideQisLarge)
        {//此处设计快速算法，仅计算首尾两行/列
          int d0L = dp0L + dq0L; 
          int d3L = dp3L + dq3L;

          int dpL = dp0L + dp3L;
          int dqL = dq0L + dq3L;

          int dL = d0L + d3L;//总变化率

          bPartPNoFilter = bPartQNoFilter = false;
          if (spsPaletteEnabledFlag)
          {
            // check if each of PUs is palette coded
            bPartPNoFilter = bPartPNoFilter || CU::isPLT(cuP);
            bPartQNoFilter = bPartQNoFilter || CU::isPLT(cuQ);
          }
          //根据像素的变化率，来进行一次滤波强弱选择，与beta和tc都有
          if (dL < iBeta)//滤波开关判断
          {
            const bool filterP = (dpL < iSideThreshold);
            const bool filterQ = (dqL < iSideThreshold);
            //对于垂直边界，src0和src3表示第0行和第3行的像素；对于水平边界，src0和src3表示第0列和第3列的像素
            Pel* src0 = piTmpSrc + iSrcStep * (iIdx*pelsInPart + iBlkIdx * 4 + 0);
            Pel* src3 = piTmpSrc + iSrcStep * (iIdx*pelsInPart + iBlkIdx * 4 + 3);

            // adjust decision so that it is not read beyond p5 is maxFilterLengthP is 5 and q5 if maxFilterLengthQ is 5
            // 调整决策，使其读数不超过p5为maxFilterLengthP为5，如果maxFilterLengthQ为5，则为q5
            const bool swL = xUseStrongFiltering(src0, iOffset, 2 * d0L, iBeta, iTc, sidePisLarge, sideQisLarge, maxFilterLengthP, maxFilterLengthQ)
              && xUseStrongFiltering(src3, iOffset, 2 * d3L, iBeta, iTc, sidePisLarge, sideQisLarge, maxFilterLengthP, maxFilterLengthQ);
            if (swL)//更强滤波
            {
              useLongtapFilter = true;
              // 对于垂直边界，滤波四行的像素
              // 对于水平边界，滤波四列的像素
              for (int i = 0; i < DEBLOCK_SMALLEST_BLOCK / 2; i++)
              {
                xPelFilterLuma(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + i), iOffset, iTc, swL, bPartPNoFilter, bPartQNoFilter, iThrCut, filterP, filterQ, clpRng, sidePisLarge, sideQisLarge, maxFilterLengthP, maxFilterLengthQ);
              }
            }

          }
        }
        if (!useLongtapFilter)
        {
          const int d0 = dp0 + dq0;
          const int d3 = dp3 + dq3;

          const int dp = dp0 + dp3;
          const int dq = dq0 + dq3;
          const int d  = d0 + d3;

          bPartPNoFilter = bPartQNoFilter = false;
          if (spsPaletteEnabledFlag)
          {
            // check if each of PUs is palette coded
            bPartPNoFilter = bPartPNoFilter || CU::isPLT(cuP);
            bPartQNoFilter = bPartQNoFilter || CU::isPLT(cuQ);
          }

          if (d < iBeta)//滤波开关决策
          {
            bool bFilterP = false;
            bool bFilterQ = false;
            if (maxFilterLengthP > 1 && maxFilterLengthQ > 1)
            {
              bFilterP = (dp < iSideThreshold);
              bFilterQ = (dq < iSideThreshold);
            }
            bool sw = false;
            if (maxFilterLengthP > 2 && maxFilterLengthQ > 2)//滤波强度决策
            {
              sw = xUseStrongFiltering(piTmpSrc + iSrcStep * (iIdx * pelsInPart + iBlkIdx * 4 + 0), iOffset, 2 * d0,
                                       iBeta, iTc)
                   && xUseStrongFiltering(piTmpSrc + iSrcStep * (iIdx * pelsInPart + iBlkIdx * 4 + 3), iOffset, 2 * d3,
                                          iBeta, iTc);
            }
            for (int i = 0; i < DEBLOCK_SMALLEST_BLOCK / 2; i++)
            {
              xPelFilterLuma(piTmpSrc + iSrcStep * (iIdx * pelsInPart + iBlkIdx * 4 + i), iOffset, iTc, sw,
                             bPartPNoFilter, bPartQNoFilter, iThrCut, bFilterP, bFilterQ, clpRng);
            }
          }
        }
      }
    }
  }
}

//xEdgeFilterChroma函数过程和亮度xEdgeFilterLuma类似，不同的是该过程遍历边界时是以2为单位进行遍历的
void DeblockingFilter::xEdgeFilterChroma(const CodingUnit& cu, const DeblockEdgeDir edgeDir, const int iEdge)
{
  const Position lumaPos   = cu.Y().valid() ? cu.Y().pos() : recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].pos() );
  const Size     lumaSize  = cu.Y().valid() ? cu.Y().size() : recalcSize( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].size() );

  const PreCalcValues& pcv = *cu.cs->pcv;
  unsigned  rasterIdx      = getRasterIdx( lumaPos, pcv );
  PelBuf     picYuvRecCb = m_enc ? m_encPicYuvBuffer.getBuf(cu.block(COMPONENT_Cb)) : cu.cs->getRecoBuf(cu.block(COMPONENT_Cb));
  PelBuf     picYuvRecCr = m_enc ? m_encPicYuvBuffer.getBuf(cu.block(COMPONENT_Cr)) : cu.cs->getRecoBuf(cu.block(COMPONENT_Cr));
  Pel       *piSrcCb       = picYuvRecCb.buf;
  Pel       *piSrcCr       = picYuvRecCr.buf;
  const int  iStride       = picYuvRecCb.stride;
  const SPS &sps           = *cu.cs->sps;
  const PPS &pps           = *cu.cs->pps;
  const Slice  &slice      = *cu.slice;
  const ChromaFormat nChromaFormat   = sps.getChromaFormatIdc();
  const unsigned uiPelsInPartChromaH = pcv.minCUWidth  >> ::getComponentScaleX(COMPONENT_Cb, nChromaFormat);
  const unsigned uiPelsInPartChromaV = pcv.minCUHeight >> ::getComponentScaleY(COMPONENT_Cb, nChromaFormat);

  int       iOffset, iSrcStep;
  unsigned  uiLoopLength;

  bool      bPartPNoFilter  = false;
  bool      bPartQNoFilter  = false;
  const int tcOffsetDiv2[2]   = { slice.getDeblockingFilterCbTcOffsetDiv2(), slice.getDeblockingFilterCrTcOffsetDiv2() };
  const int betaOffsetDiv2[2] = { slice.getDeblockingFilterCbBetaOffsetDiv2(), slice.getDeblockingFilterCrBetaOffsetDiv2() };

  // Vertical Position
  unsigned uiEdgeNumInCtuVert = rasterIdx % pcv.partsInCtuWidth + iEdge;
  unsigned uiEdgeNumInCtuHor  = rasterIdx / pcv.partsInCtuWidth + iEdge;

  if( ( uiPelsInPartChromaH < DEBLOCK_SMALLEST_BLOCK ) && ( uiPelsInPartChromaV < DEBLOCK_SMALLEST_BLOCK ) &&
      (
        ( ( uiEdgeNumInCtuVert % ( DEBLOCK_SMALLEST_BLOCK / uiPelsInPartChromaH ) ) && ( edgeDir == EDGE_VER ) ) ||
        ( ( uiEdgeNumInCtuHor  % ( DEBLOCK_SMALLEST_BLOCK / uiPelsInPartChromaV ) ) && ( edgeDir == EDGE_HOR ) )
      )
    )
  {
    return;
  }

  unsigned uiNumParts =  ( edgeDir == EDGE_VER ) ? lumaSize.height / pcv.minCUHeight : lumaSize.width / pcv.minCUWidth ;
  int   uiNumPelsLuma = pcv.minCUWidth;
  unsigned uiBsAbsIdx;
  unsigned bS[2];

  Pel* piTmpSrcCb = piSrcCb;
  Pel* piTmpSrcCr = piSrcCr;
  int xoffset, yoffset;
  Position pos( lumaPos.x, lumaPos.y );

  if( edgeDir == EDGE_VER )
  {
    xoffset      = 0;
    yoffset      = uiNumPelsLuma;
    iOffset      = 1;
    iSrcStep     = iStride;
    piTmpSrcCb  += iEdge*uiPelsInPartChromaH;
    piTmpSrcCr  += iEdge*uiPelsInPartChromaH;
    uiLoopLength = uiPelsInPartChromaV;
    pos          = Position{ lumaPos.x + iEdge*uiNumPelsLuma, lumaPos.y - yoffset };
  }
  else  // (edgeDir == EDGE_HOR)
  {
    xoffset      = uiNumPelsLuma;
    yoffset      = 0;
    iOffset      = iStride;
    iSrcStep     = 1;
    piTmpSrcCb  += iEdge*iStride*uiPelsInPartChromaV;
    piTmpSrcCr  += iEdge*iStride*uiPelsInPartChromaV;
    uiLoopLength = uiPelsInPartChromaH;
    pos          = Position{ lumaPos.x - xoffset, lumaPos.y + iEdge*uiNumPelsLuma };
  }

  const int iBitdepthScale = 1 << (sps.getBitDepth(CHANNEL_TYPE_CHROMA) - 8);

  for( int iIdx = 0; iIdx < uiNumParts; iIdx++ )
  {
    pos.x += xoffset;
    pos.y += yoffset;

    uiBsAbsIdx = getRasterIdx( pos, pcv );
    unsigned tmpBs = m_aapucBS[edgeDir][uiBsAbsIdx];

    tmpBs = m_aapucBS[edgeDir][uiBsAbsIdx];
    bS[0] = BsGet(tmpBs, COMPONENT_Cb);
    bS[1] = BsGet(tmpBs, COMPONENT_Cr);

    if (bS[0] > 0 || bS[1] > 0)
    {
      const CodingUnit& cuQ =  cu;
      CodingUnit& cuP1 = *cu.cs->getCU( recalcPosition( cu.chromaFormat, CHANNEL_TYPE_LUMA, cu.chType, pos.offset( xoffset - uiNumPelsLuma, yoffset - uiNumPelsLuma ) ), cu.chType );
      CodingUnit& cuP  = *cu.cs->getCU( recalcPosition( cu.chromaFormat, CHANNEL_TYPE_LUMA, (cuP1.isSepTree() ? CHANNEL_TYPE_CHROMA : cu.chType), pos.offset( xoffset - uiNumPelsLuma, yoffset - uiNumPelsLuma ) ), (cuP1.isSepTree() ? CHANNEL_TYPE_CHROMA : cu.chType));

      if (edgeDir == EDGE_VER)
      {
        CHECK(!isAvailableLeft(cu, cuP, !pps.getLoopFilterAcrossSlicesEnabledFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag(),
          !( pps.getSubPicFromCU(cu).getloopFilterAcrossEnabledFlag() && pps.getSubPicFromCU(cuP).getloopFilterAcrossEnabledFlag())), "Neighbour not available");
      }
      else  // (iDir == EDGE_HOR)
      {
        CHECK(!isAvailableAbove(cu, cuP, !pps.getLoopFilterAcrossSlicesEnabledFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag(),
          !( pps.getSubPicFromCU(cu).getloopFilterAcrossEnabledFlag() && pps.getSubPicFromCU(cuP).getloopFilterAcrossEnabledFlag())), "Neighbour not available");
      }

      bPartPNoFilter = bPartQNoFilter = false;
      if ( sps.getPLTMode())
      {
        // check if each of PUs is palette coded
        bPartPNoFilter = bPartPNoFilter || CU::isPLT(cuP);
        bPartQNoFilter = bPartQNoFilter || CU::isPLT(cuQ);
      }

      const int maxFilterLengthP = m_maxFilterLengthP[COMPONENT_Cb][(pos.x-m_ctuXLumaSamples)>>m_shiftHor][(pos.y-m_ctuYLumaSamples)>>m_shiftVer];
      const int maxFilterLengthQ = m_maxFilterLengthQ[COMPONENT_Cb][(pos.x-m_ctuXLumaSamples)>>m_shiftHor][(pos.y-m_ctuYLumaSamples)>>m_shiftVer];
      bool largeBoundary         = false;
      bool isChromaHorCTBBoundary = false;
      if ( maxFilterLengthP >= 3 && maxFilterLengthQ >= 3 )
      {
        largeBoundary = true;
      }

      if (edgeDir == EDGE_HOR && pos.y % cuP.slice->getSPS()->getCTUSize() == 0)
      {
        isChromaHorCTBBoundary = true;
      }

      for( int chromaIdx = 0; chromaIdx < 2; chromaIdx++ )
      {
        if ((bS[chromaIdx] == 2) || (largeBoundary && (bS[chromaIdx] == 1)))
        {
          const ClpRng &clpRng(cu.cs->slice->clpRng(ComponentID(chromaIdx + 1)));
          Pel *         piTmpSrcChroma = (chromaIdx == 0) ? piTmpSrcCb : piTmpSrcCr;

          const TransformUnit &tuQ = *cuQ.cs->getTU(
            recalcPosition(cu.chromaFormat, CHANNEL_TYPE_LUMA, CHANNEL_TYPE_CHROMA, pos), CHANNEL_TYPE_CHROMA);
          const TransformUnit &tuP =
            *cuP.cs->getTU(recalcPosition(cu.chromaFormat, CHANNEL_TYPE_LUMA, CHANNEL_TYPE_CHROMA,
                                          (edgeDir == EDGE_VER) ? pos.offset(-1, 0) : pos.offset(0, -1)),
                           CHANNEL_TYPE_CHROMA);

          const QpParam cQP(tuP, ComponentID(chromaIdx + 1), -MAX_INT, false);
          const QpParam cQQ(tuQ, ComponentID(chromaIdx + 1), -MAX_INT, false);

          const int qpBdOffset = tuP.cs->sps->getQpBDOffset(toChannelType(ComponentID(chromaIdx + 1)));
          int       baseQp_P   = cQP.Qp(0) - qpBdOffset;
          int       baseQp_Q   = cQQ.Qp(0) - qpBdOffset;
          int       iQP        = ((baseQp_Q + baseQp_P + 1) >> 1);

          const int iIndexTC =
            Clip3<int>(0, MAX_QP + DEFAULT_INTRA_TC_OFFSET,
                       iQP + DEFAULT_INTRA_TC_OFFSET * (bS[chromaIdx] - 1) + (tcOffsetDiv2[chromaIdx] << 1));
          const int bitDepthChroma = sps.getBitDepth(CHANNEL_TYPE_CHROMA);
          const int iTc            = bitDepthChroma < 10
                            ? ((sm_tcTable[iIndexTC] + (1 << (9 - bitDepthChroma))) >> (10 - bitDepthChroma))
                            : ((sm_tcTable[iIndexTC]) << (bitDepthChroma - 10));
          bool useLongFilter = false;
          if (largeBoundary)
          {
            const int indexB = Clip3<int>(0, MAX_QP, iQP + (betaOffsetDiv2[chromaIdx] << 1));
            const int beta   = sm_betaTable[indexB] * iBitdepthScale;

            const int dp0 =
              xCalcDP(piTmpSrcChroma + iSrcStep * (iIdx * uiLoopLength + 0), iOffset, isChromaHorCTBBoundary);
            const int dq0 = xCalcDQ(piTmpSrcChroma + iSrcStep * (iIdx * uiLoopLength + 0), iOffset);

            const int subSamplingShift = (edgeDir == EDGE_VER) ? m_shiftVer : m_shiftHor;

            const int dp3 =
              (subSamplingShift == 1)
                ? xCalcDP(piTmpSrcChroma + iSrcStep * (iIdx * uiLoopLength + 1), iOffset, isChromaHorCTBBoundary)
                : xCalcDP(piTmpSrcChroma + iSrcStep * (iIdx * uiLoopLength + 3), iOffset, isChromaHorCTBBoundary);
            const int dq3 = (subSamplingShift == 1)
                              ? xCalcDQ(piTmpSrcChroma + iSrcStep * (iIdx * uiLoopLength + 1), iOffset)
                              : xCalcDQ(piTmpSrcChroma + iSrcStep * (iIdx * uiLoopLength + 3), iOffset);

            const int d0 = dp0 + dq0;
            const int d3 = dp3 + dq3;
            const int d  = d0 + d3;

            if (d < beta)
            {
              useLongFilter = true;
              const bool sw = xUseStrongFiltering(piTmpSrcChroma + iSrcStep * (iIdx * uiLoopLength + 0), iOffset,
                                                  2 * d0, beta, iTc, false, false, 7, 7, isChromaHorCTBBoundary)
                              && xUseStrongFiltering(
                                piTmpSrcChroma + iSrcStep * (iIdx * uiLoopLength + ((subSamplingShift == 1) ? 1 : 3)),
                                iOffset, 2 * d3, beta, iTc, false, false, 7, 7, isChromaHorCTBBoundary);

              for (unsigned step = 0; step < uiLoopLength; step++)
              {
                xPelFilterChroma(piTmpSrcChroma + iSrcStep * (step + iIdx * uiLoopLength), iOffset, iTc, sw,
                                 bPartPNoFilter, bPartQNoFilter, clpRng, largeBoundary, isChromaHorCTBBoundary);
              }
            }
          }
          if (!useLongFilter)
          {
            for (unsigned step = 0; step < uiLoopLength; step++)
            {
              xPelFilterChroma(piTmpSrcChroma + iSrcStep * (step + iIdx * uiLoopLength), iOffset, iTc, false,
                               bPartPNoFilter, bPartQNoFilter, clpRng, largeBoundary, isChromaHorCTBBoundary);
            }
          }
        }
      }
    }
  }
}



/**
 - Deblocking for the luminance component with strong or weak filter
 .
 \param piSrc           pointer to picture data
 \param iOffset         offset value for picture data
 \param tc              tc value
 \param sw              decision strong/weak filter
 \param bPartPNoFilter  indicator to disable filtering on partP
 \param bPartQNoFilter  indicator to disable filtering on partQ
 \param iThrCut         threshold value for weak filter decision
 \param bFilterSecondP  decision weak filter/no filter for partP
 \param bFilterSecondQ  decision weak filter/no filter for partQ
 \param bitDepthLuma    luma bit depth
*/
inline void DeblockingFilter::xBilinearFilter(Pel* srcP, Pel* srcQ, int offset, int refMiddle, int refP, int refQ, int numberPSide, int numberQSide, const int* dbCoeffsP, const int* dbCoeffsQ, int tc) const
{
  const char tc7[7] = { 6, 5, 4, 3, 2, 1, 1 };
  const char tc3[3] = { 6, 4, 2 };

  const char *tcP = (numberPSide == 3) ? tc3 : tc7;
  const char *tcQ = (numberQSide == 3) ? tc3 : tc7;

  for (int pos = 0; pos < numberPSide; pos++)
  {
    int src    = srcP[-offset * pos];
    int cvalue = (tc * tcP[pos]) >> 1;
    srcP[-offset * pos] =
      Clip3(src - cvalue, src + cvalue, ((refMiddle * dbCoeffsP[pos] + refP * (64 - dbCoeffsP[pos]) + 32) >> 6));
  }
  for (int pos = 0; pos < numberQSide; pos++)
  {
    int src    = srcQ[offset * pos];
    int cvalue = (tc * tcQ[pos]) >> 1;
    srcQ[offset * pos] =
      Clip3(src - cvalue, src + cvalue, ((refMiddle * dbCoeffsQ[pos] + refQ * (64 - dbCoeffsQ[pos]) + 32) >> 6));
  }
}

inline void DeblockingFilter::xFilteringPandQ(Pel* src, int offset, int numberPSide, int numberQSide, int tc) const
{
  CHECK(numberPSide <= 3 && numberQSide <= 3, "Short filtering in long filtering function");
  Pel* srcP = src-offset;
  Pel* srcQ = src;

  int refP = 0;
  int refQ = 0;
  int refMiddle = 0;

  const int dbCoeffs7[7] = { 59, 50, 41,32,23,14,5 };
  const int dbCoeffs3[3] = { 53, 32, 11 };
  const int dbCoeffs5[5] = { 58, 45, 32,19,6};
  const int* dbCoeffsP   = numberPSide == 7 ? dbCoeffs7 : (numberPSide==5) ? dbCoeffs5 : dbCoeffs3;
  const int* dbCoeffsQ   = numberQSide == 7 ? dbCoeffs7 : (numberQSide==5) ? dbCoeffs5 : dbCoeffs3;

  switch (numberPSide)
  {
    case 7: refP = (srcP[-6*offset]   + srcP[-7 * offset] + 1) >> 1; break;
    case 3: refP = (srcP[-2 * offset] + srcP[-3 * offset] + 1) >> 1; break;
    case 5: refP = (srcP[-4 * offset] + srcP[-5 * offset] + 1) >> 1; break;
  }

  switch (numberQSide)
  {
    case 7: refQ = (srcQ[6 * offset] + srcQ[7 * offset] + 1) >> 1; break;
    case 3: refQ = (srcQ[2 * offset] + srcQ[3 * offset] + 1) >> 1; break;
    case 5: refQ = (srcQ[4 * offset] + srcQ[5 * offset] + 1) >> 1; break;
  }

  if (numberPSide == numberQSide)
  {
    if (numberPSide == 5)
    {
      refMiddle = (2 * (srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset]) + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + 8) >> 4;
    }
    else
    {
      refMiddle = (2 * (srcP[0] + srcQ[0]) + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + srcP[-5 * offset] + srcQ[5 * offset] + +srcP[-6 * offset] + srcQ[6 * offset] + 8) >> 4;
    }
  }
  else
  {
    Pel* srcPt = srcP;
    Pel* srcQt = srcQ;
    int offsetP = -offset;
    int offsetQ = offset;

    int newNumberQSide = numberQSide;
    int newNumberPSide = numberPSide;
    if (numberQSide > numberPSide)
    {
      std::swap(srcPt, srcQt);
      std::swap(offsetP, offsetQ);
      newNumberQSide = numberPSide;
      newNumberPSide = numberQSide;
    }

    if (newNumberPSide == 7 && newNumberQSide == 5)
    {
      refMiddle = (2 * (srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset]) + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + srcP[-5 * offset] + srcQ[5 * offset] + 8) >> 4;
    }
    else if (newNumberPSide == 7 && newNumberQSide == 3)
    {
      refMiddle = (2 * (srcPt[0] + srcQt[0]) + srcQt[0] + 2 * (srcQt[offsetQ] + srcQt[2 * offsetQ]) + srcPt[offsetP] + srcQt[offsetQ] + srcPt[2 * offsetP] + srcPt[3 * offsetP] + srcPt[4 * offsetP] + srcPt[5 * offsetP] + srcPt[6 * offsetP] + 8) >> 4;
    }
    else //if (newNumberPSide == 5 && newNumberQSide == 3)
    {
      refMiddle = (srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + 4) >> 3;
    }
  }
  xBilinearFilter(srcP,srcQ,offset,refMiddle,refP,refQ,numberPSide,numberQSide,dbCoeffsP,dbCoeffsQ,tc);
}

inline void DeblockingFilter::xPelFilterLuma(Pel* piSrc, const int iOffset, const int tc, const bool sw, const bool bPartPNoFilter, const bool bPartQNoFilter, const int iThrCut, const bool bFilterSecondP, const bool bFilterSecondQ, const ClpRng& clpRng, bool sidePisLarge, bool sideQisLarge, int maxFilterLengthP, int maxFilterLengthQ) const
{
  int delta;
  /******** 初始化像素位置、像素值 *******/
  const Pel m4  = piSrc[ 0          ];
  const Pel m3  = piSrc[-iOffset    ];
  const Pel m5  = piSrc[ iOffset    ];
  const Pel m2  = piSrc[-iOffset * 2];
  const Pel m6  = piSrc[ iOffset * 2];
  const Pel m1  = piSrc[-iOffset * 3];
  const Pel m7  = piSrc[ iOffset * 3];
  const Pel m0  = piSrc[-iOffset * 4];

  const Pel mP1 = piSrc[-iOffset * 5];
  const Pel mP2 = piSrc[-iOffset * 6];
  const Pel mP3 = piSrc[-iOffset * 7];
  const Pel m8  = piSrc[ iOffset * 4];
  const Pel m9  = piSrc[ iOffset * 5];
  const Pel m10 = piSrc[ iOffset * 6];
  const char tc3[3] = { 3, 2, 1};
  if (sw)//强滤波
  {
    if (sidePisLarge || sideQisLarge)//如果是大块，那么进行双线性滤波
    {
      xFilteringPandQ(piSrc, iOffset, sidePisLarge ? maxFilterLengthP : 3, sideQisLarge ? maxFilterLengthQ : 3, tc);
    }
    else
    {/* Weak filter 弱滤波 */
      piSrc[-iOffset]     = Clip3(m3 - tc3[0] * tc, m3 + tc3[0] * tc, ((m1 + 2 * m2 + 2 * m3 + 2 * m4 + m5 + 4) >> 3));
      piSrc[0]            = Clip3(m4 - tc3[0] * tc, m4 + tc3[0] * tc, ((m2 + 2 * m3 + 2 * m4 + 2 * m5 + m6 + 4) >> 3));
      piSrc[-iOffset * 2] = Clip3(m2 - tc3[1] * tc, m2 + tc3[1] * tc, ((m1 + m2 + m3 + m4 + 2) >> 2));
      piSrc[iOffset]      = Clip3(m5 - tc3[1] * tc, m5 + tc3[1] * tc, ((m3 + m4 + m5 + m6 + 2) >> 2));
      piSrc[-iOffset * 3] = Clip3(m1 - tc3[2] * tc, m1 + tc3[2] * tc, ((2 * m0 + 3 * m1 + m2 + m3 + m4 + 4) >> 3));
      piSrc[iOffset * 2]  = Clip3(m6 - tc3[2] * tc, m6 + tc3[2] * tc, ((m3 + m4 + m5 + 3 * m6 + 2 * m7 + 4) >> 3));
    }
  }
  else
  {
    /* Weak filter */
    delta = ( 9 * ( m4 - m3 ) - 3 * ( m5 - m2 ) + 8 ) >> 4;

    if ( abs(delta) < iThrCut )
    {//如果不满足该条件，说明不连续可能是内容导致的
      delta = Clip3( -tc, tc, delta );
      piSrc[-iOffset] = ClipPel( m3 + delta, clpRng);
      piSrc[0]        = ClipPel( m4 - delta, clpRng);

      const int tc2 = tc >> 1;
      if( bFilterSecondP )
      {
        const int delta1 = Clip3( -tc2, tc2, ( ( ( ( m1 + m3 + 1 ) >> 1 ) - m2 + delta ) >> 1 ) );
        piSrc[-iOffset * 2] = ClipPel( m2 + delta1, clpRng);
      }
      if( bFilterSecondQ )
      {
        const int delta2 = Clip3( -tc2, tc2, ( ( ( ( m6 + m4 + 1 ) >> 1 ) - m5 - delta ) >> 1 ) );
        piSrc[iOffset] = ClipPel( m5 + delta2, clpRng);
      }
    }
  }

  if(bPartPNoFilter)
  {
    piSrc[-iOffset    ] = m3;
    piSrc[-iOffset * 2] = m2;
    piSrc[-iOffset * 3] = m1;
    if (sidePisLarge)
    {
      piSrc[-iOffset * 4] = m0;
      piSrc[-iOffset * 5] = mP1;
      piSrc[-iOffset * 6] = mP2;
      piSrc[-iOffset * 7] = mP3;
    }
  }

  if(bPartQNoFilter)
  {
    piSrc[ 0          ] = m4;
    piSrc[ iOffset    ] = m5;
    piSrc[ iOffset * 2] = m6;
    if (sideQisLarge)
    {
      piSrc[iOffset * 3] = m7;
      piSrc[iOffset * 4] = m8;
      piSrc[iOffset * 5] = m9;
      piSrc[iOffset * 6] = m10;
    }
  }
}

/**
 - Deblocking of one line/column for the chrominance component
 .
 \param piSrc           pointer to picture data
 \param iOffset         offset value for picture data
 \param tc              tc value
 \param bPartPNoFilter  indicator to disable filtering on partP
 \param bPartQNoFilter  indicator to disable filtering on partQ
 \param bitDepthChroma  chroma bit depth
 */
inline void DeblockingFilter::xPelFilterChroma(Pel* piSrc, const int iOffset, const int tc, const bool sw, const bool bPartPNoFilter, const bool bPartQNoFilter, const ClpRng& clpRng, const bool largeBoundary, const bool isChromaHorCTBBoundary) const
{
  int delta;

  const Pel m0 = piSrc[-iOffset * 4];
  const Pel m1 = piSrc[-iOffset * 3];
  const Pel m2 = piSrc[-iOffset * 2];
  const Pel m3 = piSrc[-iOffset];
  const Pel m4 = piSrc[0];
  const Pel m5 = piSrc[iOffset];
  const Pel m6 = piSrc[iOffset * 2];
  const Pel m7 = piSrc[iOffset * 3];

  if (sw)
  {
    if (isChromaHorCTBBoundary)
    {
      piSrc[-iOffset * 1] = Clip3(m3 - tc, m3 + tc, ((3 * m2 + 2 * m3 + m4 + m5 + m6 + 4) >> 3)); // p0
      piSrc[0] = Clip3(m4 - tc, m4 + tc, ((2 * m2 + m3 + 2 * m4 + m5 + m6 + m7 + 4) >> 3)); // q0
      piSrc[iOffset * 1] = Clip3(m5 - tc, m5 + tc, ((m2 + m3 + m4 + 2 * m5 + m6 + 2 * m7 + 4) >> 3));  // q1
      piSrc[iOffset * 2] = Clip3(m6 - tc, m6 + tc, ((m3 + m4 + m5 + 2 * m6 + 3 * m7 + 4) >> 3));       // q2
    }
    else
    {
      piSrc[-iOffset * 3] = Clip3(m1 - tc, m1 + tc, ((3 * m0 + 2 * m1 + m2 + m3 + m4 + 4) >> 3));       // p2
      piSrc[-iOffset * 2] = Clip3(m2 - tc, m2 + tc, ((2 * m0 + m1 + 2 * m2 + m3 + m4 + m5 + 4) >> 3));  // p1
      piSrc[-iOffset * 1] = Clip3(m3 - tc, m3 + tc, ((m0 + m1 + m2 + 2 * m3 + m4 + m5 + m6 + 4) >> 3)); // p0
      piSrc[0] = Clip3(m4 - tc, m4 + tc, ((m1 + m2 + m3 + 2 * m4 + m5 + m6 + m7 + 4) >> 3)); // q0
      piSrc[iOffset * 1] = Clip3(m5 - tc, m5 + tc, ((m2 + m3 + m4 + 2 * m5 + m6 + 2 * m7 + 4) >> 3));  // q1
      piSrc[iOffset * 2] = Clip3(m6 - tc, m6 + tc, ((m3 + m4 + m5 + 2 * m6 + 3 * m7 + 4) >> 3));       // q2
    }
  }
  else
  {
    delta           = Clip3(-tc, tc, ((((m4 - m3) << 2) + m2 - m5 + 4) >> 3));
    piSrc[-iOffset] = ClipPel(m3 + delta, clpRng);
    piSrc[0]        = ClipPel(m4 - delta, clpRng);
  }


  if( bPartPNoFilter )
  {
    if (largeBoundary)
    {
      piSrc[-iOffset * 3] = m1; // p2
      piSrc[-iOffset * 2] = m2; // p1
    }
    piSrc[-iOffset] = m3;
  }
  if( bPartQNoFilter )
  {
    if (largeBoundary)
    {
      piSrc[iOffset * 1] = m5; // q1
      piSrc[iOffset * 2] = m6; // q2
    }
    piSrc[ 0      ] = m4;
  }
}

/**
 - Decision between strong and weak filter 进行滤波强度决策
 .
 \param offset         offset value for picture data
 \param d               d value
 \param beta            beta value
 \param tc              tc value
 \param piSrc           pointer to picture data
 */
inline bool DeblockingFilter::xUseStrongFiltering(Pel* piSrc, const int iOffset, const int d, const int beta, const int tc, bool sidePisLarge, bool sideQisLarge, int maxFilterLengthP, int maxFilterLengthQ, bool isChromaHorCTBBoundary) const
{
  const Pel m4 = piSrc[ 0          ];//          **             **   **             ** 
  const Pel m3 = piSrc[-iOffset    ];//          m0   m1   m2   m3   m4   m5   m6   m7
  const Pel m7 = piSrc[ iOffset * 3];//                              |
  const Pel m0 = piSrc[-iOffset * 4];//                    **    当前起始位置[0]（Q块）
  const Pel m2 = piSrc[-iOffset * 2];
  int       sp3 = abs(m0 - m3);//最左边的位置减去左侧边缘
  if (isChromaHorCTBBoundary)//如果是色度CTU边缘，则看相邻边缘
  {
    sp3 = abs(m2 - m3);
  }
  int       sq3      = abs(m7 - m4);//最右边的位置减去右侧边缘
  const int d_strong = sp3 + sq3;// 两者的差距

  if (sidePisLarge || sideQisLarge)//对于大块，进一步增加长度
  {
    Pel mP4;
    Pel m11;
    if (sidePisLarge)//如果P是大块
    {
      if (maxFilterLengthP == 7)//如果P块滤波长度为7，则扩展到左侧第七个像素
      {
        const Pel mP5 = piSrc[-iOffset * 5];
        const Pel mP6 = piSrc[-iOffset * 6];
        const Pel mP7 = piSrc[-iOffset * 7];;
        mP4 = piSrc[-iOffset * 8];
        sp3 = sp3 + abs(mP5 - mP6 - mP7 + mP4);
      }
      else
      {
        mP4 = piSrc[-iOffset * 6];
      }
      sp3 = (sp3 + abs(m0 - mP4) + 1) >> 1;
    }
    if (sideQisLarge)//如果Q是大块
    {
      if (maxFilterLengthQ == 7)
      {
        const Pel m8 = piSrc[iOffset * 4];
        const Pel m9 = piSrc[iOffset * 5];
        const Pel m10 = piSrc[iOffset * 6];;
        m11 = piSrc[iOffset * 7];
        sq3 = sq3 + abs(m8 - m9 - m10 + m11);
      }
      else
      {
        m11 = piSrc[iOffset * 5];
      }
      sq3 = (sq3 + abs(m11 - m7) + 1) >> 1;
    }
    return ((sp3 + sq3) < (beta * 3 >> 5)) && (d < (beta >> 4)) && (abs(m3 - m4) < ((tc * 5 + 1) >> 1));
  }
  else
  {
    return ((d_strong < (beta >> 3)) && (d < (beta >> 2)) && (abs(m3 - m4) < ((tc * 5 + 1) >> 1)));
  }
}

inline int DeblockingFilter::xCalcDP(Pel* piSrc, const int iOffset, const bool isChromaHorCTBBoundary) const
{
  if (isChromaHorCTBBoundary)
  {
    return abs(piSrc[-iOffset * 2] - 2 * piSrc[-iOffset * 2] + piSrc[-iOffset]);
  }
  else
  {
    return abs(piSrc[-iOffset * 3] - 2 * piSrc[-iOffset * 2] + piSrc[-iOffset]);
  }
}

inline int DeblockingFilter::xCalcDQ( Pel* piSrc, const int iOffset ) const
{
  return abs( piSrc[0] - 2 * piSrc[iOffset] + piSrc[iOffset * 2] );
}

inline unsigned DeblockingFilter::BsSet(unsigned val, const ComponentID compIdx) const { return (val << (compIdx << 1)); }
inline unsigned DeblockingFilter::BsGet(unsigned val, const ComponentID compIdx) const { return ((val >> (compIdx << 1)) & 3); }

//! \}
