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

/** \file     EncCu.cpp
    \brief    Coding Unit (CU) encoder class
*/

#include "EncCu.h"

#include "EncLib.h"
#include "Analyze.h"
#include "AQp.h"

#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
#include "MCTS.h"


#include "CommonLib/dtrace_buffer.h"

#include <stdio.h>
#include <cmath>
#include <algorithm>



//! \ingroup EncoderLib
//! \{
string u_EncTestModeType[] = 
{
 "HASH_INTER", "MERGE_SKIP", "INTER_ME", "AFFINE",
                               "MERGE_GEO",
                               "INTRA",
                               "PALETTE",
                               "QT",
                               "BT_H",
                               "BT_V",
                               "TT_H",
                               "TT_V",
                               "DONT_SPLIT",   // dummy mode to collect the data from the unsplit coding
#if REUSE_CU_RESULTS
 "RECO_CACHED",
#endif
 "IMV_LIST",
 "IBC",         // ibc mode
 "IBC_MERGE",   // ibc merge mode
 "INVALID"
};
// ====================================================================================================================
EncCu::EncCu() : m_GeoModeTest
{
  GeoMotionInfo(0, 1), GeoMotionInfo(1, 0),GeoMotionInfo(0, 2), GeoMotionInfo(1, 2), GeoMotionInfo(2, 0),
  GeoMotionInfo(2, 1), GeoMotionInfo(0, 3),GeoMotionInfo(1, 3), GeoMotionInfo(2, 3), GeoMotionInfo(3, 0),
  GeoMotionInfo(3, 1), GeoMotionInfo(3, 2),GeoMotionInfo(0, 4), GeoMotionInfo(1, 4), GeoMotionInfo(2, 4),
  GeoMotionInfo(3, 4), GeoMotionInfo(4, 0),GeoMotionInfo(4, 1), GeoMotionInfo(4, 2), GeoMotionInfo(4, 3),
  GeoMotionInfo(0, 5), GeoMotionInfo(1, 5),GeoMotionInfo(2, 5), GeoMotionInfo(3, 5), GeoMotionInfo(4, 5),
  GeoMotionInfo(5, 0), GeoMotionInfo(5, 1),GeoMotionInfo(5, 2), GeoMotionInfo(5, 3), GeoMotionInfo(5, 4)
}
{}

void EncCu::create( EncCfg* encCfg )
{
  unsigned      uiMaxWidth    = encCfg->getMaxCUWidth();
  unsigned      uiMaxHeight   = encCfg->getMaxCUHeight();
  ChromaFormat  chromaFormat  = encCfg->getChromaFormatIdc();

  unsigned      numWidths     = gp_sizeIdxInfo->numWidths();
  unsigned      numHeights    = gp_sizeIdxInfo->numHeights();
  m_pTempCS = new CodingStructure**  [numWidths];
  m_pBestCS = new CodingStructure**  [numWidths];
  m_pTempCS2 = new CodingStructure** [numWidths];
  m_pBestCS2 = new CodingStructure** [numWidths];

  for( unsigned w = 0; w < numWidths; w++ )
  {
    m_pTempCS[w] = new CodingStructure*  [numHeights];
    m_pBestCS[w] = new CodingStructure*  [numHeights];
    m_pTempCS2[w] = new CodingStructure* [numHeights];
    m_pBestCS2[w] = new CodingStructure* [numHeights];

    for( unsigned h = 0; h < numHeights; h++ )
    {
      unsigned width  = gp_sizeIdxInfo->sizeFrom( w );
      unsigned height = gp_sizeIdxInfo->sizeFrom( h );

      if( gp_sizeIdxInfo->isCuSize( width ) && gp_sizeIdxInfo->isCuSize( height ) )
      {
        m_pTempCS[w][h] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
        m_pBestCS[w][h] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );

        m_pTempCS[w][h]->create(chromaFormat, Area(0, 0, width, height), false, (bool)encCfg->getPLTMode());
        m_pBestCS[w][h]->create(chromaFormat, Area(0, 0, width, height), false, (bool)encCfg->getPLTMode());

        m_pTempCS2[w][h] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
        m_pBestCS2[w][h] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );

        m_pTempCS2[w][h]->create(chromaFormat, Area(0, 0, width, height), false, (bool)encCfg->getPLTMode());
        m_pBestCS2[w][h]->create(chromaFormat, Area(0, 0, width, height), false, (bool)encCfg->getPLTMode());
      }
      else
      {
        m_pTempCS[w][h] = nullptr;
        m_pBestCS[w][h] = nullptr;
        m_pTempCS2[w][h] = nullptr;
        m_pBestCS2[w][h] = nullptr;
      }
    }
  }

  m_cuChromaQpOffsetIdxPlus1 = 0;

  unsigned maxDepth = numWidths + numHeights;

  m_modeCtrl = new EncModeCtrlMTnoRQT();

  m_modeCtrl->create( *encCfg );

  for (unsigned ui = 0; ui < MMVD_MRG_MAX_RD_BUF_NUM; ui++)
  {
    m_acMergeBuffer[ui].create( chromaFormat, Area( 0, 0, uiMaxWidth, uiMaxHeight ) );
  }
  for (unsigned ui = 0; ui < MRG_MAX_NUM_CANDS; ui++)
  {
    m_acRealMergeBuffer[ui].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
    m_acMergeTmpBuffer[ui].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
  }
  for( unsigned ui = 0; ui < GEO_MAX_TRY_WEIGHTED_SAD; ui++ )
  {
    m_acGeoWeightedBuffer[ui].create( chromaFormat, Area( 0, 0, uiMaxWidth, uiMaxHeight ) );
  }

  m_CtxBuffer.resize( maxDepth );
  m_CurrCtx = 0;
}


void EncCu::destroy()
{
  unsigned numWidths  = gp_sizeIdxInfo->numWidths();
  unsigned numHeights = gp_sizeIdxInfo->numHeights();

  for( unsigned w = 0; w < numWidths; w++ )
  {
    for( unsigned h = 0; h < numHeights; h++ )
    {
      if (m_pBestCS[w][h])
      {
        m_pBestCS[w][h]->destroy();
      }
      if (m_pTempCS[w][h])
      {
        m_pTempCS[w][h]->destroy();
      }

      delete m_pBestCS[w][h];
      delete m_pTempCS[w][h];

      if (m_pBestCS2[w][h])
      {
        m_pBestCS2[w][h]->destroy();
      }
      if (m_pTempCS2[w][h])
      {
        m_pTempCS2[w][h]->destroy();
      }

      delete m_pBestCS2[w][h];
      delete m_pTempCS2[w][h];
    }

    delete[] m_pTempCS[w];
    delete[] m_pBestCS[w];
    delete[] m_pTempCS2[w];
    delete[] m_pBestCS2[w];
  }

  delete[] m_pBestCS; m_pBestCS = nullptr;
  delete[] m_pTempCS; m_pTempCS = nullptr;
  delete[] m_pBestCS2; m_pBestCS2 = nullptr;
  delete[] m_pTempCS2; m_pTempCS2 = nullptr;

#if REUSE_CU_RESULTS
  if (m_tmpStorageLCU)
  {
    m_tmpStorageLCU->destroy();
    delete m_tmpStorageLCU;  m_tmpStorageLCU = nullptr;
  }
#endif

#if REUSE_CU_RESULTS
  m_modeCtrl->destroy();

#endif
  delete m_modeCtrl;
  m_modeCtrl = nullptr;

  for (unsigned ui = 0; ui < MMVD_MRG_MAX_RD_BUF_NUM; ui++)
  {
    m_acMergeBuffer[ui].destroy();
  }
  for (unsigned ui = 0; ui < MRG_MAX_NUM_CANDS; ui++)
  {
    m_acRealMergeBuffer[ui].destroy();
    m_acMergeTmpBuffer[ui].destroy();
  }
  for (unsigned ui = 0; ui < GEO_MAX_TRY_WEIGHTED_SAD; ui++)
  {
    m_acGeoWeightedBuffer[ui].destroy();
  }
}

EncCu::~EncCu()
{
}

/** \param    pcEncLib      pointer of encoder class
 */
void EncCu::init( EncLib* pcEncLib, const SPS& sps )
{
  m_pcEncCfg           = pcEncLib;
  m_pcIntraSearch      = pcEncLib->getIntraSearch();
  m_pcInterSearch      = pcEncLib->getInterSearch();
  m_pcTrQuant          = pcEncLib->getTrQuant();
  m_pcRdCost           = pcEncLib->getRdCost ();
  m_CABACEstimator     = pcEncLib->getCABACEncoder()->getCABACEstimator( &sps );
  m_CABACEstimator->setEncCu(this);
  m_CtxCache           = pcEncLib->getCtxCache();
  m_pcRateCtrl         = pcEncLib->getRateCtrl();
  m_pcSliceEncoder     = pcEncLib->getSliceEncoder();
  m_deblockingFilter   = pcEncLib->getDeblockingFilter();
  m_GeoCostList.init(GEO_NUM_PARTITION_MODE, m_pcEncCfg->getMaxNumGeoCand());
  m_AFFBestSATDCost = MAX_DOUBLE;

  DecCu::init( m_pcTrQuant, m_pcIntraSearch, m_pcInterSearch );

  m_modeCtrl->init( m_pcEncCfg, m_pcRateCtrl, m_pcRdCost );

  m_pcInterSearch->setModeCtrl( m_modeCtrl );
  m_modeCtrl->setInterSearch(m_pcInterSearch);
  m_pcIntraSearch->setModeCtrl( m_modeCtrl );
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void EncCu::compressCtu( CodingStructure& cs, const UnitArea& area, const unsigned ctuRsAddr, const int prevQP[], const int currQP[] )
{
  /*
  input params:
  cs: picture 级别的coding structure
  area: 当前正编码CTU的区域
  ctuRsAddr: 当前正编码CTU的raster-scan顺序
  递归调用xCompressCU
  */
  // 进行模式控制类实例的CTU级初始化
  m_modeCtrl->initCTUEncoding( *cs.slice );
  cs.treeType = TREE_D;

  cs.slice->m_mapPltCost[0].clear();
  cs.slice->m_mapPltCost[1].clear();
  // init the partitioning manager
  // 是管理划分的变量，包含划分的栈、CU/PU/TU划分信息（划分深度、UnitArea等信息）、
  // 设置划分限制的方法以及划分合理性检查（canSplit）
  QTBTPartitioner partitioner;

  // 进行划分类实例的初始化，接收传输过来的数据
  partitioner.initCtu(area, CH_L, *cs.slice);

  if (m_pcEncCfg->getIBCMode())
  {
    //如果使用IBC，初始化IBC搜索范围 根据当前CU处于CTU中的位置限制IBC搜索范围
    if (area.lx() == 0 && area.ly() == 0)
    {
      m_pcInterSearch->resetIbcSearch();
    }
    m_pcInterSearch->resetCtuRecord();
    m_ctuIbcSearchRangeX = m_pcEncCfg->getIBCLocalSearchRangeX();
    m_ctuIbcSearchRangeY = m_pcEncCfg->getIBCLocalSearchRangeY();
  }
  // IBC优先通过哈希搜索的方式进行块匹配
  if (m_pcEncCfg->getIBCMode() && m_pcEncCfg->getIBCHashSearch() && (m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_ADAPTIVE_SEARCHRANGE))
  {
    const int hashHitRatio = m_ibcHashMap.getHashHitRatio(area.Y()); // in percent
    //缩小搜索范围
    if (hashHitRatio < 5) // 5%
    {
      m_ctuIbcSearchRangeX >>= 1;
      m_ctuIbcSearchRangeY >>= 1;
    }
    if (cs.slice->getNumRefIdx(REF_PIC_LIST_0) > 0)
    {
      m_ctuIbcSearchRangeX >>= 1;
      m_ctuIbcSearchRangeY >>= 1;
    }
  }
  // init current context pointer
  m_CurrCtx = m_CtxBuffer.data();

  // tempCS和bestCS 使用多个低速的数据预处理模块处理高速的数据输入流, e.g.
  // 一个buffer写入数据的时候，另外一个buffer进行数据处理,
  // tempCS存储test模式的数据，bestCS存储当前最优模式
  CodingStructure *tempCS = m_pTempCS[gp_sizeIdxInfo->idxFrom( area.lumaSize().width )][gp_sizeIdxInfo->idxFrom( area.lumaSize().height )];
  CodingStructure *bestCS = m_pBestCS[gp_sizeIdxInfo->idxFrom( area.lumaSize().width )][gp_sizeIdxInfo->idxFrom( area.lumaSize().height )];

  // 将帧级的CS信息传入CTU级的CS
  cs.initSubStructure(*tempCS, partitioner.chType, partitioner.currArea(), false);
  cs.initSubStructure(*bestCS, partitioner.chType, partitioner.currArea(), false);
  tempCS->currQP[CH_L] = bestCS->currQP[CH_L] =
  tempCS->baseQP       = bestCS->baseQP       = currQP[CH_L];
  tempCS->prevQP[CH_L] = bestCS->prevQP[CH_L] = prevQP[CH_L];

  // 第一次调用xCompressCU对亮度分量进行预测,调用xCheckModeSplit进入CU的递归划分进行RDO的过程，
  // xCheckModeSplit调用xCompressCU进行划分结果的入栈
  xCompressCU(tempCS, bestCS, partitioner);
  cs.slice->m_mapPltCost[0].clear();
  cs.slice->m_mapPltCost[1].clear();
  // all signals were already copied during compression if the CTU was split - at this point only the structures are copied to the top level CS
  // copyUnsplitCTUSignals为true表示CTU不划分
  const bool copyUnsplitCTUSignals = bestCS->cus.size() == 1;
 
  // 将当前CTU的最佳CS信息传入帧级的CS
  cs.useSubStructure(*bestCS, partitioner.chType, CS::getArea(*bestCS, area, partitioner.chType), copyUnsplitCTUSignals,
                     false, false, copyUnsplitCTUSignals, true);

  // I帧并且亮色度单独划分，两棵树，而P帧和B帧是亮色度共用一棵树
  if (CS::isDualITree (cs) && isChromaEnabled (cs.pcv->chrFormat))
  {
    m_CABACEstimator->getCtx() = m_CurrCtx->start;
    partitioner.initCtu(area, CH_C, *cs.slice);

    cs.initSubStructure(*tempCS, partitioner.chType, partitioner.currArea(), false);
    cs.initSubStructure(*bestCS, partitioner.chType, partitioner.currArea(), false);
    tempCS->currQP[CH_C] = bestCS->currQP[CH_C] =
    tempCS->baseQP       = bestCS->baseQP       = currQP[CH_C];
    tempCS->prevQP[CH_C] = bestCS->prevQP[CH_C] = prevQP[CH_C];
    // 第二次调用xCompressCU对色度分量进行预测
    xCompressCU(tempCS, bestCS, partitioner);
    const bool copyUnsplitCTUSignals = bestCS->cus.size() == 1;
    cs.useSubStructure(*bestCS, partitioner.chType, CS::getArea(*bestCS, area, partitioner.chType),
                       copyUnsplitCTUSignals, false, false, copyUnsplitCTUSignals, true);
  }

  // 使用RC，在这里更新MSE和像素值
  if (m_pcEncCfg->getUseRateCtrl())
  {
    (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_actualMSE = (double)bestCS->dist / (double)m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr).m_numberOfPixel;
  }

  // reset context states and uninit context pointer
  // 重新设置上下文状态
  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  m_CurrCtx                  = 0;


  // Ensure that a coding was found
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  CHECK( bestCS->cus.empty()                                   , "No possible encoding found" );
  CHECK( bestCS->cus[0]->predMode == NUMBER_OF_PREDICTION_MODES, "No possible encoding found" );
  CHECK( bestCS->cost             == MAX_DOUBLE                , "No possible encoding found" );
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

static int xCalcHADs8x8_ISlice(const Pel *piOrg, const int iStrideOrg)
{
  int k, i, j, jj;
  int diff[64], m1[8][8], m2[8][8], m3[8][8], iSumHad = 0;

  for (k = 0; k < 64; k += 8)
  {
    diff[k + 0] = piOrg[0];
    diff[k + 1] = piOrg[1];
    diff[k + 2] = piOrg[2];
    diff[k + 3] = piOrg[3];
    diff[k + 4] = piOrg[4];
    diff[k + 5] = piOrg[5];
    diff[k + 6] = piOrg[6];
    diff[k + 7] = piOrg[7];

    piOrg += iStrideOrg;
  }

  //horizontal
  for (j = 0; j < 8; j++)
  {
    jj = j << 3;
    m2[j][0] = diff[jj    ] + diff[jj + 4];
    m2[j][1] = diff[jj + 1] + diff[jj + 5];
    m2[j][2] = diff[jj + 2] + diff[jj + 6];
    m2[j][3] = diff[jj + 3] + diff[jj + 7];
    m2[j][4] = diff[jj    ] - diff[jj + 4];
    m2[j][5] = diff[jj + 1] - diff[jj + 5];
    m2[j][6] = diff[jj + 2] - diff[jj + 6];
    m2[j][7] = diff[jj + 3] - diff[jj + 7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for (i = 0; i < 8; i++)
  {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];

    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      iSumHad += abs(m2[i][j]);
    }
  }
  iSumHad -= abs(m2[0][0]);
  iSumHad = (iSumHad + 2) >> 2;
  return(iSumHad);
}

int  EncCu::updateCtuDataISlice(const CPelBuf buf)
{
  int  xBl, yBl;
  const int iBlkSize = 8;
  const Pel* pOrgInit = buf.buf;
  int  iStrideOrig = buf.stride;

  int iSumHad = 0;
  for( yBl = 0; ( yBl + iBlkSize ) <= buf.height; yBl += iBlkSize )
  {
    for( xBl = 0; ( xBl + iBlkSize ) <= buf.width; xBl += iBlkSize )
    {
      const Pel* pOrg = pOrgInit + iStrideOrig*yBl + xBl;
      iSumHad += xCalcHADs8x8_ISlice( pOrg, iStrideOrig );
    }
  }
  return( iSumHad );
}

bool EncCu::xCheckBestMode( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  bool bestCSUpdated = false;

  if (!tempCS->cus.empty())   // 确认当前CU是否为空
  {
   
    if( tempCS->cus.size() == 1 ) //如果没有块划分
    {
      // 提取cus的第一个cu，因为没有块划分所以只提取一个CU就好了。
      const CodingUnit& cu = *tempCS->cus.front();
      CHECK( cu.skip && !cu.firstPU->mergeFlag, "Skip flag without a merge flag is not allowed!" );
    }

#if WCG_EXT
    DTRACE_BEST_MODE( tempCS, bestCS, m_pcRdCost->getLambda( true ) );
#else
    DTRACE_BEST_MODE( tempCS, bestCS, m_pcRdCost->getLambda() );
#endif

    if( m_modeCtrl->useModeResult( encTestMode, tempCS, partitioner ) )
    {
      // 交换temp与best
      std::swap( tempCS, bestCS );
      // store temp best CI for next CU coding
      m_CurrCtx->best = m_CABACEstimator->getCtx();
      m_bestModeUpdated = true;
      bestCSUpdated = true;
    }
  }

  // reset context states
  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  return bestCSUpdated;
}

void EncCu::xCompressCU( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& partitioner, double maxCostAllowed )
{
  // xCheckModeSplit函数会调用xCompressCU进行尝试更小的QT、BT和TT划分
  // tempCS用来存储当前compress区域各种模式下的数据
  CHECK(maxCostAllowed < 0, "Wrong value of maxCostAllowed!");

  uint32_t compBegin; // 要处理的起始分量
  uint32_t numComp; // 要处理的分量数量
  bool jointPLT = false;
  // 是否是独立树结构TREE_L或者TREE_C
  if (partitioner.isSepTree( *tempCS )) 
  {
    //非I帧或者单数并且不是TREE_D
    if( !CS::isDualITree(*tempCS) && partitioner.treeType != TREE_D ) // TREE_L或者TREE_C情况下
    {
      compBegin = COMPONENT_Y;
      numComp = (tempCS->area.chromaFormat != CHROMA_400)?3: 1;
      jointPLT = true;
    }
    else // 双树，那就要分开进行处理
    {
      if (isLuma(partitioner.chType))
      {
        compBegin = COMPONENT_Y;
        numComp   = 1;
      }
      else
      {
        compBegin = COMPONENT_Cb;
        numComp   = 2;
      }
    }
  }
  else // 单树，亮度色度共同划分，就jointPLT
  {
    compBegin = COMPONENT_Y;
    numComp = (tempCS->area.chromaFormat != CHROMA_400) ? 3 : 1;
    jointPLT = true;
  }
   
  // 存储PLT模式的变量
  SplitSeries splitmode = -1;
  uint8_t   bestLastPLTSize[MAX_NUM_CHANNEL_TYPE];
  Pel       bestLastPLT[MAX_NUM_COMPONENT][MAXPLTPREDSIZE]; // store LastPLT for
  uint8_t   curLastPLTSize[MAX_NUM_CHANNEL_TYPE];
  Pel       curLastPLT[MAX_NUM_COMPONENT][MAXPLTPREDSIZE]; // store LastPLT if no partition
  
  for (int i = compBegin; i < (compBegin + numComp); i++)
  {
    ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
    bestLastPLTSize[comID] = 0;
    curLastPLTSize[comID] = tempCS->prevPLT.curPLTSize[comID];
    memcpy(curLastPLT[i], tempCS->prevPLT.curPLT[i], tempCS->prevPLT.curPLTSize[comID] * sizeof(Pel));
  }

  Slice&   slice      = *tempCS->slice;
  const PPS &pps      = *tempCS->pps;
  const SPS &sps      = *tempCS->sps;
  const uint32_t uiLPelX  = tempCS->area.Y().lumaPos().x;
  const uint32_t uiTPelY  = tempCS->area.Y().lumaPos().y;

  // 进行划分得到当前CU时，划分类实例的一些状态变量，因为当前CU可能进一步划分也可能不划分，类实例可能会发生变化
  const ModeType modeTypeParent  = partitioner.modeType;//父节点的模式类型INTER OR INTRA
  const TreeType treeTypeParent  = partitioner.treeType;//父节点的树类型
  const ChannelType chTypeParent = partitioner.chType;//父节点的通道类型
  const UnitArea currCsArea = clipArea( CS::getArea( *bestCS, bestCS->area, partitioner.chType ), *tempCS->picture );


  // 该函数对m_modeCtrl进行初始化，确定划分最小、最大深度，获取 baseQP、minQP、maxQP
  // 由堆栈控制，将当前CU可以划分的模式（四叉划分、二叉划分和三叉划分）、编码模式（帧内、帧间Merge、帧间inter等）入栈
  // m_modeCtrl维护需要测试的mode，通过一个do-while循环，尝试各种划分模式。
  m_modeCtrl->initCULevel( partitioner, *tempCS );
  

#if GDR_ENABLED
  if (m_pcEncCfg->getGdrEnabled())
  {
    bool isInGdrInterval = slice.getPicHeader()->getInGdrInterval();

    // 1.0 applicable to inter picture only
    if (isInGdrInterval)
    {
      int gdrPocStart = m_pcEncCfg->getGdrPocStart();
      int gdrInterval = m_pcEncCfg->getGdrInterval();

      int picWidth = slice.getPPS()->getPicWidthInLumaSamples();
      int m1, m2, n1;

      int curPoc = slice.getPOC();
      int gdrPoc = (curPoc - gdrPocStart) % gdrInterval;

      int begGdrX = 0;
      int endGdrX = 0;

      double dd = (picWidth / (double)gdrInterval);
      int mm = (int)((picWidth / (double)gdrInterval) + 0.49999);
      m1 = ((mm + 7) >> 3) << 3;
      m2 = ((mm + 0) >> 3) << 3;

      if (dd > mm && m1 == m2)
      {
        m1 = m1 + 8;
      }

      n1 = (picWidth - m2 * gdrInterval) / 8;

      if (gdrPoc < n1)
      {
        begGdrX = m1 * gdrPoc;
        endGdrX = begGdrX + m1;
      }
      else
      {
        begGdrX = m1 * n1 + m2 * (gdrPoc - n1);
        endGdrX = begGdrX + m2;
        if (picWidth <= endGdrX)
        {
          begGdrX = picWidth;
          endGdrX = picWidth;
        }
      }

      bool isInRefreshArea = tempCS->withinRefresh(begGdrX, endGdrX);

      if (isInRefreshArea)
      {
        m_modeCtrl->forceIntraMode();
      }
      else if (tempCS->containRefresh(begGdrX, endGdrX) || tempCS->overlapRefresh(begGdrX, endGdrX))
      {
        // 1.3.1 enable only vertical splits (QT, BT_V, TT_V)
        m_modeCtrl->forceVerSplitOnly();

        // 1.3.2 remove TT_V if it does not satisfy the condition
        if (tempCS->refreshCrossTTV(begGdrX, endGdrX))
        {
          m_modeCtrl->forceRemoveTTV();
        }
      }

      if (tempCS->area.lwidth() != tempCS->area.lheight())
      {
        m_modeCtrl->forceRemoveQT();
      }

      if (!m_modeCtrl->anyPredModeLeft())
      {
        m_modeCtrl->forceRemoveDontSplit();
      }

      if (isInRefreshArea && !m_modeCtrl->anyIntraIBCMode() && (tempCS->area.lwidth() == 4 || tempCS->area.lheight() == 4))
      {
        m_modeCtrl->finishCULevel(partitioner);
        return;
      }
    }
  }
#endif

  // 如果当前QT深度和Mt深度为0，不为I帧且SBT或MTS开启，则运行。SBT sub-block transform默认false
  if( partitioner.currQtDepth == 0 && partitioner.currMtDepth == 0 && !tempCS->slice->isIntra() && ( sps.getUseSBT() || sps.getUseInterMTS() ) )
  {
    auto slsSbt = dynamic_cast<SaveLoadEncInfoSbt*>( m_modeCtrl );
    int maxSLSize = sps.getUseSBT() ? tempCS->slice->getSPS()->getMaxTbSize() : MTS_INTER_MAX_CU_SIZE;
    slsSbt->resetSaveloadSbt( maxSLSize );
  }
  m_sbtCostSave[0] = m_sbtCostSave[1] = MAX_DOUBLE;
  // 上下文模型起点
  m_CurrCtx->start = m_CABACEstimator->getCtx();
  // 是否使用色度QP调整
  if( slice.getUseChromaQpAdj() )
  {
    // TODO M0133 : double check encoder decisions with respect to chroma QG detection and actual encode
    int lgMinCuSize = sps.getLog2MinCodingBlockSize() +
      std::max<int>(0, floorLog2(sps.getCTUSize()) - sps.getLog2MinCodingBlockSize() - int((slice.getCuChromaQpOffsetSubdiv()+1) / 2));
    if( partitioner.currQgChromaEnable() )
    {
      m_cuChromaQpOffsetIdxPlus1 = ( ( uiLPelX >> lgMinCuSize ) + ( uiTPelY >> lgMinCuSize ) ) % ( pps.getChromaQpOffsetListLen() + 1 );
    }
  }
  else
  {
    m_cuChromaQpOffsetIdxPlus1 = 0;
  }

  // m_modeCtrl初始化后，如果没有可以test的模式，那么直接return
  if( !m_modeCtrl->anyMode() )
  {
    m_modeCtrl->finishCULevel( partitioner );
    return;
  }

  // 确定CU的位置和长宽
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cux", uiLPelX ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cuy", uiTPelY ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cuw", tempCS->area.lwidth() ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cuh", tempCS->area.lheight() ) );
  DTRACE( g_trace_ctx, D_COMMON, "@(%4d,%4d) [%2dx%2d]\n", tempCS->area.lx(), tempCS->area.ly(), tempCS->area.lwidth(), tempCS->area.lheight() );


  m_pcInterSearch->resetSavedAffineMotion();

  // 最佳的整像素帧间预测的cost
  double bestIntPelCost = MAX_DOUBLE;
  // 若使用色彩变换
  // 当前CS与最佳CS的 cost 均初始化为最大值，且选中第一色彩空间
  if (tempCS->slice->getSPS()->getUseColorTrans())
  {
    tempCS->tmpColorSpaceCost = MAX_DOUBLE;
    bestCS->tmpColorSpaceCost = MAX_DOUBLE;
    tempCS->firstColorSpaceSelected = true;
    bestCS->firstColorSpaceSelected = true;
  }
  // 若使用色彩变换，且不是双重intra树时
  // 当前CS与最佳CS均设置不止测试第一色彩空间，且两色彩空间的 Intra cost 初始化为最大值
  if (tempCS->slice->getSPS()->getUseColorTrans() && !CS::isDualITree(*tempCS))
  {
    tempCS->firstColorSpaceTestOnly = false;
    bestCS->firstColorSpaceTestOnly = false;
    tempCS->tmpColorSpaceIntraCost[0] = MAX_DOUBLE;
    tempCS->tmpColorSpaceIntraCost[1] = MAX_DOUBLE;
    bestCS->tmpColorSpaceIntraCost[0] = MAX_DOUBLE;
    bestCS->tmpColorSpaceIntraCost[1] = MAX_DOUBLE;

    if (tempCS->bestParent && tempCS->bestParent->firstColorSpaceTestOnly)
    {
      tempCS->firstColorSpaceTestOnly = bestCS->firstColorSpaceTestOnly = true;
    }
  }

  if (tempCS->slice->getCheckLDC())
  {
    m_bestBcwCost[0] = m_bestBcwCost[1] = std::numeric_limits<double>::max();
    m_bestBcwIdx[0] = m_bestBcwIdx[1] = -1;

  }
  /*----------------这里开始测试各种模式------------------*/
  int u_count = 0;
  vector<EncTestModeType> u_bestdivision;
  vector<double>          u_costlist;
  // 遍历模式列表，根据待测试模式不同调用不同的函数
  // 每个模式选择的函数中都会调用xCheckBestMode函数将最佳模式下的tempCS中的内容复制到bestCS中。
  do
  { 
    u_count++;
    for (int i = compBegin; i < (compBegin + numComp); i++)
    {
      ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
      tempCS->prevPLT.curPLTSize[comID] = curLastPLTSize[comID];
      memcpy(tempCS->prevPLT.curPLT[i], curLastPLT[i], curLastPLTSize[comID] * sizeof(Pel));
    }
    
    // 出栈，获得当前需要测试的模式
    EncTestMode currTestMode = m_modeCtrl->currTestMode();
    currTestMode.maxCostAllowed = maxCostAllowed; //当前测试模式允许的最大cost
    // delta QP,量化相关的参数
    if (pps.getUseDQP() && partitioner.isSepTree(*tempCS) && isChroma( partitioner.chType ))
    {
      const Position chromaCentral(tempCS->area.Cb().chromaPos().offset(tempCS->area.Cb().chromaSize().width >> 1, tempCS->area.Cb().chromaSize().height >> 1));
      const Position lumaRefPos(chromaCentral.x << getComponentScaleX(COMPONENT_Cb, tempCS->area.chromaFormat), chromaCentral.y << getComponentScaleY(COMPONENT_Cb, tempCS->area.chromaFormat));
      const CodingStructure* baseCS = bestCS->picture->cs;
      const CodingUnit* colLumaCu = baseCS->getCU(lumaRefPos, CHANNEL_TYPE_LUMA);

      if (colLumaCu)
      {
        currTestMode.qp = colLumaCu->qp;
      }
    }

#if SHARP_LUMA_DELTA_QP || ENABLE_QPA_SUB_CTU
    if (partitioner.currQgEnable() && (
#if SHARP_LUMA_DELTA_QP
        (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()) ||
#endif
        (m_pcEncCfg->getSmoothQPReductionEnable()) ||
#if ENABLE_QPA_SUB_CTU
        (m_pcEncCfg->getUsePerceptQPA() && !m_pcEncCfg->getUseRateCtrl() && pps.getUseDQP())
#else
        false
#endif
      ))
    {
      if (currTestMode.qp >= 0)
      {
        updateLambda (&slice, currTestMode.qp,
 #if WCG_EXT && ER_CHROMA_QP_WCG_PPS
                      m_pcEncCfg->getWCGChromaQPControl().isEnabled(),
 #endif
                      CS::isDualITree (*tempCS) || (partitioner.currDepth == 0));
      }
    }
#endif

    // 帧间AMVP，常规AMVP模式或者Affine AMVP模式
    if( currTestMode.type == ETM_INTER_ME )
    {
      // AMVR 
      if( ( currTestMode.opts & ETO_IMV ) != 0 )
      {
        const bool skipAltHpelIF = ( int( ( currTestMode.opts & ETO_IMV ) >> ETO_IMV_SHIFT ) == 4 ) && ( bestIntPelCost > 1.25 * bestCS->cost );
        if (!skipAltHpelIF)
        {
          tempCS->bestCS = bestCS;
          xCheckRDCostInterIMV(tempCS, bestCS, partitioner, currTestMode, bestIntPelCost);
          tempCS->bestCS = nullptr;
        }
      }
      else // 非AMVR
      {
        tempCS->bestCS = bestCS;
        xCheckRDCostInter( tempCS, bestCS, partitioner, currTestMode );
        tempCS->bestCS = nullptr;
      }

    }
    // HashInter
    else if (currTestMode.type == ETM_HASH_INTER)
    {
      //基于hash的快速搜索
      xCheckRDCostHashInter( tempCS, bestCS, partitioner, currTestMode );
    }
    // Affine Merge模式
    else if( currTestMode.type == ETM_AFFINE ) 
    {
      xCheckRDCostAffineMerge2Nx2N( tempCS, bestCS, partitioner, currTestMode );
    }
#if REUSE_CU_RESULTS
    // Reuse Cached 这个不知道干什么
    else if( currTestMode.type == ETM_RECO_CACHED )
    {
      xReuseCachedResult( tempCS, bestCS, partitioner );
    }
#endif
    // 常规Merge模式和Skip模式
    else if( currTestMode.type == ETM_MERGE_SKIP )
    { 
      xCheckRDCostMerge2Nx2N( tempCS, bestCS, partitioner, currTestMode );
      CodingUnit* cu = bestCS->getCU(partitioner.chType);
      if (cu)
      {
        cu->mmvdSkip = cu->skip == false ? false : cu->mmvdSkip;
      }
    }
    // GEO模式，没去研究
    else if( currTestMode.type == ETM_MERGE_GEO )
    {//几何划分模式 
      xCheckRDCostMergeGeo2Nx2N( tempCS, bestCS, partitioner, currTestMode );
    }
    // 帧内预测
    else if( currTestMode.type == ETM_INTRA )
    {
      // 启用Adaptive Color Transform且变换且不是双树，ACT是针对SCC YUV444格式的视频，只有I帧才双树
      // 默认情况下不启用ACT变换
      if (slice.getSPS()->getUseColorTrans() && !CS::isDualITree(*tempCS))
      {
        bool skipSecColorSpace = false;
        skipSecColorSpace = xCheckRDCostIntra(tempCS, bestCS, partitioner, currTestMode, (m_pcEncCfg->getRGBFormatFlag() ? true : false));
        if ((m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless()) && !m_pcEncCfg->getRGBFormatFlag())
        {
          skipSecColorSpace = true;
        }
        if (!skipSecColorSpace && !tempCS->firstColorSpaceTestOnly)
        {
          xCheckRDCostIntra(tempCS, bestCS, partitioner, currTestMode, (m_pcEncCfg->getRGBFormatFlag() ? false : true));
        }

        if (!tempCS->firstColorSpaceTestOnly)
        {
          if (tempCS->tmpColorSpaceIntraCost[0] != MAX_DOUBLE && tempCS->tmpColorSpaceIntraCost[1] != MAX_DOUBLE)
          {
            double skipCostRatio = m_pcEncCfg->getRGBFormatFlag() ? 1.1 : 1.0;
            if (tempCS->tmpColorSpaceIntraCost[1] > (skipCostRatio*tempCS->tmpColorSpaceIntraCost[0]))
            {
              tempCS->firstColorSpaceTestOnly = bestCS->firstColorSpaceTestOnly = true;
            }
          }
        }
        else
        {
          CHECK(tempCS->tmpColorSpaceIntraCost[1] != MAX_DOUBLE, "the RD test of the second color space should be skipped");
        }
      }
      else
      {
        // 选出最佳帧内模式和变换类型。
        xCheckRDCostIntra(tempCS, bestCS, partitioner, currTestMode, false);
      }
      u_bestdivision.push_back(currTestMode.type);
      u_costlist.push_back(bestCS->cost);
    }
    else if (currTestMode.type == ETM_PALETTE)
    {//PLT
      xCheckPLT( tempCS, bestCS, partitioner, currTestMode );
    }
    else if (currTestMode.type == ETM_IBC)
    {//IBC 
      xCheckRDCostIBCMode(tempCS, bestCS, partitioner, currTestMode);
    }
    else if (currTestMode.type == ETM_IBC_MERGE)
    {//IBC Merge
      xCheckRDCostIBCModeMerge2Nx2N(tempCS, bestCS, partitioner, currTestMode);
    }
    // 划分模式（四叉树、水平三叉树、垂直三叉树、水平二 叉树、垂直二叉树）
    else if( isModeSplit( currTestMode ) ) // 四叉、二叉、三叉划分为true
    {
      // QT,BT, TT划分
      // 若当前CU已划分，则当前BestCs的cu数目就不为零。
      if (bestCS->cus.size() != 0)
      {
        splitmode = bestCS->cus[0]->splitSeries;
      }
      // 检验当前CS结构modeType是否和划分类中设定的modeType相同，如果不同，则终止程序
      // 帧内/帧间
      assert( partitioner.modeType == tempCS->modeType );

      // 这里的父节点意思是进行CU划分之后，树的上一层和这一层是父子关系
      // 防止出现帧内色度小块,提高最坏情况下的吞吐量。
      // 根据当前树的结构、划分类型以及区域尺寸判断signalModeConsVal
      // LDT_MODE_TYPE_INHERIT子块 RDO 测试模式（intra or inter）与父结点相同。
      // LDT_MODE_TYPE_INFER 子块 RDO 仅测试 Intra
      // LDT_MODE_TYPE_SIGNAL 子块 RDO 先测试 Inter 后测试 Intra      

      // 1. DualTree或者父节点不是MODE_TYPE_ALL（inter与intra均尝试）或者YUV444格式或者YUV400格式，子块RDO继承父节点
      // 2. QT/TH/TV，最小亮度区域为当前区域的1/4； BV/BH，最小亮度区域为当前区域1/2；最小色度块=最小亮度区域 >> scale(色度格式)
      // 3. 划分后最小色度块 >= 16 且 色度块尺寸不是 2*N 时，子块RDO继承父节点
      // 4. 不满足第3点，最小亮度块 < 32 或 当前帧为 intra 时子块RDO为intra
      // 5. 不满足第4点，也不满足第3点时，先inter在intra

      int signalModeConsVal = tempCS->signalModeCons( getPartSplit( currTestMode ), partitioner, modeTypeParent );
      // 对于SCIPU（最小帧内色度预测单元），其所有的Cb都是inter模式或者所有CB都是intra模式
      // 因此，需要两次RDO过程，第一次RDO仅测试inter；第二次RDO仅测试intra模式

      int numRoundRdo = signalModeConsVal == LDT_MODE_TYPE_SIGNAL ? 2 : 1;
      bool skipInterPass = false;
      /*-----------上边不太懂-----------*/
      // 遍历 RDO 轮次
      for( int i = 0; i < numRoundRdo; i++ )
      {
        //change cons modes
        //  若当前信号模式值为 LDT_MODE_TYPE_SIGNAL
        //  则当前测试模式类型在第一轮设置为 INTER，在第二轮设置为 INTRA
        if( signalModeConsVal == LDT_MODE_TYPE_SIGNAL )// 需要传输
        {
          CHECK( numRoundRdo != 2, "numRoundRdo shall be 2 - [LDT_MODE_TYPE_SIGNAL]" );
          tempCS->modeType = partitioner.modeType = (i == 0) ? MODE_TYPE_INTER : MODE_TYPE_INTRA;
        }
        // 信号模式值为 LDT_MODE_TYPE_INFER,则当前测试模式类型为 INTRA
        else if( signalModeConsVal == LDT_MODE_TYPE_INFER )//不需要传输，推导出为帧内模式
        {
          CHECK( numRoundRdo != 1, "numRoundRdo shall be 1 - [LDT_MODE_TYPE_INFER]" );
          tempCS->modeType = partitioner.modeType = MODE_TYPE_INTRA;
        }
        //信号模式值为 LDT_MODE_TYPE_INHERIT,则当前测试模式类型为父结点模式类
        else if( signalModeConsVal == LDT_MODE_TYPE_INHERIT )//继承
        {
          CHECK( numRoundRdo != 1, "numRoundRdo shall be 1 - [LDT_MODE_TYPE_INHERIT]" );
          tempCS->modeType = partitioner.modeType = modeTypeParent;
        }

        //for lite intra encoding fast algorithm, set the status to save inter coding info
        //  对于lite帧内编码快速算法，将状态设置为保存帧间编码信息
        // 若父结点模式类型为 MODE_TYPE_ALL（可以尝试所有类型），且当前测试模式类型为 Inter
        // 则 SaveCuCostInSCIPU，且设置 NumCuInSCIPU 为0
        if( modeTypeParent == MODE_TYPE_ALL && tempCS->modeType == MODE_TYPE_INTER )
        {
          m_pcIntraSearch->setSaveCuCostInSCIPU( true );
          m_pcIntraSearch->setNumCuInSCIPU( 0 );
        }
        // 若父结点模式类型为 MODE_TYPE_ALL（可以尝试所有类型），且当前测试模式类型不为 Inter
        // 则 SaveCuCostInSCIPU 为 false，且当前测试模式 MODE_TYPE_ALL 时设置 NumCuInSCIPU 为0
        else if( modeTypeParent == MODE_TYPE_ALL && tempCS->modeType != MODE_TYPE_INTER )
        {
          m_pcIntraSearch->setSaveCuCostInSCIPU( false );
          if( tempCS->modeType == MODE_TYPE_ALL )
          {
            m_pcIntraSearch->setNumCuInSCIPU( 0 );
          }
        }

        // 进行QT、BT和TT划分
        xCheckModeSplit( tempCS, bestCS, partitioner, currTestMode, modeTypeParent, skipInterPass );
        u_bestdivision.push_back(currTestMode.type);
        u_costlist.push_back(bestCS->cost);
        //recover cons modes
        tempCS->modeType = partitioner.modeType = modeTypeParent;
        tempCS->treeType = partitioner.treeType = treeTypeParent;
        partitioner.chType = chTypeParent;
        if( modeTypeParent == MODE_TYPE_ALL )
        {
          m_pcIntraSearch->setSaveCuCostInSCIPU( false );
          if( numRoundRdo == 2 && tempCS->modeType == MODE_TYPE_INTRA )
          {
            m_pcIntraSearch->initCuAreaCostInSCIPU();
          }
        }
        if( skipInterPass )
        {
          break;
        }
      }// for( int i = 0; i < numRoundRdo; i++ )
#if GDR_ENABLED
      if (bestCS->cus.size() > 0 && splitmode != bestCS->cus[0]->splitSeries)
#else
      if (splitmode != bestCS->cus[0]->splitSeries)
#endif
      {
        splitmode = bestCS->cus[0]->splitSeries;
        const CodingUnit&     cu = *bestCS->cus.front();
        cu.cs->prevPLT = bestCS->prevPLT;
        for (int i = compBegin; i < (compBegin + numComp); i++)
        {
          ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
          bestLastPLTSize[comID] = bestCS->cus[0]->cs->prevPLT.curPLTSize[comID];
          memcpy(bestLastPLT[i], bestCS->cus[0]->cs->prevPLT.curPLT[i], bestCS->cus[0]->cs->prevPLT.curPLTSize[comID] * sizeof(Pel));
        }
      }
    }
    else
    {
      THROW( "Don't know how to handle mode: type = " << currTestMode.type << ", options = " << currTestMode.opts );
    }
  } while( m_modeCtrl->nextMode( *tempCS, partitioner ) ); // 先进后出栈


  //////////////////////////////////////////////////////////////////////////
  //if (u_count > 1)  { 
  if (bestCS->area.lwidth() == 64 || bestCS->area.lheight() == 64)
  {
    auto str = (partitioner.chType == CHANNEL_TYPE_LUMA) ? " Luma" : "Chroma";
    cout << str << "(" << bestCS->area.lx() << "," << bestCS->area.ly()
                                                          << ") " << bestCS->area.lwidth() << "x"
         << bestCS->area.lheight()   <<" : count " << u_count;
    for (int i = 0; i < u_costlist.size(); i++)
    {
      cout << " {" << u_costlist[i] << "," << u_EncTestModeType[u_bestdivision[i]] << "}";
    }
    cout << endl;
  
  }

  // Finishing CU
  // 将最优划分模式和最优划分方法都存储在bestCS中
  //  在运行完以后，m_ComprCUCtxList的size减少了1
  // .原本的那层出了intra没有其他模式，所以当前CU已经算测试完了，就把栈中这个CU的信息消去，返回上一层父CU
  if( tempCS->cost == MAX_DOUBLE && bestCS->cost == MAX_DOUBLE )
  {
    //although some coding modes were planned to be tried in RDO, no coding mode actually finished encoding due to early termination
    //thus tempCS->cost and bestCS->cost are both MAX_DOUBLE; in this case, skip the following process for normal case 
    // 虽然计划在RDO中尝试一些编码模式，但由于提前终止，没有编码模式真正完成编码 因此
    // tempCS->cost 和 bestCS->cost 都是 MAX_DOUBLE; 在这种情况下，对于正常情况，请跳过以下过程
    m_modeCtrl->finishCULevel( partitioner );
    return;
  }
  // 更新参数
  // set context states
  m_CABACEstimator->getCtx() = m_CurrCtx->best;

  // QP from last processed CU for further processing
  //copy the qp of the last non-chroma CU
  int numCUInThisNode = (int)bestCS->cus.size();
  if( numCUInThisNode > 1 && bestCS->cus.back()->chType == CHANNEL_TYPE_CHROMA && !CS::isDualITree( *bestCS ) )
  {
    CHECK( bestCS->cus[numCUInThisNode-2]->chType != CHANNEL_TYPE_LUMA, "wrong chType" );
    bestCS->prevQP[partitioner.chType] = bestCS->cus[numCUInThisNode-2]->qp;
  }
  else
  {
  bestCS->prevQP[partitioner.chType] = bestCS->cus.back()->qp;
  }
  if ((!slice.isIntra() || slice.getSPS()->getIBCFlag())
    && partitioner.chType == CHANNEL_TYPE_LUMA
    && bestCS->cus.size() == 1 && (bestCS->cus.back()->predMode == MODE_INTER || bestCS->cus.back()->predMode == MODE_IBC)
    && bestCS->area.Y() == (*bestCS->cus.back()).Y()
    )
  {
    const CodingUnit&     cu = *bestCS->cus.front();

    bool isIbcSmallBlk = CU::isIBC(cu) && (cu.lwidth() * cu.lheight() <= 16);
    CU::saveMotionInHMVP( cu, isIbcSmallBlk );// 更新HMVP列表
  }

  // 将预测像素和重建像素复制到bestCS->picture中
  bestCS->picture->getPredBuf(currCsArea).copyFrom(bestCS->getPredBuf(currCsArea));
  bestCS->picture->getRecoBuf( currCsArea ).copyFrom( bestCS->getRecoBuf( currCsArea ) );
  m_modeCtrl->finishCULevel( partitioner );
  if( m_pcIntraSearch->getSaveCuCostInSCIPU() && bestCS->cus.size() == 1 )
  {
    m_pcIntraSearch->saveCuAreaCostInSCIPU( Area( partitioner.currArea().lumaPos(), partitioner.currArea().lumaSize() ), bestCS->cost );
  }
  // PLT 相关
  if (bestCS->cus.size() == 1) // no partition
  {
    CHECK(bestCS->cus[0]->tileIdx != bestCS->pps->getTileIdx(bestCS->area.lumaPos()), "Wrong tile index!");
    if (bestCS->cus[0]->predMode == MODE_PLT)
    {
      for (int i = compBegin; i < (compBegin + numComp); i++)
      {
        ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
        bestCS->prevPLT.curPLTSize[comID] = curLastPLTSize[comID];
        memcpy(bestCS->prevPLT.curPLT[i], curLastPLT[i], curLastPLTSize[comID] * sizeof(Pel));
      }
      bestCS->reorderPrevPLT(bestCS->prevPLT, bestCS->cus[0]->curPLTSize, bestCS->cus[0]->curPLT, bestCS->cus[0]->reuseflag, compBegin, numComp, jointPLT);
    }
    else
    {
      for (int i = compBegin; i<(compBegin + numComp); i++)
      {
        ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
        bestCS->prevPLT.curPLTSize[comID] = curLastPLTSize[comID];
        memcpy(bestCS->prevPLT.curPLT[i], curLastPLT[i], bestCS->prevPLT.curPLTSize[comID] * sizeof(Pel));
      }
    }
  }
  else
  {
    for (int i = compBegin; i<(compBegin + numComp); i++)
    {
      ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
      bestCS->prevPLT.curPLTSize[comID] = bestLastPLTSize[comID];
      memcpy(bestCS->prevPLT.curPLT[i], bestLastPLT[i], bestCS->prevPLT.curPLTSize[comID] * sizeof(Pel));
    }
  }
  const CodingUnit&     cu = *bestCS->cus.front();
  cu.cs->prevPLT = bestCS->prevPLT;
  // Assert if Best prediction mode is NONE
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  CHECK( bestCS->cus.empty()                                   , "No possible encoding found" );
  CHECK( bestCS->cus[0]->predMode == NUMBER_OF_PREDICTION_MODES, "No possible encoding found" );
  CHECK( bestCS->cost             == MAX_DOUBLE                , "No possible encoding found" );
}

#if SHARP_LUMA_DELTA_QP || ENABLE_QPA_SUB_CTU
void EncCu::updateLambda (Slice* slice, const int dQP,
 #if WCG_EXT && ER_CHROMA_QP_WCG_PPS
                          const bool useWCGChromaControl,
 #endif
                          const bool updateRdCostLambda)
{
#if WCG_EXT && ER_CHROMA_QP_WCG_PPS
  if (useWCGChromaControl)
  {
    const double lambda = m_pcSliceEncoder->initializeLambda (slice, m_pcSliceEncoder->getGopId(), slice->getSliceQp(), (double)dQP);
    const int clippedQP = Clip3 (-slice->getSPS()->getQpBDOffset (CHANNEL_TYPE_LUMA), MAX_QP, dQP);

    m_pcSliceEncoder->setUpLambda (slice, lambda, clippedQP);
    return;
  }
#endif
  int iQP = dQP;
  const double oldQP     = (double)slice->getSliceQpBase();
#if ENABLE_QPA_SUB_CTU
  const double oldLambda = (m_pcEncCfg->getUsePerceptQPA() && !m_pcEncCfg->getUseRateCtrl() && slice->getPPS()->getUseDQP()) ? slice->getLambdas()[0] :
                           m_pcSliceEncoder->calculateLambda (slice, m_pcSliceEncoder->getGopId(), oldQP, oldQP, iQP);
#else
  const double oldLambda = m_pcSliceEncoder->calculateLambda (slice, m_pcSliceEncoder->getGopId(), oldQP, oldQP, iQP);
#endif
  const double newLambda = oldLambda * pow (2.0, ((double)dQP - oldQP) / 3.0);
#if RDOQ_CHROMA_LAMBDA
  const double lambdaArray[MAX_NUM_COMPONENT] = {newLambda / m_pcRdCost->getDistortionWeight (COMPONENT_Y),
                                                 newLambda / m_pcRdCost->getDistortionWeight (COMPONENT_Cb),
                                                 newLambda / m_pcRdCost->getDistortionWeight (COMPONENT_Cr)};
  m_pcTrQuant->setLambdas (lambdaArray);
#else
  m_pcTrQuant->setLambda (newLambda);
#endif
  if (updateRdCostLambda)
  {
    m_pcRdCost->setLambda (newLambda, slice->getSPS()->getBitDepths());
#if WCG_EXT
#if !JVET_W0043
    if (!(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || m_pcEncCfg->getSmoothQPReductionEnable()))
#else
    if (!m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled())
#endif
    {
      m_pcRdCost->saveUnadjustedLambda();
    }
#endif
  }
}
#endif // SHARP_LUMA_DELTA_QP || ENABLE_QPA_SUB_CTU

void EncCu::xCheckModeSplit(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, const ModeType modeTypeParent, bool &skipInterPass )
{
  const int qp = encTestMode.qp;// 当前测试qp
  const Slice &slice          = *tempCS->slice;//当前slice
  const int oldPrevQp         = tempCS->prevQP[partitioner.chType];
  const auto oldMotionLut     = tempCS->motionLut;
#if ENABLE_QPA_SUB_CTU
  const PPS &pps              = *tempCS->pps;
  const uint32_t currDepth    = partitioner.currDepth;
#endif
  const auto oldPLT           = tempCS->prevPLT;
  // 获取需要测试的划分模式
  const PartSplit split = getPartSplit( encTestMode ); 
  // 子节点的模式类型
  const ModeType modeTypeChild = partitioner.modeType;

  CHECK( split == CU_DONT_SPLIT, "No proper split provided!" );

  tempCS->initStructData( qp );
  // 获取上下文
  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  // 定位上下文各部分起始位置
  const TempCtx ctxStartSP( m_CtxCache, SubCtx( Ctx::SplitFlag,   m_CABACEstimator->getCtx() ) );
  const TempCtx ctxStartQt( m_CtxCache, SubCtx( Ctx::SplitQtFlag, m_CABACEstimator->getCtx() ) );
  const TempCtx ctxStartHv( m_CtxCache, SubCtx( Ctx::SplitHvFlag, m_CABACEstimator->getCtx() ) );
  const TempCtx ctxStart12( m_CtxCache, SubCtx( Ctx::Split12Flag, m_CABACEstimator->getCtx() ) );
  const TempCtx ctxStartMC( m_CtxCache, SubCtx( Ctx::ModeConsFlag, m_CABACEstimator->getCtx() ) );
  m_CABACEstimator->resetBits();// 重置上下文比特
  // 设置划分 CU 的模式
  m_CABACEstimator->split_cu_mode( split, *tempCS, partitioner );
  // 设置模式限制
  m_CABACEstimator->mode_constraint( split, *tempCS, partitioner, modeTypeChild );

  // 计算预测 cost 的因子在当前 QP > 30 时为 1.1，否则为 1.075
  const double factor = ( tempCS->currQP[partitioner.chType] > 30 ? 1.1 : 1.075 );
  // 计算根据上下文预测的 cost
  tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();
  if (!tempCS->useDbCost)
    CHECK(bestCS->costDbOffset != 0, "error");
  // 计算上次划分的失真？？
  const double cost   = m_pcRdCost->calcRdCost( uint64_t( m_CABACEstimator->getEstFracBits() + ( ( bestCS->fracBits ) / factor ) ), Distortion( bestCS->dist / factor ) ) + bestCS->costDbOffset / factor;
  // 更新上下文参数
  m_CABACEstimator->getCtx() = SubCtx( Ctx::SplitFlag,   ctxStartSP );
  m_CABACEstimator->getCtx() = SubCtx( Ctx::SplitQtFlag, ctxStartQt );
  m_CABACEstimator->getCtx() = SubCtx( Ctx::SplitHvFlag, ctxStartHv );
  m_CABACEstimator->getCtx() = SubCtx( Ctx::Split12Flag, ctxStart12 );
  m_CABACEstimator->getCtx() = SubCtx( Ctx::ModeConsFlag, ctxStartMC );

  // 若上下文预测的 cost > bestCS cost，则重置上下文状态且确认 bestCS
  // 如果tmpCS优于bestCS，返回，这是建立在best与tmp编码模式不同的基础上，但是从后面代码来看，
  // bset与tmp是一样的，也能理解，因为只有对比划分就是在两个一样的基础上对比划分与不划分的RDcost。
  // 之所以会有不同，是因为循环遍历mpm时，会先选择出不做划分下的最佳编码模式,若是预估的cost(这个预估一定准吗？)比原本的大那就remake，这样可以运算快点

  if (cost > bestCS->cost + bestCS->costDbOffset
#if ENABLE_QPA_SUB_CTU
    || (m_pcEncCfg->getUsePerceptQPA() && !m_pcEncCfg->getUseRateCtrl() && pps.getUseDQP() && (slice.getCuQpDeltaSubdiv() > 0) && (split == CU_HORZ_SPLIT || split == CU_VERT_SPLIT) &&
        (currDepth == 0)) // force quad-split or no split at CTU level
#endif
    )
  {
    // 
    xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
    return;
  }

  // 如果父节点所有类型都行且子节点为MODE_TYPE_INTRA，则chromaNotSplit为true
  // 为TRUE时，第一次执行compressCU时，仅对亮度进行划分，第二次执行compressCU时，对色度划分
  const bool chromaNotSplit = modeTypeParent == MODE_TYPE_ALL && modeTypeChild == MODE_TYPE_INTRA ? true : false;
  // 如果当前树类型不为TREE_D，则将数类型设为TREE_L，
  // 若chromaNotSplit为true，则当前 tempCS 划分树类型设置为TREE_L，
  // 若色度要划分，则设置为TREE_D
  if( partitioner.treeType != TREE_D )
  {
    tempCS->treeType = TREE_L;
  }
  else
  {
    if( chromaNotSplit ) // 色度不进行划分
    {
      CHECK( partitioner.chType != CHANNEL_TYPE_LUMA, "chType must be luma" );
      tempCS->treeType = partitioner.treeType = TREE_L;
    }
    else
    {
      tempCS->treeType = partitioner.treeType = TREE_D;
    }
  }

  // 根据CU划分模式划分当前区域 深度++
  partitioner.splitCurrArea( split, *tempCS );

  // QG(Quantization group): 把一帧图像划分为固定大小的正方形像素块，大小由PPS指定，且必须处于CTU和最小CU之间，
  //  同一个QG内的所有非零系数的CU共享一个QP; HEVC中使用相邻QG的信息预测当前QG内的QP, 
  // predQP =(QP_A + Qp_B + 1) >> 1
  bool qgEnableChildren       = partitioner.currQgEnable();       // QG possible at children level
  //bool qgEnableChildren = partitioner.currQgEnable(); // QG possible at children level
  bool qgChromaEnableChildren = partitioner.currQgChromaEnable(); // Chroma QG possible at children level
  
  //这是一个指向当前上下文的指针变量，++表示下一个变量，而CU在做了进一步划分后，深度也会加1，进而当前上下文也会改变。一开始会有初始化，后面可能会有修改
  m_CurrCtx++;
  // 重建和预测像素值都置0
  tempCS->getRecoBuf().fill( 0 );
  tempCS->getPredBuf().fill(0);

  AffineMVInfo tmpMVInfo;
  bool isAffMVInfoSaved;
#if GDR_ENABLED
  AffineMVInfoSolid tmpMVInfoSolid;
  m_pcInterSearch->savePrevAffMVInfo(0, tmpMVInfo, tmpMVInfoSolid, isAffMVInfoSaved);
#else
  m_pcInterSearch->savePrevAffMVInfo(0, tmpMVInfo, isAffMVInfoSaved);
#endif
  BlkUniMvInfo tmpUniMvInfo;
  bool         isUniMvInfoSaved = false;
  if (!tempCS->slice->isIntra())
  {
    m_pcInterSearch->savePrevUniMvInfo(tempCS->area.Y(), tmpUniMvInfo, isUniMvInfoSaved);
  }

  do
  {
    // 得到划分后的当前子区域
    const auto &subCUArea  = partitioner.currArea();

    if( tempCS->picture->Y().contains( subCUArea.lumaPos() ) )
    {

      // 当前划分的索引
      const unsigned wIdx    = gp_sizeIdxInfo->idxFrom( subCUArea.lwidth () );
      const unsigned hIdx    = gp_sizeIdxInfo->idxFrom( subCUArea.lheight() );

      // 从已划分CU的wIdx和hIdx推出新的tempCs
      CodingStructure *tempSubCS = m_pTempCS[wIdx][hIdx];
      CodingStructure *bestSubCS = m_pBestCS[wIdx][hIdx];

      // 将双方的父节点设为bestCS
      tempCS->initSubStructure( *tempSubCS, partitioner.chType, subCUArea, false );
      tempCS->initSubStructure( *bestSubCS, partitioner.chType, subCUArea, false );

      tempSubCS->bestParent = bestSubCS->bestParent = bestCS;
      double newMaxCostAllowed = isLuma(partitioner.chType) ? std::min(encTestMode.maxCostAllowed, bestCS->cost - m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist)) : MAX_DOUBLE;
      newMaxCostAllowed = std::max(0.0, newMaxCostAllowed);

      // tempCS和bestCS参数是根据划分后子CU的尺寸从m_pTempCS和m_pBestCS得到的CS结构
      xCompressCU(tempSubCS, bestSubCS, partitioner, newMaxCostAllowed);
      // 子 CU 的 bestParent 设置为空
      tempSubCS->bestParent = bestSubCS->bestParent = nullptr;

      // 子CU的cost是正无穷（？），结束划分
      if( bestSubCS->cost == MAX_DOUBLE )
      {
        CHECK( split == CU_QUAD_SPLIT, "Split decision reusing cannot skip quad split" );
        // 将当前 cost 设置为正无穷
        tempCS->cost = MAX_DOUBLE;
        tempCS->costDbOffset = 0;
        tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();
        m_CurrCtx--;// 退回上下文
        partitioner.exitCurrSplit();// 退出当前划分
        // 检查当前模式是否为最佳模式，若为最佳模式则与 bestCS 交换
        xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
        // 若当前通道为 LUMA
        // 则还原 LUT
        if( partitioner.chType == CHANNEL_TYPE_LUMA )
        {
          tempCS->motionLut = oldMotionLut;
        }
        return;   // 结束划分尝试
      }
      // 是否需要保存残差信号
      bool keepResi = KEEP_PRED_AND_RESI_SIGNALS;
      tempCS->useSubStructure( *bestSubCS, partitioner.chType, CS::getArea( *tempCS, subCUArea, partitioner.chType ), KEEP_PRED_AND_RESI_SIGNALS, true, keepResi, keepResi, true );
      // 若当前 QG 可用
      // 则用子 CU prevQP 更新当前 tempCS 的 prevQP
      if( partitioner.currQgEnable() )
      {
        tempCS->prevQP[partitioner.chType] = bestSubCS->prevQP[partitioner.chType];
      }
      // 循环遍历所有子CU，check是否有不相容的模式
      if( partitioner.isConsInter() )
      {
        // 循环遍历所有子 CU，检查预测模式是否均为 INTER
        for( int i = 0; i < bestSubCS->cus.size(); i++ )
        {
          CHECK( bestSubCS->cus[i]->predMode != MODE_INTER, "all CUs must be inter mode in an Inter coding region (SCIPU)" );
        }
      }
      // 若当前限制尝试 INTRA
      else if( partitioner.isConsIntra() )
      {
        // 循环遍历所有子 CU，检查预测模式是否均为 INTRA
        for( int i = 0; i < bestSubCS->cus.size(); i++ )
        {
          CHECK( bestSubCS->cus[i]->predMode == MODE_INTER, "all CUs must not be inter mode in an Intra coding region (SCIPU)" );
        }
      }
      // tempSubCS和bestSubCS的清空初始化
      tempSubCS->releaseIntermediateData();
      bestSubCS->releaseIntermediateData();
      // 若当前帧不是 I 帧，且当前限制只尝试 INTRA
      if( !tempCS->slice->isIntra() && partitioner.isConsIntra() )
      {
        // 如果计算到目前的tempCS的cost已经大于bestCS的cost，则提前终止块划分的尝试
        tempCS->cost = m_pcRdCost->calcRdCost( tempCS->fracBits, tempCS->dist );

        if( tempCS->cost > bestCS->cost )
        {
          tempCS->cost = MAX_DOUBLE;
          tempCS->costDbOffset = 0;

          tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();
          m_CurrCtx--;
          // 划分的子cu区域，在partitioner中出栈
          partitioner.exitCurrSplit();
          // 若当前通道为 LUMA
          // 则还原 LUT
          if( partitioner.chType == CHANNEL_TYPE_LUMA )
          {
            tempCS->motionLut = oldMotionLut;
          }
          return;
        }
      }
    }
  } while (partitioner.nextPart(*tempCS));   // 尝试所有的划分子CU

  partitioner.exitCurrSplit();   // 划分的子cu区域，在partitioner中出栈


  m_CurrCtx--;
  // 这部分代码需要refer to https://blog.csdn.net/dfhg54/article/details/124291272
  if( chromaNotSplit )
  {
    //Note: In local dual tree region, the chroma CU refers to the central luma CU's QP.
    //If the luma CU QP shall be predQP (no residual in it and before it in the QG), it must be revised to predQP before encoding the chroma CU
    //Otherwise, the chroma CU uses predQP+deltaQP in encoding but is decoded as using predQP, thus causing encoder-decoded mismatch on chroma qp.
    //  若使用 DQP
    if( tempCS->pps->getUseDQP() )
    {
      //find parent CS that including all coded CUs in the QG before this node
      CodingStructure* qgCS = tempCS;
      bool deltaQpCodedBeforeThisNode = false;
      if( partitioner.currArea().lumaPos() != partitioner.currQgPos )
      {
        int numParentNodeToQgCS = 0;
        while( qgCS->area.lumaPos() != partitioner.currQgPos )
        {
          CHECK( qgCS->parent == nullptr, "parent of qgCS shall exsit" );
          qgCS = qgCS->parent;
          numParentNodeToQgCS++;
        }

        //check whether deltaQP has been coded (in luma CU or luma&chroma CU) before this node
        CodingStructure* parentCS = tempCS->parent;
        // 逐层遍历当前父节点的所有CU，检查deltaQP在当前节点之前是否已经编码
        for( int i = 0; i < numParentNodeToQgCS; i++ )
        {
          //checking each parent
          CHECK( parentCS == nullptr, "parentCS shall exsit" );
          for( const auto &cu : parentCS->cus )
          {
            if( cu->rootCbf && !isChroma( cu->chType ) )
            {
              deltaQpCodedBeforeThisNode = true;
              break;
            }
          }
          parentCS = parentCS->parent;
        }
      }

      //revise luma CU qp before the first luma CU with residual in the SCIPU to predQP
      if( !deltaQpCodedBeforeThisNode )
      {
        //get pred QP of the QG
        const CodingUnit* cuFirst = qgCS->getCU( CHANNEL_TYPE_LUMA );
        CHECK( cuFirst->lumaPos() != partitioner.currQgPos, "First cu of the Qg is wrong" );
        int predQp = CU::predictQP( *cuFirst, qgCS->prevQP[CHANNEL_TYPE_LUMA] );

        //revise to predQP
        int firstCuHasResidual = (int)tempCS->cus.size();
        for( int i = 0; i < tempCS->cus.size(); i++ )
        {
          if( tempCS->cus[i]->rootCbf )
          {
            firstCuHasResidual = i;
            break;
          }
        }

        for( int i = 0; i < firstCuHasResidual; i++ )
        {
          tempCS->cus[i]->qp = predQp;
        }
      }
    }
    assert( tempCS->treeType == TREE_L );
    uint32_t numCuPuTu[6];
    tempCS->picture->cs->getNumCuPuTuOffset( numCuPuTu );
    tempCS->picture->cs->useSubStructure( *tempCS, partitioner.chType, CS::getArea( *tempCS, partitioner.currArea(), partitioner.chType ), false, true, false, false, false );

    if (isChromaEnabled(tempCS->pcv->chrFormat))
    {
      partitioner.chType = CHANNEL_TYPE_CHROMA;
      tempCS->treeType = partitioner.treeType = TREE_C;

      m_CurrCtx++;

      const unsigned   wIdx         = gp_sizeIdxInfo->idxFrom(partitioner.currArea().lwidth());
      const unsigned   hIdx         = gp_sizeIdxInfo->idxFrom(partitioner.currArea().lheight());
      CodingStructure *tempCSChroma = m_pTempCS2[wIdx][hIdx];
      CodingStructure *bestCSChroma = m_pBestCS2[wIdx][hIdx];
      tempCS->initSubStructure(*tempCSChroma, partitioner.chType, partitioner.currArea(), false);
      tempCS->initSubStructure(*bestCSChroma, partitioner.chType, partitioner.currArea(), false);
      tempCS->treeType = TREE_D;
      xCompressCU(tempCSChroma, bestCSChroma, partitioner);

      // attach chromaCS to luma CS and update cost
      bool keepResi = KEEP_PRED_AND_RESI_SIGNALS;
      // bestCSChroma->treeType = tempCSChroma->treeType = TREE_C;
      CHECK(bestCSChroma->treeType != TREE_C || tempCSChroma->treeType != TREE_C, "wrong treeType for chroma CS");
      tempCS->useSubStructure(*bestCSChroma, partitioner.chType,
                              CS::getArea(*bestCSChroma, partitioner.currArea(), partitioner.chType),
                              KEEP_PRED_AND_RESI_SIGNALS, true, keepResi, true, true);

      // release tmp resource
      tempCSChroma->releaseIntermediateData();
      bestCSChroma->releaseIntermediateData();
      // tempCS->picture->cs->releaseIntermediateData();
      m_CurrCtx--;
    }
    tempCS->picture->cs->clearCuPuTuIdxMap( partitioner.currArea(), numCuPuTu[0], numCuPuTu[1], numCuPuTu[2], numCuPuTu + 3 );


    //recover luma tree status
    partitioner.chType = CHANNEL_TYPE_LUMA;
    partitioner.treeType = TREE_D;
    partitioner.modeType = MODE_TYPE_ALL;
  }
  else
  {
    if (!qgChromaEnableChildren) // check at deepest cQG level only
    {
      xCheckChromaQPOffset( *tempCS, partitioner );
    }
  }

  // Finally, generate split-signaling bits for RD-cost check
  const PartSplit implicitSplit = partitioner.getImplicitSplit( *tempCS );

  {
    bool enforceQT = implicitSplit == CU_QUAD_SPLIT;

    // LARGE CTU bug
    if( m_pcEncCfg->getUseFastLCTU() )
    {
      unsigned minDepth = 0;
      unsigned maxDepth = floorLog2(tempCS->sps->getCTUSize()) - floorLog2(tempCS->sps->getMinQTSize(slice.getSliceType(), partitioner.chType));

      if( auto ad = dynamic_cast<AdaptiveDepthPartitioner*>( &partitioner ) )
      {
        ad->setMaxMinDepth( minDepth, maxDepth, *tempCS );
      }

      if( minDepth > partitioner.currQtDepth )
      {
        // enforce QT
        enforceQT = true;
      }
    }

    if( !enforceQT )
    {
      m_CABACEstimator->resetBits();

      m_CABACEstimator->split_cu_mode( split, *tempCS, partitioner );
      partitioner.modeType = modeTypeParent;
      m_CABACEstimator->mode_constraint( split, *tempCS, partitioner, modeTypeChild );
      tempCS->fracBits += m_CABACEstimator->getEstFracBits(); // split bits
    }
  }

  tempCS->cost = m_pcRdCost->calcRdCost( tempCS->fracBits, tempCS->dist );

  // Check Delta QP bits for splitted structure
  if( !qgEnableChildren ) // check at deepest QG level only
  {
    xCheckDQP(*tempCS, partitioner, true);
  }

  // If the configuration being tested exceeds the maximum number of bytes for a slice / slice-segment, then
  // a proper RD evaluation cannot be performed. Therefore, termination of the
  // slice/slice-segment must be made prior to this CTU.
  // This can be achieved by forcing the decision to be that of the rpcTempCU.
  // The exception is each slice / slice-segment must have at least one CTU.
  if (bestCS->cost != MAX_DOUBLE)
  {
  }
  else
  {
    bestCS->costDbOffset = 0;
  }
  tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();
  if( tempCS->cus.size() > 0 && modeTypeParent == MODE_TYPE_ALL && modeTypeChild == MODE_TYPE_INTER )
  {
    int areaSizeNoResiCu = 0;
    for( int k = 0; k < tempCS->cus.size(); k++ )
    {
      areaSizeNoResiCu += (tempCS->cus[k]->rootCbf == false) ? tempCS->cus[k]->lumaSize().area() : 0;
    }
    if( areaSizeNoResiCu >= (tempCS->area.lumaSize().area() >> 1) )
    {
      skipInterPass = true;
    }
  }
  // RD check for sub partitioned coding structure.
  xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );

#if GDR_ENABLED
  if (isAffMVInfoSaved)
  {
    m_pcInterSearch->addAffMVInfo(tmpMVInfo, tmpMVInfoSolid);
  }
#else
  if (isAffMVInfoSaved)
  {
    m_pcInterSearch->addAffMVInfo(tmpMVInfo);
  }
#endif

  if (!tempCS->slice->isIntra() && isUniMvInfoSaved)
  {
    m_pcInterSearch->addUniMvInfo(tmpUniMvInfo);
  }

  tempCS->motionLut = oldMotionLut;

  tempCS->prevPLT   = oldPLT;

  tempCS->releaseIntermediateData();

  tempCS->prevQP[partitioner.chType] = oldPrevQp;
}

bool EncCu::xCheckRDCostIntra(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, bool adaptiveColorTrans)
{
  /*
  进行了相应变换块可选变换模式集的遍历，通过并计算RD Cost选出最佳变换模式
  主要分为两个阶段
  1. mts_flag标志被设置为0，并遍历预测模式根据RD Cost选择出最佳预测模式。对于每个预测模式，分别进行DCT-2和DCT-2+LFNST变换。
    1.1 对于DCT-2变换，通过比较DCT-2和TransformSkip的SAD决定使用DCT-2还是TransformSkip
    1.2 对于LFNST变换，通过遍历lfnstIdx为1或者2的情况选出最佳LFNST变换矩阵
  2. mts_flag标志被设置为1，遍历MTS变换集并计算相应的RD Cost。
    2.1 快速算法：通过比较当前MTS变换核的 RD Cost与在第一阶段的存储的最佳DCT-2的RD Cost决定是否进行MTS下一变换核的遍历

  // 亮度块宽高都小于32才会进行MTS
    1. 最大尺寸为64：DCT-2 或DCT-2+LFNST
    2. 最大尺寸小于等于32：DCT-2和TransformerSkip(和DCT-2竞争) 或 DCT-2+LFNST 或 MTS

  // 最大尺寸小于等于32: 
     1. DCT-2, Cb Cr单独编码或jointCbCr联合编码
     2. DCT-2+LFNST, Cb Cr单独编码或jointCbCr联合编码
  */

  // 最佳的帧间预测模式的RDcost
  double          bestInterCost             = m_modeCtrl->getBestInterCost();
  // MTS不开启的RDcost
  double          costSize2Nx2NmtsFirstPass = m_modeCtrl->getMtsSize2Nx2NFirstPassCost();

  // 为true表示只测试trGrp=0的情况
  bool            skipSecondMtsPass         = m_modeCtrl->getSkipSecondMTSPass();
  const SPS&      sps                       = *tempCS->sps;
  const int       maxSizeMTS                = MTS_INTRA_MAX_CU_SIZE;

  // 亮度块宽高都小于32才会进行MTS
  // 最大尺寸为64：DCT-2 或DCT-2+LFNST
  // 最大尺寸为32：DCT-2+TransformerSkip 或 DCT-2+LFNST 或 MTS

  uint8_t         considerMtsSecondPass     = ( sps.getUseIntraMTS() && isLuma( partitioner.chType ) && partitioner.currArea().lwidth() <= maxSizeMTS && partitioner.currArea().lheight() <= maxSizeMTS ) ? 1 : 0;

  bool   useIntraSubPartitions   = false; //ISP模式
  double maxCostAllowedForChroma = MAX_DOUBLE; // 色度最大允许cost
  // 当前CU最佳模式
  const  CodingUnit *bestCU      = bestCS->getCU( partitioner.chType );
  Distortion interHad = m_modeCtrl->getInterHad(); // 帧间HAD


  double dct2Cost                =   MAX_DOUBLE;//不开启MTS且不开启LFNST情况下的RDcost
  double bestNonDCT2Cost         = MAX_DOUBLE;//开启MTS或开启LFNST情况下的最佳RDcost
  // 如果对应的trGrp中有测试的模式成为最佳模式，此为对应的trGrp的最佳RDcost
  double trGrpBestCost     [ 4 ] = { MAX_DOUBLE, MAX_DOUBLE, MAX_DOUBLE, MAX_DOUBLE };
  // 所有模式的如果有测试的模式成为最佳模式，此为最佳RDcost
  double globalBestCost          =   MAX_DOUBLE;

  // 如果对应的trGrp中有测试的模式成为最佳模式，此为true
  bool   bestSelFlag       [ 4 ] = { false, false, false, false };
  // 如果对应的trGrp需要测试，初始化为true，后边在遍历一个grp之后会检查是否测试下一个grp
  bool   trGrpCheck        [ 4 ] = { true, true, true, true };

  // 对应trGrp要测试MTS中的哪些情况由下面两个变量决定
  // 表示多变换核的4个类型。0：DST7_DST7 1：DCT8_DST7 2：DST7_DST8 3：DCT8_DCT8
  int    startMTSIdx       [ 4 ] = { 0, 1, 2, 3 }; 
  int    endMTSIdx         [ 4 ] = { 0, 1, 2, 3 }; //由下面的trGrpIdx来控制

  // 当前trGrp的最佳RDcost与不开启MTS且不开启LFNST情况下的RDcost的比例如果超过阈值，
  // 就会停止测试下一个trGrp。这个阈值就由trGrpStopThreshold决定
  double trGrpStopThreshold[ 3 ] = { 1.001, 1.001, 1.001 }; 

  // 如果有测试的模式成为最佳模式，此为最佳模式的mtsFlag
  int    bestMtsFlag             =   0;
  // 如果有测试的模式成为最佳模式，此为最佳模式的lfnstIdx
  int    bestLfnstIdx            =   0;

  // 如果当前CU可以测试LFNST开启的情况，此为2否则就为0
  // 0表示不使用LFNST，1和2表示LFNST的变换核的选择
  const int  maxLfnstIdx         = ( partitioner.isSepTree( *tempCS ) && partitioner.chType == CHANNEL_TYPE_CHROMA && ( partitioner.currArea().lwidth() < 8 || partitioner.currArea().lheight() < 8 ) )  || ( partitioner.currArea().lwidth() > sps.getMaxTbSize() || partitioner.currArea().lheight() > sps.getMaxTbSize() ) ? 0 : 2;

  bool       skipOtherLfnst      = false;//为true表示跳过测试其余的LFNST的情况

  // 要测试的LFNST中的哪些情况由下面两个变量决定
  int        startLfnstIdx       = 0;
  int        endLfnstIdx         = sps.getUseLFNST() ? maxLfnstIdx : 0; // 0/2

  // 使用LFNST时候，grpNumMax的大小等于MTSIntraMaxCand？？也就是4(LD是3，AI是4)个
  int grpNumMax = sps.getUseLFNST() ? m_pcEncCfg->getMTSIntraMaxCand() : 1;
  m_modeCtrl->setISPWasTested(false);
  m_pcIntraSearch->invalidateBestModeCost();
  if (sps.getUseColorTrans() && !CS::isDualITree(*tempCS))
  {
    if ((m_pcEncCfg->getRGBFormatFlag() && adaptiveColorTrans) || (!m_pcEncCfg->getRGBFormatFlag() && !adaptiveColorTrans))
    {
      m_pcIntraSearch->invalidateBestRdModeFirstColorSpace();
    }
  }
  // cbf是code-block-flag标志，如果YUV中的cbf全部为0，则帧内的核查全部跳过不执行
  bool foundZeroRootCbf = false; //发现RootCbf为0的情况
  if (sps.getUseColorTrans())
  {
    CHECK(tempCS->treeType != TREE_D || partitioner.treeType != TREE_D, "localtree should not be applied when adaptive color transform is enabled");
    CHECK(tempCS->modeType != MODE_TYPE_ALL || partitioner.modeType != MODE_TYPE_ALL, "localtree should not be applied when adaptive color transform is enabled");
    CHECK(adaptiveColorTrans && (CS::isDualITree(*tempCS) || partitioner.chType != CHANNEL_TYPE_LUMA), "adaptive color transform cannot be applied to dual-tree");
  }

  // 这里开始3个for进行变换模式集的遍历，计算RDcost选出最佳变换模式
  // 遍历MTS的四种组合 MTS_DST7_DST7、 MTS_DCT8_DST7、 MTS_DST7_DCT8、MTS_DCT8_DCT8
  // SPS有getUseImplicitMTS来判断隐式MTS，但是不知在哪用了
  for( int trGrpIdx = 0; trGrpIdx < grpNumMax; trGrpIdx++ ) // [0, 1/4)
  {
    // 决定当前trGrp的mtsFlag的变化范围，trGrp的数量一般为4，第一个trGrp会测试mtsFlag为1和0两种情况，
    // 剩余trGrp只会测试mtsFlag为1的情况
    const uint8_t startMtsFlag = trGrpIdx > 0; // trGrpIdx=0时候为0， 其余时候都为1
    // 使用MTS且最大尺寸为32时候为1，否则为0
    const uint8_t endMtsFlag   = sps.getUseLFNST() ? considerMtsSecondPass : 0; // 0/1

    // trGrpIdx == 0是第一轮变换，considerMtsSecondPass是进行二次变换
    if( ( trGrpIdx == 0 || ( !skipSecondMtsPass && considerMtsSecondPass ) ) && trGrpCheck[ trGrpIdx ] )
    {
      // 帧内编码CU，CU级的RD检测以LFNST索引值为循环索引
      // VVC有8x8和4x4两种尺寸的LFNST，由TU大小决定用哪种
      // 每个尺寸有4个变换集，每个变换集包含2个不同transform kernel
      // 变换集由TU的帧内预测模式隐式推断，不是通过for遍历来实现选取的。
      // 变换核由lfnstIdx显式决定，lfnstIdx=0,1,2。lfnstIdx=0表示不开启LFNST,即不使用二次变换
      // 1，2分别选择不同的变换核
      for( int lfnstIdx = startLfnstIdx; lfnstIdx <= endLfnstIdx; lfnstIdx++ ) // [0, 0/2]
      {

        // MTS的选择由第一个for循环和第三个for循环控制。
        // LFNST的选择由第二个for循环控制，控制是否开启以及使用哪个变换核。
        // LFNST变换集由TU的帧内预测模式隐式推断
        // mtsFlag为0和1表示是否开启MTS。1表示开启MTS，0表示不开启。
        // mtsFlag=0时，变换只会采用DCT2+DCT2和变换skip两种。
        // mtsFlag=1时，第一层for循环控制MTS_DST7_DST7、 MTS_DCT8_DST7、 
        // MTS_DST7_DCT8、MTS_DCT8_DCT8变换核的遍历选择
        for( uint8_t mtsFlag = startMtsFlag; mtsFlag <= endMtsFlag; mtsFlag++ )// [0, 1]
        {
          if (sps.getUseColorTrans() && !CS::isDualITree(*tempCS))
          {
            m_pcIntraSearch->setSavedRdModeIdx(trGrpIdx*(NUM_LFNST_NUM_PER_SET * 2) + lfnstIdx * 2 + mtsFlag);
          }
          // MTS是单独对最大尺寸小于等于32的进行变换
          // LFNST是对DCT-2变换后的二次变换，不能同时开启
          // 这段意思就是mts和lfnst不能同时使用
          if (mtsFlag > 0 && lfnstIdx > 0)
          {
            continue;
          }
          //3) if interHad is 0, only try further modes if some intra mode was already better than inter
          // 3) 如果interHad为0，只有当某些intra模式已经优于inter时，才继续尝试其他模式
          if( sps.getUseLFNST() && m_pcEncCfg->getUsePbIntraFast() && !tempCS->slice->isIntra() && bestCU && CU::isInter( *bestCS->getCU( partitioner.chType ) ) && interHad == 0 )
          {
            continue;
          }


          /*--------------设置tempCS和CU-------------------*/
          tempCS->initStructData( encTestMode.qp );

          CodingUnit &cu      = tempCS->addCU( CS::getArea( *tempCS, tempCS->area, partitioner.chType ), partitioner.chType );

          partitioner.setCUData( cu );
          cu.slice            = tempCS->slice;
          cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
          cu.skip             = false;
          cu.mmvdSkip = false;
          cu.predMode         = MODE_INTRA;
          cu.chromaQpAdj      = m_cuChromaQpOffsetIdxPlus1;
          cu.qp               = encTestMode.qp;
          cu.lfnstIdx         = lfnstIdx; // 表示使用的是第几个变换核
          cu.mtsFlag          = mtsFlag;
          cu.ispMode          = NOT_INTRA_SUBPARTITIONS; // 不进行ISP
          cu.colorTransform = adaptiveColorTrans;
          CU::addPUs( cu );
          tempCS->interHad    = interHad;
          // 为true表示更新了最佳模式
          m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;
          /*--------------设置tempCS和CU代码结束-------------------*/


          bool validCandRet = false;
          // 亮度分量帧内预测
          if( isLuma( partitioner.chType ) )
          {
            //ISP uses the value of the best cost so far (luma if it is the fast version) to avoid test non-necessary subpartitions 
            // ISP使用到目前为止的最佳成本值(如果是快速版本，则使用luma)来避免测试不必要的子分区
            double bestCostSoFar = partitioner.isSepTree(*tempCS) ? m_modeCtrl->getBestCostWithoutSplitFlags() : bestCU && bestCU->predMode == MODE_INTRA ? bestCS->lumaCost : bestCS->cost;

            if (partitioner.isSepTree(*tempCS) && encTestMode.maxCostAllowed < bestCostSoFar)
            {
              bestCostSoFar = encTestMode.maxCostAllowed; //目前为止的最佳成本，用来避免不必要的ISP测试
            }

            //亮度分量预测，如果无效则跳过之后的步骤
            validCandRet = m_pcIntraSearch->estIntraPredLumaQT(cu, partitioner, bestCostSoFar, mtsFlag, startMTSIdx[trGrpIdx], endMTSIdx[trGrpIdx], (trGrpIdx > 0), !cu.colorTransform ? bestCS : nullptr);

            // 如果没有找到有效的模式或者ISP模式且cbf=0，则进行下一轮选择
            if ((!validCandRet || (cu.ispMode && cu.firstTU->cbf[COMPONENT_Y] == 0)))
            {
              continue;
            }
            if (m_pcEncCfg->getUseFastISP() && validCandRet && !mtsFlag && !lfnstIdx && !cu.colorTransform)
            {
              m_modeCtrl->setISPMode(cu.ispMode);
              m_modeCtrl->setISPLfnstIdx(cu.lfnstIdx);
              m_modeCtrl->setMIPFlagISPPass(cu.mipFlag);
              m_modeCtrl->setBestISPIntraModeRelCU(cu.ispMode ? PU::getFinalIntraMode(*cu.firstPU, CHANNEL_TYPE_LUMA) : UINT8_MAX);
              m_modeCtrl->setBestDCT2NonISPCostRelCU(m_modeCtrl->getMtsFirstPassNoIspCost());
            }

            if (sps.getUseColorTrans() && m_pcEncCfg->getRGBFormatFlag() && !CS::isDualITree(*tempCS) && !cu.colorTransform)
            {
              double curLumaCost = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);
              if (curLumaCost > bestCS->cost)
              {
                continue;
              }
            }

            // 使用ISP模式标志
            useIntraSubPartitions = cu.ispMode != NOT_INTRA_SUBPARTITIONS;

            if( !partitioner.isSepTree( *tempCS ) )
            {
              // 当前亮度分量的RD cost
              tempCS->lumaCost = m_pcRdCost->calcRdCost( tempCS->fracBits, tempCS->dist );
              if( useIntraSubPartitions )
              {
                //the difference between the best cost so far and the current luma cost is stored to avoid testing the Cr component if the cost of luma + Cb is larger than the best cost
                // 到目前为止的最佳成本和当前的luma成本之间的差值被储存起来，
                // 以避免在luma+ Cb的成本大于最佳成本时测试Cr成分。
                maxCostAllowedForChroma = bestCS->cost < MAX_DOUBLE ? bestCS->cost - tempCS->lumaCost : MAX_DOUBLE;
              }
            }

            // 帧间HAD？？
            if (m_pcEncCfg->getUsePbIntraFast() && tempCS->dist == std::numeric_limits<Distortion>::max()
                && tempCS->interHad == 0)
            {
              interHad = 0;
              // JEM assumes only perfect reconstructions can from now on beat the inter mode
              m_modeCtrl->enforceInterHad( 0 );
              continue;
            }

            // 如果不是双树，需要拷贝亮度的重建值和预测值，后边色度预测使用。
            if( !partitioner.isSepTree( *tempCS ) )
            {
              if (!cu.colorTransform)
              {
                cu.cs->picture->getRecoBuf(cu.Y()).copyFrom(cu.cs->getRecoBuf(COMPONENT_Y));
                cu.cs->picture->getPredBuf(cu.Y()).copyFrom(cu.cs->getPredBuf(COMPONENT_Y));
              }
              else
              {
                cu.cs->picture->getRecoBuf(cu).copyFrom(cu.cs->getRecoBuf(cu));
                cu.cs->picture->getPredBuf(cu).copyFrom(cu.cs->getPredBuf(cu));
              }
            }
          }

          // 色度分量帧内预测
          // I帧是先亮度后色度预测，B帧和P帧是亮度色度同时进行（对于一个CTU来说）
          if( tempCS->area.chromaFormat != CHROMA_400 && ( partitioner.chType == CHANNEL_TYPE_CHROMA || !cu.isSepTree() ) && !cu.colorTransform )
          { 
            TUIntraSubPartitioner subTuPartitioner( partitioner );
            m_pcIntraSearch->estIntraPredChromaQT( cu, ( !useIntraSubPartitions || ( cu.isSepTree() && !isLuma( CHANNEL_TYPE_CHROMA ) ) ) ? partitioner : subTuPartitioner, maxCostAllowedForChroma );
            if( useIntraSubPartitions && !cu.ispMode )
            {
              //At this point the temp cost is larger than the best cost. Therefore, we can already skip the remaining calculations
              // 此时，临时成本大于最佳成本。因此，我们可以跳过其余的计算
              continue;
            }
          }

          cu.rootCbf = false;

          for( uint32_t t = 0; t < getNumberValidTBlocks( *cu.cs->pcv ); t++ )
          {
            cu.rootCbf |= cu.firstTU->cbf[t] != 0;
          }

          if (!cu.rootCbf)
          {
            cu.colorTransform = false;
            foundZeroRootCbf = true;
          }

          // Get total bits for current mode: encode CU
          // 获取当前模式的总比特数:对CU进行编码
          m_CABACEstimator->resetBits();
          // 跳过变换量化标志
          if ((!cu.cs->slice->isIntra() || cu.cs->slice->getSPS()->getIBCFlag())
            && cu.Y().valid()
            )
          {
            m_CABACEstimator->cu_skip_flag ( cu );//设置预测模式
          }
          m_CABACEstimator->pred_mode      ( cu );
          m_CABACEstimator->adaptive_color_transform(cu);
          m_CABACEstimator->cu_pred_data   ( cu );

          // Encode Coefficients
          // 编码系数
          CUCtx cuCtx;
          cuCtx.isDQPCoded = true;
          cuCtx.isChromaQpAdjCoded = true;
          m_CABACEstimator->cu_residual( cu, partitioner, cuCtx );

          // RDO进行筛选
          tempCS->fracBits = m_CABACEstimator->getEstFracBits(); // 编码码率
          tempCS->cost     = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist); // RD cost


          double tmpCostWithoutSplitFlags = tempCS->cost;
          xEncodeDontSplit( *tempCS, partitioner );

          xCheckDQP( *tempCS, partitioner );
          xCheckChromaQPOffset( *tempCS, partitioner );

          // Check if low frequency non-separable transform (LFNST) is too expensive
          // 检查低频不可分变换(LFNST)是否太贵
          if( lfnstIdx && !cuCtx.lfnstLastScanPos && !cu.ispMode )
          {
            bool cbfAtZeroDepth = cu.isSepTree() ?
                                       cu.rootCbf
                                     : (tempCS->area.chromaFormat != CHROMA_400 && std::min( cu.firstTU->blocks[ 1 ].width, cu.firstTU->blocks[ 1 ].height ) < 4) ?
                                            TU::getCbfAtDepth( *cu.firstTU, COMPONENT_Y, 0 )
                                          : cu.rootCbf;
            if( cbfAtZeroDepth )
            {
              tempCS->cost = MAX_DOUBLE;
              tmpCostWithoutSplitFlags = MAX_DOUBLE;
            }
          }

          if (isLuma(partitioner.chType) && cu.firstTU->mtsIdx[COMPONENT_Y] > MTS_SKIP)
          {
            CHECK(!cuCtx.mtsLastScanPos, "MTS is disallowed to only contain DC coefficient");
          }

          // 不开启MTS和LFNST的时候，就只有DCT-2和变换跳过
          if( mtsFlag == 0 && lfnstIdx == 0 )
          {
            dct2Cost = tempCS->cost; // 设置DCT-2的cost
          }
          else if (tmpCostWithoutSplitFlags < bestNonDCT2Cost) 
          { 
            bestNonDCT2Cost = tmpCostWithoutSplitFlags;
          }

          if( tempCS->cost < bestCS->cost )//如果当前Cost小于最佳Cost
          {
            m_modeCtrl->setBestCostWithoutSplitFlags( tmpCostWithoutSplitFlags );
          }

          if (!mtsFlag)
          {
            static_cast<double &>(costSize2Nx2NmtsFirstPass) = tempCS->cost;
          }

          if( sps.getUseLFNST() && !tempCS->cus.empty() )
          {
            skipOtherLfnst = m_modeCtrl->checkSkipOtherLfnst( encTestMode, tempCS, partitioner );
          }

          xCalDebCost( *tempCS, partitioner );
          tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();


#if WCG_EXT
          DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda( true ) );
#else
          DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
#endif
          if (sps.getUseColorTrans() && !CS::isDualITree(*tempCS))
          {
            int colorSpaceIdx = ((m_pcEncCfg->getRGBFormatFlag() && adaptiveColorTrans) || (!m_pcEncCfg->getRGBFormatFlag() && !adaptiveColorTrans)) ? 0 : 1;
            if (tempCS->cost < tempCS->tmpColorSpaceIntraCost[colorSpaceIdx])
            {
              tempCS->tmpColorSpaceIntraCost[colorSpaceIdx] = tempCS->cost;
              bestCS->tmpColorSpaceIntraCost[colorSpaceIdx] = tempCS->cost;
            }
          }

          if( !sps.getUseLFNST() )//SPS层关闭LFNST
          {
            // 检查最佳模式
            xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
          }
          else//SPS层开启LFNST
          {
            if( xCheckBestMode( tempCS, bestCS, partitioner, encTestMode ) )
            {
              trGrpBestCost[ trGrpIdx ] = globalBestCost = bestCS->cost;
              bestSelFlag  [ trGrpIdx ] = true;
              bestMtsFlag               = mtsFlag;
              bestLfnstIdx              = lfnstIdx;
              if( bestCS->cus.size() == 1 )
              {
                CodingUnit &cu = *bestCS->cus.front();
                if (cu.firstTU->mtsIdx[COMPONENT_Y] == MTS_SKIP)
                {
                  if( ( floorLog2( cu.firstTU->blocks[ COMPONENT_Y ].width ) + floorLog2( cu.firstTU->blocks[ COMPONENT_Y ].height ) ) >= 6 )
                  {
                    endLfnstIdx = 0;
                  }
                }
              }
            }

            //we decide to skip the non-DCT-II transforms and LFNST according to the ISP results
            // 根据ISP结果，我们决定跳过非DCT-2变换和LFNST变换
            if ((endMtsFlag > 0 || endLfnstIdx > 0) && (cu.ispMode || (bestCS && bestCS->cus[0]->ispMode)) && tempCS->slice->isIntra() && m_pcEncCfg->getUseFastISP())
            {
              double bestCostDct2NoIsp = m_modeCtrl->getMtsFirstPassNoIspCost();
              double bestIspCost       = m_modeCtrl->getIspCost();
              CHECKD( bestCostDct2NoIsp <= bestIspCost, "wrong cost!" );
              double threshold = 1.4;

              double lfnstThreshold = 1.01 * threshold;

              
              if( m_modeCtrl->getStopNonDCT2Transforms() || bestCostDct2NoIsp > bestIspCost*lfnstThreshold ) //跳过剩下的LFNST
              {
                endLfnstIdx = lfnstIdx;
              }

              // 跳过第二阶段的MTS
              if ( m_modeCtrl->getStopNonDCT2Transforms() || bestCostDct2NoIsp > bestIspCost*threshold )
              {
                skipSecondMtsPass = true;
                m_modeCtrl->setSkipSecondMTSPass( true );
                break;
              }
            }

            //now we check whether the second pass of SIZE_2Nx2N and the whole Intra SIZE_NxN should be skipped or not
            // 现在我们检查是否应该跳过SIZE_2Nx2N的第二遍以及整个内部的SIZE_NxN
            // 当使用DCT-2时，对于P帧和B帧，如果最佳预测模式不是帧内预测模式，如果性能比最佳帧间模式的性能更糟糕则跳过MTS的检查
            if( !mtsFlag && !tempCS->slice->isIntra() && bestCU && bestCU->predMode != MODE_INTRA )
            {
              // 如果“使用DCT2的2Nx2N”比最佳帧间模式更糟糕，则跳过帧内检查
              const double thEmtInterFastSkipIntra = 1.4; // Skip checking Intra if "2Nx2N using DCT2" is worse than best Inter mode
              if( costSize2Nx2NmtsFirstPass > thEmtInterFastSkipIntra * bestInterCost )
              {
                skipSecondMtsPass = true;
                m_modeCtrl->setSkipSecondMTSPass( true );
                break;
              }
            }
          }   // if(sps.getUseLFNST())

        } //for emtCuFlag
        if( skipOtherLfnst ) //跳过其他的lfnst变换
        {
          startLfnstIdx = lfnstIdx;
          endLfnstIdx   = lfnstIdx;
          break;
        }
      } //for lfnstIdx
    } //if (!skipSecondMtsPass && considerMtsSecondPass && trGrpCheck[iGrpIdx])


    // trGrpCheck初始化为true，这里要判断是否要测试下一个trGrp。必须当前trGrp有测试的模式成为最佳模式，
    // 且最佳模式是开启MTS或LFNST，并且当前trGrp的最佳RDcost与不开启MTS且不开启LFNST情况下的RDcost的
    // 比例不超过阈值，才会测试下一个trGrp
    if( sps.getUseLFNST() && trGrpIdx < 3 )
    {
      trGrpCheck[ trGrpIdx + 1 ] = false;
      if( bestSelFlag[ trGrpIdx ] && considerMtsSecondPass )
      {
        // 若最优代价比DCT2变换的代价小，且相差的部分在一定范围内则trGrpCheck[ trGrpIdx + 1 ] = true,
        // 会选择trGrpIdx+1对应的变换:（因为startMTSIdx[ trGrpIdx ]、endMTSIdx[ trGrpIdx
        // ]，对应xRecurIntraCodingLumaQT中的mtsFirstCheckId、mtsLastCheckId）
        double dCostRatio = dct2Cost / trGrpBestCost[ trGrpIdx ];
        trGrpCheck[ trGrpIdx + 1 ] = ( bestMtsFlag != 0 || bestLfnstIdx != 0 ) && dCostRatio < trGrpStopThreshold[ trGrpIdx ];
      }
    }
  } //trGrpIdx

  if(!adaptiveColorTrans)
  {
    m_modeCtrl->setBestNonDCT2Cost(bestNonDCT2Cost);
  }
  return foundZeroRootCbf;
}


void EncCu::xCheckPLT(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  if (((partitioner.currArea().lumaSize().width * partitioner.currArea().lumaSize().height <= 16) && (isLuma(partitioner.chType)) )
        || ((partitioner.currArea().chromaSize().width * partitioner.currArea().chromaSize().height <= 16) && (!isLuma(partitioner.chType)) && partitioner.isSepTree(*tempCS) )
      || (partitioner.isLocalSepTree(*tempCS)  && (!isLuma(partitioner.chType))  )  )
  {
    return;
  }
  tempCS->initStructData(encTestMode.qp);
  CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);
  partitioner.setCUData(cu);
  cu.slice = tempCS->slice;
  cu.tileIdx = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
  cu.skip = false;
  cu.mmvdSkip = false;
  cu.predMode = MODE_PLT;

  cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
  cu.qp = encTestMode.qp;
  cu.bdpcmMode = 0;

  tempCS->addPU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);
  tempCS->addTU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);
  // Search
  tempCS->dist = 0;
  //主要函数为PLTSearch 内部调用derivePLTLossy获取调色板 再reorder 最后按照两种扫描顺序进行游程编码
  if (cu.isSepTree())
  {
    if (isLuma(partitioner.chType))
    {
      m_pcIntraSearch->PLTSearch(*tempCS, partitioner, COMPONENT_Y, 1);
    }
    if (tempCS->area.chromaFormat != CHROMA_400 && (partitioner.chType == CHANNEL_TYPE_CHROMA))
    {
      m_pcIntraSearch->PLTSearch(*tempCS, partitioner, COMPONENT_Cb, 2);
    }
  }
  else
  {
    if( cu.chromaFormat != CHROMA_400 )
    {
      m_pcIntraSearch->PLTSearch(*tempCS, partitioner, COMPONENT_Y, 3);
    }
    else
    {
      m_pcIntraSearch->PLTSearch(*tempCS, partitioner, COMPONENT_Y, 1);
    }
  }


  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  m_CABACEstimator->resetBits();
  if ((!cu.cs->slice->isIntra() || cu.cs->slice->getSPS()->getIBCFlag())
    && cu.Y().valid())
  {
    m_CABACEstimator->cu_skip_flag(cu);
  }
  m_CABACEstimator->pred_mode(cu);

  // signaling
  CUCtx cuCtx;
  cuCtx.isDQPCoded = true;
  cuCtx.isChromaQpAdjCoded = true;
  if (cu.isSepTree())
  {
    if (isLuma(partitioner.chType))
    {
      m_CABACEstimator->cu_palette_info(cu, COMPONENT_Y, 1, cuCtx);
    }
    if (tempCS->area.chromaFormat != CHROMA_400 && (partitioner.chType == CHANNEL_TYPE_CHROMA))
    {
      m_CABACEstimator->cu_palette_info(cu, COMPONENT_Cb, 2, cuCtx);
    }
  }
  else
  {
    if( cu.chromaFormat != CHROMA_400 )
    {
      m_CABACEstimator->cu_palette_info(cu, COMPONENT_Y, 3, cuCtx);
    }
    else
    {
      m_CABACEstimator->cu_palette_info(cu, COMPONENT_Y, 1, cuCtx);
    }
  }
  tempCS->fracBits = m_CABACEstimator->getEstFracBits();
  tempCS->cost = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);

  xEncodeDontSplit(*tempCS, partitioner);
  xCheckDQP(*tempCS, partitioner);
  xCheckChromaQPOffset( *tempCS, partitioner );
  xCalDebCost(*tempCS, partitioner);
  tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();

  const Area currCuArea = cu.block(getFirstComponentOfChannel(partitioner.chType));
  cu.slice->m_mapPltCost[isChroma(partitioner.chType)][currCuArea.pos()][currCuArea.size()] = tempCS->cost;
#if WCG_EXT
  DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda(true));
#else
  DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
#endif
  xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
}

void EncCu::xCheckDQP( CodingStructure& cs, Partitioner& partitioner, bool bKeepCtx )
{
  CHECK( bKeepCtx && cs.cus.size() <= 1 && partitioner.getImplicitSplit( cs ) == CU_DONT_SPLIT, "bKeepCtx should only be set in split case" );
  CHECK( !bKeepCtx && cs.cus.size() > 1, "bKeepCtx should never be set for non-split case" );

  if( !cs.pps->getUseDQP() )
  {
    return;
  }

  if (partitioner.isSepTree(cs) && isChroma(partitioner.chType))
  {
    return;
  }

  if( !partitioner.currQgEnable() ) // do not consider split or leaf/not leaf QG condition (checked by caller)
  {
    return;
  }


  CodingUnit* cuFirst = cs.getCU( partitioner.chType );

  CHECK( !cuFirst, "No CU available" );

  bool hasResidual = false;
  for( const auto &cu : cs.cus )
  {
    //not include the chroma CU because chroma CU is decided based on corresponding luma QP and deltaQP is not signaled at chroma CU
    if( cu->rootCbf && !isChroma( cu->chType ))
    {
      hasResidual = true;
      break;
    }
  }

  int predQP = CU::predictQP( *cuFirst, cs.prevQP[partitioner.chType] );

  if( hasResidual )
  {
    TempCtx ctxTemp( m_CtxCache );
    if (!bKeepCtx)
    {
      ctxTemp = SubCtx(Ctx::DeltaQP, m_CABACEstimator->getCtx());
    }

    m_CABACEstimator->resetBits();
    m_CABACEstimator->cu_qp_delta( *cuFirst, predQP, cuFirst->qp );

    cs.fracBits += m_CABACEstimator->getEstFracBits(); // dQP bits
    cs.cost      = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);

    if (!bKeepCtx)
    {
      m_CABACEstimator->getCtx() = SubCtx(Ctx::DeltaQP, ctxTemp);
    }

    // NOTE: reset QPs for CUs without residuals up to first coded CU
    for( const auto &cu : cs.cus )
    {
      //not include the chroma CU because chroma CU is decided based on corresponding luma QP and deltaQP is not signaled at chroma CU
      if( cu->rootCbf && !isChroma( cu->chType ))
      {
        break;
      }
      cu->qp = predQP;
    }
  }
  else
  {
    // No residuals: reset CU QP to predicted value
    for( const auto &cu : cs.cus )
    {
      cu->qp = predQP;
    }
  }
}

void EncCu::xCheckChromaQPOffset( CodingStructure& cs, Partitioner& partitioner )
{
  // doesn't apply if CU chroma QP offset is disabled
  if( !cs.slice->getUseChromaQpAdj() )
  {
    return;
  }

  // doesn't apply to luma CUs
  if( partitioner.isSepTree(cs) && isLuma(partitioner.chType) )
  {
    return;
  }

  // check cost only at cQG top-level (everything below shall not be influenced by adj coding: it occurs only once)
  if( !partitioner.currQgChromaEnable() )
  {
    return;
  }

  // check if chroma is coded or not
  bool isCoded = false;
  for( auto &cu : cs.cus )
  {
    SizeType channelWidth = !cu->isSepTree() ? cu->lwidth() : cu->chromaSize().width;
    SizeType channelHeight = !cu->isSepTree() ? cu->lheight() : cu->chromaSize().height;

    for( const TransformUnit &tu : CU::traverseTUs(*cu) )
    {
      if( tu.cbf[COMPONENT_Cb] || tu.cbf[COMPONENT_Cr] || channelWidth > 64 || channelHeight > 64)
      {
        isCoded = true;
        break;
      }
    }
    if (isCoded)
    {
      // estimate cost for coding cu_chroma_qp_offset
      TempCtx ctxTempAdjFlag( m_CtxCache );
      TempCtx ctxTempAdjIdc( m_CtxCache );
      ctxTempAdjFlag = SubCtx( Ctx::ChromaQpAdjFlag, m_CABACEstimator->getCtx() );
      ctxTempAdjIdc = SubCtx( Ctx::ChromaQpAdjIdc,   m_CABACEstimator->getCtx() );
      m_CABACEstimator->resetBits();
      m_CABACEstimator->cu_chroma_qp_offset( *cu );
      cs.fracBits += m_CABACEstimator->getEstFracBits();
      cs.cost      = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);
      m_CABACEstimator->getCtx() = SubCtx( Ctx::ChromaQpAdjFlag, ctxTempAdjFlag );
      m_CABACEstimator->getCtx() = SubCtx( Ctx::ChromaQpAdjIdc,  ctxTempAdjIdc  );
      break;
    }
    else
    {
      // chroma QP adj is forced to 0 for leading uncoded CUs
      cu->chromaQpAdj = 0;
    }
  }
}

void EncCu::xCheckRDCostHashInter( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  bool isPerfectMatch = false;

  tempCS->initStructData(encTestMode.qp);
  m_pcInterSearch->resetBufferedUniMotions();
  m_pcInterSearch->setAffineModeSelected(false);
  CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);

  partitioner.setCUData(cu);
  cu.slice = tempCS->slice;
  cu.tileIdx = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
  cu.skip = false;
  cu.predMode = MODE_INTER;
  cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
  cu.qp = encTestMode.qp;
  CU::addPUs(cu);
  cu.mmvdSkip = false;
  cu.firstPU->mmvdMergeFlag = false;
  //进行Hash搜索 内部调用 xHashInterEstimation
  if (m_pcInterSearch->predInterHashSearch(cu, partitioner, isPerfectMatch))
  {
    double equBcwCost = MAX_DOUBLE;

    m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;

    xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 0, 0, &equBcwCost);

    if ( m_bestModeUpdated && bestCS->cost != MAX_DOUBLE )
    {
      xCalDebCost( *bestCS, partitioner );
    }
  }
  tempCS->initStructData(encTestMode.qp);
  int minSize = min(cu.lwidth(), cu.lheight());
  if (minSize < 64)
  {
    isPerfectMatch = false;
  }
  m_modeCtrl->setIsHashPerfectMatch(isPerfectMatch);
}


void EncCu::xCheckRDCostMerge2Nx2N( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  /*
  选出Merge/Skip模式中最佳的候选MV，根据常规Merge模式、带有运动矢量差的Merge模式(MMVD)以及
  CIIP模式(联合帧内帧间模式)产生的预测值和原始值得SATD Cost，选出最佳的运动信息。主要过程：

  1. xCheckRDCostMerge2Nx2N函数，构造Merge模式候选列表，选出Merge/Skip模式中最佳的候选MV，对常规Merge模式、带有运动矢量差的Merge模式(MMVD)以及联合帧内帧间模式(CIIP)计算SAD
  cost，选出最佳的运动信息：

  2. 获取Merge模式候选列表并初始化RD选择的模式信息列表。模式信息包括Merge候选模式的索引，是否是常规Merge模式、MMVD模式、CIIP模式。常规Merge模式列表大小为6，MMVD复用常规Merge列表的0/1索引，分别进行4方向8步长的搜索，共2*8*4=64个。再将候选Merge模式信息加入到RD模式列表。

  3. 如果使用Merge快速算法或者CIIP模式可用，如果最佳模式是非跳过，会缩减进行RD
  Cost细选的模式数(RdModeList列表大小)。非跳过模式，会计算常规Merge、CIIP、MMVD模式的satd
  cost减少RD列表数量，会将代价小的模式移动到列表前边。
     1. 遍历常规Merge候选模式，进行运动补偿获得预测值，并使用DMVR对其MV进行细化，计算SAD
  cost，更新RD模式列表，将代价小的模式移到列表前面
     2. CIIP模式。最多选择常规Merge模式粗选后的RD cost列表的前4个模式，进行MC预测，和亮度分量帧内Planar预测值加权，计算SAD
  cost，更新RD模式列表，将代价小的模式移动到列表前面
     3. 选择merge列表的前两项，进行2\*2\*8共64种MMVD候选模式，进行运动补偿获得预测值，计算SAD
  cost，更新RD模式列表，将代价小的模式移到列表前面
     4. 使用阈值限制SATD列表数量：第i个模式代价 > 第1个模式的代价*MRG_FAST_RATIO(1.25)，则将uiNumMrgSATDCand设置为i
     5. CIIP可用，对选中的最佳CIIP模式进行正式三个分量的intra预测值计算，亮度用planar，色度用DM

     如果最佳模式是MMVD Skip模式，则进行细选的模式数为可用常规Merge模式数+MMVD模式数，否则(如果最佳模式是Skip模式)，进行细选的模式数为可用常规Merge模式

  4. 遍历候选模式列表进行SATD细选。两次迭代，第一次常规Merge模式，第二次Skip模式(即跳过残差编码)。在每次遍历缩减后的RD模式列表，对预测残差进行变换、量化，计算RD
  Cost,选出最佳的merge模式。、
     1. 判断当前候选是三种Merge模式中的哪一种，为相应的模式做预测的准备；
     2.
  对当前候选进行MV的细化操作，MV细化之后再去重新求出当前PU对应Merge模式的预测值。对于CIIP模式，进行三个分量的加权操作；对于MMVD模式，通过MC重新计算MMVD模式预测值；对于regular模式，直接拷贝上面已经计算好的预测值，因为在上面已经对regular模式下的MV细化过了。
     3. 编码当前Merge候选模式的inter预测值的残差，并完成SATD的比较，最终选出SATD最小的那个Merge模式

  5. SATD细选后选出三种Merge中最优的一种，计算出相应的预测值以及残差，最后计算SATD下的RDcost，再去和GEO、Affine之类的模式做SATD的RDcsot的比较
  */
  const Slice &slice = *tempCS->slice;

  CHECK( slice.getSliceType() == I_SLICE, "Merge modes not available for I-slices" );

  tempCS->initStructData( encTestMode.qp );

  MergeCtx mergeCtx; // Merge上下文模型
  const SPS &sps = *tempCS->sps;

#if GDR_ENABLED
  bool isEncodeGdrClean = false;
  CodingStructure *cs;
#endif

  // 启用SbTMVP （如果相邻块使用了同位图作为参考帧 那么将相邻块的MV应用于当前块）
  if (sps.getSbTMVPEnabledFlag())
  {
    Size bufSize = g_miScaling.scale( tempCS->area.lumaSize() );
    mergeCtx.subPuMvpMiBuf    = MotionBuf( m_SubPuMiBuf,    bufSize );
  }

  Mv   refinedMvdL0[MAX_NUM_PARTS_IN_CTU][MRG_MAX_NUM_CANDS];
  setMergeBestSATDCost( MAX_DOUBLE );//设置最佳Merge模式的RD Cost为MAX_DOUBLE

  {
    // first get merge candidates 首先获得Merge候选列表
    CodingUnit cu( tempCS->area );
    cu.cs       = tempCS;
    cu.predMode = MODE_INTER;
    cu.slice    = tempCS->slice;
    cu.tileIdx  = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );

    PredictionUnit pu( tempCS->area );
    pu.cu = &cu;
    pu.cs = tempCS;
    // 获得常规Merge获选列表 最多6个
    PU::getInterMergeCandidates(pu, mergeCtx, 0);
 
    /*
    获得MMVD Merge获选列表
    在skip和merge模式中。
    MMVD是对常规Merge列表的前两个MV进行细化，,每一个初始MV在每个方向上的
    每种步长都会形成一个新的MV，包含起始点、搜索方向、搜索步长，因此一个初始的MV可以扩展出32个新的MV，
    所以两个初始MV共生成64个新的MV，分别对这些新的MV也会通过运动补偿得到当前CU的预测值。

    1. 复用常规Merge模式候选列表，将Merge候选列表的前两个候选MV作为初始MV
    2. 对于第1步得到初始的MV，分别以该初始MV在参考帧中所指向的位置作为起始点，在上下左右四个方向上，
    进行1/4,1/2,1,2,4,8,16,32共8种步长的搜索
    3. 在Skip和Merge模式下，将所有得到的64个新的预测值之间进行率失真代价比较，选出最优的MV，
    在编码时需要编码该MV的三个信息：初始MV在Merge list中的索引值、搜索方向和搜索步长。

    */
    PU::getInterMMVDMergeCandidates(pu, mergeCtx); // 复用Merge模式列表的前两个
    pu.regularMergeFlag = true;//常规Merge的flag，一开始默认当前PU为regular模式
#if GDR_ENABLED
    cs = pu.cs;
    isEncodeGdrClean = cs->sps->getGDREnabledFlag() && cs->pcv->isEncoder && ((cs->picHeader->getInGdrInterval() && cs->isClean(pu.Y().topRight(), CHANNEL_TYPE_LUMA)) || (cs->picHeader->getNumVerVirtualBoundaries() == 0));
#endif
  }
  // 为true表示对应merge候选在非skip下不存在有关残差的语法元素
  bool candHasNoResidual[MRG_MAX_NUM_CANDS + MMVD_ADD_NUM];

  for (uint32_t ui = 0; ui < MRG_MAX_NUM_CANDS + MMVD_ADD_NUM; ui++)
  {
    candHasNoResidual[ui] = false;
  }

  bool        bestIsSkip     = false;//最佳模式是Skip模式
  bool        bestIsMMVDSkip = true;//最佳模式是MMVD Skip模式
  PelUnitBuf  acMergeBuffer[MRG_MAX_NUM_CANDS];//存储常规Merge模式预测像素
  PelUnitBuf  acMergeTmpBuffer[MRG_MAX_NUM_CANDS];//用来存储常规Merge模式预测像素的临时空间
  PelUnitBuf  acMergeRealBuffer[MMVD_MRG_MAX_RD_BUF_NUM];// Merge运动补偿的真实预测值
  PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM];//用来存储MMVD Merge模式的临时空间
  PelUnitBuf *singleMergeTempBuffer;//单Merge列表候选的预测值缓存
  int         insertPos;

  // 进行SATD Cost计算的模式数目，初始化为可用常规Merge数目+64，后面会缩减
  unsigned    uiNumMrgSATDCand = mergeCtx.numValidMergeCand + MMVD_ADD_NUM;

  struct ModeInfo
  {
    uint32_t mergeCand; //merge索引
    bool     isRegularMerge; 
    bool     isMMVD; 
    bool     isCIIP;
    ModeInfo() : mergeCand(0), isRegularMerge(false), isMMVD(false), isCIIP(false) {}
    ModeInfo(const uint32_t mergeCand, const bool isRegularMerge, const bool isMMVD, const bool isCIIP) :
      mergeCand(mergeCand), isRegularMerge(isRegularMerge), isMMVD(isMMVD), isCIIP(isCIIP) {}
  };
  // RDcost的模式列表
  static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  RdModeList;
  bool                                        mrgTempBufSet = false;


  // 常规merge+mmvd 6+64(8*4*2 以两个base mv 在4个方向 8个步长) = 70
  const int candNum = mergeCtx.numValidMergeCand + (tempCS->sps->getUseMMVD() ? std::min<int>(MMVD_BASE_MV_NUM, mergeCtx.numValidMergeCand) * MMVD_MAX_REFINE_NUM : 0);

  // 加入RD模式列表
  for (int i = 0; i < candNum; i++)
  {
    if (i < mergeCtx.numValidMergeCand)
    {
      //先加入普通merge模式
      RdModeList.push_back(ModeInfo(i, true, false, false));
    }
    else
    {//再加入MMVD merge模式
      RdModeList.push_back(ModeInfo(std::min(MMVD_ADD_NUM, i - mergeCtx.numValidMergeCand), false, true, false));
    }
  }

  // 初始化一个和当前CU相同大小的区域
  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
  for (unsigned i = 0; i < MMVD_MRG_MAX_RD_BUF_NUM; i++)
  {
    // 初始化每一个候选的预测值缓存
    acMergeRealBuffer[i] = m_acMergeBuffer[i].getBuf(localUnitArea);
    if (i < MMVD_MRG_MAX_RD_NUM)
    {
      acMergeTempBuffer[i] = acMergeRealBuffer + i;
    }
    else
    {
      singleMergeTempBuffer = acMergeRealBuffer + i;
    }
  }

  bool isIntrainterEnabled = sps.getUseCiip();//是否使用CIIP 组合Intra Inter加权预测
  // 如果块尺寸小于64或宽和高大于最大CU边长（128）时。CIIP模式不可用
  if (bestCS->area.lwidth() * bestCS->area.lheight() < 64 || bestCS->area.lwidth() >= MAX_CU_SIZE || bestCS->area.lheight() >= MAX_CU_SIZE)
  {
    isIntrainterEnabled = false;
  }
  // 记录Merge候选对象是否尝试了Skip模式
  bool isTestSkipMerge[MRG_MAX_NUM_CANDS]; // record if the merge candidate has tried skip mode
  for (uint32_t idx = 0; idx < MRG_MAX_NUM_CANDS; idx++)
  {
    isTestSkipMerge[idx] = false;
  }

  // CIIP模式可用或者使用快速Merge模式，缩减进行RD Cost细选的模式数 
  if( m_pcEncCfg->getUseFastMerge() || isIntrainterEnabled)
  {
    uiNumMrgSATDCand = NUM_MRG_SATD_CAND;//SATD候选模式数：4
    if (isIntrainterEnabled)
    {
      uiNumMrgSATDCand += 1;//增加一个CIIP模式
    }
    bestIsSkip       = false; // 最好的不是跳过

    if( auto blkCache = dynamic_cast< CacheBlkInfoCtrl* >( m_modeCtrl ) )
    {
      if (slice.getSPS()->getIBCFlag())
      {
        ComprCUCtx cuECtx = m_modeCtrl->getComprCUCtx();
        bestIsSkip = blkCache->isSkip(tempCS->area) && cuECtx.bestCU;
      }
      else
      bestIsSkip = blkCache->isSkip( tempCS->area ); //最优的模式是否是Skip
      bestIsMMVDSkip = blkCache->isMMVDSkip(tempCS->area); //最优的模式是否是MMVDSkip
    } 

    if (isIntrainterEnabled) // always perform low complexity check
    {
      bestIsSkip = false; //最优的模式不是SKip
    }

    // 候选的代价列表（最多构造6+64个）
    static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> candCostList;


    // 1. Pass: get SATD-cost for selected candidates and reduce their count
    // 1. 非跳过，获取Merge，CIIP，MMVD模式的SATD-cost,减少RD列表数量
    if( !bestIsSkip ) //如果最佳模式不是Skip模式
    {
      RdModeList.clear();
      mrgTempBufSet       = true;
      const TempCtx ctxStart(m_CtxCache, m_CABACEstimator->getCtx());

      CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );
      const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda( ) * FRAC_BITS_SCALE;
      partitioner.setCUData( cu );
      cu.slice            = tempCS->slice;
      cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
      cu.skip             = false;
      cu.mmvdSkip = false;
      cu.geoFlag          = false; //当前CU不为GEO模式
    //cu.affine
      cu.predMode         = MODE_INTER;
    //cu.LICFlag
      cu.chromaQpAdj      = m_cuChromaQpOffsetIdxPlus1;
      cu.qp               = encTestMode.qp;
    //cu.emtFlag  is set below

      PredictionUnit &pu  = tempCS->addPU( cu, partitioner.chType );

      DistParam distParam;
      // 是否使用SATD
      const bool bUseHadamard = !tempCS->slice->getDisableSATDForRD();
      m_pcRdCost->setDistParam (distParam, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth (CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);

      const UnitArea localUnitArea( tempCS->area.chromaFormat, Area( 0, 0, tempCS->area.Y().width, tempCS->area.Y().height) );

      // 遍历常规可用Merge候选项，计算SAD并更新候选列表
      // 此时的循环是循环regular模式的Merge列表，此时当前CU默认是regular_Merge模式，
      // MMVD以及CIIP都是在Regular_Merge的Merge列表SAD率失真代价比较完之后的基础上再去改进的
      for( uint32_t uiMergeCand = 0; uiMergeCand < mergeCtx.numValidMergeCand; uiMergeCand++ )
      {
        mergeCtx.setMergeInfo( pu, uiMergeCand );

        PU::spanMotionInfo( pu, mergeCtx );
        pu.mvRefine = true; // MV可细化
        distParam.cur = singleMergeTempBuffer->Y();
        acMergeTmpBuffer[uiMergeCand] = m_acMergeTmpBuffer[uiMergeCand].getBuf(localUnitArea);
        //运动补偿存入缓存singleMergeTempBuffer中
        m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, true, &(acMergeTmpBuffer[uiMergeCand]));
        acMergeBuffer[uiMergeCand] = m_acRealMergeBuffer[uiMergeCand].getBuf(localUnitArea);
        acMergeBuffer[uiMergeCand].copyFrom(*singleMergeTempBuffer);

        pu.mvRefine = false;
        // 对于双向的预测候选，直接进行MV的细化（即DMVR）
        if( mergeCtx.interDirNeighbours[uiMergeCand] == 3 && mergeCtx.mrgTypeNeighbours[uiMergeCand] == MRG_TYPE_DEFAULT_N )
        {
          mergeCtx.mvFieldNeighbours[2*uiMergeCand].mv   = pu.mv[0];
          mergeCtx.mvFieldNeighbours[2*uiMergeCand+1].mv = pu.mv[1];
          {
            int dx, dy, i, j, num = 0;
            dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
            dx = std::min<int>(pu.lumaSize().width, DMVR_SUBCU_WIDTH);
            if (PU::checkDMVRCondition(pu))
            {
              for (i = 0; i < (pu.lumaSize().height); i += dy)
              {
                for (j = 0; j < (pu.lumaSize().width); j += dx)
                {
                  refinedMvdL0[num][uiMergeCand] = pu.mvdL0SubPu[num];
                  num++;
                }
              }
            }
          }
        }

        // 计算SAD 的RD cost
        Distortion uiSad = distParam.distFunc(distParam); // SAD
        m_CABACEstimator->getCtx() = ctxStart;
        uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
        double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra; // cost
        insertPos = -1;

#if GDR_ENABLED
        // Non-RD cost for regular merge
        if (isEncodeGdrClean)
        {
          bool isSolid = true;
          bool isValid = true;

          if (mergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0].refIdx >= 0)
          {
            Mv mv = mergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0].mv;
            int ridx = mergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0].refIdx;

            mergeCtx.mvValid[(uiMergeCand << 1) + 0] = cs->isClean(pu.Y().bottomRight(), mv, REF_PIC_LIST_0, ridx);

            isSolid = isSolid && mergeCtx.mvSolid[(uiMergeCand << 1) + 0];
            isValid = isValid && mergeCtx.mvValid[(uiMergeCand << 1) + 0];
          }

          if (mergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1].refIdx >= 0) \
          {
            Mv mv = mergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1].mv;
            int ridx = mergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1].refIdx;

            mergeCtx.mvValid[(uiMergeCand << 1) + 1] = cs->isClean(pu.Y().bottomRight(), mv, REF_PIC_LIST_1, ridx);

            isSolid = isSolid && mergeCtx.mvSolid[(uiMergeCand << 1) + 1];
            isValid = isValid && mergeCtx.mvValid[(uiMergeCand << 1) + 1];
          }

          if (!isValid || !isSolid)
          {
            cost = MAX_DOUBLE;
          }
        }
#endif
        //更新代价列表
        updateCandList(ModeInfo(uiMergeCand, true, false, false), cost, RdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
        //插入成功
        if (insertPos != -1)
        {
          if (insertPos == RdModeList.size() - 1)
          {
            swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
          }
          else
          {
            for (uint32_t i = uint32_t(RdModeList.size()) - 1; i > insertPos; i--)
            {
              swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
            }
            swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
          }
        }
#if !GDR_ENABLED
        CHECK(std::min(uiMergeCand + 1, uiNumMrgSATDCand) != RdModeList.size(), "");
#endif
      }

      // CIIP模式可用， 计算RD cost
      if (isIntrainterEnabled)
      {
        // prepare for Intra bits calculation
        pu.ciipFlag = true;

        // save the to-be-tested merge candidates 保存要测试的CIIP候选模式
        uint32_t CiipMergeCand[NUM_MRG_SATD_CAND]; // 4
        for (uint32_t mergeCnt = 0; mergeCnt < std::min(NUM_MRG_SATD_CAND, (const int)mergeCtx.numValidMergeCand); mergeCnt++)
        {
          CiipMergeCand[mergeCnt] = RdModeList[mergeCnt].mergeCand;
        }

        // 最多从RDCost列表中选出前四个候选进行CIIP的预测。分别进行MV，与亮度分量Planar帧内模式加权
        for (uint32_t mergeCnt = 0; mergeCnt < std::min(std::min(NUM_MRG_SATD_CAND, (const int)mergeCtx.numValidMergeCand), 4); mergeCnt++)
        {
          uint32_t mergeCand = CiipMergeCand[mergeCnt];
          acMergeTmpBuffer[mergeCand] = m_acMergeTmpBuffer[mergeCand].getBuf(localUnitArea);

          // estimate merge bits 估计Merge 的bit数
          mergeCtx.setMergeInfo(pu, mergeCand);

          // first round 帧内使用Planar模式
          pu.intraDir[0] = PLANAR_IDX;
          uint32_t intraCnt = 0;
          // generate intrainter Y prediction  产生帧内帧间联合预测像素
          // 仅在第一个Merge候选模式计算一次Planar预测像素
          if (mergeCnt == 0)
          {
            m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Y());
            m_pcIntraSearch->predIntraAng(COMPONENT_Y, pu.cs->getPredBuf(pu).Y(), pu);
            m_pcIntraSearch->switchBuffer(pu, COMPONENT_Y, pu.cs->getPredBuf(pu).Y(), m_pcIntraSearch->getPredictorPtr2(COMPONENT_Y, intraCnt));
          }
          //加载相应Merge模式产生的帧间预测像素
          pu.cs->getPredBuf(pu).copyFrom(acMergeTmpBuffer[mergeCand]);
          if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
          {
            pu.cs->getPredBuf(pu).Y().rspSignal(m_pcReshape->getFwdLUT());
          }
          // 对帧内帧间进行加权预测 Y分量
          m_pcIntraSearch->geneWeightedPred(COMPONENT_Y, pu.cs->getPredBuf(pu).Y(), pu, m_pcIntraSearch->getPredictorPtr2(COMPONENT_Y, intraCnt));

          // calculate cost
          if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
          {
            pu.cs->getPredBuf(pu).Y().rspSignal(m_pcReshape->getInvLUT());
          }
          distParam.cur = pu.cs->getPredBuf(pu).Y();
          Distortion sadValue = distParam.distFunc(distParam);
          if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
          {
            pu.cs->getPredBuf(pu).Y().rspSignal(m_pcReshape->getFwdLUT());
          }
          m_CABACEstimator->getCtx() = ctxStart;
          pu.regularMergeFlag = false;
          uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
          double cost = (double)sadValue + (double)fracBits * sqrtLambdaForFirstPassIntra;
#if GDR_ENABLED
          // Non-RD cost for CIIP merge
          if (isEncodeGdrClean)
          {
            bool isSolid = true;
            bool isValid = true;

            if (mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 0].refIdx >= 0)
            {
              Mv mv = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 0].mv;
              int ridx = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 0].refIdx;

              mergeCtx.mvValid[(mergeCand << 1) + 0] = cs->isClean(pu.Y().bottomRight(), mv, REF_PIC_LIST_0, ridx);

              isSolid = isSolid && mergeCtx.mvSolid[(mergeCand << 1) + 0];
              isValid = isValid && mergeCtx.mvValid[(mergeCand << 1) + 0];
            }

            if (mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 1].refIdx >= 0)
            {
              Mv mv = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 1].mv;
              int ridx = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 1].refIdx;

              mergeCtx.mvValid[(mergeCand << 1) + 1] = cs->isClean(pu.Y().bottomRight(), mv, REF_PIC_LIST_1, ridx);

              isSolid = isSolid && mergeCtx.mvSolid[(mergeCand << 1) + 1];
              isValid = isValid && mergeCtx.mvValid[(mergeCand << 1) + 1];
            }

            if (!isValid || !isSolid)
            {
              cost = MAX_DOUBLE;
            }
          }
#endif

          insertPos = -1;

          // 更新候选模式列表
          updateCandList(ModeInfo(mergeCand, false, false, true), cost, RdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
          if (insertPos != -1)
          {
            for (int i = int(RdModeList.size()) - 1; i > insertPos; i--)
            {
              swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
            }
            swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
          }
        }
        pu.ciipFlag = false;
      }// if (isIntrainterEnabled) CIIP模式

      // MMVD模式，计算RD cost
      //选出最优的MMVD的Merge候选（初始MV+搜索方向+搜索步长）
      if ( pu.cs->sps->getUseMMVD() )
      {
        cu.mmvdSkip = true;
        pu.regularMergeFlag = true;//普通Merge列表依旧可用，因为初始的MV还是要从普通Merge中获取

        const int tempNum = (mergeCtx.numValidMergeCand > 1) ? MMVD_ADD_NUM : MMVD_ADD_NUM >> 1;
        // 对MMVD候选循环遍历，由于每个初始MV共有4种搜索方向，8种搜索步长，即每个初始MV产生32种细化MV
        for (int mmvdMergeCand = 0; mmvdMergeCand < tempNum; mmvdMergeCand++)
        {
          //初始MV索引，merge列表的0/1（就是两个初始MV的起点，每个初始MV有32种4*8的步长+方向的组合）
          int baseIdx = mmvdMergeCand / MMVD_MAX_REFINE_NUM; // 0/1
          int refineStep = (mmvdMergeCand - (baseIdx * MMVD_MAX_REFINE_NUM)) / 4;//8种步长
          if (refineStep >= m_pcEncCfg->getMmvdDisNum())
          {
            continue;
          }
#if GDR_ENABLED
          if (isEncodeGdrClean)
          {
            pu.mvSolid[REF_PIC_LIST_0] = true;
            pu.mvSolid[REF_PIC_LIST_1] = true;

            pu.mvValid[REF_PIC_LIST_0] = true;
            pu.mvValid[REF_PIC_LIST_1] = true;
          }
#endif

          //设置MMVD候选的信息，得到每个扩展MV具体的搜索初始点以及每个方向以及对应的步长
          // 对每种MV进行各个方向各步长的搜索
          mergeCtx.setMmvdMergeCandiInfo(pu, mmvdMergeCand);

          PU::spanMotionInfo(pu, mergeCtx);
          pu.mvRefine = true;
          distParam.cur = singleMergeTempBuffer->Y();
          pu.mmvdEncOptMode = (refineStep > 2 ? 2 : 1);
          CHECK(!pu.mmvdMergeFlag, "MMVD merge should be set");
          // Don't do chroma MC here
          //MMVD的merge候选运动补偿
          m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, false);
          pu.mmvdEncOptMode = 0;
          pu.mvRefine = false;
          Distortion uiSad = distParam.distFunc(distParam);//计算失真

          m_CABACEstimator->getCtx() = ctxStart;
          uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);//计算比特
          double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;//RDcost
          insertPos = -1;

#if GDR_ENABLED
          if (isEncodeGdrClean)
          {
            bool isSolid = true;
            bool isValid = true;

            if (pu.refIdx[0] >= 0)
            {
              isSolid = isSolid && pu.mvSolid[0];
              isValid = isValid && pu.mvValid[0];
            }

            if (pu.refIdx[1] >= 0)
            {
              isSolid = isSolid && pu.mvSolid[1];
              isValid = isValid && pu.mvValid[1];
            }

            if (!isSolid || !isValid)
            {
              cost = MAX_DOUBLE;
            }
          }
#endif   
          // 更新候选列表
          updateCandList(ModeInfo(mmvdMergeCand, false, true, false), cost, RdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
          if (insertPos != -1)
          {
            for (int i = int(RdModeList.size()) - 1; i > insertPos; i--)
            {
              swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
            }
            swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
          }
        }
      } // MMVD

      // Try to limit number of candidates using SATD-costs
      // 尽量限制使用SATD-Cost的模式数
      // 如果某个模式的Cost大于候选列表中的第一个模式的Cost*1.25，则后面的模式不再进行SATD-Cost计算
      for( uint32_t i = 1; i < uiNumMrgSATDCand; i++ )
      {
        if( candCostList[i] > MRG_FAST_RATIO * candCostList[0] )
        {
          uiNumMrgSATDCand = i;
          break;
        }
      }

      setMergeBestSATDCost( candCostList[0] );//设置Merge模式的最佳SATD Cost
      // CIIP模式可用且存在色度分量
      if (isIntrainterEnabled && isChromaEnabled(pu.cs->pcv->chrFormat))
      {
        pu.ciipFlag = true;
        for (uint32_t mergeCnt = 0; mergeCnt < uiNumMrgSATDCand; mergeCnt++)
        {
          // 色度CIIP planar模式像素
          if (RdModeList[mergeCnt].isCIIP)
          {
            pu.intraDir[0] = PLANAR_IDX;
            pu.intraDir[1] = DM_CHROMA_IDX;
            if (pu.chromaSize().width == 2)
            {
              continue;
            }
            uint32_t bufIdx = 0;
            m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cb());
            m_pcIntraSearch->predIntraAng(COMPONENT_Cb, pu.cs->getPredBuf(pu).Cb(), pu);
            m_pcIntraSearch->switchBuffer(pu, COMPONENT_Cb, pu.cs->getPredBuf(pu).Cb(), m_pcIntraSearch->getPredictorPtr2(COMPONENT_Cb, bufIdx));

            m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cr());
            m_pcIntraSearch->predIntraAng(COMPONENT_Cr, pu.cs->getPredBuf(pu).Cr(), pu);
            m_pcIntraSearch->switchBuffer(pu, COMPONENT_Cr, pu.cs->getPredBuf(pu).Cr(), m_pcIntraSearch->getPredictorPtr2(COMPONENT_Cr, bufIdx));
          }
        }
        pu.ciipFlag = false;
      }

      tempCS->initStructData( encTestMode.qp );
      m_CABACEstimator->getCtx() = ctxStart;
    }
    else
    {
      if (bestIsMMVDSkip)//如果最佳的模式是MMVD Skip模式
      {
        uiNumMrgSATDCand = mergeCtx.numValidMergeCand + ((mergeCtx.numValidMergeCand > 1) ? MMVD_ADD_NUM : MMVD_ADD_NUM >> 1);
      }
      else// 最佳模式是Skip模式
      {
        uiNumMrgSATDCand = mergeCtx.numValidMergeCand;
      }
    }
  }
  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;
  uint32_t iteration;
  uint32_t iterationBegin = 0;
  iteration = 2;

 // 两次迭代，一次是Skip模式跳过残差编码；另一次是常规Merge模式
  for (uint32_t uiNoResidualPass = iterationBegin; uiNoResidualPass < iteration; ++uiNoResidualPass)
  {
    // 遍历SATD选出的Merge列表
    for( uint32_t uiMrgHADIdx = 0; uiMrgHADIdx < uiNumMrgSATDCand; uiMrgHADIdx++ )
    {
      // 当前SATD列表中对应的merge候选模式
      uint32_t uiMergeCand = RdModeList[uiMrgHADIdx].mergeCand;

      // CIIP不支持Skip模式 所以直接跳过
      // intrainter does not support skip mode
      if (uiNoResidualPass != 0 && RdModeList[uiMrgHADIdx].isCIIP) 
      {
        if (isTestSkipMerge[uiMergeCand])
        {
          continue;
        }
      }
      // 0且跳过模式的时候
      if (((uiNoResidualPass != 0) && candHasNoResidual[uiMrgHADIdx])
       || ( (uiNoResidualPass == 0) && bestIsSkip ) )
      {
        continue;
      }

      // first get merge candidates 首先获得Merge候选对象
      CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

      partitioner.setCUData( cu );
      cu.slice            = tempCS->slice;
      cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
      cu.skip             = false;
      cu.mmvdSkip = false;
      cu.geoFlag          = false;
    //cu.affine
      cu.predMode         = MODE_INTER;
    //cu.LICFlag
      cu.chromaQpAdj      = m_cuChromaQpOffsetIdxPlus1;
      cu.qp               = encTestMode.qp;
      PredictionUnit &pu  = tempCS->addPU( cu, partitioner.chType );

      // 是CIIP模式
      if (uiNoResidualPass == 0 && RdModeList[uiMrgHADIdx].isCIIP)
      {
        cu.mmvdSkip = false;
        mergeCtx.setMergeInfo(pu, uiMergeCand);
        pu.ciipFlag = true;
        pu.regularMergeFlag = false;
        pu.intraDir[0] = PLANAR_IDX;
        CHECK(pu.intraDir[0]<0 || pu.intraDir[0]>(NUM_LUMA_MODE - 1), "out of intra mode");
        pu.intraDir[1] = DM_CHROMA_IDX;
      }
      else if (RdModeList[uiMrgHADIdx].isMMVD)//待测模式是MMVD模式
      {
        cu.mmvdSkip = true;
        pu.regularMergeFlag = true;
        mergeCtx.setMmvdMergeCandiInfo(pu, uiMergeCand); //设置MMVD候选信息
      }
      else //是普通的merge
      {
        cu.mmvdSkip = false;
        pu.regularMergeFlag = true;
        mergeCtx.setMergeInfo(pu, uiMergeCand);
      }
      PU::spanMotionInfo( pu, mergeCtx );

      if( m_pcEncCfg->getMCTSEncConstraint() )
      {
        bool isDMVR = PU::checkDMVRCondition( pu );
        if( ( isDMVR && MCTSHelper::isRefBlockAtRestrictedTileBoundary( pu ) ) || ( !isDMVR && !( MCTSHelper::checkMvBufferForMCTSConstraint( pu ) ) ) )
        {
          // Do not use this mode
          tempCS->initStructData( encTestMode.qp );
          continue;
        }
      }

      // 设置Merge预测的临时Buffer，进行DMVR
      if( mrgTempBufSet )
      {
        {
          int dx, dy, i, j, num = 0;
          dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
          dx = std::min<int>(pu.lumaSize().width, DMVR_SUBCU_WIDTH);
          if (PU::checkDMVRCondition(pu))//DMVR条件（解码端MV精度自适应 不是很明白）
          {
            for (i = 0; i < (pu.lumaSize().height); i += dy)
            {
              for (j = 0; j < (pu.lumaSize().width); j += dx)
              {
                pu.mvdL0SubPu[num] = refinedMvdL0[num][uiMergeCand];
                num++;
              }
            }
          }
        }

        // CIIP
        if (pu.ciipFlag)
        {
          uint32_t bufIdx = 0;
          PelBuf tmpBuf = tempCS->getPredBuf(pu).Y();//将Y预测值存储到tmpBuf中
          tmpBuf.copyFrom(acMergeTmpBuffer[uiMergeCand].Y());
          if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
          {
            tmpBuf.rspSignal(m_pcReshape->getFwdLUT());
          }
          // 帧内帧间加权 Y分量
          m_pcIntraSearch->geneWeightedPred(COMPONENT_Y, tmpBuf, pu, m_pcIntraSearch->getPredictorPtr2(COMPONENT_Y, bufIdx));
          // 色度的加权
          if (isChromaEnabled(pu.chromaFormat))
          {
            if (pu.chromaSize().width > 2)
            {
              tmpBuf = tempCS->getPredBuf(pu).Cb();
              tmpBuf.copyFrom(acMergeTmpBuffer[uiMergeCand].Cb());
              m_pcIntraSearch->geneWeightedPred(COMPONENT_Cb, tmpBuf, pu,
                                                m_pcIntraSearch->getPredictorPtr2(COMPONENT_Cb, bufIdx));
              tmpBuf = tempCS->getPredBuf(pu).Cr();
              tmpBuf.copyFrom(acMergeTmpBuffer[uiMergeCand].Cr());
              m_pcIntraSearch->geneWeightedPred(COMPONENT_Cr, tmpBuf, pu,
                                                m_pcIntraSearch->getPredictorPtr2(COMPONENT_Cr, bufIdx));
            }
            else
            {
              tmpBuf = tempCS->getPredBuf(pu).Cb();
              tmpBuf.copyFrom(acMergeTmpBuffer[uiMergeCand].Cb());
              tmpBuf = tempCS->getPredBuf(pu).Cr();
              tmpBuf.copyFrom(acMergeTmpBuffer[uiMergeCand].Cr());
            }
          }
        }
        else//MMVD 普通merge
        {
          if (RdModeList[uiMrgHADIdx].isMMVD)//MMVD
          {
            pu.mmvdEncOptMode = 0;
            m_pcInterSearch->motionCompensation(pu);//运动补偿
          }
          // CIIP不支持Skip模式
          else if (uiNoResidualPass != 0 && RdModeList[uiMrgHADIdx].isCIIP)
          {
            tempCS->getPredBuf().copyFrom(acMergeBuffer[uiMergeCand]);
          }
          else // 常规
          {
            tempCS->getPredBuf().copyFrom(*acMergeTempBuffer[uiMrgHADIdx]);
          }
        }
      }//  if (mrgTempBufSet)
      else
      {
        pu.mvRefine = true;
        m_pcInterSearch->motionCompensation( pu );//运动补偿
        pu.mvRefine = false;
      }
      if (!cu.mmvdSkip && !pu.ciipFlag && uiNoResidualPass != 0)
      {
        CHECK(uiMergeCand >= mergeCtx.numValidMergeCand, "out of normal merge");
        isTestSkipMerge[uiMergeCand] = true;//对该Merge模式检查了Skip模式
      }

#if GDR_ENABLED
      if (isEncodeGdrClean)
      {
        bool isSolid = true;
        bool isValid = true;

        if (pu.refIdx[0] >= 0)
        {
          isSolid = isSolid && pu.mvSolid[0];
          isValid = isValid && pu.mvValid[0];
        }

        if (pu.refIdx[1] >= 0)
        {
          isSolid = isSolid && pu.mvSolid[1];
          isValid = isValid && pu.mvValid[1];
        }

        if (isSolid && isValid)
        {
          xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass, uiNoResidualPass == 0 ? &candHasNoResidual[uiMrgHADIdx] : NULL);
        }
      }
      else
      {
        xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass, uiNoResidualPass == 0 ? &candHasNoResidual[uiMrgHADIdx] : NULL);
      }
#else

      // 编码残差
      xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass, uiNoResidualPass == 0 ? &candHasNoResidual[uiMrgHADIdx] : NULL );
#endif

      if( m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip && !pu.ciipFlag)
      {
        bestIsSkip = !bestCS->cus.empty() && bestCS->getCU( partitioner.chType )->rootCbf == 0;
      }
      tempCS->initStructData( encTestMode.qp );
    }// end loop uiMrgHADIdx
    
     //第一次遍历结果 用于提前终止
    if( uiNoResidualPass == 0 && m_pcEncCfg->getUseEarlySkipDetection() )
    {
      const CodingUnit     &bestCU = *bestCS->getCU( partitioner.chType );
      const PredictionUnit &bestPU = *bestCS->getPU( partitioner.chType );

      if( bestCU.rootCbf == 0 )
      {
        if( bestPU.mergeFlag )
        {
          m_modeCtrl->setEarlySkipDetected();
        }
        else if( m_pcEncCfg->getMotionEstimationSearchMethod() != MESEARCH_SELECTIVE )
        {
          int absolute_MV = 0;

          for( uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            if( slice.getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
            {
              absolute_MV += bestPU.mvd[uiRefListIdx].getAbsHor() + bestPU.mvd[uiRefListIdx].getAbsVer();
            }
          }

          if( absolute_MV == 0 )
          {
            m_modeCtrl->setEarlySkipDetected();
          }
        }
      }
    }
  }
  //SATD细选后选出三种Merge中最优的一种，计算出相应的预测值以及残差，最后计算SATD下的RDcost，
  // 再去和GEO、Affine之类的模式做SATD的RDcsot的比较
  if ( m_bestModeUpdated && bestCS->cost != MAX_DOUBLE )
  {
    xCalDebCost( *bestCS, partitioner );
  }
}

void EncCu::xCheckRDCostMergeGeo2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode)
{

  const Slice &slice = *tempCS->slice;
  CHECK(slice.getSliceType() == I_SLICE, "Merge modes not available for I-slices");

  tempCS->initStructData(encTestMode.qp);

  MergeCtx mergeCtx;
  const SPS &sps = *tempCS->sps;

  if (sps.getSbTMVPEnabledFlag())
  {
    Size bufSize = g_miScaling.scale(tempCS->area.lumaSize());
    mergeCtx.subPuMvpMiBuf = MotionBuf(m_SubPuMiBuf, bufSize);
  }

  CodingUnit &cu = tempCS->addCU(tempCS->area, pm.chType);
  pm.setCUData(cu);
  cu.predMode = MODE_INTER;
  cu.slice = tempCS->slice;
  cu.tileIdx = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
  cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
  cu.qp = encTestMode.qp;
  cu.affine = false;
  cu.mtsFlag = false;
  cu.BcwIdx = BCW_DEFAULT;
  cu.geoFlag = true;
  cu.imv = 0;
  cu.mmvdSkip = false;
  cu.skip = false;
  cu.mipFlag = false;
  cu.bdpcmMode = 0;

  PredictionUnit &pu = tempCS->addPU(cu, pm.chType);
#if GDR_ENABLED
  CodingStructure &cs = *pu.cs;
  const bool isEncodeGdrClean = cs.sps->getGDREnabledFlag() && cs.pcv->isEncoder && ((cs.picHeader->getInGdrInterval() && cs.isClean(pu.Y().topRight(), CHANNEL_TYPE_LUMA)) || (cs.picHeader->getNumVerVirtualBoundaries() == 0));
#endif

  pu.mergeFlag = true;
  pu.regularMergeFlag = false;

  PU::getGeoMergeCandidates(pu, mergeCtx);//获得GEO Merge候选列表

  GeoComboCostList comboList;
  int bitsCandTB = floorLog2(GEO_NUM_PARTITION_MODE);
  PelUnitBuf geoBuffer[MRG_MAX_NUM_CANDS];
  PelUnitBuf geoTempBuf[MRG_MAX_NUM_CANDS];
  PelUnitBuf geoCombinations[GEO_MAX_TRY_WEIGHTED_SAD];
  DistParam distParam;

  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
  const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda();
  uint8_t maxNumMergeCandidates = cu.cs->sps->getMaxNumGeoCand();
  DistParam distParamWholeBlk;
  m_pcRdCost->setDistParam(distParamWholeBlk, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y().buf, m_acMergeBuffer[0].Y().stride, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
  Distortion bestWholeBlkSad = MAX_UINT64;
  double bestWholeBlkCost = MAX_DOUBLE;
  Distortion sadWholeBlk[GEO_MAX_NUM_UNI_CANDS];
  int        pocMrg[GEO_MAX_NUM_UNI_CANDS];
  Mv         MrgMv[GEO_MAX_NUM_UNI_CANDS];
  bool       isSkipThisCand[GEO_MAX_NUM_UNI_CANDS];
#if GDR_ENABLED
  bool MrgSolid[GEO_MAX_NUM_UNI_CANDS];
  bool MrgValid[GEO_MAX_NUM_UNI_CANDS];

  if (isEncodeGdrClean)
  {
    for (int i = 0; i < maxNumMergeCandidates; i++)
    {
      MrgSolid[i] = true;
      MrgValid[i] = true;
    }
  }
#endif
  for (int i = 0; i < maxNumMergeCandidates; i++)
  {
    isSkipThisCand[i] = false;
  }
  //使用mereg list中mv进行运动补偿 
  for (uint8_t mergeCand = 0; mergeCand < maxNumMergeCandidates; mergeCand++)
  {
    geoBuffer[mergeCand] = m_acMergeBuffer[mergeCand].getBuf(localUnitArea);
    mergeCtx.setMergeInfo(pu, mergeCand);
    int MrgList = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 0].refIdx == -1 ? 1 : 0;
    RefPicList MrgeRefPicList = (MrgList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
    int MrgrefIdx = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + MrgList].refIdx;
    pocMrg[mergeCand] = tempCS->slice->getRefPic(MrgeRefPicList, MrgrefIdx)->getPOC();
    MrgMv[mergeCand] = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + MrgList].mv;
#if GDR_ENABLED
    if (isEncodeGdrClean)
    {
      MrgSolid[mergeCand] = mergeCtx.mvSolid[(mergeCand << 1) + MrgList];
      MrgValid[mergeCand] = mergeCtx.mvValid[(mergeCand << 1) + MrgList];
    }
#endif
    if (mergeCand)
    {
      //如果和前面已经检测过的存在重复 那么跳过
      for (int i = 0; i < mergeCand; i++)
      {
        if (pocMrg[mergeCand] == pocMrg[i] && MrgMv[mergeCand] == MrgMv[i])
        {
          isSkipThisCand[mergeCand] = true;
          break;
        }
      }
    }
    PU::spanMotionInfo(pu, mergeCtx);
    if (m_pcEncCfg->getMCTSEncConstraint() && (!(MCTSHelper::checkMvBufferForMCTSConstraint(pu))))
    {
      tempCS->initStructData(encTestMode.qp);
      return;
    }
    m_pcInterSearch->motionCompensation(pu, geoBuffer[mergeCand]);//运动补偿
    //保存merge List中MV的运动补偿结果
    geoTempBuf[mergeCand] = m_acMergeTmpBuffer[mergeCand].getBuf(localUnitArea);
    geoTempBuf[mergeCand].Y().copyFrom(geoBuffer[mergeCand].Y());
    geoTempBuf[mergeCand].Y().roundToOutputBitdepth(geoTempBuf[mergeCand].Y(), cu.slice->clpRng(COMPONENT_Y));
    distParamWholeBlk.cur.buf = geoTempBuf[mergeCand].Y().buf;
    distParamWholeBlk.cur.stride = geoTempBuf[mergeCand].Y().stride;
    sadWholeBlk[mergeCand] = distParamWholeBlk.distFunc(distParamWholeBlk);//计算失真
#if GDR_ENABLED
    bool allOk = (sadWholeBlk[mergeCand] < bestWholeBlkSad);
    if (isEncodeGdrClean)
    {
      bool isSolid = mergeCtx.mvSolid[(mergeCand << 1) + MrgList];
      bool isValid = mergeCtx.mvValid[(mergeCand << 1) + MrgList];
      allOk = allOk && isSolid && isValid;
    }
#endif
#if GDR_ENABLED
    if (allOk)
#else
    if (sadWholeBlk[mergeCand] < bestWholeBlkSad)
#endif
    {
      bestWholeBlkSad = sadWholeBlk[mergeCand];
      int bitsCand = mergeCand + 1;
      bestWholeBlkCost = (double)bestWholeBlkSad + (double)bitsCand * sqrtLambdaForFirstPass;
    }
  }
  bool isGeo = true;
  for (uint8_t mergeCand = 1; mergeCand < maxNumMergeCandidates; mergeCand++)
  {
    isGeo &= isSkipThisCand[mergeCand];
  }
  if (isGeo)
  {
    return;
  }

  int wIdx = floorLog2(cu.lwidth()) - GEO_MIN_CU_LOG2;
  int hIdx = floorLog2(cu.lheight()) - GEO_MIN_CU_LOG2;
  //遍历64种模式 先求出各个划分下 使用不同merge MV的预测结果
  for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
  {
    int maskStride = 0, maskStride2 = 0;
    int stepX = 1;
    Pel* SADmask;
    int16_t angle = g_GeoParams[splitDir][0];
    //推导出SADmask 用于融合划分边界处的像素
    if (g_angle2mirror[angle] == 2)
    {
      maskStride = -GEO_WEIGHT_MASK_SIZE;
      maskStride2 = -(int)cu.lwidth();
      SADmask = &g_globalGeoEncSADmask[g_angle2mask[g_GeoParams[splitDir][0]]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
    }
    else if (g_angle2mirror[angle] == 1)
    {
      stepX = -1;
      maskStride2 = cu.lwidth();
      maskStride = GEO_WEIGHT_MASK_SIZE;
      SADmask = &g_globalGeoEncSADmask[g_angle2mask[g_GeoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
    }
    else
    {
      maskStride = GEO_WEIGHT_MASK_SIZE;
      maskStride2 = -(int)cu.lwidth();
      SADmask = &g_globalGeoEncSADmask[g_angle2mask[g_GeoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
    }
    Distortion sadSmall = 0, sadLarge = 0;
    //对于一种划分方式 遍历mergelisi
    for (uint8_t mergeCand = 0; mergeCand < maxNumMergeCandidates; mergeCand++)
    {
      int bitsCand = mergeCand + 1;
#if GDR_ENABLED
      if (isEncodeGdrClean)
      {
        double cost0, cost1;

        m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), geoTempBuf[mergeCand].Y().buf, geoTempBuf[mergeCand].Y().stride, SADmask, maskStride, stepX, maskStride2, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
        sadLarge = distParam.distFunc(distParam);
        sadSmall = sadWholeBlk[mergeCand] - sadLarge;

        if (MrgSolid[mergeCand] && MrgValid[mergeCand])
        {
          cost0 = (double)sadLarge + (double)bitsCand * sqrtLambdaForFirstPass;
          cost1 = (double)sadSmall + (double)bitsCand * sqrtLambdaForFirstPass;
        }
        else
        {
          cost0 = MAX_DOUBLE;
          cost1 = MAX_DOUBLE;
        }

        m_GeoCostList.insert(splitDir, 0, mergeCand, cost0);
        m_GeoCostList.insert(splitDir, 1, mergeCand, cost1);
      }
      else
      {
        m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), geoTempBuf[mergeCand].Y().buf, geoTempBuf[mergeCand].Y().stride, SADmask, maskStride, stepX, maskStride2, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
        sadLarge = distParam.distFunc(distParam);
        m_GeoCostList.insert(splitDir, 0, mergeCand, (double)sadLarge + (double)bitsCand * sqrtLambdaForFirstPass);
        sadSmall = sadWholeBlk[mergeCand] - sadLarge;
        m_GeoCostList.insert(splitDir, 1, mergeCand, (double)sadSmall + (double)bitsCand * sqrtLambdaForFirstPass);
      }
#else
      //使用之前保存的mergeList MV运动补偿结果 以及SADMASK求出该种划分情况下的结果
      m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), geoTempBuf[mergeCand].Y().buf, geoTempBuf[mergeCand].Y().stride, SADmask, maskStride, stepX, maskStride2, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
      sadLarge = distParam.distFunc(distParam);
      //m_GeoCostList维护一个三维数组 singleDistList[partIdx][geoIdx][mergeIdx]=cost
      m_GeoCostList.insert(splitDir, 0, mergeCand, (double)sadLarge + (double)bitsCand * sqrtLambdaForFirstPass);
      sadSmall = sadWholeBlk[mergeCand] - sadLarge;
      m_GeoCostList.insert(splitDir, 1, mergeCand, (double)sadSmall + (double)bitsCand * sqrtLambdaForFirstPass);
#endif
    }
  }
  //遍历64种划分方式 开始组合
  for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
  {
    //将两部分的cost组合 形成最终的cost
    for (int GeoMotionIdx = 0; GeoMotionIdx < maxNumMergeCandidates * (maxNumMergeCandidates - 1); GeoMotionIdx++)
    {
      unsigned int mergeCand0 = m_GeoModeTest[GeoMotionIdx].m_candIdx0;
      unsigned int mergeCand1 = m_GeoModeTest[GeoMotionIdx].m_candIdx1;
      double tempCost = m_GeoCostList.singleDistList[0][splitDir][mergeCand0].cost + m_GeoCostList.singleDistList[1][splitDir][mergeCand1].cost;
      if (tempCost > bestWholeBlkCost)
      {
        continue;
      }
      tempCost = tempCost + (double)bitsCandTB * sqrtLambdaForFirstPass;
#if GDR_ENABLED
      if (isEncodeGdrClean)
      {
        int idx0 = mergeCand0;
        int idx1 = mergeCand1;
        bool isSolid0 = mergeCtx.mvSolid[(idx0 << 1) + 0] && mergeCtx.mvSolid[(idx0 << 1) + 1];
        bool isSolid1 = mergeCtx.mvSolid[(idx1 << 1) + 0] && mergeCtx.mvSolid[(idx1 << 1) + 1];
        bool isValid0 = mergeCtx.mvValid[(idx0 << 1) + 0] && mergeCtx.mvValid[(idx0 << 1) + 1];
        bool isValid1 = mergeCtx.mvValid[(idx1 << 1) + 0] && mergeCtx.mvValid[(idx1 << 1) + 1];

        if (!isSolid0 || !isSolid1 || !isValid0 || !isValid1)
        {
          tempCost = MAX_DOUBLE;
        }
      }
#endif
      comboList.list.push_back(GeoMergeCombo(splitDir, mergeCand0, mergeCand1, tempCost));
    }
  }
  if (comboList.list.empty())
  {
    return;
  }
  comboList.sortByCost();//排序
  bool geocandHasNoResidual[GEO_MAX_TRY_WEIGHTED_SAD];
  for (int mergeCand = 0; mergeCand < GEO_MAX_TRY_WEIGHTED_SAD; mergeCand++)
  {
    geocandHasNoResidual[mergeCand] = false;
  }
  bool bestIsSkip = false;
  int geoNumCobo = (int)comboList.list.size();
  static_vector<uint8_t, GEO_MAX_TRY_WEIGHTED_SAD> geoRdModeList;
  static_vector<double, GEO_MAX_TRY_WEIGHTED_SAD> geocandCostList;

  DistParam distParamSAD2;
  const bool useHadamard = !tempCS->slice->getDisableSATDForRD();
  m_pcRdCost->setDistParam(distParamSAD2, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, useHadamard);
  int geoNumMrgSATDCand = min(GEO_MAX_TRY_WEIGHTED_SATD, geoNumCobo);
  //对上面获得的列表 中的两个部分组合 RDO
  for (uint8_t candidateIdx = 0; candidateIdx < min(geoNumCobo, GEO_MAX_TRY_WEIGHTED_SAD); candidateIdx++)
  {
    int splitDir = comboList.list[candidateIdx].splitDir;
    int mergeCand0 = comboList.list[candidateIdx].mergeIdx0;
    int mergeCand1 = comboList.list[candidateIdx].mergeIdx1;

    geoCombinations[candidateIdx] = m_acGeoWeightedBuffer[candidateIdx].getBuf(localUnitArea);
    m_pcInterSearch->weightedGeoBlk(pu, splitDir, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx], geoBuffer[mergeCand0], geoBuffer[mergeCand1]);
    distParamSAD2.cur = geoCombinations[candidateIdx].Y();
    Distortion sad = distParamSAD2.distFunc(distParamSAD2);
    int mvBits = 2;
    mergeCand1 -= mergeCand1 < mergeCand0 ? 0 : 1;
    mvBits += mergeCand0;
    mvBits += mergeCand1;
    double updateCost = (double)sad + (double)(bitsCandTB + mvBits) * sqrtLambdaForFirstPass;
#if GDR_ENABLED
    if (isEncodeGdrClean)
    {
      int idx0 = mergeCand0;
      int idx1 = mergeCand1;
      bool isSolid0 = mergeCtx.mvSolid[(idx0 << 1) + 0] && mergeCtx.mvSolid[(idx0 << 1) + 1];
      bool isSolid1 = mergeCtx.mvSolid[(idx1 << 1) + 0] && mergeCtx.mvSolid[(idx1 << 1) + 1];
      bool isValid0 = mergeCtx.mvValid[(idx0 << 1) + 0] && mergeCtx.mvValid[(idx0 << 1) + 1];
      bool isValid1 = mergeCtx.mvValid[(idx1 << 1) + 0] && mergeCtx.mvValid[(idx1 << 1) + 1];

      if (!isSolid0 || !isSolid1 || !isValid0 || !isValid1)
      {
        updateCost = MAX_DOUBLE;
      }
    }
#endif
    comboList.list[candidateIdx].cost = updateCost;
    updateCandList(candidateIdx, updateCost, geoRdModeList, geocandCostList, geoNumMrgSATDCand);
  }
  //跳过一些不如之前的
  for (uint8_t i = 0; i < geoNumMrgSATDCand; i++)
  {
    if (geocandCostList[i] > MRG_FAST_RATIO * geocandCostList[0] || geocandCostList[i] > getMergeBestSATDCost() || geocandCostList[i] > getAFFBestSATDCost())
    {
      geoNumMrgSATDCand = i;
      break;
    }
  }
  for (uint8_t i = 0; i < geoNumMrgSATDCand && isChromaEnabled(pu.chromaFormat); i++)
  {
    uint8_t candidateIdx = geoRdModeList[i];
    int splitDir = comboList.list[candidateIdx].splitDir;
    int mergeCand0 = comboList.list[candidateIdx].mergeIdx0;
    int mergeCand1 = comboList.list[candidateIdx].mergeIdx1;
    geoCombinations[candidateIdx] = m_acGeoWeightedBuffer[candidateIdx].getBuf(localUnitArea);
    m_pcInterSearch->weightedGeoBlk(pu, splitDir, CHANNEL_TYPE_CHROMA, geoCombinations[candidateIdx], geoBuffer[mergeCand0], geoBuffer[mergeCand1]);
  }

  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;
  tempCS->initStructData(encTestMode.qp);
  uint8_t iteration;
  uint8_t iterationBegin = 0;
  iteration = 2;
  //第一轮 有残差的merge 第二轮 skip
  for (uint8_t noResidualPass = iterationBegin; noResidualPass < iteration; ++noResidualPass)
  {
    for (uint8_t mrgHADIdx = 0; mrgHADIdx < geoNumMrgSATDCand; mrgHADIdx++)
    {
      uint8_t candidateIdx = geoRdModeList[mrgHADIdx];
      if (((noResidualPass != 0) && geocandHasNoResidual[candidateIdx])
        || ((noResidualPass == 0) && bestIsSkip))
      {
        continue;
      }
      CodingUnit &cu = tempCS->addCU(tempCS->area, pm.chType);
      pm.setCUData(cu);
      cu.predMode = MODE_INTER;
      cu.slice = tempCS->slice;
      cu.tileIdx = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
      cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
      cu.qp = encTestMode.qp;
      cu.affine = false;
      cu.mtsFlag = false;
      cu.BcwIdx = BCW_DEFAULT;
      cu.geoFlag = true;
      cu.imv = 0;
      cu.mmvdSkip = false;
      cu.skip = false;
      cu.mipFlag = false;
      cu.bdpcmMode = 0;
      PredictionUnit &pu = tempCS->addPU(cu, pm.chType);
      pu.mergeFlag = true;
      pu.regularMergeFlag = false;
      pu.geoSplitDir = comboList.list[candidateIdx].splitDir;
      pu.geoMergeIdx0 = comboList.list[candidateIdx].mergeIdx0;
      pu.geoMergeIdx1 = comboList.list[candidateIdx].mergeIdx1;
      pu.mmvdMergeFlag = false;
      pu.mmvdMergeIdx = MAX_UINT;

      PU::spanGeoMotionInfo(pu, mergeCtx, pu.geoSplitDir, pu.geoMergeIdx0, pu.geoMergeIdx1);
      tempCS->getPredBuf().copyFrom(geoCombinations[candidateIdx]);

#if GDR_ENABLED
      if (isEncodeGdrClean)
      {
        int idx0 = pu.geoMergeIdx0;
        int idx1 = pu.geoMergeIdx1;
        bool isSolid0 = mergeCtx.mvSolid[(idx0 << 1) + 0] && mergeCtx.mvSolid[(idx0 << 1) + 1];
        bool isSolid1 = mergeCtx.mvSolid[(idx1 << 1) + 0] && mergeCtx.mvSolid[(idx1 << 1) + 1];
        bool isValid0 = mergeCtx.mvValid[(idx0 << 1) + 0] && mergeCtx.mvValid[(idx0 << 1) + 1];
        bool isValid1 = mergeCtx.mvValid[(idx1 << 1) + 0] && mergeCtx.mvValid[(idx1 << 1) + 1];

        if (isSolid0 && isSolid1 && isValid0 && isValid1)
        {
          xEncodeInterResidual(tempCS, bestCS, pm, encTestMode, noResidualPass, (noResidualPass == 0 ? &geocandHasNoResidual[candidateIdx] : NULL));
        }
      }
      else
      {
        xEncodeInterResidual(tempCS, bestCS, pm, encTestMode, noResidualPass, (noResidualPass == 0 ? &geocandHasNoResidual[candidateIdx] : NULL));
      }
#else
      xEncodeInterResidual(tempCS, bestCS, pm, encTestMode, noResidualPass, (noResidualPass == 0 ? &geocandHasNoResidual[candidateIdx] : NULL));
#endif

      if (m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip)
      {
        bestIsSkip = bestCS->getCU(pm.chType)->rootCbf == 0;
      }
      tempCS->initStructData(encTestMode.qp);
    }
  }
  if (m_bestModeUpdated && bestCS->cost != MAX_DOUBLE)
  {
    xCalDebCost(*bestCS, pm);
  }
}

void EncCu::xCheckRDCostAffineMerge2Nx2N( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  /*
  1. 获取Affine Merge候选列表
  2. 若开启快速Merge模式，根据当前CU的编码信息缩减进行细选的Merge模式数目
  2.1
  若当前CU最佳模式不是Skip模式，则先遍历可用候选Merge模式，进行运动补偿计算预测值，然后根据SAD和Merge模式比特数更新RD代价列表，如果第i个模式代价
  > 第一个模式的代价*MRG_FAST_RATIO(1.25)，则将细选模式数设置为i 2.2 否则，进行细选的模式数为可用的Merge模式数
  3. 进行细选，细选过程和xCheckRDCostMerge2Nx2N的细选过程类似

  */
  if( m_modeCtrl->getFastDeltaQp() )
  {
    return;
  }
  // 块大小都要大于等于8
  if ( bestCS->area.lumaSize().width < 8 || bestCS->area.lumaSize().height < 8 )
  {
    return;
  }
#if GDR_ENABLED
  CodingStructure *cs;
  bool isEncodeGdrClean = false;
#endif
  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;
  const Slice &slice = *tempCS->slice;

  CHECK( slice.getSliceType() == I_SLICE, "Affine Merge modes not available for I-slices" );

  tempCS->initStructData( encTestMode.qp );
  // AffineMerge模式候选列表，长度为5，最大存储5组3控制点的MV
  AffineMergeCtx affineMergeCtx;//Affine Merge上下文
  const SPS &sps = *tempCS->sps;
  if (sps.getMaxNumAffineMergeCand() == 0)
  {
    return;
  }

  setAFFBestSATDCost(MAX_DOUBLE);

  MergeCtx mrgCtx;
  if (sps.getSbTMVPEnabledFlag())
  {
    Size bufSize = g_miScaling.scale( tempCS->area.lumaSize() );
    mrgCtx.subPuMvpMiBuf = MotionBuf( m_SubPuMiBuf, bufSize );
    // merge候选列表，主要记录ATMVP模式的一些信息，包含在AffineMerge候选列表中
    affineMergeCtx.mrgCtx = &mrgCtx; 
  }

  {
    // first get merge candidates 首先获得Merge候选模式
    CodingUnit cu( tempCS->area );
    cu.cs = tempCS;
    cu.predMode = MODE_INTER;
    cu.slice = tempCS->slice;
    cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
    cu.mmvdSkip = false;

    PredictionUnit pu( tempCS->area );
    pu.cu = &cu;
    pu.cs = tempCS;
    pu.regularMergeFlag = false;
#if GDR_ENABLED
    cs = pu.cs;
    isEncodeGdrClean = cs->sps->getGDREnabledFlag() && cs->pcv->isEncoder && ((cs->picHeader->getInGdrInterval() && cs->isClean(pu.Y().topRight(), CHANNEL_TYPE_LUMA)) || (cs->picHeader->getNumVerVirtualBoundaries() == 0));
#endif
    // Merge候选列表构建，列表大小为5
    PU::getAffineMergeCand( pu, affineMergeCtx );

    if ( affineMergeCtx.numValidMergeCand <= 0 )
    {
      return;
    }
  }

  bool candHasNoResidual[AFFINE_MRG_MAX_NUM_CANDS]; // 4
  for ( uint32_t ui = 0; ui < affineMergeCtx.numValidMergeCand; ui++ )
  {
    candHasNoResidual[ui] = false;
  }

  bool                                        bestIsSkip = false;
  uint32_t                                    uiNumMrgSATDCand = affineMergeCtx.numValidMergeCand;
  // 记录5组Affine Merge候选分别经MC得到的pred像素
  PelUnitBuf                                  acMergeBuffer[AFFINE_MRG_MAX_NUM_CANDS];
  static_vector<uint32_t, AFFINE_MRG_MAX_NUM_CANDS>  RdModeList;
  bool                                        mrgTempBufSet = false;

  // 5种affine merge索引压栈
  for ( uint32_t i = 0; i < AFFINE_MRG_MAX_NUM_CANDS; i++ )
  {
    RdModeList.push_back( i );
  }

  // Merge快速算法决定有多少模式进行SATD细选
  if ( m_pcEncCfg->getUseFastMerge() )
  {
    // 最小为2
    uiNumMrgSATDCand = std::min( NUM_AFF_MRG_SATD_CAND, affineMergeCtx.numValidMergeCand );
    bestIsSkip = false;

    if ( auto blkCache = dynamic_cast<CacheBlkInfoCtrl*>(m_modeCtrl) )
    {
      bestIsSkip = blkCache->isSkip( tempCS->area );
    }

    static_vector<double, AFFINE_MRG_MAX_NUM_CANDS> candCostList;

    // 1. Pass: get SATD-cost for selected candidates and reduce their count
    // 使用SAD缩减Merge进行SATD cost的模式数
    if ( !bestIsSkip )
    {
      RdModeList.clear();
      mrgTempBufSet = true;
      const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda( );

      CodingUnit &cu = tempCS->addCU( tempCS->area, partitioner.chType );

      partitioner.setCUData( cu );
      cu.slice = tempCS->slice;
      cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
      cu.skip = false;
      cu.affine = true;
      cu.predMode = MODE_INTER;
      cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
      cu.qp = encTestMode.qp;

      PredictionUnit &pu = tempCS->addPU( cu, partitioner.chType );

      DistParam distParam;
      // 是否使用SATD
      const bool bUseHadamard = !tempCS->slice->getDisableSATDForRD();
      m_pcRdCost->setDistParam( distParam, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth( CHANNEL_TYPE_LUMA ), COMPONENT_Y, bUseHadamard );

      const UnitArea localUnitArea( tempCS->area.chromaFormat, Area( 0, 0, tempCS->area.Y().width, tempCS->area.Y().height ) );

      // 遍历merge列表的5种进行SAD粗选，选出用来进行SATD细选的模式
      for ( uint32_t uiMergeCand = 0; uiMergeCand < affineMergeCtx.numValidMergeCand; uiMergeCand++ )
      {
        acMergeBuffer[uiMergeCand] = m_acMergeBuffer[uiMergeCand].getBuf( localUnitArea );

        // set merge information

        pu.interDir = affineMergeCtx.interDirNeighbours[uiMergeCand];
        pu.mergeFlag = true; // 
        pu.regularMergeFlag = false;
        pu.mergeIdx = uiMergeCand; // 在affine merge模式列表的索引
        cu.affineType = affineMergeCtx.affineType[uiMergeCand]; // 4/6参数模型
        cu.BcwIdx = affineMergeCtx.BcwIdx[uiMergeCand];

        pu.mergeType = affineMergeCtx.mergeType[uiMergeCand];
        if ( pu.mergeType == MRG_TYPE_SUBPU_ATMVP ) //  sbTMVP
        {
          pu.refIdx[0] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0][0].refIdx;
          pu.refIdx[1] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1][0].refIdx;
          // 构造AffineMerge候选列表时，ATMVP提供的候选，就记录在mrgCtx中
          PU::spanMotionInfo( pu, mrgCtx );
        }
        else
        {
          PU::setAllAffineMvField( pu, affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0], REF_PIC_LIST_0 );
          PU::setAllAffineMvField( pu, affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1], REF_PIC_LIST_1 );

          PU::spanMotionInfo( pu );
        }

#if GDR_ENABLED
        if (isEncodeGdrClean)
        {
          Mv zero = Mv(0, 0);
          bool isValid = cs->isSubPuClean(pu, &zero);
          affineMergeCtx.mvValid[(uiMergeCand << 1) + 0][0] = isValid;
          affineMergeCtx.mvValid[(uiMergeCand << 1) + 0][1] = isValid;
          affineMergeCtx.mvValid[(uiMergeCand << 1) + 0][2] = isValid;
          affineMergeCtx.mvValid[(uiMergeCand << 1) + 1][0] = isValid;
          affineMergeCtx.mvValid[(uiMergeCand << 1) + 1][1] = isValid;
          affineMergeCtx.mvValid[(uiMergeCand << 1) + 1][2] = isValid;
        }
#endif
        distParam.cur = acMergeBuffer[uiMergeCand].Y();

        // 运动补偿，由控制点运动信息获得当前块的预测像素pred
        m_pcInterSearch->motionCompensation( pu, acMergeBuffer[uiMergeCand], REF_PIC_LIST_X, true, false );

        Distortion uiSad = distParam.distFunc( distParam );
        uint32_t   uiBitsCand = uiMergeCand + 1;
        if ( uiMergeCand == tempCS->picHeader->getMaxNumAffineMergeCand() - 1 )
        {
          uiBitsCand--;
        }
        double cost = (double)uiSad + (double)uiBitsCand * sqrtLambdaForFirstPass;
#if GDR_ENABLED
        if (isEncodeGdrClean)
        {
          bool isSolid0 = affineMergeCtx.mvSolid[(uiMergeCand << 1) + 0][0] && affineMergeCtx.mvSolid[(uiMergeCand << 1) + 0][1] && affineMergeCtx.mvSolid[(uiMergeCand << 1) + 0][2];
          bool isSolid1 = affineMergeCtx.mvSolid[(uiMergeCand << 1) + 1][0] && affineMergeCtx.mvSolid[(uiMergeCand << 1) + 1][1] && affineMergeCtx.mvSolid[(uiMergeCand << 1) + 1][2];
          bool isValid0 = affineMergeCtx.mvValid[(uiMergeCand << 1) + 0][0] && affineMergeCtx.mvValid[(uiMergeCand << 1) + 0][1] && affineMergeCtx.mvValid[(uiMergeCand << 1) + 0][2];
          bool isValid1 = affineMergeCtx.mvValid[(uiMergeCand << 1) + 1][0] && affineMergeCtx.mvValid[(uiMergeCand << 1) + 1][1] && affineMergeCtx.mvValid[(uiMergeCand << 1) + 1][2];

          bool isSolid = isSolid0 && isSolid1;
          bool isValid = isValid0 && isValid1;

          if (!isSolid || !isValid)
          {
            cost = MAX_DOUBLE;
          }
        }
#endif
        // 根据SAD cost更新模式列表
        updateCandList( uiMergeCand, cost, RdModeList, candCostList
          , uiNumMrgSATDCand );

        CHECK( std::min( uiMergeCand + 1, uiNumMrgSATDCand ) != RdModeList.size(), "" );
      }

      // Try to limit number of candidates using SATD-costs
      //SATD比之前的最好结果的1.25倍还大 说明这个失真较多 直接去除 不进行RDO优化
      for ( uint32_t i = 1; i < uiNumMrgSATDCand; i++ )
      {
        if ( candCostList[i] > MRG_FAST_RATIO * candCostList[0] )
        {
          uiNumMrgSATDCand = i;
          break;
        }
      }

      tempCS->initStructData( encTestMode.qp );
      setAFFBestSATDCost(candCostList[0]); // 设置Affine Merge的最佳失真

    }
    else
    {
      uiNumMrgSATDCand = affineMergeCtx.numValidMergeCand;
    }
  }

  uint32_t iteration;
  uint32_t iterationBegin = 0;
  iteration = 2;

  // 两遍SATD细选，第一遍Affine merge 第二遍skip
  for (uint32_t uiNoResidualPass = iterationBegin; uiNoResidualPass < iteration; ++uiNoResidualPass)
  {
    for ( uint32_t uiMrgHADIdx = 0; uiMrgHADIdx < uiNumMrgSATDCand; uiMrgHADIdx++ )
    {
      // RD列表的索引
      uint32_t uiMergeCand = RdModeList[uiMrgHADIdx];

      if ( ((uiNoResidualPass != 0) && candHasNoResidual[uiMergeCand])
        || ((uiNoResidualPass == 0) && bestIsSkip) )
      {
        continue;
      }

      // first get merge candidates
      CodingUnit &cu = tempCS->addCU( tempCS->area, partitioner.chType );

      partitioner.setCUData( cu );
      cu.slice = tempCS->slice;
      cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
      cu.skip = false;
      cu.affine = true;
      cu.predMode = MODE_INTER;
      cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
      cu.qp = encTestMode.qp;
      PredictionUnit &pu = tempCS->addPU( cu, partitioner.chType );

      // set merge information
      pu.mergeFlag = true;
      pu.mergeIdx = uiMergeCand;
      pu.interDir = affineMergeCtx.interDirNeighbours[uiMergeCand];
      cu.affineType = affineMergeCtx.affineType[uiMergeCand];
      cu.BcwIdx = affineMergeCtx.BcwIdx[uiMergeCand];

      pu.mergeType = affineMergeCtx.mergeType[uiMergeCand];
      if ( pu.mergeType == MRG_TYPE_SUBPU_ATMVP )
      {
        pu.refIdx[0] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0][0].refIdx;
        pu.refIdx[1] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1][0].refIdx;
        PU::spanMotionInfo( pu, mrgCtx );
      }
      else
      {
        PU::setAllAffineMvField( pu, affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0], REF_PIC_LIST_0 );
        PU::setAllAffineMvField( pu, affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1], REF_PIC_LIST_1 );

        PU::spanMotionInfo( pu );
      }

      if( m_pcEncCfg->getMCTSEncConstraint() && ( !( MCTSHelper::checkMvBufferForMCTSConstraint( *cu.firstPU ) ) ) )
      {
        // Do not use this mode
        tempCS->initStructData( encTestMode.qp );
        return;
      }

      // 非skip时为true。因为非skip模式已经处理过并得到pred像素，无需MC
      if ( mrgTempBufSet )
      {
        tempCS->getPredBuf().copyFrom(acMergeBuffer[uiMergeCand], true, false);   // Copy Luma Only
        m_pcInterSearch->motionCompensation(pu, REF_PIC_LIST_X, false, true);
      }
      else
      {
        m_pcInterSearch->motionCompensation( pu );
      }

#if GDR_ENABLED
      bool isSolid = true;
      bool isValid = true;

      if (isEncodeGdrClean)
      {
        if (bestIsSkip)
        {
          Mv zero = Mv(0, 0);
          bool isValid = cs->isSubPuClean(pu, &zero);
          affineMergeCtx.mvValid[(uiMergeCand << 1) + 0][0] = isValid;
          affineMergeCtx.mvValid[(uiMergeCand << 1) + 0][1] = isValid;
          affineMergeCtx.mvValid[(uiMergeCand << 1) + 0][2] = isValid;
          affineMergeCtx.mvValid[(uiMergeCand << 1) + 1][0] = isValid;
          affineMergeCtx.mvValid[(uiMergeCand << 1) + 1][1] = isValid;
          affineMergeCtx.mvValid[(uiMergeCand << 1) + 1][2] = isValid;
        }

        bool isSolid0 = affineMergeCtx.mvSolid[(uiMergeCand << 1) + 0][0] && affineMergeCtx.mvSolid[(uiMergeCand << 1) + 0][1] && affineMergeCtx.mvSolid[(uiMergeCand << 1) + 0][2];
        bool isSolid1 = affineMergeCtx.mvSolid[(uiMergeCand << 1) + 1][0] && affineMergeCtx.mvSolid[(uiMergeCand << 1) + 1][1] && affineMergeCtx.mvSolid[(uiMergeCand << 1) + 1][2];
        bool isValid0 = affineMergeCtx.mvValid[(uiMergeCand << 1) + 0][0] && affineMergeCtx.mvValid[(uiMergeCand << 1) + 0][1] && affineMergeCtx.mvValid[(uiMergeCand << 1) + 0][2];
        bool isValid1 = affineMergeCtx.mvValid[(uiMergeCand << 1) + 1][0] && affineMergeCtx.mvValid[(uiMergeCand << 1) + 1][1] && affineMergeCtx.mvValid[(uiMergeCand << 1) + 1][2];

        isSolid = isSolid0 && isSolid1;
        isValid = isValid0 && isValid1;

        if (isSolid && isValid)
        {
          xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass, (uiNoResidualPass == 0 ? &candHasNoResidual[uiMergeCand] : NULL));
        }
      }
      else
      {
        xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass, (uiNoResidualPass == 0 ? &candHasNoResidual[uiMergeCand] : NULL));
      }
#else
      // 残差进行变换、量化变化，计算编码比特
      xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass, ( uiNoResidualPass == 0 ? &candHasNoResidual[uiMergeCand] : NULL ) );
#endif

      if ( m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip )
      {
#if GDR_ENABLED
        if (bestCS->getCU(partitioner.chType))
        {
          bestIsSkip = bestCS->getCU(partitioner.chType)->rootCbf == 0;
        }
        else
        {
          bestIsSkip = false;
        }
#else
        bestIsSkip = bestCS->getCU(partitioner.chType)->rootCbf == 0;
#endif
      }
      tempCS->initStructData( encTestMode.qp );
    }// end loop uiMrgHADIdx

    if ( uiNoResidualPass == 0 && m_pcEncCfg->getUseEarlySkipDetection() )
    {
      const CodingUnit     &bestCU = *bestCS->getCU( partitioner.chType );
      const PredictionUnit &bestPU = *bestCS->getPU( partitioner.chType );

      if ( bestCU.rootCbf == 0 )
      {
        if ( bestPU.mergeFlag )
        {
          m_modeCtrl->setEarlySkipDetected();
        }
        else if ( m_pcEncCfg->getMotionEstimationSearchMethod() != MESEARCH_SELECTIVE )
        {
          int absolute_MV = 0;

          for ( uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            if ( slice.getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
            {
              absolute_MV += bestPU.mvd[uiRefListIdx].getAbsHor() + bestPU.mvd[uiRefListIdx].getAbsVer();
            }
          }

          if ( absolute_MV == 0 )
          {
            m_modeCtrl->setEarlySkipDetected();
          }
        }
      }
    }
  }
  if ( m_bestModeUpdated && bestCS->cost != MAX_DOUBLE )
  {
    xCalDebCost( *bestCS, partitioner );
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////
// ibc merge/skip mode check
void EncCu::xCheckRDCostIBCModeMerge2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  assert(partitioner.chType != CHANNEL_TYPE_CHROMA); // chroma IBC is derived
  if (tempCS->area.lwidth() == 128 || tempCS->area.lheight() == 128) // disable IBC mode larger than 64x64
  {
    return;
  }
  const SPS &sps = *tempCS->sps;

  tempCS->initStructData(encTestMode.qp);
  MergeCtx mergeCtx;

  if (sps.getSbTMVPEnabledFlag())
  {
    Size bufSize = g_miScaling.scale(tempCS->area.lumaSize());
    mergeCtx.subPuMvpMiBuf = MotionBuf(m_SubPuMiBuf, bufSize);
  }

#if GDR_ENABLED
  bool gdrClean = true;
#endif
  {
    // first get merge candidates
    CodingUnit cu(tempCS->area);
    cu.cs = tempCS;
    cu.predMode = MODE_IBC;
    cu.slice = tempCS->slice;
    cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
    PredictionUnit pu(tempCS->area);
    pu.cu = &cu;
    pu.cs = tempCS;
    cu.mmvdSkip = false;
    pu.mmvdMergeFlag = false;
    pu.regularMergeFlag = false;
    cu.geoFlag = false;
    PU::getIBCMergeCandidates(pu, mergeCtx);//获取IBCmerge列表 左 上 历史 0
#if GDR_ENABLED
    gdrClean = tempCS->isClean(pu.Y().topRight(), CHANNEL_TYPE_LUMA) || tempCS->picHeader->getNumVerVirtualBoundaries() == 0;
#endif
  }
#if GDR_ENABLED
  const bool isEncodeGdrClean = tempCS->sps->getGDREnabledFlag() && tempCS->pcv->isEncoder && tempCS->picHeader->getInGdrInterval() && gdrClean;
  bool *MrgSolid = nullptr;
  bool *MrgValid = nullptr;
#endif

  int candHasNoResidual[MRG_MAX_NUM_CANDS];
  for (unsigned int ui = 0; ui < mergeCtx.numValidMergeCand; ui++)
  {
    candHasNoResidual[ui] = 0;
  }

#if GDR_ENABLED
  if (isEncodeGdrClean)
  {
    MrgSolid = new bool[MRG_MAX_NUM_CANDS];
    MrgValid = new bool[MRG_MAX_NUM_CANDS];
    for (int i = 0; i < MRG_MAX_NUM_CANDS; i++)
    {
      MrgSolid[i] = false;
      MrgValid[i] = false;
    }
  }
#endif

  bool                                        bestIsSkip = false;
  unsigned                                    numMrgSATDCand = mergeCtx.numValidMergeCand;
  static_vector<unsigned, MRG_MAX_NUM_CANDS>  RdModeList(MRG_MAX_NUM_CANDS);
  for (unsigned i = 0; i < MRG_MAX_NUM_CANDS; i++)
  {
    RdModeList[i] = i;
  }

  static_vector<double, MRG_MAX_NUM_CANDS> candCostList(MRG_MAX_NUM_CANDS, MAX_DOUBLE);
  // 1. Pass: get SATD-cost for selected candidates and reduce their count
  {
    const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda();

    CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, (const ChannelType) partitioner.chType),
                                   (const ChannelType) partitioner.chType);

    partitioner.setCUData(cu);
    cu.slice       = tempCS->slice;
    cu.tileIdx     = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
    cu.skip        = false;
    cu.predMode    = MODE_IBC;
    cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
    cu.qp          = encTestMode.qp;
    cu.mmvdSkip    = false;
    cu.geoFlag     = false;
    DistParam       distParam;
    const bool      bUseHadamard = !cu.slice->getDisableSATDForRD();
    PredictionUnit &pu           = tempCS->addPU(cu, partitioner.chType);   // tempCS->addPU(cu);
    pu.mmvdMergeFlag             = false;
    pu.regularMergeFlag          = false;
    Picture *     refPic         = pu.cu->slice->getPic();
    const CPelBuf refBuf         = refPic->getRecoBuf(pu.blocks[COMPONENT_Y]);
    const Pel *   piRefSrch      = refBuf.buf;
    //LMCS相关的映射处理
    if (tempCS->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
    {
      const CompArea &area = cu.blocks[COMPONENT_Y];
      CompArea        tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
      PelBuf          tmpLuma = m_tmpStorageLCU->getBuf(tmpArea);
      tmpLuma.copyFrom(tempCS->getOrgBuf().Y());
      tmpLuma.rspSignal(m_pcReshape->getFwdLUT());
      m_pcRdCost->setDistParam(distParam, tmpLuma, refBuf, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y,
                               bUseHadamard);
    }
    else
    {
      m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), refBuf, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
    }
    int            refStride = refBuf.stride;
    const UnitArea localUnitArea(tempCS->area.chromaFormat,
                                 Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
    int            numValidBv = mergeCtx.numValidMergeCand;
    //遍历merge列表
    for (unsigned int mergeCand = 0; mergeCand < mergeCtx.numValidMergeCand; mergeCand++)
    {
      mergeCtx.setMergeInfo(pu, mergeCand);   // set bv info in merge mode
      const int          cuPelX    = pu.Y().x;
      const int          cuPelY    = pu.Y().y;
      int                roiWidth  = pu.lwidth();
      int                roiHeight = pu.lheight();
      const int          picWidth  = pu.cs->slice->getPPS()->getPicWidthInLumaSamples();
      const int          picHeight = pu.cs->slice->getPPS()->getPicHeightInLumaSamples();
      const unsigned int lcuWidth  = pu.cs->slice->getSPS()->getMaxCUWidth();
      int                xPred     = pu.bv.getHor();
      int                yPred     = pu.bv.getVer();
      //检测是否合法BV
      if (!m_pcInterSearch->searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, xPred, yPred,
                                     lcuWidth))   // not valid bv derived
      {
        numValidBv--;
        continue;
      }
      PU::spanMotionInfo(pu, mergeCtx);

      distParam.cur.buf = piRefSrch + refStride * yPred + xPred;

      Distortion   sad      = distParam.distFunc(distParam);
      unsigned int bitsCand = mergeCand + 1;
      if (mergeCand == tempCS->sps->getMaxNumMergeCand() - 1)
      {
        bitsCand--;
      }
      double cost = (double) sad + (double) bitsCand * sqrtLambdaForFirstPass;
#if GDR_ENABLED
      if (isEncodeGdrClean)
      {
        bool isSolid = true;
        bool isValid = true;

        if (mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 0].refIdx >= 0)
        {
          Mv mv = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 0].mv;
          int ridx = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 0].refIdx;

          mergeCtx.mvValid[(mergeCand << 1) + 0] = tempCS->isClean(pu.Y().bottomRight(), mv, REF_PIC_LIST_0, ridx, true);

          isSolid = isSolid && mergeCtx.mvSolid[(mergeCand << 1) + 0];
          isValid = isValid && mergeCtx.mvValid[(mergeCand << 1) + 0];
        }

        if (mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 1].refIdx >= 0)
        {
          Mv mv = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 1].mv;
          int ridx = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 1].refIdx;

          mergeCtx.mvValid[(mergeCand << 1) + 1] = tempCS->isClean(pu.Y().bottomRight(), mv, REF_PIC_LIST_1, ridx, true);

          isSolid = isSolid && mergeCtx.mvSolid[(mergeCand << 1) + 1];
          isValid = isValid && mergeCtx.mvValid[(mergeCand << 1) + 1];
        }

        if (!isValid || !isSolid)
        {
          cost = MAX_DOUBLE;
          numValidBv--;
        }
      }
#endif
      updateCandList(mergeCand, cost, RdModeList, candCostList, numMrgSATDCand);
    }

    // Try to limit number of candidates using SATD-costs
    //远大于已经有的 除去
    if (numValidBv)
    {
      numMrgSATDCand = numValidBv;
      for (unsigned int i = 1; i < numValidBv; i++)
      {
        if (candCostList[i] > MRG_FAST_RATIO * candCostList[0])
        {
          numMrgSATDCand = i;
          break;
        }
      }
    }
    else
    {
      //merge 列表为0
      tempCS->dist         = 0;
      tempCS->fracBits     = 0;
      tempCS->cost         = MAX_DOUBLE;
      tempCS->costDbOffset = 0;
      tempCS->initStructData(encTestMode.qp);
      return;
    }

    tempCS->initStructData(encTestMode.qp);
  }

  const unsigned int iteration = 2;
  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;
  // 2. Pass: check candidates using full RD test
  //两轮 第一轮为merge 第二轮为skip
  for (unsigned int numResidualPass = 0; numResidualPass < iteration; numResidualPass++)
  {
    for (unsigned int mrgHADIdx = 0; mrgHADIdx < numMrgSATDCand; mrgHADIdx++)
    {
      unsigned int mergeCand = RdModeList[mrgHADIdx];
      if (!(numResidualPass == 1 && candHasNoResidual[mergeCand] == 1))
      {
        if (!(bestIsSkip && (numResidualPass == 0)))
        {
          {
            // first get merge candidates
            CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, (const ChannelType)partitioner.chType), (const ChannelType)partitioner.chType);

            partitioner.setCUData(cu);
            cu.slice = tempCS->slice;
            cu.tileIdx = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
            cu.skip = false;
            cu.predMode = MODE_IBC;
            cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
            cu.qp = encTestMode.qp;
            cu.sbtInfo = 0;

            PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType);// tempCS->addPU(cu);
            pu.intraDir[0] = DC_IDX; // set intra pred for ibc block
            pu.intraDir[1] = PLANAR_IDX; // set intra pred for ibc block
            cu.mmvdSkip = false;
            pu.mmvdMergeFlag = false;
            pu.regularMergeFlag = false;
            cu.geoFlag = false;
            mergeCtx.setMergeInfo(pu, mergeCand);
            PU::spanMotionInfo(pu, mergeCtx);

            assert(mergeCtx.mrgTypeNeighbours[mergeCand] == MRG_TYPE_IBC); //  should be IBC candidate at this round
            const bool chroma = !pu.cu->isSepTree();
#if GDR_ENABLED
            // redo validation again for Skip
            {
              CodingStructure &cs = *pu.cs;

              if (isEncodeGdrClean)
              {
                if (mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 0].refIdx >= 0)
                {
                  Mv mv = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 0].mv;
                  int ridx = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 0].refIdx;

                  mergeCtx.mvValid[(mergeCand << 1) + 0] = cs.isClean(pu.Y().bottomRight(), mv, REF_PIC_LIST_0, ridx, true);
                }

                if (mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 1].refIdx >= 0)
                {
                  Mv mv = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 1].mv;
                  int ridx = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 1].refIdx;
                  mergeCtx.mvValid[(mergeCand << 1) + 1] = cs.isClean(pu.Y().bottomRight(), mv, REF_PIC_LIST_1, ridx, true);
                }
              }
            }
#endif
            //  MC
            m_pcInterSearch->motionCompensation(pu,REF_PIC_LIST_0, true, chroma);
            m_CABACEstimator->getCtx() = m_CurrCtx->start;

#if GDR_ENABLED
            if (isEncodeGdrClean)
            {
              bool mvSolid = true;
              bool mvValid = true;
              if (mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 0].refIdx >= 0)
              {
                mvSolid = mvSolid && mergeCtx.mvSolid[(mergeCand << 1) + 0];
                mvValid = mvValid && mergeCtx.mvValid[(mergeCand << 1) + 0];
              }
              if (mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 1].refIdx >= 0)
              {
                mvSolid = mvSolid && mergeCtx.mvSolid[(mergeCand << 1) + 1];
                mvValid = mvValid && mergeCtx.mvValid[(mergeCand << 1) + 1];
              }

              if (mvSolid && mvValid)
              {
                m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, (numResidualPass != 0), true, chroma);
              }
            }
            else
            {
              m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, (numResidualPass != 0), true, chroma);
            }
#else
            m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, (numResidualPass != 0), true, chroma);
#endif
            if (tempCS->slice->getSPS()->getUseColorTrans())
            {
              bestCS->tmpColorSpaceCost = tempCS->tmpColorSpaceCost;
              bestCS->firstColorSpaceSelected = tempCS->firstColorSpaceSelected;
            }
            xEncodeDontSplit(*tempCS, partitioner);

#if ENABLE_QPA_SUB_CTU
            xCheckDQP (*tempCS, partitioner);
#else
            // this if-check is redundant
            if (tempCS->pps->getUseDQP() && partitioner.currQgEnable())
            {
              xCheckDQP(*tempCS, partitioner);
            }
#endif
            xCheckChromaQPOffset( *tempCS, partitioner );


            DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
            xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);

            tempCS->initStructData(encTestMode.qp);
          }

          if (m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip)
          {
            if (bestCS->getCU(partitioner.chType) == NULL)
            {
              bestIsSkip = 0;
            }
            else
            {
              bestIsSkip = bestCS->getCU(partitioner.chType)->rootCbf == 0;
            }
          }
        }
      }
    }
  }
  if ( m_bestModeUpdated && bestCS->cost != MAX_DOUBLE )
  {
    xCalDebCost( *bestCS, partitioner );
  }
#if GDR_ENABLED
  if (isEncodeGdrClean)
  {
    delete[] MrgSolid;
    delete[] MrgValid;
  }
#endif
}

void EncCu::xCheckRDCostIBCMode(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  if (tempCS->area.lwidth() == 128 || tempCS->area.lheight() == 128) // disable IBC mode larger than 64x64
  {
    return;
  }

  tempCS->initStructData(encTestMode.qp);

  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;

  CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);

  partitioner.setCUData(cu);
  cu.slice       = tempCS->slice;
  cu.tileIdx     = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
  cu.skip        = false;
  cu.predMode    = MODE_IBC;
  cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
  cu.qp          = encTestMode.qp;
  cu.imv         = 0;
  cu.sbtInfo     = 0;

  CU::addPUs(cu);

  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;

  PredictionUnit &pu  = *cu.firstPU;
  cu.mmvdSkip         = false;
  pu.mmvdMergeFlag    = false;
  pu.regularMergeFlag = false;

  pu.intraDir[0] = DC_IDX;       // set intra pred for ibc block
  pu.intraDir[1] = PLANAR_IDX;   // set intra pred for ibc block

  pu.interDir               = 1;             // use list 0 for IBC mode
  pu.refIdx[REF_PIC_LIST_0] = MAX_NUM_REF;   // last idx in the list
  //predIBCSearch进行运动搜索
  bool bValid =
    m_pcInterSearch->predIBCSearch(cu, partitioner, m_ctuIbcSearchRangeX, m_ctuIbcSearchRangeY, m_ibcHashMap);

  if (bValid)
  {
    PU::spanMotionInfo(pu);
    const bool chroma = !pu.cu->isSepTree();
    //  MC
    m_pcInterSearch->motionCompensation(pu, REF_PIC_LIST_0, true, chroma);

    m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, false, true, chroma);
    if (tempCS->slice->getSPS()->getUseColorTrans())
    {
      bestCS->tmpColorSpaceCost       = tempCS->tmpColorSpaceCost;
      bestCS->firstColorSpaceSelected = tempCS->firstColorSpaceSelected;
    }

    xEncodeDontSplit(*tempCS, partitioner);

#if ENABLE_QPA_SUB_CTU
    xCheckDQP(*tempCS, partitioner);
#else
    // this if-check is redundant
    if (tempCS->pps->getUseDQP() && partitioner.currQgEnable())
    {
      xCheckDQP(*tempCS, partitioner);
    }
#endif
    xCheckChromaQPOffset(*tempCS, partitioner);

    tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();
    if (m_bestModeUpdated)
    {
      xCalDebCost(*tempCS, partitioner);
    }

    DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
    xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
  }   // bValid
  else
  {
    tempCS->dist         = 0;
    tempCS->fracBits     = 0;
    tempCS->cost         = MAX_DOUBLE;
    tempCS->costDbOffset = 0;
  }
}
  // check ibc mode in encoder RD
  //////////////////////////////////////////////////////////////////////////////////////////////

// 进行常规AMVP和affine AMVP模式
void EncCu::xCheckRDCostInter( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  //重置tempCS
  tempCS->initStructData( encTestMode.qp );

  //重置m_affineModeSelected，为true表示affine被选为最佳模式
  m_pcInterSearch->setAffineModeSelected(false);

  m_pcInterSearch->resetBufferedUniMotions();
  //B帧开启BCW，权重数为5，否则为1
  int bcwLoopNum = (tempCS->slice->isInterB() ? BCW_NUM : 1);
  bcwLoopNum = (tempCS->sps->getUseBcw() ? bcwLoopNum : 1);
  
  // BCW像素数应该>=256
  if( tempCS->area.lwidth() * tempCS->area.lheight() < BCW_SIZE_CONSTRAINT )
  {
    bcwLoopNum = 1;
  }

  double curBestCost = bestCS->cost; // 当前最佳代价
  double equBcwCost = MAX_DOUBLE;

  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;
  //遍历BCW的5种权重，双向权重{4, 5, 3, 10, -2}
  for( int bcwLoopIdx = 0; bcwLoopIdx < bcwLoopNum; bcwLoopIdx++ )
  {
    // BCW快速算法，CU目前的测试的模式为帧间时，只测试均等权重和最佳BCW权重
    if( m_pcEncCfg->getUseBcwFast() )
    {
      auto blkCache = dynamic_cast< CacheBlkInfoCtrl* >(m_modeCtrl);

      if( blkCache )
      {
        bool isBestInter = blkCache->getInter(bestCS->area);
        uint8_t bestBcwIdx = blkCache->getBcwIdx(bestCS->area);
        //default为2 也就是均等权重 
        if( isBestInter && g_BcwSearchOrder[bcwLoopIdx] != BCW_DEFAULT && g_BcwSearchOrder[bcwLoopIdx] != bestBcwIdx )
        {
          continue;
        }
      }
    }
    // RA只测试BCW权重列表{4, 5, 3, 10, -2}的前三种
    if( !tempCS->slice->getCheckLDC() )
    {
      if( bcwLoopIdx != 0 && bcwLoopIdx != 3 && bcwLoopIdx != 4 )
      {
        continue;
      }
    }

    CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);

    partitioner.setCUData(cu);
    cu.slice    = tempCS->slice;
    cu.tileIdx  = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
    cu.skip     = false;
    cu.mmvdSkip = false;
    // cu.affine
    cu.predMode    = MODE_INTER;
    cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
    cu.qp          = encTestMode.qp;
    CU::addPUs(cu);

    cu.BcwIdx       = g_BcwSearchOrder[bcwLoopIdx];
    uint8_t bcwIdx  = cu.BcwIdx;   // 当前测试的BCW权重
    bool    testBcw = (bcwIdx != BCW_DEFAULT); // 开启BCW

#if GDR_ENABLED
  const bool isEncodeGdrClean = tempCS->sps->getGDREnabledFlag() && tempCS->pcv->isEncoder && ((tempCS->picHeader->getInGdrInterval() && tempCS->isClean(cu.Y().topRight(), CHANNEL_TYPE_LUMA)) || (tempCS->picHeader->getNumVerVirtualBoundaries() == 0));
#endif


    /*
    * 获取帧间常规AMVP模式和Affine AMVP模式的最优运动信息和预测值。
    一、 检查常规AMVP模式
    1. 首先检查单向预测模式：对前向参考帧列表L0和后向参考帧列表L1中的所有参考帧进行遍历，分别进行AMVP，选出最佳预测MV（MVP），之后在最佳MVP基础上调用xMotionEstimation函数进行运动估计选出最佳MV，并记录单向预测中最佳的运动信息及RD Cost
    2 检查双向预测模式（对于4x4、4x8和8x4PU禁用双向预测）：
    2.1 使用单向预测得到的最佳运动信息，进行四次迭代，调用调用xMotionEstimation函数进行运动估计，得到每个参考帧下最优的运动信息（在双向预测进行运动估计前，需要先计算另一个反向参考帧得到的预测块，然后在运动估计中，从原始块减去另一方向的预测块），然后进行四次迭代，分别选出参考帧L0 L1在双向预测时的最佳MV
    2.2 检查对称MVD模式（symmetric MVD）：使用之前单向预测和双向预测得到的参考帧L0的最优运动信息，选出最优的MVP，再通过对称运动估计选出最佳MV，如果所得Cost小于之前双向预测模式的Cost，则记录所得最优运动信息及相关代价信息
    3 比较单向预测和双向预测模式的Cost，并记录最优模式信息存入在PU中
    二、检查Affine AMVP模式(CU尺寸宽度和高度均大于8)
    1. 先检查4参数模型，调用xPredAffineInterSearch函数进行运动估计并计算相关Cost
    2. 如果四参数模型的Cost小于1.05倍的常规AMVP的Cost，则检查6参数模型
    三、进行运动补偿motionCompensation得到预测值
    四、设置加权预测的参数
    */
    m_pcInterSearch->predInterSearch(cu, partitioner);

    bcwIdx = CU::getValidBcwIdx(cu);
    if (testBcw && bcwIdx == BCW_DEFAULT)   // Enabled Bcw but the search results is uni.
    {
      tempCS->initStructData(encTestMode.qp);
      continue;
    }
    CHECK(!(testBcw || (!testBcw && bcwIdx == BCW_DEFAULT)), " !( bTestBcw || (!bTestBcw && bcwIdx == BCW_DEFAULT ) )");
    //为true表示不开启BCW时最佳模式为单向预测
    bool isEqualUni = false; 
    if (m_pcEncCfg->getUseBcwFast())
    {
      if (cu.firstPU->interDir != 3 && testBcw == 0)
      {
        isEqualUni = true;
      }
    }

#if GDR_ENABLED
    // 2.0 xCheckRDCostInter: check residual (compare with bestCS)
    //GRD Gradual Decoder Refresh  把一个I帧分成多片 
    //https://soaringlee.blog.csdn.net/article/details/108441044
    if (isEncodeGdrClean)
    {
      bool isClean = true;

      if (cu.affine && cu.firstPU)
      {
        bool L0ok = true, L1ok = true, L3ok = true;

        L0ok = L0ok && cu.firstPU->mvAffiSolid[0][0] && cu.firstPU->mvAffiSolid[0][1] && cu.firstPU->mvAffiSolid[0][2];
        L0ok = L0ok && cu.firstPU->mvAffiValid[0][0] && cu.firstPU->mvAffiValid[0][1] && cu.firstPU->mvAffiValid[0][2];

        L1ok = L1ok && cu.firstPU->mvAffiSolid[1][0] && cu.firstPU->mvAffiSolid[1][1] && cu.firstPU->mvAffiSolid[1][2];
        L1ok = L1ok && cu.firstPU->mvAffiValid[1][0] && cu.firstPU->mvAffiValid[1][1] && cu.firstPU->mvAffiValid[1][2];

        L3ok = L0ok && L1ok;

        if (cu.firstPU->interDir == 1 && !L0ok)
        {
          isClean = false;
        }
        if (cu.firstPU->interDir == 2 && !L1ok)
        {
          isClean = false;
        }
        if (cu.firstPU->interDir == 3 && !L3ok)
        {
          isClean = false;
        }
      }
      else if (cu.firstPU)
      {
        bool L0ok = true;
        bool L1ok = true;
        bool L3ok = true;

        L0ok = L0ok && cu.firstPU->mvSolid[0];
        L0ok = L0ok && cu.firstPU->mvValid[0];

        L1ok = L1ok && cu.firstPU->mvSolid[1];
        L1ok = L1ok && cu.firstPU->mvValid[1];

        L3ok = L0ok && L1ok;

        if (cu.firstPU->interDir == 1 && !L0ok)
        {
          isClean = false;
        }
        if (cu.firstPU->interDir == 2 && !L1ok)
        {
          isClean = false;
        }
        if (cu.firstPU->interDir == 3 && !L3ok)
        {
          isClean = false;
        }
      }
      else
      {
        isClean = false;
      }

      if (isClean)
      {
        xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 0
          , 0
          , &equBcwCost
        );
      }
    }
    else
    {
      xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 0
        , 0
        , &equBcwCost
      );
    }
#else

    // 对残差变换量化，调用encodeResAndCalcRdInterCU进行RDcost计算选出最佳帧间模式，
    // 包含对SBT的RD决策过程。encodeResAndCalcRdInterCU包含对MTS的RD决策过程
    xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 0, 0, &equBcwCost);
#endif


#if GDR_ENABLED
  if (g_BcwSearchOrder[bcwLoopIdx] == BCW_DEFAULT && bestCS->cus.size() > 0)
    m_pcInterSearch->setAffineModeSelected((bestCS->cus.front()->affine && !(bestCS->cus.front()->firstPU->mergeFlag)));
#else
    if (g_BcwSearchOrder[bcwLoopIdx] == BCW_DEFAULT)
    {
      m_pcInterSearch->setAffineModeSelected(
        (bestCS->cus.front()->affine && !(bestCS->cus.front()->firstPU->mergeFlag)));
    }
#endif

    tempCS->initStructData(encTestMode.qp);

    double skipTH = MAX_DOUBLE;
    skipTH        = (m_pcEncCfg->getUseBcwFast() ? 1.05 : MAX_DOUBLE);
    //不开启BCW时的RDcost如果超过之前的最佳RDcost的1.05倍，就跳过之后BCW的权重测试
    if (equBcwCost > curBestCost * skipTH)
    {
      break;
    }
    //当序列中只有第一帧是I帧时，如果不开启BCW时最佳模式为单向预测，就跳过之后BCW的权重测试
    if (m_pcEncCfg->getUseBcwFast())
    {
      if (isEqualUni == true && m_pcEncCfg->getIntraPeriod() == -1)
      {
        break;
      }
    }
    //满足一定条件，就跳过之后BCW的权重测试
    if (g_BcwSearchOrder[bcwLoopIdx] == BCW_DEFAULT && xIsBcwSkip(cu) && m_pcEncCfg->getUseBcwFast())
    {
      break;
    }
  }   // for( UChar bcwLoopIdx = 0; bcwLoopIdx < bcwLoopNum; bcwLoopIdx++ )
  if ( m_bestModeUpdated && bestCS->cost != MAX_DOUBLE )
  {
    xCalDebCost( *bestCS, partitioner );
  }
}
// 和xCheckRDCostInter函数类似，xCheckRDCostInterIMV函数会提前设置cu.imv
// cu.imv指定AMVP模式下的像素精度
bool EncCu::xCheckRDCostInterIMV(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, double &bestIntPelCost)
{
  // 常规AMVP模式imv可以为0，1，2，3，分别为1/4，1，4，1/2像素精度；
  // affine AMVP模式下imv可以为0，1，2，分别为1/4，1/16，1像素精度
  int iIMV = int( ( encTestMode.opts & ETO_IMV ) >> ETO_IMV_SHIFT );
  m_pcInterSearch->setAffineModeSelected(false);
  // Only Half-Pel, int-Pel, 4-Pel and fast 4-Pel allowed
  CHECK(iIMV < 1 || iIMV > 4, "Unsupported IMV Mode");
  //是否在测试半像素精度
  const bool testAltHpelFilter = iIMV == 4;
  // Fast 4-Pel Mode

  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;

  EncTestMode encTestModeBase = encTestMode;                                        // copy for clearing non-IMV options
  encTestModeBase.opts        = EncTestModeOpts( encTestModeBase.opts & ETO_IMV );  // clear non-IMV options (is that intended?)

  tempCS->initStructData( encTestMode.qp );

  m_pcInterSearch->resetBufferedUniMotions();
  int bcwLoopNum = (tempCS->slice->isInterB() ? BCW_NUM : 1);
  bcwLoopNum = (tempCS->slice->getSPS()->getUseBcw() ? bcwLoopNum : 1);

  if( tempCS->area.lwidth() * tempCS->area.lheight() < BCW_SIZE_CONSTRAINT )
  {
    bcwLoopNum = 1;
  }

  bool validMode = false;
  double curBestCost = bestCS->cost;
  double equBcwCost = MAX_DOUBLE;
  //CU加权的双向预测
  for( int bcwLoopIdx = 0; bcwLoopIdx < bcwLoopNum; bcwLoopIdx++ )
  {
    if( m_pcEncCfg->getUseBcwFast() )
    {
      auto blkCache = dynamic_cast< CacheBlkInfoCtrl* >(m_modeCtrl);

      if( blkCache )
      {
        bool isBestInter = blkCache->getInter(bestCS->area);
        uint8_t bestBcwIdx = blkCache->getBcwIdx(bestCS->area);

        if( isBestInter && g_BcwSearchOrder[bcwLoopIdx] != BCW_DEFAULT && g_BcwSearchOrder[bcwLoopIdx] != bestBcwIdx )
        {
          continue;
        }
      }
    }

    if( !tempCS->slice->getCheckLDC() )
    {
      if( bcwLoopIdx != 0 && bcwLoopIdx != 3 && bcwLoopIdx != 4 )
      {
        continue;
      }
    }
    //如果是LD，只能测试不开启AMVR时最好的两次RDcost下的BCW权重还有默认权重
    if( m_pcEncCfg->getUseBcwFast() && tempCS->slice->getCheckLDC() && g_BcwSearchOrder[bcwLoopIdx] != BCW_DEFAULT
      && (m_bestBcwIdx[0] >= 0 && g_BcwSearchOrder[bcwLoopIdx] != m_bestBcwIdx[0])
      && (m_bestBcwIdx[1] >= 0 && g_BcwSearchOrder[bcwLoopIdx] != m_bestBcwIdx[1]))
    {
      continue;
    }

    CodingUnit &cu = tempCS->addCU( tempCS->area, partitioner.chType );

    partitioner.setCUData( cu );
    cu.slice            = tempCS->slice;
    cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
    cu.skip             = false;
    cu.mmvdSkip = false;
  //cu.affine
    cu.predMode         = MODE_INTER;
    cu.chromaQpAdj      = m_cuChromaQpOffsetIdxPlus1;
    cu.qp               = encTestMode.qp;

    CU::addPUs( cu );

  #if GDR_ENABLED
      const bool isEncodeGdrClean = tempCS->sps->getGDREnabledFlag() && tempCS->pcv->isEncoder && ((tempCS->picHeader->getInGdrInterval() && tempCS->isClean(cu.Y().topRight(), CHANNEL_TYPE_LUMA)) || (tempCS->picHeader->getNumVerVirtualBoundaries() == 0));
  #endif

    if (testAltHpelFilter)
    {
      cu.imv = IMV_HPEL; // 1/2像素
    }
    else
    {
      cu.imv = iIMV == 1 ? IMV_FPEL : IMV_4PEL; // 整像素或4像素
    }
    // 常规AMVP下imv可以为0，1，2，3，分别为1/4，1，4，1/2像素精度；
    // affine AMVP下imv可以为0，1，2，分别为1/4，1/16，1像素精度
    bool testBcw;
    uint8_t bcwIdx;
    bool affineAmvrEanbledFlag = !testAltHpelFilter && cu.slice->getSPS()->getAffineAmvrEnabledFlag();

    cu.BcwIdx = g_BcwSearchOrder[bcwLoopIdx];
    bcwIdx = cu.BcwIdx;
    testBcw = (bcwIdx != BCW_DEFAULT);

    cu.firstPU->interDir = 10;
    // 进行帧间搜索
    m_pcInterSearch->predInterSearch( cu, partitioner );

    if ( cu.firstPU->interDir <= 3 )
    {
      bcwIdx = CU::getValidBcwIdx(cu);
    }
    else
    {
      return false;
    }

    if( m_pcEncCfg->getMCTSEncConstraint() && ( ( cu.firstPU->refIdx[L0] < 0 && cu.firstPU->refIdx[L1] < 0 ) || ( !( MCTSHelper::checkMvBufferForMCTSConstraint( *cu.firstPU ) ) ) ) )
    {
      // Do not use this mode
      tempCS->initStructData( encTestMode.qp );
      continue;
    }
    if( testBcw && bcwIdx == BCW_DEFAULT ) // Enabled Bcw but the search results is uni.
    {
      tempCS->initStructData(encTestMode.qp);
      continue;
    }
    CHECK(!(testBcw || (!testBcw && bcwIdx == BCW_DEFAULT)), " !( bTestBcw || (!bTestBcw && bcwIdx == BCW_DEFAULT ) )");

    bool isEqualUni = false;
    if( m_pcEncCfg->getUseBcwFast() )
    {
      if( cu.firstPU->interDir != 3 && testBcw == 0 )
      {
        isEqualUni = true;
      }
    }

    if ( !CU::hasSubCUNonZeroMVd( cu ) && !CU::hasSubCUNonZeroAffineMVd( cu ) )
    {
      if (m_modeCtrl->useModeResult(encTestModeBase, tempCS, partitioner))
      {
        std::swap(tempCS, bestCS);
        // store temp best CI for next CU coding
        m_CurrCtx->best = m_CABACEstimator->getCtx();
      }
      if ( affineAmvrEanbledFlag )
      {
        tempCS->initStructData( encTestMode.qp );
        continue;
      }
      else
      {
        return false;
      }
    }

  #if GDR_ENABLED
    // 2.0 xCheckRDCostInter: check residual (compare with bestCS)
    if (isEncodeGdrClean)
    {
      bool isClean = true;

      if (cu.affine && cu.firstPU)
      {
        bool L0ok = true, L1ok = true, L3ok = true;

        L0ok = L0ok && cu.firstPU->mvAffiSolid[0][0] && cu.firstPU->mvAffiSolid[0][1] && cu.firstPU->mvAffiSolid[0][2];
        L0ok = L0ok && cu.firstPU->mvAffiValid[0][0] && cu.firstPU->mvAffiValid[0][1] && cu.firstPU->mvAffiValid[0][2];

        L1ok = L1ok && cu.firstPU->mvAffiSolid[1][0] && cu.firstPU->mvAffiSolid[1][1] && cu.firstPU->mvAffiSolid[1][2];
        L1ok = L1ok && cu.firstPU->mvAffiValid[1][0] && cu.firstPU->mvAffiValid[1][1] && cu.firstPU->mvAffiValid[1][2];

        L3ok = L0ok && L1ok;

        if (cu.firstPU->interDir == 1 && !L0ok)
        {
          isClean = false;
        }
        if (cu.firstPU->interDir == 2 && !L1ok)
        {
          isClean = false;
        }
        if (cu.firstPU->interDir == 3 && !L3ok)
        {
          isClean = false;
        }
      }
      else if (cu.firstPU)
      {
        bool L0ok = cu.firstPU->mvSolid[0] && cu.firstPU->mvValid[0];
        bool L1ok = cu.firstPU->mvSolid[1] && cu.firstPU->mvValid[1];
        bool L3ok = L0ok && L1ok;

        if (cu.firstPU->interDir == 1 && !L0ok)
        {
          isClean = false;
        }
        if (cu.firstPU->interDir == 2 && !L1ok)
        {
          isClean = false;
        }
        if (cu.firstPU->interDir == 3 && !L3ok)
        {
          isClean = false;
        }
      }
      else
      {
        isClean = false;
      }

      if (isClean)
      {
        xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 0
          , 0
          , &equBcwCost
        );
      }
    }
    else
    {
      xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 0
        , 0
        , &equBcwCost
      );
    }
#else
    xEncodeInterResidual(tempCS, bestCS, partitioner, encTestModeBase, 0, 0, &equBcwCost);
#endif

    if( cu.imv == IMV_FPEL && tempCS->cost < bestIntPelCost )
    {
      bestIntPelCost = tempCS->cost;
    }
    tempCS->initStructData(encTestMode.qp);

    double skipTH = MAX_DOUBLE;
    skipTH = (m_pcEncCfg->getUseBcwFast() ? 1.05 : MAX_DOUBLE);
    if( equBcwCost > curBestCost * skipTH )
    {
      break;
    }

    if( m_pcEncCfg->getUseBcwFast() )
    {
      if( isEqualUni == true && m_pcEncCfg->getIntraPeriod() == -1 )
      {
        break;
      }
    }
    if( g_BcwSearchOrder[bcwLoopIdx] == BCW_DEFAULT && xIsBcwSkip(cu) && m_pcEncCfg->getUseBcwFast() )
    {
      break;
    }
    validMode = true;
  } // for( UChar bcwLoopIdx = 0; bcwLoopIdx < bcwLoopNum; bcwLoopIdx++ )

  if ( m_bestModeUpdated && bestCS->cost != MAX_DOUBLE )
  {
    xCalDebCost( *bestCS, partitioner );
  }

  return tempCS->slice->getSPS()->getAffineAmvrEnabledFlag() ? validMode : true;
}

void EncCu::xCalDebCost( CodingStructure &cs, Partitioner &partitioner, bool calDist )
{
  if ( cs.cost == MAX_DOUBLE )
  {
    cs.costDbOffset = 0;
  }

  if ( cs.slice->getDeblockingFilterDisable() || ( !m_pcEncCfg->getUseEncDbOpt() && !calDist ) )
  {
    return;
  }

  m_deblockingFilter->setEnc(true);
  const ChromaFormat format = cs.area.chromaFormat;
  CodingUnit*                cu = cs.getCU(partitioner.chType);
  const Position lumaPos = cu->Y().valid() ? cu->Y().pos() : recalcPosition( format, cu->chType, CHANNEL_TYPE_LUMA, cu->blocks[cu->chType].pos() );
  bool topEdgeAvai = lumaPos.y > 0 && ((lumaPos.y % 4) == 0);
  bool leftEdgeAvai = lumaPos.x > 0 && ((lumaPos.x % 4) == 0);
  bool anyEdgeAvai = topEdgeAvai || leftEdgeAvai;
  cs.costDbOffset = 0;

  if ( calDist )
  {
    ComponentID compStr = ( cu->isSepTree() && !isLuma( partitioner.chType ) ) ? COMPONENT_Cb : COMPONENT_Y;
    ComponentID compEnd = ( cu->isSepTree() && isLuma( partitioner.chType ) ) ? COMPONENT_Y : COMPONENT_Cr;
    Distortion finalDistortion = 0;
    for ( int comp = compStr; comp <= compEnd; comp++ )
    {
      const ComponentID compID = ComponentID( comp );
      CPelBuf org = cs.getOrgBuf( compID );
      CPelBuf reco = cs.getRecoBuf( compID );
      finalDistortion += getDistortionDb(cs, org, reco, compID, cs.area.block(COMPONENT_Y), false);
    }
    //updated distortion
    cs.dist = finalDistortion;
  }

  if ( anyEdgeAvai && m_pcEncCfg->getUseEncDbOpt() )
  {
    ComponentID compStr = ( cu->isSepTree() && !isLuma( partitioner.chType ) ) ? COMPONENT_Cb : COMPONENT_Y;
    ComponentID compEnd = ( cu->isSepTree() &&  isLuma( partitioner.chType ) ) ? COMPONENT_Y : COMPONENT_Cr;

    const UnitArea currCsArea = clipArea(cs.area, *cs.picture);

    PelStorage&          picDbBuf = m_deblockingFilter->getDbEncPicYuvBuffer();

    //deblock neighbour pixels
    const Size     lumaSize = cu->Y().valid() ? cu->Y().size() : recalcSize( format, cu->chType, CHANNEL_TYPE_LUMA, cu->blocks[cu->chType].size() );

    const int verOffset = lumaPos.y > 7 ? 8 : 4;
    const int horOffset = lumaPos.x > 7 ? 8 : 4;
    const UnitArea areaTop(  format, Area( lumaPos.x, lumaPos.y - verOffset, lumaSize.width, verOffset  ) );
    const UnitArea areaLeft( format, Area( lumaPos.x - horOffset, lumaPos.y, horOffset, lumaSize.height ) );
    for ( int compIdx = compStr; compIdx <= compEnd; compIdx++ )
    {
      ComponentID compId = (ComponentID)compIdx;

      //Copy current CU's reco to Deblock Pic Buffer
      const CompArea&  curCompArea = currCsArea.block( compId );
      picDbBuf.getBuf( curCompArea ).copyFrom( cs.getRecoBuf( curCompArea ) );
      if (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getSliceReshaperInfo().getUseSliceReshaper() && isLuma(compId))
      {
        picDbBuf.getBuf( curCompArea ).rspSignal( m_pcReshape->getInvLUT() );
      }

      //left neighbour
      if ( leftEdgeAvai )
      {
        const CompArea&  compArea = areaLeft.block(compId);
        picDbBuf.getBuf( compArea ).copyFrom( cs.picture->getRecoBuf( compArea ) );
        if (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getSliceReshaperInfo().getUseSliceReshaper() && isLuma(compId))
        {
          picDbBuf.getBuf( compArea ).rspSignal( m_pcReshape->getInvLUT() );
        }
      }
      //top neighbour
      if ( topEdgeAvai )
      {
        const CompArea&  compArea = areaTop.block( compId );
        picDbBuf.getBuf( compArea ).copyFrom( cs.picture->getRecoBuf( compArea ) );
        if (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getSliceReshaperInfo().getUseSliceReshaper() && isLuma(compId))
        {
          picDbBuf.getBuf( compArea ).rspSignal( m_pcReshape->getInvLUT() );
        }
      }
    }

    //deblock
    if ( leftEdgeAvai )
    {
      m_deblockingFilter->resetFilterLengths();
      m_deblockingFilter->xDeblockCU( *cu, EDGE_VER );
    }

    if (topEdgeAvai)
    {
      m_deblockingFilter->resetFilterLengths();
      m_deblockingFilter->xDeblockCU( *cu, EDGE_HOR );
    }

    //update current CU SSE
    Distortion distCur = 0;
    for ( int compIdx = compStr; compIdx <= compEnd; compIdx++ )
    {
      ComponentID compId = (ComponentID)compIdx;
      CPelBuf reco = picDbBuf.getBuf( currCsArea.block( compId ) );
      CPelBuf org = cs.getOrgBuf( compId );
      distCur += getDistortionDb(cs, org, reco, compId, currCsArea.block(COMPONENT_Y), true);
    }

    //calculate difference between DB_before_SSE and DB_after_SSE for neighbouring CUs
    Distortion distBeforeDb = 0, distAfterDb = 0;
    for (int compIdx = compStr; compIdx <= compEnd; compIdx++)
    {
      ComponentID compId = (ComponentID)compIdx;
      if ( leftEdgeAvai )
      {
        const CompArea&  compArea = areaLeft.block( compId );
        CPelBuf org = cs.picture->getOrigBuf( compArea );
        CPelBuf reco = cs.picture->getRecoBuf( compArea );
        CPelBuf recoDb = picDbBuf.getBuf( compArea );
        distBeforeDb += getDistortionDb(cs, org, reco, compId, areaLeft.block(COMPONENT_Y), false);
        distAfterDb += getDistortionDb(cs, org, recoDb, compId, areaLeft.block(COMPONENT_Y), true);
      }
      if ( topEdgeAvai )
      {
        const CompArea&  compArea = areaTop.block( compId );
        CPelBuf org = cs.picture->getOrigBuf( compArea );
        CPelBuf reco = cs.picture->getRecoBuf( compArea );
        CPelBuf recoDb = picDbBuf.getBuf( compArea );
        distBeforeDb += getDistortionDb(cs, org, reco, compId, areaTop.block(COMPONENT_Y), false);
        distAfterDb += getDistortionDb(cs, org, recoDb, compId, areaTop.block(COMPONENT_Y), true);
      }
    }

    //updated cost
    int64_t distTmp = distCur - cs.dist + distAfterDb - distBeforeDb;
    int sign = distTmp < 0 ? -1 : 1;
    distTmp = distTmp < 0 ? -distTmp : distTmp;
    cs.costDbOffset = sign * m_pcRdCost->calcRdCost( 0, distTmp );
  }

  m_deblockingFilter->setEnc( false );
}

Distortion EncCu::getDistortionDb( CodingStructure &cs, CPelBuf org, CPelBuf reco, ComponentID compID, const CompArea& compArea, bool afterDb )
{
  Distortion dist = 0;
#if WCG_EXT
  m_pcRdCost->setChromaFormat(cs.sps->getChromaFormatIdc());
  CPelBuf orgLuma = cs.picture->getOrigBuf(compArea);
  if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (
    m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())))
  {
    if ( compID == COMPONENT_Y && !afterDb && !m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled())
    {
      CompArea    tmpArea( COMPONENT_Y, cs.area.chromaFormat, Position( 0, 0 ), compArea.size() );
      PelBuf tmpRecLuma = m_tmpStorageLCU->getBuf( tmpArea );
      tmpRecLuma.copyFrom( reco );
      tmpRecLuma.rspSignal( m_pcReshape->getInvLUT() );
      dist += m_pcRdCost->getDistPart( org, tmpRecLuma, cs.sps->getBitDepth( toChannelType( compID ) ), compID, DF_SSE_WTD, &orgLuma );
    }
    else
    {
      dist += m_pcRdCost->getDistPart( org, reco, cs.sps->getBitDepth( toChannelType( compID ) ), compID, DF_SSE_WTD, &orgLuma );
    }
  }
  else if (m_pcEncCfg->getLmcs() && cs.slice->getLmcsEnabledFlag() && cs.slice->isIntra()) //intra slice
  {
    if ( compID == COMPONENT_Y && afterDb )
    {
      CompArea    tmpArea( COMPONENT_Y, cs.area.chromaFormat, Position( 0, 0 ), compArea.size() );
      PelBuf tmpRecLuma = m_tmpStorageLCU->getBuf( tmpArea );
      tmpRecLuma.copyFrom( reco );
      tmpRecLuma.rspSignal( m_pcReshape->getFwdLUT() );
      dist += m_pcRdCost->getDistPart( org, tmpRecLuma, cs.sps->getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
    }
    else
    {
      dist += m_pcRdCost->getDistPart( org, reco, cs.sps->getBitDepth(toChannelType( compID ) ), compID, DF_SSE );
    }
  }
  else
#endif
  {
    dist = m_pcRdCost->getDistPart( org, reco, cs.sps->getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
  }
  return dist;
}

void EncCu::xEncodeInterResidual(   CodingStructure *&tempCS
                                  , CodingStructure *&bestCS
                                  , Partitioner &partitioner
                                  , const EncTestMode& encTestMode
                                  , int residualPass
                                  , bool* bestHasNonResi
                                  , double* equBcwCost
  )
{

  CodingUnit*            cu        = tempCS->getCU( partitioner.chType );
  double   bestCostInternal        = MAX_DOUBLE;
  double           bestCost        = bestCS->cost;
  double           bestCostBegin   = bestCS->cost;
  CodingUnit*      prevBestCU      = bestCS->getCU( partitioner.chType );
  uint8_t          prevBestSbt     = ( prevBestCU == nullptr ) ? 0 : prevBestCU->sbtInfo;
  bool              swapped        = false; // avoid unwanted data copy
  bool             reloadCU        = false;

  const PredictionUnit& pu = *cu->firstPU;

  // clang-format off
  const int affineShiftTab[3] =
  {
    MV_PRECISION_INTERNAL - MV_PRECISION_QUARTER,
    MV_PRECISION_INTERNAL - MV_PRECISION_SIXTEENTH,
    MV_PRECISION_INTERNAL - MV_PRECISION_INT
  };

  const int normalShiftTab[NUM_IMV_MODES] =
  {
    MV_PRECISION_INTERNAL - MV_PRECISION_QUARTER,
    MV_PRECISION_INTERNAL - MV_PRECISION_INT,
    MV_PRECISION_INTERNAL - MV_PRECISION_4PEL,
    MV_PRECISION_INTERNAL - MV_PRECISION_HALF,
  };
  // clang-format on

  int mvShift;

  for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    if (pu.refIdx[refList] >= 0)
    {
      if (!cu->affine)
      {
        mvShift = normalShiftTab[cu->imv];
        Mv signaledmvd(pu.mvd[refList].getHor() >> mvShift, pu.mvd[refList].getVer() >> mvShift);
        if (!((signaledmvd.getHor() >= MVD_MIN) && (signaledmvd.getHor() <= MVD_MAX)) || !((signaledmvd.getVer() >= MVD_MIN) && (signaledmvd.getVer() <= MVD_MAX)))
        {
          return;
        }
      }
      else
      {
        for (int ctrlP = 1 + (cu->affineType == AFFINEMODEL_6PARAM); ctrlP >= 0; ctrlP--)
        {
          mvShift = affineShiftTab[cu->imv];
          Mv signaledmvd(pu.mvdAffi[refList][ctrlP].getHor() >> mvShift, pu.mvdAffi[refList][ctrlP].getVer() >> mvShift);
          if (!((signaledmvd.getHor() >= MVD_MIN) && (signaledmvd.getHor() <= MVD_MAX)) || !((signaledmvd.getVer() >= MVD_MIN) && (signaledmvd.getVer() <= MVD_MAX)))
          {
            return;
          }
        }
      }
    }
  }
  // avoid MV exceeding 18-bit dynamic range
  const int maxMv = 1 << 17;
  if (!cu->affine && !pu.mergeFlag)
  {
    if ( (pu.refIdx[0] >= 0 && (pu.mv[0].getAbsHor() >= maxMv || pu.mv[0].getAbsVer() >= maxMv))
      || (pu.refIdx[1] >= 0 && (pu.mv[1].getAbsHor() >= maxMv || pu.mv[1].getAbsVer() >= maxMv)))
    {
      return;
    }
  }
  if (cu->affine && !pu.mergeFlag)
  {
    for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
    {
      if (pu.refIdx[refList] >= 0)
      {
        for (int ctrlP = 1 + (cu->affineType == AFFINEMODEL_6PARAM); ctrlP >= 0; ctrlP--)
        {
          if (pu.mvAffi[refList][ctrlP].getAbsHor() >= maxMv || pu.mvAffi[refList][ctrlP].getAbsVer() >= maxMv)
          {
            return;
          }
        }
      }
    }
  }
  const bool mtsAllowed = tempCS->sps->getUseInterMTS() && CU::isInter( *cu ) && partitioner.currArea().lwidth() <= MTS_INTER_MAX_CU_SIZE && partitioner.currArea().lheight() <= MTS_INTER_MAX_CU_SIZE;
  uint8_t sbtAllowed = cu->checkAllowedSbt();//是否允许使用SBT，判断当前预测模式、CU尺寸是否符合要求
  //SBT 对一个CU残差的一个子部分进行变换处理
  //SBT resolution-dependent fast algorithm: not try size-64 SBT in RDO for low-resolution sequences (now resolution below HD)
  // SBT分辨率相关快速算法：对于低分辨率序列（现在分辨率低于HD），不要尝试RDO中的size-64sbt
  if( tempCS->pps->getPicWidthInLumaSamples() < (uint32_t)m_pcEncCfg->getSBTFast64WidthTh() )
  {
    sbtAllowed = ((cu->lwidth() > 32 || cu->lheight() > 32)) ? 0 : sbtAllowed;
  }
  uint8_t numRDOTried = 0;
  Distortion sbtOffDist = 0;
  bool    sbtOffRootCbf = 0;
  double  sbtOffCost      = MAX_DOUBLE;
  double  currBestCost = MAX_DOUBLE;
  bool    doPreAnalyzeResi = ( sbtAllowed || mtsAllowed ) && residualPass == 0;

  m_pcInterSearch->initTuAnalyzer();
  if( doPreAnalyzeResi )
  {
    m_pcInterSearch->calcMinDistSbt( *tempCS, *cu, sbtAllowed );
  }

  auto    slsSbt = dynamic_cast<SaveLoadEncInfoSbt*>( m_modeCtrl );
  int     slShift = 4 + std::min( (int)gp_sizeIdxInfo->idxFrom( cu->lwidth() ) + (int)gp_sizeIdxInfo->idxFrom( cu->lheight() ), 9 );
  Distortion curPuSse = m_pcInterSearch->getEstDistSbt( NUMBER_SBT_MODE );
  uint8_t currBestSbt = 0;
  uint8_t currBestTrs = MAX_UCHAR;
  uint8_t histBestSbt = MAX_UCHAR;
  uint8_t histBestTrs = MAX_UCHAR;
  m_pcInterSearch->setHistBestTrs( MAX_UCHAR, MAX_UCHAR );
  if( doPreAnalyzeResi )
  {
    if( m_pcInterSearch->getSkipSbtAll() && !mtsAllowed ) //emt is off
    {
      histBestSbt = 0; //try DCT2
      m_pcInterSearch->setHistBestTrs( histBestSbt, histBestTrs );
    }
    else
    {
      assert( curPuSse != std::numeric_limits<uint64_t>::max() );
      uint16_t compositeSbtTrs = slsSbt->findBestSbt( cu->cs->area, (uint32_t)( curPuSse >> slShift ) );
      histBestSbt = ( compositeSbtTrs >> 0 ) & 0xff;
      histBestTrs = ( compositeSbtTrs >> 8 ) & 0xff;
      if( m_pcInterSearch->getSkipSbtAll() && CU::isSbtMode( histBestSbt ) ) //special case, skip SBT when loading SBT
      {
        histBestSbt = 0; //try DCT2
      }
      m_pcInterSearch->setHistBestTrs( histBestSbt, histBestTrs );
    }
  }

  {
    if( reloadCU )
    {
      if( bestCost == bestCS->cost ) //The first EMT pass didn't become the bestCS, so we clear the TUs generated
      {
        tempCS->clearTUs();
      }
      else if( false == swapped )
      {
        tempCS->initStructData( encTestMode.qp );
        tempCS->copyStructure( *bestCS, partitioner.chType );
        tempCS->getPredBuf().copyFrom( bestCS->getPredBuf() );
        bestCost = bestCS->cost;
        cu       = tempCS->getCU( partitioner.chType );
        swapped = true;
      }
      else
      {
        tempCS->clearTUs();
        bestCost = bestCS->cost;
        cu       = tempCS->getCU( partitioner.chType );
      }

      //we need to restart the distortion for the new tempCS, the bit count and the cost
      tempCS->dist     = 0;
      tempCS->fracBits = 0;
      tempCS->cost     = MAX_DOUBLE;
      tempCS->costDbOffset = 0;
    }

    reloadCU    = true; // enable cu reloading
    cu->skip    = false;
    cu->sbtInfo = 0;

    const bool skipResidual = residualPass == 1;
    if( skipResidual || histBestSbt == MAX_UCHAR || !CU::isSbtMode( histBestSbt ) )
    {
    m_pcInterSearch->encodeResAndCalcRdInterCU( *tempCS, partitioner, skipResidual );
    if (tempCS->slice->getSPS()->getUseColorTrans())
    {
      bestCS->tmpColorSpaceCost = tempCS->tmpColorSpaceCost;
      bestCS->firstColorSpaceSelected = tempCS->firstColorSpaceSelected;
    }
    numRDOTried += mtsAllowed ? 2 : 1;
    xEncodeDontSplit( *tempCS, partitioner );

    xCheckDQP( *tempCS, partitioner );
    xCheckChromaQPOffset( *tempCS, partitioner );


    if( NULL != bestHasNonResi && (bestCostInternal > tempCS->cost) )
    {
      bestCostInternal = tempCS->cost;
      if (!(tempCS->getPU(partitioner.chType)->ciipFlag))
      *bestHasNonResi  = !cu->rootCbf;
    }

    if (cu->rootCbf == false)
    {
      if (tempCS->getPU(partitioner.chType)->ciipFlag)
      {
        tempCS->cost = MAX_DOUBLE;
        tempCS->costDbOffset = 0;
        return;
      }
    }
    currBestCost = tempCS->cost;
    sbtOffCost = tempCS->cost;
    sbtOffDist = tempCS->dist;
    sbtOffRootCbf = cu->rootCbf;
    currBestSbt = CU::getSbtInfo(cu->firstTU->mtsIdx[COMPONENT_Y] > MTS_SKIP ? SBT_OFF_MTS : SBT_OFF_DCT, 0);
    currBestTrs = cu->firstTU->mtsIdx[COMPONENT_Y];

#if WCG_EXT
    DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda( true ) );
#else
    DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
#endif
    xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );

    }

    uint8_t numSbtRdo = CU::numSbtModeRdo( sbtAllowed );
    //early termination if all SBT modes are not allowed
    //normative
    if( !sbtAllowed || skipResidual )
    {
      numSbtRdo = 0;
    }
    //fast algorithm
    if( ( histBestSbt != MAX_UCHAR && !CU::isSbtMode( histBestSbt ) ) || m_pcInterSearch->getSkipSbtAll() )
    {
      numSbtRdo = 0;
    }
    if( bestCost != MAX_DOUBLE && sbtOffCost != MAX_DOUBLE )
    {
      double th = 1.07;
      if( !( prevBestSbt == 0 || m_sbtCostSave[0] == MAX_DOUBLE ) )
      {
        assert( m_sbtCostSave[1] <= m_sbtCostSave[0] );
        th *= ( m_sbtCostSave[0] / m_sbtCostSave[1] );
      }
      if( sbtOffCost > bestCost * th )
      {
        numSbtRdo = 0;
      }
    }
    if( !sbtOffRootCbf && sbtOffCost != MAX_DOUBLE )
    {
      double th = Clip3( 0.05, 0.55, ( 27 - cu->qp ) * 0.02 + 0.35 );
      if( sbtOffCost < m_pcRdCost->calcRdCost( ( cu->lwidth() * cu->lheight() ) << SCALE_BITS, 0 ) * th )
      {
        numSbtRdo = 0;
      }
    }

    if( histBestSbt != MAX_UCHAR && numSbtRdo != 0 )
    {
      numSbtRdo = 1;
      m_pcInterSearch->initSbtRdoOrder( CU::getSbtMode( CU::getSbtIdx( histBestSbt ), CU::getSbtPos( histBestSbt ) ) );
    }

    for( int sbtModeIdx = 0; sbtModeIdx < numSbtRdo; sbtModeIdx++ )
    {
      uint8_t sbtMode = m_pcInterSearch->getSbtRdoOrder( sbtModeIdx );
      uint8_t sbtIdx = CU::getSbtIdxFromSbtMode( sbtMode );
      uint8_t sbtPos = CU::getSbtPosFromSbtMode( sbtMode );

      //fast algorithm (early skip, save & load)
      if( histBestSbt == MAX_UCHAR )
      {
        uint8_t skipCode = m_pcInterSearch->skipSbtByRDCost( cu->lwidth(), cu->lheight(), cu->mtDepth, sbtIdx, sbtPos, bestCS->cost, sbtOffDist, sbtOffCost, sbtOffRootCbf );
        if( skipCode != MAX_UCHAR )
        {
          continue;
        }

        if( sbtModeIdx > 0 )
        {
          uint8_t prevSbtMode = m_pcInterSearch->getSbtRdoOrder( sbtModeIdx - 1 );
          //make sure the prevSbtMode is the same size as the current SBT mode (otherwise the estimated dist may not be comparable)
          if( CU::isSameSbtSize( prevSbtMode, sbtMode ) )
          {
            Distortion currEstDist = m_pcInterSearch->getEstDistSbt( sbtMode );
            Distortion prevEstDist = m_pcInterSearch->getEstDistSbt( prevSbtMode );
            if( currEstDist > prevEstDist * 1.15 )
            {
              continue;
            }
          }
        }
      }

      //init tempCS and TU
      if( bestCost == bestCS->cost ) //The first EMT pass didn't become the bestCS, so we clear the TUs generated
      {
        tempCS->clearTUs();
      }
      else if( false == swapped )
      {
        tempCS->initStructData( encTestMode.qp );
        tempCS->copyStructure( *bestCS, partitioner.chType );
        tempCS->getPredBuf().copyFrom( bestCS->getPredBuf() );
        bestCost = bestCS->cost;
        cu = tempCS->getCU( partitioner.chType );
        swapped = true;
      }
      else
      {
        tempCS->clearTUs();
        bestCost = bestCS->cost;
        cu = tempCS->getCU( partitioner.chType );
      }

      //we need to restart the distortion for the new tempCS, the bit count and the cost
      tempCS->dist = 0;
      tempCS->fracBits = 0;
      tempCS->cost = MAX_DOUBLE;
      cu->skip = false;

      //set SBT info
      cu->setSbtIdx( sbtIdx );
      cu->setSbtPos( sbtPos );

      //try residual coding
      m_pcInterSearch->encodeResAndCalcRdInterCU( *tempCS, partitioner, skipResidual );
      if (tempCS->slice->getSPS()->getUseColorTrans())
      {
        bestCS->tmpColorSpaceCost = tempCS->tmpColorSpaceCost;
        bestCS->firstColorSpaceSelected = tempCS->firstColorSpaceSelected;
      }
      numRDOTried++;

      xEncodeDontSplit( *tempCS, partitioner );

      xCheckDQP( *tempCS, partitioner );
      xCheckChromaQPOffset( *tempCS, partitioner );

      if( NULL != bestHasNonResi && ( bestCostInternal > tempCS->cost ) )
      {
        bestCostInternal = tempCS->cost;
        if( !( tempCS->getPU( partitioner.chType )->ciipFlag ) )
          *bestHasNonResi = !cu->rootCbf;
      }

      if( tempCS->cost < currBestCost )
      {
        currBestSbt = cu->sbtInfo;
        currBestTrs = tempCS->tus[cu->sbtInfo ? cu->getSbtPos() : 0]->mtsIdx[COMPONENT_Y];
        assert( currBestTrs == 0 || currBestTrs == 1 );
        currBestCost = tempCS->cost;
      }

#if WCG_EXT
      DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda( true ) );
#else
      DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
#endif
      xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
    }

    if( bestCostBegin != bestCS->cost )
    {
      m_sbtCostSave[0] = sbtOffCost;
      m_sbtCostSave[1] = currBestCost;
    }
  } //end emt loop

  if( histBestSbt == MAX_UCHAR && doPreAnalyzeResi && numRDOTried > 1 )
  {
    slsSbt->saveBestSbt( cu->cs->area, (uint32_t)( curPuSse >> slShift ), currBestSbt, currBestTrs );
  }
  tempCS->cost = currBestCost;
  if( ETM_INTER_ME == encTestMode.type )
  {
    if( equBcwCost != NULL )
    {
      if( tempCS->cost < ( *equBcwCost ) && cu->BcwIdx == BCW_DEFAULT )
      {
        ( *equBcwCost ) = tempCS->cost;
      }
    }
    else
    {
      CHECK( equBcwCost == NULL, "equBcwCost == NULL" );
    }
    if( tempCS->slice->getCheckLDC() && !cu->imv && cu->BcwIdx != BCW_DEFAULT && tempCS->cost < m_bestBcwCost[1] )
    {
      if( tempCS->cost < m_bestBcwCost[0] )
      {
        m_bestBcwCost[1] = m_bestBcwCost[0];
        m_bestBcwCost[0] = tempCS->cost;
        m_bestBcwIdx[1] = m_bestBcwIdx[0];
        m_bestBcwIdx[0] = cu->BcwIdx;
      }
      else
      {
        m_bestBcwCost[1] = tempCS->cost;
        m_bestBcwIdx[1] = cu->BcwIdx;
      }
    }
  }
}


void EncCu::xEncodeDontSplit( CodingStructure &cs, Partitioner &partitioner )
{
  m_CABACEstimator->resetBits();

  m_CABACEstimator->split_cu_mode( CU_DONT_SPLIT, cs, partitioner );
  if( partitioner.treeType == TREE_C )
  {
    CHECK( m_CABACEstimator->getEstFracBits() != 0, "must be 0 bit" );
  }

  cs.fracBits += m_CABACEstimator->getEstFracBits(); // split bits
  cs.cost      = m_pcRdCost->calcRdCost( cs.fracBits, cs.dist );
}

#if REUSE_CU_RESULTS
void EncCu::xReuseCachedResult( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner )
{
  m_pcRdCost->setChromaFormat(tempCS->sps->getChromaFormatIdc());
  BestEncInfoCache* bestEncCache = dynamic_cast<BestEncInfoCache*>( m_modeCtrl );
  CHECK( !bestEncCache, "If this mode is chosen, mode controller has to implement the mode caching capabilities" );
  EncTestMode cachedMode;

  if( bestEncCache->setCsFrom( *tempCS, cachedMode, partitioner ) )
  {
    CodingUnit& cu = *tempCS->cus.front();
    partitioner.setCUData( cu );

    if (CU::isIntra(cu) || CU::isPLT(cu))
    {
      xReconIntraQT( cu );
    }
    else
    {
      xDeriveCUMV( cu );
      xReconInter( cu );
    }

    Distortion finalDistortion = 0;
    tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();
    if ( m_pcEncCfg->getUseEncDbOpt() )
    {
      xCalDebCost( *tempCS, partitioner, true );
      finalDistortion = tempCS->dist;
    }
    else
    {
      const SPS &sps                = *tempCS->sps;
      const int  numValidComponents = getNumberValidComponents(tempCS->area.chromaFormat);

      for (int comp = 0; comp < numValidComponents; comp++)
      {
        const ComponentID compID = ComponentID(comp);

        if (partitioner.isSepTree(*tempCS) && toChannelType(compID) != partitioner.chType)
        {
          continue;
        }

        CPelBuf reco = tempCS->getRecoBuf(compID);
        CPelBuf org  = tempCS->getOrgBuf(compID);

#if WCG_EXT
        if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()
            || (m_pcEncCfg->getLmcs() && (tempCS->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())))
        {
          const CPelBuf orgLuma = tempCS->getOrgBuf(tempCS->area.blocks[COMPONENT_Y]);
          if (compID == COMPONENT_Y && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
          {
            const CompArea &area = cu.blocks[COMPONENT_Y];
            CompArea        tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
            PelBuf          tmpRecLuma = m_tmpStorageLCU->getBuf(tmpArea);
            tmpRecLuma.copyFrom(reco);
            tmpRecLuma.rspSignal(m_pcReshape->getInvLUT());
            finalDistortion += m_pcRdCost->getDistPart(org, tmpRecLuma, sps.getBitDepth(toChannelType(compID)), compID,
                                                       DF_SSE_WTD, &orgLuma);
          }
          else
          {
            finalDistortion +=
              m_pcRdCost->getDistPart(org, reco, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
          }
        }
        else
#endif
        {
          finalDistortion += m_pcRdCost->getDistPart(org, reco, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE);
        }
      }
    }

    m_CABACEstimator->getCtx() = m_CurrCtx->start;
    m_CABACEstimator->resetBits();

    CUCtx cuCtx;
    cuCtx.isDQPCoded = true;
    cuCtx.isChromaQpAdjCoded = true;
    m_CABACEstimator->coding_unit( cu, partitioner, cuCtx );


    tempCS->dist     = finalDistortion;
    tempCS->fracBits = m_CABACEstimator->getEstFracBits();
    tempCS->cost     = m_pcRdCost->calcRdCost( tempCS->fracBits, tempCS->dist );

    xEncodeDontSplit( *tempCS,         partitioner );
    xCheckDQP       ( *tempCS,         partitioner );
    xCheckChromaQPOffset( *tempCS,     partitioner );
    xCheckBestMode  (  tempCS, bestCS, partitioner, cachedMode );
  }
  else
  {
    THROW( "Should never happen!" );
  }
}
#endif


//! \}
