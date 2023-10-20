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

/** \file     EncSearch.cpp
 *  \brief    encoder intra search class
 */

#include "IntraSearch.h"

#include "EncModeCtrl.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Rom.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"

#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"

#include <math.h>
#include <limits>
 //! \ingroup EncoderLib
 //! \{
#define PLTCtx(c) SubCtx( Ctx::Palette, c )
IntraSearch::IntraSearch()
  : m_pSplitCS      (nullptr)
  , m_pFullCS       (nullptr)
  , m_pBestCS       (nullptr)
  , m_pcEncCfg      (nullptr)
  , m_pcTrQuant     (nullptr)
  , m_pcRdCost      (nullptr)
  , m_pcReshape     (nullptr)
  , m_CABACEstimator(nullptr)
  , m_CtxCache      (nullptr)
  , m_isInitialized (false)
{
  for( uint32_t ch = 0; ch < MAX_NUM_TBLOCKS; ch++ )
  {
    m_pSharedPredTransformSkip[ch] = nullptr;
  }
  m_minErrorIndexMap = nullptr;
  for (unsigned i = 0; i < (MAXPLTSIZE + 1); i++)
  {
    m_indexError[i] = nullptr;
  }
  for (unsigned i = 0; i < NUM_TRELLIS_STATE; i++)
  {
    m_statePtRDOQ[i] = nullptr;
  }
}


void IntraSearch::destroy()
{
  CHECK( !m_isInitialized, "Not initialized" );

  if( m_pcEncCfg )
  {
    const uint32_t uiNumLayersToAllocateSplit = 1;
    const uint32_t uiNumLayersToAllocateFull  = 1;
    const int uiNumSaveLayersToAllocate = 2;

    for( uint32_t layer = 0; layer < uiNumSaveLayersToAllocate; layer++ )
    {
      m_pSaveCS[layer]->destroy();
      delete m_pSaveCS[layer];
    }

    uint32_t numWidths  = gp_sizeIdxInfo->numWidths();
    uint32_t numHeights = gp_sizeIdxInfo->numHeights();

    for( uint32_t width = 0; width < numWidths; width++ )
    {
      for( uint32_t height = 0; height < numHeights; height++ )
      {
        if( gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( width ) ) && gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( height ) ) )
        {
          for( uint32_t layer = 0; layer < uiNumLayersToAllocateSplit; layer++ )
          {
            m_pSplitCS[width][height][layer]->destroy();

            delete m_pSplitCS[width][height][layer];
          }

          for( uint32_t layer = 0; layer < uiNumLayersToAllocateFull; layer++ )
          {
            m_pFullCS[width][height][layer]->destroy();

            delete m_pFullCS[width][height][layer];
          }

          delete[] m_pSplitCS[width][height];
          delete[] m_pFullCS [width][height];

          m_pBestCS[width][height]->destroy();
          m_pTempCS[width][height]->destroy();

          delete m_pTempCS[width][height];
          delete m_pBestCS[width][height];
        }
      }

      delete[] m_pSplitCS[width];
      delete[] m_pFullCS [width];

      delete[] m_pTempCS[width];
      delete[] m_pBestCS[width];
    }

    delete[] m_pSplitCS;
    delete[] m_pFullCS;

    delete[] m_pBestCS;
    delete[] m_pTempCS;

    delete[] m_pSaveCS;
  }

  m_pSplitCS = m_pFullCS = nullptr;

  m_pBestCS = m_pTempCS = nullptr;

  m_pSaveCS = nullptr;

  for( uint32_t ch = 0; ch < MAX_NUM_TBLOCKS; ch++ )
  {
    delete[] m_pSharedPredTransformSkip[ch];
    m_pSharedPredTransformSkip[ch] = nullptr;
  }

  m_tmpStorageLCU.destroy();
  m_colorTransResiBuf.destroy();
  m_isInitialized = false;
  if (m_indexError[0] != nullptr)
  {
    for (unsigned i = 0; i < (MAXPLTSIZE + 1); i++)
    {
      delete[] m_indexError[i];
      m_indexError[i] = nullptr;
    }
  }
  if (m_minErrorIndexMap != nullptr)
  {
    delete[] m_minErrorIndexMap;
    m_minErrorIndexMap = nullptr;
  }
  if (m_statePtRDOQ[0] != nullptr)
  {
    for (unsigned i = 0; i < NUM_TRELLIS_STATE; i++)
    {
      delete[] m_statePtRDOQ[i];
      m_statePtRDOQ[i] = nullptr;
    }
  }
}

IntraSearch::~IntraSearch()
{
  if( m_isInitialized )
  {
    destroy();
  }
}

void IntraSearch::init( EncCfg*        pcEncCfg,
                        TrQuant*       pcTrQuant,
                        RdCost*        pcRdCost,
                        CABACWriter*   CABACEstimator,
                        CtxCache*      ctxCache,
                        const uint32_t     maxCUWidth,
                        const uint32_t     maxCUHeight,
                        const uint32_t     maxTotalCUDepth
                       , EncReshape*   pcReshape
                       , const unsigned bitDepthY
)
{
  CHECK(m_isInitialized, "Already initialized");
  m_pcEncCfg                     = pcEncCfg;
  m_pcTrQuant                    = pcTrQuant;
  m_pcRdCost                     = pcRdCost;
  m_CABACEstimator               = CABACEstimator;
  m_CtxCache                     = ctxCache;
  m_pcReshape                    = pcReshape;

  const ChromaFormat cform = pcEncCfg->getChromaFormatIdc();

  IntraPrediction::init( cform, pcEncCfg->getBitDepth( CHANNEL_TYPE_LUMA ) );
  m_tmpStorageLCU.create(UnitArea(cform, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
  m_colorTransResiBuf.create(UnitArea(cform, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));

  for( uint32_t ch = 0; ch < MAX_NUM_TBLOCKS; ch++ )
  {
    m_pSharedPredTransformSkip[ch] = new Pel[MAX_CU_SIZE * MAX_CU_SIZE];
  }

  uint32_t numWidths  = gp_sizeIdxInfo->numWidths();
  uint32_t numHeights = gp_sizeIdxInfo->numHeights();

  const uint32_t uiNumLayersToAllocateSplit = 1;
  const uint32_t uiNumLayersToAllocateFull  = 1;

  m_pBestCS = new CodingStructure**[numWidths];
  m_pTempCS = new CodingStructure**[numWidths];

  m_pFullCS  = new CodingStructure***[numWidths];
  m_pSplitCS = new CodingStructure***[numWidths];

  for( uint32_t width = 0; width < numWidths; width++ )
  {
    m_pBestCS[width] = new CodingStructure*[numHeights];
    m_pTempCS[width] = new CodingStructure*[numHeights];

    m_pFullCS [width] = new CodingStructure**[numHeights];
    m_pSplitCS[width] = new CodingStructure**[numHeights];

    for( uint32_t height = 0; height < numHeights; height++ )
    {
      if(  gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( width ) ) && gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( height ) ) )
      {
        m_pBestCS[width][height] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
        m_pTempCS[width][height] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );

        m_pBestCS[width][height]->create(m_pcEncCfg->getChromaFormatIdc(), Area(0, 0, gp_sizeIdxInfo->sizeFrom(width), gp_sizeIdxInfo->sizeFrom(height)), false, (bool)pcEncCfg->getPLTMode());
        m_pTempCS[width][height]->create(m_pcEncCfg->getChromaFormatIdc(), Area(0, 0, gp_sizeIdxInfo->sizeFrom(width), gp_sizeIdxInfo->sizeFrom(height)), false, (bool)pcEncCfg->getPLTMode());

        m_pFullCS [width][height] = new CodingStructure*[uiNumLayersToAllocateFull];
        m_pSplitCS[width][height] = new CodingStructure*[uiNumLayersToAllocateSplit];

        for( uint32_t layer = 0; layer < uiNumLayersToAllocateFull; layer++ )
        {
          m_pFullCS [width][height][layer] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );

          m_pFullCS[width][height][layer]->create(m_pcEncCfg->getChromaFormatIdc(), Area(0, 0, gp_sizeIdxInfo->sizeFrom(width), gp_sizeIdxInfo->sizeFrom(height)), false, (bool)pcEncCfg->getPLTMode());
        }

        for( uint32_t layer = 0; layer < uiNumLayersToAllocateSplit; layer++ )
        {
          m_pSplitCS[width][height][layer] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
          m_pSplitCS[width][height][layer]->create(m_pcEncCfg->getChromaFormatIdc(), Area(0, 0, gp_sizeIdxInfo->sizeFrom(width), gp_sizeIdxInfo->sizeFrom(height)), false, (bool)pcEncCfg->getPLTMode());
        }
      }
      else
      {
        m_pBestCS[width][height] = nullptr;
        m_pTempCS[width][height] = nullptr;

        m_pFullCS [width][height] = nullptr;
        m_pSplitCS[width][height] = nullptr;
      }
    }
  }

  const int uiNumSaveLayersToAllocate = 2;

  m_pSaveCS = new CodingStructure*[uiNumSaveLayersToAllocate];

  for( uint32_t depth = 0; depth < uiNumSaveLayersToAllocate; depth++ )
  {
    m_pSaveCS[depth] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
    m_pSaveCS[depth]->create(UnitArea(cform, Area(0, 0, maxCUWidth, maxCUHeight)), false, (bool)pcEncCfg->getPLTMode());
  }

  m_isInitialized = true;
  if (pcEncCfg->getPLTMode())
  {
    if (m_indexError[0] == nullptr)
    {
      for (unsigned i = 0; i < (MAXPLTSIZE + 1); i++)
      {
        m_indexError[i] = new double[MAX_CU_BLKSIZE_PLT*MAX_CU_BLKSIZE_PLT];
      }
    }
    if (m_minErrorIndexMap == nullptr)
    {
      m_minErrorIndexMap = new uint8_t[MAX_CU_BLKSIZE_PLT*MAX_CU_BLKSIZE_PLT];
    }
    if (m_statePtRDOQ[0] == nullptr)
    {
      for (unsigned i = 0; i < NUM_TRELLIS_STATE; i++)
      {
        m_statePtRDOQ[i] = new uint8_t[MAX_CU_BLKSIZE_PLT*MAX_CU_BLKSIZE_PLT];
      }
    }
  }
}


//////////////////////////////////////////////////////////////////////////
// INTRA PREDICTION
//////////////////////////////////////////////////////////////////////////
static constexpr double COST_UNKNOWN = -65536.0;

double IntraSearch::findInterCUCost( CodingUnit &cu )
{
  if( cu.isConsIntra() && !cu.slice->isIntra() )
  {
    //search corresponding inter CU cost
    for( int i = 0; i < m_numCuInSCIPU; i++ )
    {
      if( cu.lumaPos() == m_cuAreaInSCIPU[i].pos() && cu.lumaSize() == m_cuAreaInSCIPU[i].size() )
      {
        return m_cuCostInSCIPU[i];
      }
    }
  }
  return COST_UNKNOWN;
}

#if GDR_ENABLED
int IntraSearch::getNumTopRecons(PredictionUnit &pu, int luma_dirMode, bool isChroma)
{
  int w = isChroma ? pu.Cb().width  : pu.Y().width;
  int h = isChroma ? pu.Cb().height : pu.Y().height;

  int numOfTopRecons = w;

  static const int angTable[32] = { 0,    1,    2,    3,    4,    6,     8,   10,   12,   14,   16,   18,   20,   23,   26,   29,   32,   35,   39,  45,  51,  57,  64,  73,  86, 102, 128, 171, 256, 341, 512, 1024 };
  static const int invAngTable[32] = {
    0,   16384, 8192, 5461, 4096, 2731, 2048, 1638, 1365, 1170, 1024, 910, 819, 712, 630, 565,
    512, 468,   420,  364,  321,  287,  256,  224,  191,  161,  128,  96,  64,  48,  32,  16
  };   // (512 * 32) / Angle

  const int refIdx             = pu.multiRefIdx;
  const int predModeIntra      = getModifiedWideAngle(w, h, luma_dirMode);
  const int isModeVer          = predModeIntra >= DIA_IDX;
  const int intraPredAngleMode = (isModeVer) ? predModeIntra - VER_IDX : -(predModeIntra - HOR_IDX);

  const int absAngMode         = abs(intraPredAngleMode);
  const int signAng            = intraPredAngleMode < 0 ? -1 : 1;
  const int absAng             = (luma_dirMode > DC_IDX && luma_dirMode < NUM_LUMA_MODE) ? angTable[absAngMode] : 0;

  const int invAngle           = invAngTable[absAngMode];
  const int intraPredAngle     = signAng * absAng;

  const int sideSize = isModeVer ? h : w;
  const int maxScale = 2;

  const int angularScale = std::min(maxScale, floorLog2(sideSize) - (floorLog2(3 * invAngle - 2) - 8));

  bool applyPDPC;


  // 1.0 derive PDPC
  applyPDPC  = (refIdx == 0) ? true : false;
  if (luma_dirMode > DC_IDX && luma_dirMode < NUM_LUMA_MODE)
  {
    if (intraPredAngleMode < 0)
    {
      applyPDPC &= false;
    }
    else if (intraPredAngleMode > 0)
    {
      applyPDPC &= (angularScale >= 0);
    }
  }

  // 2.0 calculate number of recons
  switch (luma_dirMode)
  {
  case PLANAR_IDX:
    numOfTopRecons = applyPDPC ? (w + 1) : (w + 1);
    break;

  case DC_IDX:
    numOfTopRecons = applyPDPC ? (w) : (w);
    break;

  case HOR_IDX:
    numOfTopRecons = applyPDPC ? (w) : (w);
    break;

  case VER_IDX:
    numOfTopRecons = applyPDPC ? (w) : (w);
    break;

  default:
    // 2..66
    // note: There should be a way to reduce the number of top recons, in case of non PDPC
    applyPDPC |= isChroma;

    if (predModeIntra >= DIA_IDX)
    {
      if (intraPredAngle < 0)
      {
        numOfTopRecons = (applyPDPC) ? (w + w) : (w + 1);
      }
      else
      {
        numOfTopRecons = (applyPDPC) ? (w + w) : (w + w);
      }
    }
    else
    {
      if (intraPredAngle < 0)
      {
        numOfTopRecons = (applyPDPC) ? (w + w) : (w);
      }
      else
      {
        numOfTopRecons = (applyPDPC) ? (w + w) : (w);
      }
    }
    break;
  }

  return numOfTopRecons;
}

bool IntraSearch::isValidIntraPredLuma(PredictionUnit &pu, int luma_dirMode)
{
  bool isValid  = true;
  PicHeader *ph = pu.cs->picHeader;

  if (ph->getInGdrInterval())
  {
    int x = pu.Y().x;

    // count num of recons on the top
    int virX             = ph->getVirtualBoundariesPosX(0);
    int numOfTopRecons = getNumTopRecons(pu, luma_dirMode, false);

    // check if recon is out of boundary
    if (x < virX && virX < (x + numOfTopRecons))
    {
      isValid = false;
    }
  }

  return isValid;
}

bool IntraSearch::isValidIntraPredChroma(PredictionUnit &pu, int luma_dirMode, int chroma_dirMode)
{
  bool isValid = true;
  CodingStructure *cs = pu.cs;
  PicHeader       *ph = cs->picHeader;

  if (ph->getInGdrInterval())
  {
    // note: chroma cordinate
    int cbX = pu.Cb().x;
    //int cbY = pu.Cb().y;
    int cbW = pu.Cb().width;
    int cbH = pu.Cb().height;

    int chromaScaleX = getComponentScaleX(COMPONENT_Cb, cs->area.chromaFormat);
    int chromaScaleY = getComponentScaleY(COMPONENT_Cb, cs->area.chromaFormat);

    int lumaX = cbX << chromaScaleX;
    // int lumaY = cbY << chromaScaleY;
    int lumaW = cbW << chromaScaleX;
    int lumaH = cbH << chromaScaleY;

    int numOfTopRecons = lumaW;
    int virX           = ph->getVirtualBoundariesPosX(0);

    // count num of recons on the top
    switch (chroma_dirMode)
    {

    case LM_CHROMA_IDX :
      numOfTopRecons = lumaW;
      break;

    case MDLM_L_IDX :
      numOfTopRecons = lumaW;
      break;

    // note: could reduce the actual #of
    case MDLM_T_IDX:
      numOfTopRecons = (lumaW + lumaH);
      break;

    case DM_CHROMA_IDX :
      numOfTopRecons = getNumTopRecons(pu, luma_dirMode, true) << chromaScaleX;
      break;

    default :
      numOfTopRecons = getNumTopRecons(pu, chroma_dirMode, true) << chromaScaleX;
      break;
    }

    // check if recon is out of boundary
    if (lumaX < virX && virX < (lumaX + numOfTopRecons))
    {
      isValid = false;
    }
  }

  return isValid;
}
#endif

 
/*
* 完成亮度分量的预测，并选出亮度预测的最佳模式
* MTS 多变换核选择
* LFNST 低频不可分变换
* SBT 子块变换
一、初始化各种参数；
二、为了减少最终RDcost的次数，降低编码端的复杂度，estIntraPredLumaQT使用帧内快速搜索算法，主要包含三个阶段：
   1、RMD：为了减少运算复杂度，直接对残差进行哈达玛变换求得SATD失真计算代价，将最小代价前几的模式加入全率失真模式列表中（不进行变换量化等操作）
   2、MPM：根据相邻块推导出当前块的最可能模式，加入到全率失真模式列表中。
   3、RDO：遍历全率失真模式列表中的模式，进行完整的变换、量化、反变换、反量化等操作，计算失真和比特并求出RD
Cost，选择代价最小的模式作为当前块的最优角度模式。

具体过程如下：
  1、第一轮SATD粗选：对35种非扩展的传统的亮度帧内角度预测模式进行第一轮SATD循环粗选，这里粗选的时候使用predIntraAng函数计算预测值，然后计算SAD和SATD（HAD哈达玛变换后的SAD），并选其最小值计算每种模式的Cost，然后调用updateCandList函数更新全率失真候选模式列表uiRdModeList，候选列表长度为numModesForFullRD。

  2、第二轮SATD粗选：

  （1）对第一轮选出的numModesForFullRD种非扩展角度模式，分别对其左右相邻的扩展角度模式使用predIntraAng函数计算预测值，然后计算SAD和SATD，并选其最小值计算每种模式的Cost，然后调用updateCandList函数更新全率失真候选模式列表uiRdModeList，候选列表长度为numModesForFullRD。（numModesForFullRD通常为3，使用MIP时翻倍为6）

  （2）遍历多参考行MRL模式，使用getIntraMPMs函数获取MPM列表，然后对每个扩展的参考行（1、2）进行遍历，选出最有可能的几种模式加入到全率失真优化列表中。粗选过程与之前类似，循环遍历MPM列表中的每种模式，再调用predIntraAng函数计算预测值，然后计算SAD和SATD，并选其最小值计算每种模式的Cost，调用updateCandList函数更新全率失真候选模式列表uiRdModeList。

  （3）使用哈达玛变换对MIP模式单独进行候选模式的推导。根据当前CU块的尺寸获取到MIP的模式数，然后对每一种模式调用predIntraMip函数计算预测值，然后计算SAD和SATD，并选其最小值计算每种模式的Cost，再调用updateCandList函数更新全率失真候选模式列表uiRdModeList。在这之后，调用reduceHadCandList函数对全率失真候选模式列表uiRdModeList进行缩减处理。

  （4）获取FastUDI最可能的MPM模式列表，仅遍历左相邻CU和上相邻CU的模式，判断它们是否在全率失真候选列表中，若不在则将其加入进去。同样地，对ISP模式列表m_ispCandListHor进行相同的处理。

  （5）如果需要测试ISP模式，则需为ISP在全率失真候选模式列表中保留位置，即将所有的ISP模式添加进uiRdModeList。

  3、第三轮RD
Cost细选：遍历RD候选模式列表，如果当前模式为ISP模式，则将ISP列表重新排列（仅排列一次），然后调用xIntraCodingLumaISP函数进行ISP模式的残差变换、量化并计算RD
Cost；否则调用xRecurIntraCodingLumaQT函数进行普通的残差变换、量化并计算RD Cost。

  4、收尾工作：判断最优模式是否是MIP以及是否使用多参考行，并将最优模式保存入pu.intraDir里。

MIP和角度预测是竞争关系，最后选出的最优模式是代价最小的角度预测模式或者MIP模式。
*/

bool IntraSearch::estIntraPredLumaQT(CodingUnit &cu, Partitioner &partitioner, const double bestCostSoFar, bool mtsCheckRangeFlag, int mtsFirstCheckId, int mtsLastCheckId, bool moreProbMTSIdxFirst, CodingStructure* bestCS)
{
  // mtsFirstCheckId和mtsLastCheckId对应于上层的startMTSIdx[trGrpIdx], endMTSIdx[trGrpIdx]，
  // 进行的是变换核的选择 
  // trGrpIdx也就是xCheckRDCostIntra中第一个for循环遍历的4个，进行MTS变换核的选择
  // 后边可能出现检查/check的字眼，说的是正在进行筛选的是xCheckRDCostIntra中for循环遍历到的变换类型
  CodingStructure       &cs            = *cu.cs; // 当前CU的编码结构
  const SPS             &sps           = *cs.sps;  //序列参数集
 
  const uint32_t         logWidth      = floorLog2(partitioner.currArea().lwidth());
  const uint32_t         logHeight     = floorLog2(partitioner.currArea().lheight());

  // Lambda calculation at equivalent Qp of 4 is recommended because at that Qp, the quantization divisor is 1.

  // 建议在等效qp为4时进行lambda的计算，因为在该qp时，用于量化的除数为1。
  const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda( ) * FRAC_BITS_SCALE;

  //===== loop over partitions =====
  // 上下文模型更新一些参数信息
  const TempCtx ctxStart          ( m_CtxCache, m_CABACEstimator->getCtx() );
  // MIP模式
  const TempCtx ctxStartMipFlag    ( m_CtxCache, SubCtx( Ctx::MipFlag,          m_CABACEstimator->getCtx() ) );
  // ISP模式
  const TempCtx ctxStartIspMode    ( m_CtxCache, SubCtx( Ctx::ISPMode,          m_CABACEstimator->getCtx() ) );
  // Planar模式
  const TempCtx ctxStartPlanarFlag ( m_CtxCache, SubCtx( Ctx::IntraLumaPlanarFlag, m_CABACEstimator->getCtx() ) );
  // 其他帧内模式
  const TempCtx ctxStartIntraMode(m_CtxCache, SubCtx(Ctx::IntraLumaMpmFlag, m_CABACEstimator->getCtx()));
  // 多参考行索引
  const TempCtx ctxStartMrlIdx      ( m_CtxCache, SubCtx( Ctx::MultiRefLineIdx,        m_CABACEstimator->getCtx() ) );

  CHECK( !cu.firstPU, "CU has no PUs" );
  // variables for saving fast intra modes scan results across multiple LFNST passes


  /*
  * LFNST快速算法LFNSTLoadFlag、LFNSTSaveFlag两个标志位
  * lfnstIdx=0需要进行完整的一轮、二轮粗选和RD cost细选过程，并将第一轮SATD粗选非扩展模式
  * 第二轮SATD粗选扩展模式、MRL的MPM列表及MIP模式的全率失真候选列表保存下来。
  * 当lfnstIdx=1，2时直接加载lfnstIdx=0保存的全率失真候选列表，并且不再将ISP模式添加进候选模式列表，
  * 这样可以大大降低编码端的复杂度
  */
  bool LFNSTLoadFlag = sps.getUseLFNST() && cu.lfnstIdx != 0;
  bool LFNSTSaveFlag = sps.getUseLFNST() && cu.lfnstIdx == 0;

  // LFNST是二次变换，在DCT-2后使用，MTS是一次变换，和LFNST互斥
  // 判断当前是否检查的是DCT-2，SPS没有开启帧内MTS，LFNSTSaveFlag=true，
  // 说明变换是DCT2，就要把DCT-2的结果保存下来给lfnst后边的变换使用
  // SPS开启MTS，cu.mtsFlag==0说明当前遍历的是不使用LFNST的，那就是正常的DCT-2变换或变换跳过
  // cu.mtsFlag=1说明使用了lfnst内核进行变换，这时候cu.mtsFlag == 0为false，所以不需要保存DCT-2的结果
  LFNSTSaveFlag &= sps.getUseIntraMTS() ? cu.mtsFlag == 0 : true;

  const uint32_t lfnstIdx = cu.lfnstIdx; // 当前CU使用的LFNST的索引，分别为0/1/2
  double costInterCU = findInterCUCost( cu ); // 帧间CU的cost
  
  // 当前亮度CU的宽和高
  const int width  = partitioner.currArea().lwidth();
  const int height = partitioner.currArea().lheight();

  // Marking MTS usage for faster MTS
  // 0: MTS is either not applicable for current CU (cuWidth > MTS_INTRA_MAX_CU_SIZE or cuHeight > MTS_INTRA_MAX_CU_SIZE), not active in the config file or the fast decision algorithm is not used in this case
  // 1: MTS fast algorithm can be applied for the current CU, and the DCT2 is being checked
  // 2: MTS is being checked for current CU. Stored results of DCT2 can be utilized for speedup

  // 标记 MTS 使用情况以加快 MTS 
  // 0：MTS 不适用于当前CU(长宽最小尺寸大于32)，在配置文件中未激活或在此情况下未使用快速决策算法 
  // 1：MTS快速算法可应用于当前CU，当前CU正在检查 DCT2 
  // 2：正在检查当前 CU 的 MTS，可以利用存储的 DCT2 结果进行加速(二次变换)
  uint8_t mtsUsageFlag = 0;
  const int maxSizeEMT = MTS_INTRA_MAX_CU_SIZE;

  /*
  * DCT-2变换和MTS变换，为降低复杂度，使用mtsUsageFlag标记MTS快速算法
  对于DCT-2变换和MTS变换，为降低复杂度，使用mtsUsageFlag变量标记MTS以实现快速算法。
  mtsUsageFlag=0时，表示MTS不适用于当前CU，即一次变换只需要检查DCT-2，此时需要进行第一轮SATD粗选和第二轮SATD粗选，并将粗选结果保存下来。
  mtsUsageFlag=1时，表示当前CU可以使用DCT-2和MTS，且当前正在检查DCT-2，此时需要进行第一轮SATD粗选和第二轮SATD粗选，并将粗选过程保存下来。
  mtsUsageFlag=2时，表示当前CU正在检查MTS，这里由于之前检查DCT-2的时候已经进行了一轮和二轮的SATD粗选，所以可以直接将粗选结果加载出来。
  */
  // 当前CU最大尺寸小于等于32且SPS开启MTS
  if( width <= maxSizeEMT && height <= maxSizeEMT && sps.getUseIntraMTS() )
  {
    // SPS层开启LFNST，当前正在检查的是MTS，则mtsUsageFlag为2，否则为1
    mtsUsageFlag = ( sps.getUseLFNST() && cu.mtsFlag == 1 ) ? 2 : 1;
  }

  // CU总像素小于64且不开启LFNST, 不使用MTS
  if( width * height < 64 && !m_pcEncCfg->getUseFastLFNST() )
  {
    mtsUsageFlag = 0;
  }

  // 单树或ISP模式会关闭ACT。这里只考虑ACT关闭的状况
  // ACT是将原始颜色空间中的预测残差自适应地转换到另一个颜色空间中，
  // 以减少 4:4:4 色度格式视频序列的三个颜色分量之间的相关性。
  // 开启ACT变换且不是双树
  const bool colorTransformIsEnabled = sps.getUseColorTrans() && !CS::isDualITree(cs);
  // 是否是第一种颜色空间
  const bool isFirstColorSpace       = colorTransformIsEnabled && ((m_pcEncCfg->getRGBFormatFlag() && cu.colorTransform) || (!m_pcEncCfg->getRGBFormatFlag() && !cu.colorTransform));
  // 是否第二种颜色空间
  const bool isSecondColorSpace      = colorTransformIsEnabled && ((m_pcEncCfg->getRGBFormatFlag() && !cu.colorTransform) || (!m_pcEncCfg->getRGBFormatFlag() && cu.colorTransform));
  
  // 定义一个当前最优的代价,这是前边已经进行过RDO计算得到的最优cost
  double bestCurrentCost = bestCostSoFar;

  // 是否可以使用ISP：SPS层开启ISP且不使用MTS，且lfnstIdx=0且尺寸符合ISP的限制
  bool ispCanBeUsed   = sps.getUseISP() && cu.mtsFlag == 0 && cu.lfnstIdx == 0 && CU::canUseISP(width, height, cu.cs->sps->getMaxTbSize());

  // 有ISP模式就不能使用ACT
  bool saveDataForISP = ispCanBeUsed && (!colorTransformIsEnabled || isFirstColorSpace);
  // 测试ISP
  bool testISP        = ispCanBeUsed && (!colorTransformIsEnabled || !cu.colorTransform);

  if ( saveDataForISP )
  {
    //reset the intra modes lists variables
    // 重置ISP帧内模式列表变量
    // ISP水平或垂直划分的候选列表
    m_ispCandListHor.clear();
    m_ispCandListVer.clear();
  }
  // ISP模式初始化
  // ISP预测模式主要是将当前块划分为多个子块，每个子块的重建信号可用于生成下一个子块的预测值。
  // 所有子块的预测模式都相同，且只能是PLANAR模式或者DC模式或者帧内角度模式。
  if( testISP )
  {
    //reset the variables used for the tests
    // 重置用于测试的变量
    m_regIntraRDListWithCosts.clear();
    // 水平划分所得子分区数
    int numTotalPartsHor = (int)height >> floorLog2(CU::getISPSplitDim(width, height, TU_1D_HORZ_SPLIT));
    // 垂直划分所得子分区数
    int numTotalPartsVer = (int)width  >> floorLog2(CU::getISPSplitDim(width, height, TU_1D_VERT_SPLIT));//初始化LfnstIdx为0时ISP测试数据

    m_ispTestedModes[0].init( numTotalPartsHor, numTotalPartsVer );
    //the total number of subpartitions is modified to take into account the cases where LFNST cannot be combined with ISP due to size restrictions
    
    // ISP只能垂直或水平划分CU
    // 根据当前CU划分后TU的尺寸的大小是否满足LFNST使用条件，决定是否跳过该(划分方式和LFNST的组合)的检查。
    // 划分后仍然能满足LFNST的情况就使用，否则这种划分模式为0也就是不考虑划分
    numTotalPartsHor = sps.getUseLFNST() && CU::canUseLfnstWithISP(cu.Y(), HOR_INTRA_SUBPARTITIONS) ? numTotalPartsHor : 0;
    numTotalPartsVer = sps.getUseLFNST() && CU::canUseLfnstWithISP(cu.Y(), VER_INTRA_SUBPARTITIONS) ? numTotalPartsVer : 0;

    for (int j = 1; j < NUM_LFNST_NUM_PER_SET; j++)
    {
      m_ispTestedModes[j].init(numTotalPartsHor, numTotalPartsVer);
    }
  } 

  // BDPCM:Block-based Differential Pulse-Code Modulation，区块差分脉冲编码调制模式，
  // 应用在帧内预测的模式。应用时，都会推断成Transform Skip，并且有横向的或者是纵向的区别。
  // 纵向模式时候，解码过程中残差r(i,j)会减去上一行的重构值

  const bool testBDPCM = sps.getBDPCMEnabledFlag() && CU::bdpcmAllowed(cu, ComponentID(partitioner.chType)) && cu.mtsFlag == 0 && cu.lfnstIdx == 0;

  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> hadModeList;//哈达玛变换（即SATD）粗选后的模式列表
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM>   candCostList;//代价候选列表
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM>   candHadList;//哈达玛变换（SATD/HAD）后候选失真列表

  auto &pu = *cu.firstPU;
#if GDR_ENABLED
  const bool isEncodeGdrClean = cs.sps->getGDREnabledFlag() && cs.pcv->isEncoder && ((cs.picHeader->getInGdrInterval() && cs.isClean(pu.Y().topRight(), CHANNEL_TYPE_LUMA)) || (cs.picHeader->getNumVerVirtualBoundaries() == 0));
#endif
  bool validReturn = false;
  {
    candHadList.clear();
    candCostList.clear();
    hadModeList.clear();

    CHECK(pu.cu != &cu, "PU is not contained in the CU");

    //===== determine set of modes to be tested (using prediction signal only) =====
    //===== 确定要测试的模式集（仅使用预测信号）======
    // 所有67种传统帧内模式的数量，不包括MIP模式，MIP模式另行处理
    int numModesAvailable = NUM_LUMA_MODE; // total number of Intra modes

    const bool fastMip    = sps.getUseMIP() && m_pcEncCfg->getUseFastMIP(); // MIP快速算法
    // MIP允许使用标志
    const bool mipAllowed = sps.getUseMIP() && isLuma(partitioner.chType) && ((cu.lfnstIdx == 0) || allowLfnstWithMip(cu.firstPU->lumaSize()));

    // MIP模式测试标志：宽高比不能超过1:8/8:1
    const bool testMip = mipAllowed && !(cu.lwidth() > (8 * cu.lheight()) || cu.lheight() > (8 * cu.lwidth()));
    // 判断当前PU块是否支持MIP模式
    const bool supportedMipBlkSize = pu.lwidth() <= MIP_MAX_WIDTH && pu.lheight() <= MIP_MAX_HEIGHT;

    static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> rdModeList; // 全率失真候选模式列表

    // 能够进入到最终的RDcost环节的最优的几种模式的数量，根据CU的尺寸选择为2还是3
    int numModesForFullRD = g_intraModeNumFastUseMPM2D[logWidth - MIN_CU_LOG2][logHeight - MIN_CU_LOG2];

#if INTRA_FULL_SEARCH
    numModesForFullRD = numModesAvailable;
#endif
    // 第二种颜色空间，直接使用第一种颜色空间保存的模式
    if (isSecondColorSpace)
    {
      rdModeList.clear();
      // 直接从第一个颜色空间的模式列表中加载测试的模式列表
      if (m_numSavedRdModeFirstColorSpace[m_savedRdModeIdx] > 0)
      {
        for (int i = 0; i < m_numSavedRdModeFirstColorSpace[m_savedRdModeIdx]; i++)
        {
          rdModeList.push_back(m_savedRdModeFirstColorSpace[m_savedRdModeIdx][i]);
        }
      }
      else
      {
        return false;
      }
    }
    else
    {
      // mtsUsageFlag的只来保存或加载数据实现快速算法
      // mtsUsageFlag != 2，需要保存前两轮SATD粗选结果 
      if (mtsUsageFlag != 2) // 一次变换使用DCT-2
      {
        // this should always be true
        CHECK(!pu.Y().valid(), "PU is not valid");
        bool isFirstLineOfCtu     = (((pu.block(COMPONENT_Y).y) & ((pu.cs->sps)->getMaxCUWidth() - 1)) == 0);
        // 使用的扩展参考行的数量，CTU的第一行或者SPS层关闭MRL，则最多只能用1行，否则最多可以用三行
        int  numOfPassesExtendRef = ((!sps.getUseMRL() || isFirstLineOfCtu) ? 1 : MRL_NUM_REF_LINES);
        // 先对参考行0进行处理，参考行1，2使用MPM列表
        pu.multiRefIdx            = 0;

        // numModesForFullRD候选列表长度根据块大小初始化为3/2。这里是主要的RD的入口
        if (numModesForFullRD != numModesAvailable)
        {
          CHECK(numModesForFullRD >= numModesAvailable, "Too many modes for full RD search");

          const CompArea &area = pu.Y(); 

          PelBuf piOrg  = cs.getOrgBuf(area); // 当前的原始像素
          PelBuf piPred = cs.getPredBuf(area); // 当前PU的预测值

          DistParam distParamSad; // SAD失真
          DistParam distParamHad; // SAD进行哈达玛变换后的失真SATD

          // 如果使用LMCS对原始像素进行处理
          if (cu.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
          {
            // 原始亮度区域的拷贝 
            CompArea tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
            PelBuf   tmpOrg = m_tmpStorageLCU.getBuf(tmpArea);
            tmpOrg.copyFrom(piOrg);

            tmpOrg.rspSignal(m_pcReshape->getFwdLUT());
            // 计算SAD和SATD失真
            m_pcRdCost->setDistParam(distParamSad, tmpOrg, piPred, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, false);   // Use SAD cost
            m_pcRdCost->setDistParam(distParamHad, tmpOrg, piPred, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, true);   // Use HAD (SATD) cost
          }
          else
          {
            // 设置计算失真SAD和SATD相关的参数信息
            m_pcRdCost->setDistParam(distParamSad, piOrg, piPred, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, false);   // Use SAD cost
            m_pcRdCost->setDistParam(distParamHad, piOrg, piPred, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, true);   // Use HAD (SATD) cost
          }

          distParamSad.applyWeight = false;
          distParamHad.applyWeight = false;

          // numModesForFullRD最开始初始化为2/3
          // 如果使用MIP快速算法且块大小允许使用MIP，添加MIP到候选模式列表中
          if (testMip && supportedMipBlkSize)
          {
            numModesForFullRD += fastMip 
              ? std::max(numModesForFullRD, floorLog2(std::min(pu.lwidth(), pu.lheight())) - 1) 
              : numModesForFullRD;
          } // numModesForFullRD这里增大

          // SATD变换候选数量，MIP模式就是6，否则是3
          const int numHadCand = (testMip ? 2 : 1) * 3;

          //*** Derive (regular) candidates using Hadamard
          //*** 使用哈达玛变换的推导角度候选（即使用SATD）
          cu.mipFlag = false; // MIP标志位先为false，因为先进行的是角度模式的，MIP和这些是独立的

          //===== init pattern for luma prediction =====
          // 传forceRefFilterFlag=true，里边if (!forceRefFilterFlag)为false
          // 即在这里不进行参数的初始化及角度的偏移等计算，只是获取参考像素和参考像素的平滑滤波
          // 初始化参数到后边进行筛选的时候才进行
          initIntraPatternChType(cu, pu.Y(), true);
          bool satdChecked[NUM_INTRA_MODE]; // 67种传统+3种CCLM(LMC + MDLM_T + MDLM_L)
          std::fill_n(satdChecked, NUM_INTRA_MODE, false); //存放已经被STAD粗选过的帧内模式的bool数组

          // 第一轮SATD粗选，使用DCT-2变换，还没有LFNST
          if (!LFNSTLoadFlag)
          {
            // 第一轮SATD粗筛，筛选67种模式种的35种(HEVC有的35种)模式
            for (int modeIdx = 0; modeIdx < numModesAvailable; modeIdx++)
            {
              uint32_t   mode      = modeIdx;
              Distortion minSadHad = 0;

              // Skip checking extended Angular modes in the first round of SATD
              // 第一轮SATD检查对HEVC的35种模式进行筛选
              if (mode > DC_IDX && (mode & 1))
              {
                continue;
              }
              // 对第一轮筛选过的标记为true
              satdChecked[mode] = true;
              // 当前CU最终选中的预测模式
              pu.intraDir[0] = modeIdx; // 这里给了PU的模式来进行预测
              // 初始化帧内预测参数，根据模式计算角度偏移，判断模式PDPC是否可用
              // 判断是否对参考像素进行滤波的代码
              initPredIntraParams(pu, pu.Y(), sps);

              // 传统角度预测模式(Planar, DC, 角度预测)的函数入口，根据模式获取预测像素piPred并使用PDPC平滑滤波
              // 在计算预测值的时候，由于一些角度模式会使用到非整数位置处的像素，亮度分量的非整数位置会使用
              // 高斯插值滤波器或三次插值滤波器对非整数位置像素插值，色度分量使用HEVC的两抽头插值滤波器
              predIntraAng(COMPONENT_Y, piPred, pu);
              // Use the min between SAD and HAD as the cost criterion
              // SAD is scaled by 2 to align with the scaling of HAD
              // 使用SAD和HAD(SATD, SAD经过哈达玛变换)之间的最小值作为代价标准
              // SAD的缩放比例为2，与HAD的缩放比例一致。
              minSadHad += std::min(distParamSad.distFunc(distParamSad) * 2, distParamHad.distFunc(distParamHad));

              // NB xFracModeBitsIntra will not affect the mode for chroma that may have already been pre-estimated.
              // NB xFracModeBitsIntra不会影响可能已经预先估计的色度模式。
              m_CABACEstimator->getCtx() = SubCtx( Ctx::MipFlag, ctxStartMipFlag );
              m_CABACEstimator->getCtx() = SubCtx( Ctx::ISPMode, ctxStartIspMode );
              m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaPlanarFlag, ctxStartPlanarFlag);
              m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaMpmFlag, ctxStartIntraMode);
              m_CABACEstimator->getCtx() = SubCtx( Ctx::MultiRefLineIdx, ctxStartMrlIdx );

              // 估计帧内编码所需要的比特数
              uint64_t fracModeBits = xFracModeBitsIntra(pu, mode, CHANNEL_TYPE_LUMA);

              // 计算每种模式的Cost，比较相互之间的Cost进行模式更新
              double cost = (double) minSadHad + (double) fracModeBits * sqrtLambdaForFirstPass;

              DTRACE(g_trace_ctx, D_INTRA_COST, "IntraHAD: %u, %llu, %f (%d)\n", minSadHad, fracModeBits, cost, mode);

#if GDR_ENABLED
              if (isEncodeGdrClean)
              {
                if (isValidIntraPredLuma(pu, mode))
                {
                  updateCandList(ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, mode), cost, rdModeList,
                                 candCostList, numModesForFullRD);
                  updateCandList(ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, mode), double(minSadHad),
                                 hadModeList, candHadList, numHadCand);
                }
              }
              else
              {
                updateCandList(ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, mode), cost, rdModeList, candCostList,
                               numModesForFullRD);
                updateCandList(ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, mode), double(minSadHad), hadModeList,
                               candHadList, numHadCand);
              }
#else
              /*
              模式一个列表，模式对应的代价一个列表
              numModesForFullRD 最终RDcost环节的最优的几种模式的数量
              rdModeList：全率失真候选模式列表
              candCostList：全率失真代价候选代价列表
              
              hadModeList： 哈达玛变换(SATD)粗选后的模式候选列表
              candHadList： 哈达玛变换后的候选代价列表
              */
              // 更新SAD之后的RD候选模式列表以及每种模式对应代价列表
              updateCandList(ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, mode), cost, rdModeList, candCostList, numModesForFullRD);

              // 更新SATD之后的模式列表HadModeList、以及对应最小失真列表CandHadList
              updateCandList(ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, mode), double(minSadHad), hadModeList, candHadList, numHadCand);
#endif
            }// 遍历67种模式，粗筛其中的35种

            // 保存第一轮SATD粗选的结果给LFNST
            if (!sps.getUseMIP() && LFNSTSaveFlag)
            {
              // save found best modes
              // 保留第一轮筛选的模式及代价列表
              m_savedNumRdModesLFNST = numModesForFullRD;
              m_savedRdModeListLFNST = rdModeList;
              m_savedModeCostLFNST   = candCostList;
              // PBINTRA fast
              // 保留第一轮粗筛的STAD模式及失真列表
              m_savedHadModeListLFNST   = hadModeList;
              m_savedHadListLFNST       = candHadList;
              LFNSTSaveFlag             = false;
            }
          }   // NSSTFlag

          // 加载为lfnst保存的第一轮粗选结果
          if (!sps.getUseMIP() && LFNSTLoadFlag)
          {
            // restore saved modes
            numModesForFullRD = m_savedNumRdModesLFNST;
            rdModeList        = m_savedRdModeListLFNST;
            candCostList      = m_savedModeCostLFNST;
            // PBINTRA fast
            hadModeList = m_savedHadModeListLFNST;
            candHadList = m_savedHadListLFNST;
          }   // !LFNSTFlag

          // sps.getUseMIP() && LFNSTLoadFlag的结果为false
          // 即使用MIP和加载一轮的数据有1/2不成立
          // 第二轮SATD进行粗选
          if (!(sps.getUseMIP() && LFNSTLoadFlag))
          {
            // 第一轮粗选的结果保存为父模式列表种
            static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> parentCandList = rdModeList;
            // Second round of SATD for extended Angular modes

#if GDR_ENABLED
            int nn = numModesForFullRD;
            if (isEncodeGdrClean)
            {
              nn = std::min((int)numModesForFullRD, (int)parentCandList.size());
            }

            for (int modeIdx = 0; modeIdx < nn; modeIdx++)
#else

            // 遍历67种角度模式，对多参考行的0进行
            // 开始对扩展角度模式进行第二轮第一次SATD粗选
            for (int modeIdx = 0; modeIdx < numModesForFullRD; modeIdx++)
#endif
            {
              // 选择一个父类模式，对其进行
              unsigned parentMode = parentCandList[modeIdx].modeId;
              // 这里对第一轮筛选的左右进行筛选3-65
              if (parentMode > (DC_IDX + 1) && parentMode < (NUM_LUMA_MODE - 1))
              {
                // 分别选择第一轮筛选模式的非扩展模式左右两个扩展模式
                for (int subModeIdx = -1; subModeIdx <= 1; subModeIdx += 2)
                {
                  unsigned mode = parentMode + subModeIdx;
                  if (!satdChecked[mode])
                  {
                    pu.intraDir[0] = mode; // 这里给了PU的模式进行预测

                    // 后边是使用上边给的那个模式来进行预测
                    initPredIntraParams(pu, pu.Y(), sps);
                    predIntraAng(COMPONENT_Y, piPred, pu);

                    // Use the min between SAD and SATD as the cost criterion
                    // SAD is scaled by 2 to align with the scaling of HAD
                    // SAD按2缩放以与HAD(SATD)的缩放对齐
                    Distortion minSadHad =
                      std::min(distParamSad.distFunc(distParamSad) * 2, distParamHad.distFunc(distParamHad));

                    // NB xFracModeBitsIntra will not affect the mode for chroma that may have already been
                    // pre-estimated.
                    m_CABACEstimator->getCtx() = SubCtx(Ctx::MipFlag, ctxStartMipFlag);
                    m_CABACEstimator->getCtx() = SubCtx(Ctx::ISPMode, ctxStartIspMode);
                    m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaPlanarFlag, ctxStartPlanarFlag);
                    m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaMpmFlag, ctxStartIntraMode);
                    m_CABACEstimator->getCtx() = SubCtx(Ctx::MultiRefLineIdx, ctxStartMrlIdx);

                    uint64_t fracModeBits = xFracModeBitsIntra(pu, mode, CHANNEL_TYPE_LUMA);
                    // 计算每种是模式的代价
                    double cost = (double) minSadHad + (double) fracModeBits * sqrtLambdaForFirstPass;

#if GDR_ENABLED
                    if (isEncodeGdrClean)
                    {
                      if (isValidIntraPredLuma(pu, mode))
                      {
                        updateCandList(ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, mode), cost, rdModeList,
                                       candCostList, numModesForFullRD);
                        updateCandList(ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, mode), double(minSadHad),
                                       hadModeList, candHadList, numHadCand);
                      }
                    }
                    else
                    {
                      updateCandList(ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, mode), cost, rdModeList,
                                     candCostList, numModesForFullRD);
                      updateCandList(ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, mode), double(minSadHad),
                                     hadModeList, candHadList, numHadCand);
                    }
#else
                    // 更新列表SAD的模式以及代价列表
                    updateCandList(ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, mode), cost, rdModeList, candCostList, numModesForFullRD);
                    // 更新SATD的模式及失真列表
                    updateCandList(ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, mode), double(minSadHad), hadModeList, candHadList, numHadCand);
#endif

                    satdChecked[mode] = true;
                  }
                }
              }
            }

            // 为ISP粗选的传统帧内预测模式的粗选结果
            if (saveDataForISP)
            {
              // we save the regular intra modes list
              // 保存常规的帧内模式列表
              m_ispCandListHor = rdModeList;
            }

            // 第一轮对参考行0的HEVC的35种模式进行SATD粗筛
            // 第二轮第一次对第一轮筛选出来模式的左右扩展模式进行SATD筛选
            // 第二轮第二次对参考行1和2进行MPM列表的筛选
            pu.multiRefIdx    = 1; // MRL的参考行
            const int numMPMs = NUM_MOST_PROBABLE_MODES; // 最可能模式的数量，即MPM的大小 6
            unsigned  multiRefMPM[numMPMs]; // 多参考行的MPM列表
            // 根据左边和上边的模式得到最可能列表
            PU::getIntraMPMs(pu, multiRefMPM);

            // 标号为0的参考行已经进行全角度的计算
            // 遍历MRL的第1和2行只使用MPM列表的6种帧内模式进行SATD粗选
            for (int mRefNum = 1; mRefNum < numOfPassesExtendRef; mRefNum++)
            {
              int multiRefIdx = MULTI_REF_LINE_IDX[mRefNum];
              // 这里还需要获取参考像素是因为上边那次获取参考像素并滤波是对参考行0来进行的，这里对参考行1和2进行
              pu.multiRefIdx = multiRefIdx; // 当前参考行索引
              {
                initIntraPatternChType(cu, pu.Y(), true); //获取参考像素并滤波
              }
              // 遍历MPM列表种的6种模式进行SATD比较选择
              for (int x = 1; x < numMPMs; x++)
              {
                uint32_t mode = multiRefMPM[x];
                {
                  pu.intraDir[0] = mode;

                  initPredIntraParams(pu, pu.Y(), sps);
                  predIntraAng(COMPONENT_Y, piPred, pu);

                  // Use the min between SAD and SATD as the cost criterion
                  // SAD is scaled by 2 to align with the scaling of HAD
                  Distortion minSadHad =
                    std::min(distParamSad.distFunc(distParamSad) * 2, distParamHad.distFunc(distParamHad));

                  // NB xFracModeBitsIntra will not affect the mode for chroma that may have already been pre-estimated.
                  m_CABACEstimator->getCtx() = SubCtx(Ctx::MipFlag, ctxStartMipFlag);
                  m_CABACEstimator->getCtx() = SubCtx(Ctx::ISPMode, ctxStartIspMode);
                  m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaPlanarFlag, ctxStartPlanarFlag);
                  m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaMpmFlag, ctxStartIntraMode);
                  m_CABACEstimator->getCtx() = SubCtx(Ctx::MultiRefLineIdx, ctxStartMrlIdx);

                  uint64_t fracModeBits = xFracModeBitsIntra(pu, mode, CHANNEL_TYPE_LUMA);

                  double cost = (double) minSadHad + (double) fracModeBits * sqrtLambdaForFirstPass;
#if GDR_ENABLED
                  if (isEncodeGdrClean)
                  {
                    if (isValidIntraPredLuma(pu, mode))
                    {
                      updateCandList(ModeInfo(false, false, multiRefIdx, NOT_INTRA_SUBPARTITIONS, mode), cost,
                                     rdModeList, candCostList, numModesForFullRD);
                      updateCandList(ModeInfo(false, false, multiRefIdx, NOT_INTRA_SUBPARTITIONS, mode),
                                     double(minSadHad), hadModeList, candHadList, numHadCand);
                    }
                  }
                  else
                  {
                    updateCandList(ModeInfo(false, false, multiRefIdx, NOT_INTRA_SUBPARTITIONS, mode), cost, rdModeList,
                                   candCostList, numModesForFullRD);
                    updateCandList(ModeInfo(false, false, multiRefIdx, NOT_INTRA_SUBPARTITIONS, mode),
                                   double(minSadHad), hadModeList, candHadList, numHadCand);
                  }
#else
                  // 更新模式候选列表
                  updateCandList(ModeInfo(false, false, multiRefIdx, NOT_INTRA_SUBPARTITIONS, mode), cost, rdModeList,
                                 candCostList, numModesForFullRD);
                  updateCandList(ModeInfo(false, false, multiRefIdx, NOT_INTRA_SUBPARTITIONS, mode), double(minSadHad),
                                 hadModeList, candHadList, numHadCand);
#endif
                }
              }// for (int x = 1; x < numMPMs; x++)遍历MPM列表
            } // for (int mRefNum = 1; mRefNum < numOfPassesExtendRef; mRefNum++) 遍历参考行
#if GDR_ENABLED
            if (!isEncodeGdrClean)
            {
              CHECKD(rdModeList.size() != numModesForFullRD, "Error: RD mode list size");
            }
#else
            CHECKD(rdModeList.size() != numModesForFullRD, "Error: RD mode list size");
#endif
            // save a different set for the next run
            // 如果使用了MIP，在给LFNST保存结果的时候也会保存MIP的粗选结果，同时对列表的大小进行修改
            if (LFNSTSaveFlag && testMip && !allowLfnstWithMip(cu.firstPU->lumaSize()))   
            {
              // save found best modes
              m_savedRdModeListLFNST = rdModeList;
              m_savedModeCostLFNST   = candCostList;
              // PBINTRA fast
              m_savedHadModeListLFNST   = hadModeList;
              m_savedHadListLFNST       = candHadList;
              m_savedNumRdModesLFNST    = g_intraModeNumFastUseMPM2D[logWidth - MIN_CU_LOG2][logHeight - MIN_CU_LOG2];
              m_savedRdModeListLFNST.resize(m_savedNumRdModesLFNST);
              m_savedModeCostLFNST.resize(m_savedNumRdModesLFNST);
              // PBINTRA fast
              m_savedHadModeListLFNST.resize(3);
              m_savedHadListLFNST.resize(3);
              LFNSTSaveFlag = false;
            }

            //*** Derive MIP candidates using Hadamard
            // MIP模式在下边进行，MIP模式和传统角度模式是互斥的
            // 进行两轮传统角度模式SATD粗选和MPM列表SATD之后
            // 第二轮第三次，对使用哈达玛变换的MIP进行单独候选模式

            // 测试MIP模式但是块不支持MIP模式
            if (testMip && !supportedMipBlkSize)
            {
              // avoid estimation for unsupported blk sizes
              // 避免不支持的块大小
              const int transpOff    = getNumModesMip(pu.Y());
              const int numModesFull = (transpOff << 1);

              for (uint32_t uiModeFull = 0; uiModeFull < numModesFull; uiModeFull++)
              {
                const bool     isTransposed = (uiModeFull >= transpOff ? true : false);
                const uint32_t mode         = (isTransposed ? uiModeFull - transpOff : uiModeFull);

                numModesForFullRD++;
                rdModeList.push_back(ModeInfo(true, isTransposed, 0, NOT_INTRA_SUBPARTITIONS, mode));
                candCostList.push_back(0);
              }
            }
            else if (testMip) // 块也支持MIP模式
            {
              // 第二轮第三次粗选，根据CU块的大小获取到MIP的模式数，对每种模式计算SAD和SATD，
              // 选择其中最小的作为每种模式的cost，更新全率失真候选模式列表rdModeList
              // 第二轮三次粗选结束后，调用reduceHadCandList函数对rdModeList进行缩减处理
              cu.mipFlag     = true; // MIP标志位
              pu.multiRefIdx = 0; // MIP只对参考行0

              // MIP的32种模式都初始化最大失真
              double mipHadCost[MAX_NUM_MIP_MODE] = { MAX_DOUBLE };

              // 初始化预测参数，获取参考像素并滤波，同角度模式一样
              initIntraPatternChType(cu, pu.Y());
              // MIP模式参数初始化，获取参考像素进行下采样、转置等操作
              initIntraMip(pu, pu.Y());
              // getNumModesMip函数根据当前CU的块尺寸获取不同的Mip模式数量16 8 6
              const int transpOff    = getNumModesMip(pu.Y());
              const int numModesFull = (transpOff << 1);
              // MIP模式的多少由CU大小决定, MIP模式的后半段会进行转置操作
              // 4x4有16x2种MIP模式
              // 4xN, Nx4, 8x8 有8x2种MIP模式
              // 其余大小有6x2种MIP模式
              for (uint32_t uiModeFull = 0; uiModeFull < numModesFull; uiModeFull++)
              {
                // 另一半MIP模式大的要进行转置，
                const bool     isTransposed = (uiModeFull >= transpOff ? true : false);
                const uint32_t mode         = (isTransposed ? uiModeFull - transpOff : uiModeFull);

                pu.mipTransposedFlag           = isTransposed;
                pu.intraDir[CHANNEL_TYPE_LUMA] = mode;
                // MIP模式预测函数入口，进行MIP预测
                //  mipSizeId =0,4x4块，下采样后边界长度2，输出边界长度4，MIP矩阵数目16，矩阵维度16x4
                //  mipSizeId =1,4xN,Nx4,8x8，下采样后边界长度4，输出边界长度4，MIP矩阵数目8，矩阵维度16x8
                //  mipSizeId =2,其余块，下采样后边界长度4，输出边界长度5，MIP矩阵数目6，矩阵维度64x7
                predIntraMip(COMPONENT_Y, piPred, pu);

                // Use the min between SAD and HAD as the cost criterion
                // SAD is scaled by 2 to align with the scaling of HAD
                // 计算每种模式的SAD和SATD失真最小值
                Distortion minSadHad =
                  std::min(distParamSad.distFunc(distParamSad) * 2, distParamHad.distFunc(distParamHad));

                m_CABACEstimator->getCtx() = SubCtx(Ctx::MipFlag, ctxStartMipFlag);

                // 熵编码等一系列的计算一下所需比特
                uint64_t fracModeBits = xFracModeBitsIntra(pu, mode, CHANNEL_TYPE_LUMA);

                double cost            = double(minSadHad) + double(fracModeBits) * sqrtLambdaForFirstPass;
                mipHadCost[uiModeFull] = cost;
                DTRACE(g_trace_ctx, D_INTRA_COST, "IntraMIP: %u, %llu, %f (%d)\n", minSadHad, fracModeBits, cost,
                       uiModeFull);

#if GDR_ENABLED
                if (isEncodeGdrClean)
                {
                  if (isValidIntraPredLuma(pu, mode))
                  {
                    updateCandList(ModeInfo(true, isTransposed, 0, NOT_INTRA_SUBPARTITIONS, mode), cost, rdModeList,
                                   candCostList, numModesForFullRD + 1);
                    updateCandList(ModeInfo(true, isTransposed, 0, NOT_INTRA_SUBPARTITIONS, mode),
                                   0.8 * double(minSadHad), hadModeList, candHadList, numHadCand);
                  }
                }
                else
                {
                  updateCandList(ModeInfo(true, isTransposed, 0, NOT_INTRA_SUBPARTITIONS, mode), cost, rdModeList,
                                 candCostList, numModesForFullRD + 1);
                  updateCandList(ModeInfo(true, isTransposed, 0, NOT_INTRA_SUBPARTITIONS, mode),
                                 0.8 * double(minSadHad), hadModeList, candHadList, numHadCand);
                }
#else
                // 更新候选列表，将最优的MIP模式加入到RDCost候选列表
                updateCandList(ModeInfo(true, isTransposed, 0, NOT_INTRA_SUBPARTITIONS, mode), cost, rdModeList, candCostList, numModesForFullRD + 1);
                updateCandList(ModeInfo(true, isTransposed, 0, NOT_INTRA_SUBPARTITIONS, mode), 0.8 * double(minSadHad), hadModeList, candHadList, numHadCand);
#endif
              }

              // 两轮四遍SATD之后，最终更新好RDcost列表后，还要对该列表进行一个缩减的处理
              // HAD代价阈值
              const double thresholdHadCost = 1.0 + 1.4 / sqrt((double) (pu.lwidth() * pu.lheight()));

              // 调用reduceHadCandList函数对全率失真候选模式列表rdModeList进行缩减处理。
              // rdModeList 候选模式列表              
              // candCostList 候选模式代价列表
              // numModesForFullRD 全RD测试的模式数
              // mipHadCost MIP的SATD Cost列表（索引是MIP模式号）
              reduceHadCandList(rdModeList, candCostList, numModesForFullRD, thresholdHadCost, mipHadCost, pu, fastMip);
            }

            // 如果使用了MIP，在给LFNST保存结果的时候也会保存MIP的粗选结果
            // 到这里LFNST保存的第一轮和第二轮1-3过程的全率失真候选列表保存结束
            if (sps.getUseMIP() && LFNSTSaveFlag)
            {
              // save found best modes
              m_savedNumRdModesLFNST = numModesForFullRD;
              m_savedRdModeListLFNST = rdModeList;
              m_savedModeCostLFNST   = candCostList;
              // PBINTRA fast
              m_savedHadModeListLFNST   = hadModeList;
              m_savedHadListLFNST       = candHadList;
              LFNSTSaveFlag             = false;
            }
          }
          else // 加载给LFNST保存的列表
          {
            // SPS层使用MIP且加载为LFNST保存到粗选结果
            // restore saved modes
            numModesForFullRD = m_savedNumRdModesLFNST;
            rdModeList        = m_savedRdModeListLFNST;
            candCostList      = m_savedModeCostLFNST;
            // PBINTRA fast
            hadModeList = m_savedHadModeListLFNST;
            candHadList = m_savedHadListLFNST;
          }


          // 快速搜索细选阶段， FastUDI是什么
          // 获取FastUDI的MPM列表，判断它们是否在全率失真候选列表中，不再就加入
          // 使用ISP模式的，MPM列表判断并加入到m_ispCandListHor
          if (m_pcEncCfg->getFastUDIUseMPMEnabled())
          {

            const int numMPMs = NUM_MOST_PROBABLE_MODES;
            unsigned  uiPreds[numMPMs];

            pu.multiRefIdx = 0; // 使用参考行0

            // 获取MPM列表中最可能的模式
            const int numCand = PU::getIntraMPMs(pu, uiPreds); 

            // 遍历MPM列表判断MPM列表中的模式在不在全率失真候选列表中
            for (int j = 0; j < numCand; j++)
            {
              bool     mostProbableModeIncluded = false;
              ModeInfo mostProbableMode( false, false, 0, NOT_INTRA_SUBPARTITIONS, uiPreds[j] );

#if GDR_ENABLED
              int nn = numModesForFullRD;
              if (isEncodeGdrClean)
              {
                nn = std::min((int) numModesForFullRD, (int) rdModeList.size());
              }

              for (int i = 0; i < nn; i++)
#else
              // 判断MPM列表中的模式在不在全率失真候选列表中
              for (int i = 0; i < numModesForFullRD; i++)
#endif
              {
                mostProbableModeIncluded |= (mostProbableMode == rdModeList[i]);
              }
#if GDR_ENABLED
              if (!isEncodeGdrClean)
              {
                if (!mostProbableModeIncluded)
                {
                  numModesForFullRD++;
                  rdModeList.push_back(mostProbableMode);
                  candCostList.push_back(0);
                }
              }
#else
              // 如果不在就加进去列表中
              if (!mostProbableModeIncluded)
              {
                numModesForFullRD++;
                rdModeList.push_back(mostProbableMode);
                candCostList.push_back(0);
              }
#endif
            }

            // 和FastUDI一样，同样判断MPM列表中的模式在不在ISP列表
            if (saveDataForISP)
            {
              // we add the MPMs to the list that contains only regular intra modes
              // 将MPM列表中的模式不在m_ispCandListHor就添加进去
              for (int j = 0; j < numCand; j++)
              {
                bool     mostProbableModeIncluded = false;
                ModeInfo mostProbableMode(false, false, 0, NOT_INTRA_SUBPARTITIONS, uiPreds[j]);

                for (int i = 0; i < m_ispCandListHor.size(); i++)
                {
                  mostProbableModeIncluded |= (mostProbableMode == m_ispCandListHor[i]);
                }
#if GDR_ENABLED
                if (!isEncodeGdrClean)
                {
                  if (!mostProbableModeIncluded)
                  {
                    m_ispCandListHor.push_back(mostProbableMode);
                  }
                }
#else
                if (!mostProbableModeIncluded)
                {
                  m_ispCandListHor.push_back(mostProbableMode);
                }
#endif
              }
            }
          }//End if(m_pcEncCfg->getFastUDIUseMPMEnabled())
        }//End if (numModesForFullRD != numModesAvailable)
        else
        {
          THROW("Full search not supported for MIP");
        }

        // 上边进行了两轮的SATD筛选
        // 第一轮对HEVC的35种模式进行SATD筛选并加入到候选列表
        // 第二轮第一次对VVC扩展模式进行SATD筛选
        // 第二轮第二次对MRL的第2、3参考行使用MPM列表进行SATD筛选
        // 第二轮第三次对MIP模式进行SATD筛选
        // 第二轮第四次对FastUDI的MPM列表种不在候选列表的加入到候选列表
        // 第二轮第五次对ISP模式加入到候选列表
        // mtsUsageFlag=0/1会保存前两轮的SATD粗选列表
        if (sps.getUseLFNST() && mtsUsageFlag == 1)//为进行MTS变换检查保存RD模式列表
        {
          // Store the modes to be checked with RD
          // 存储要进行RD Cost检查的模式
          m_savedNumRdModes[lfnstIdx] = numModesForFullRD;
          std::copy_n(rdModeList.begin(), numModesForFullRD, m_savedRdModeList[lfnstIdx]);
        }
      } // if(mtsUsage != 2)
      else   
      {
        //  mtsUsage = 2 这里就用 mtsUsage = 0/1时候保存的粗选列表
        // mtsUsage = 2 (here we potentially reduce the number of modes that will be full-RD checked)
        // LFNST快速算法
        if ((m_pcEncCfg->getUseFastLFNST() || !cu.slice->isIntra()) && m_bestModeCostValid[lfnstIdx])
        {
          numModesForFullRD = 0;

          double thresholdSkipMode = 1.0 + ((cu.lfnstIdx > 0) ? 0.1 : 1.0) * (1.4 / sqrt((double) (width * height)));

          // Skip checking the modes with much larger R-D cost than the best mode
          // 跳过检查RD Cost比最佳模式大得多的模式
          for (int i = 0; i < m_savedNumRdModes[lfnstIdx]; i++)
          {
            if (m_modeCostStore[lfnstIdx][i] <= thresholdSkipMode * m_bestModeCostStore[lfnstIdx])
            {
              rdModeList.push_back(m_savedRdModeList[lfnstIdx][i]);
              numModesForFullRD++;
            }
          }
        }
        else   
        {
          // this is necessary because we skip the candidates list calculation, since it was already obtained for the DCT-II. Now we load it
          // 从DCT-2中获得了计算数据，这里跳过了候选列表的计算，直接加载保存的数据
          // Restore the modes to be checked with RD
          numModesForFullRD = m_savedNumRdModes[lfnstIdx];
          rdModeList.resize(numModesForFullRD);
          std::copy_n(m_savedRdModeList[lfnstIdx], m_savedNumRdModes[lfnstIdx], rdModeList.begin());
          candCostList.resize(numModesForFullRD);
        }
      }

#if GDR_ENABLED
      if (!isEncodeGdrClean)
      {
        CHECK(numModesForFullRD != rdModeList.size(), "Inconsistent state!");
      }
#else
      CHECK(numModesForFullRD != rdModeList.size(), "Inconsistent state!");
#endif

      // after this point, don't use numModesForFullRD
      // 在这一点之后，不要使用numModesForFullRD
      // PBINTRA fast 快速帧内模式
      // 这段进行什么调整？？
      if (m_pcEncCfg->getUsePbIntraFast() && !cs.slice->isIntra() && rdModeList.size() < numModesAvailable
          && !cs.slice->getDisableSATDForRD() && (mtsUsageFlag != 2 || lfnstIdx > 0))
      {
        double   pbintraRatio = (lfnstIdx > 0) ? 1.25 : PBINTRA_RATIO;
        int      maxSize      = -1;
        ModeInfo bestMipMode;
        int      bestMipIdx = -1;
        for (int idx = 0; idx < rdModeList.size(); idx++)
        {
          if (rdModeList[idx].mipFlg)
          {
            bestMipMode = rdModeList[idx];
            bestMipIdx  = idx;
            break;
          }
        }
        const int numHadCand = 3;
        for (int k = numHadCand - 1; k >= 0; k--)
        {
          if (candHadList.size() < (k + 1) || candHadList[k] > cs.interHad * pbintraRatio)
          {
            maxSize = k;
          }
        }
        if (maxSize > 0)
        {
          rdModeList.resize(std::min<size_t>(rdModeList.size(), maxSize));
          if (bestMipIdx >= 0)
          {
            if (rdModeList.size() <= bestMipIdx)
            {
              rdModeList.push_back(bestMipMode);
            }
          }
          if (saveDataForISP)
          {
            m_ispCandListHor.resize(std::min<size_t>(m_ispCandListHor.size(), maxSize));
          }
        }
        if (maxSize == 0)
        {
          cs.dist     = std::numeric_limits<Distortion>::max();
          cs.interHad = 0;

          //===== reset context models =====
          m_CABACEstimator->getCtx() = SubCtx(Ctx::MipFlag, ctxStartMipFlag);
          m_CABACEstimator->getCtx() = SubCtx(Ctx::ISPMode, ctxStartIspMode);
          m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaPlanarFlag, ctxStartPlanarFlag);
          m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaMpmFlag, ctxStartIntraMode);
          m_CABACEstimator->getCtx() = SubCtx(Ctx::MultiRefLineIdx, ctxStartMrlIdx);

          return false;
        }
      }
    }


    // 这个是上边进行了传统角度模式粗选、MRL的MPM粗选以及MIP粗选后的列表，记录以下一共有多少种
    // 后边再往列表种加入的就是ISP模式了
    int numNonISPModes = (int) rdModeList.size(); // 非ISP模式的数量

    // 在进行RD Cost细选以前，将初始化后的ISP模式加入到候选模式列表中
    if ( testISP )
    {
      // we reserve positions for ISP in the common full RD list
      // 我们为ISP在通用的完整RD列表中保留位置
#if GDR_ENABLED
      if (!isEncodeGdrClean)
      {
        const int maxNumRDModesISP = sps.getUseLFNST() ? 16 * NUM_LFNST_NUM_PER_SET : 16;
        m_curIspLfnstIdx = 0;
        for (int i = 0; i < maxNumRDModesISP; i++)
        {
          rdModeList.push_back(ModeInfo(false, false, 0, INTRA_SUBPARTITIONS_RESERVED, 0));
        }
      }
#else
      // LFNST会有3个标志，0/1/2分别对应不使用，和一个变换集的两个不同变换核
      const int maxNumRDModesISP = sps.getUseLFNST() ? 16 * NUM_LFNST_NUM_PER_SET : 16;
      m_curIspLfnstIdx = 0;
      for (int i = 0; i < maxNumRDModesISP; i++)
      {
        rdModeList.push_back(ModeInfo(false, false, 0, INTRA_SUBPARTITIONS_RESERVED, 0));
      }
#endif
    }

    // 第三轮RD COST细选
    //===== check modes (using r-d costs) =====
    // 使用RD cost检查预测模式
    ModeInfo       uiBestPUMode; // 当前PU最佳亮度帧内预测模式
    int            bestBDPCMMode = 0;
    double         bestCostNonBDPCM = MAX_DOUBLE; // 定义再不使用BDPCM模式的时候的最优代价
    // 临时缓存的CS
    CodingStructure *csTemp = m_pTempCS[gp_sizeIdxInfo->idxFrom( cu.lwidth() )][gp_sizeIdxInfo->idxFrom( cu.lheight() )];
    // 最佳CS
    CodingStructure *csBest = m_pBestCS[gp_sizeIdxInfo->idxFrom( cu.lwidth() )][gp_sizeIdxInfo->idxFrom( cu.lheight() )];
    // 定义临时变量的初始化信息
    csTemp->slice = cs.slice;
    csBest->slice = cs.slice;
    csTemp->initStructData();
    csBest->initStructData();
    csTemp->picture = cs.picture;
    csBest->picture = cs.picture;

    // just to be sure
    numModesForFullRD = (int) rdModeList.size();
    TUIntraSubPartitioner subTuPartitioner( partitioner );
    if ( testISP )
    {
      m_modeCtrl->setIspCost( MAX_DOUBLE );
      m_modeCtrl->setMtsFirstPassNoIspCost( MAX_DOUBLE );
    }
    int bestLfnstIdx = cu.lfnstIdx;

    // 第三轮对RD候选列表中的模式进一步精选
    // 遍历rdModeList候选模式列表，对每种模式进行残差量化、变换后计算rdcost选择其中最小的。
    for (int mode = isSecondColorSpace ? 0 : -2 * int(testBDPCM); mode < (int) rdModeList.size(); mode++)
    {
      // set CU/PU to luma prediction mode
      //首先设置亮度CU/PU的预测模式
      ModeInfo uiOrgMode;
      if (sps.getUseColorTrans() && !m_pcEncCfg->getRGBFormatFlag() && isSecondColorSpace && mode)
      {
        continue;
      }
      // BDPCM：区块差分脉冲编码调制，用于对量化后的变换块进行编码传输。
      if (mode < 0 || (isSecondColorSpace && m_savedBDPCMModeFirstColorSpace[m_savedRdModeIdx][mode]))
      {
        cu.bdpcmMode = mode < 0 ? -mode : m_savedBDPCMModeFirstColorSpace[m_savedRdModeIdx][mode];
        uiOrgMode = ModeInfo( false, false, 0, NOT_INTRA_SUBPARTITIONS, cu.bdpcmMode == 2 ? VER_IDX : HOR_IDX );
      }
      else
      {
        cu.bdpcmMode = 0;
        uiOrgMode    = rdModeList[mode];
      }

      // 先遍历常规帧内角度模式和MIP模式之后，这里到了ISP模式
      if (!cu.bdpcmMode && rdModeList[mode].ispMod == INTRA_SUBPARTITIONS_RESERVED)
      {
        // numNonISPModes是之前存储的前两轮一共有的模式数量，
        // 后边会把那些模式插入到m_regIntraRDListWithCosts
        // 到这里就把上边的模式全都插入到m_regIntraRDListWithCosts里了，只进行一次排序
        // 传统的角度模式
        if (mode == numNonISPModes)   // the list needs to be sorted only once 列表只需排序一次
        {
          // 用ISP快速算法
          if (m_pcEncCfg->getUseFastISP())
          {
            m_modeCtrl->setBestPredModeDCT2(uiBestPUMode.modeId);
          }

          // 为ISP模式遍历准备帧内模式候选列表，m_regIntraRDListWithCosts
          // m_regIntraRDListWithCosts里边存储了前三轮SATD的常规帧内角度模式、MIP模式
          // xSortISPCandList首先对模式进行排序选出最佳帧内角度模式
          // 然后一次将PLANAR模式、最佳帧内角度模式、m_regIntraRDListWithCosts列表其余模式（除DC模式外）、DC模式、常规哈达玛候选模式列表中额外的三种模式加入保存水平划分模式列表和垂直划分模式列表
          if (!xSortISPCandList(bestCurrentCost, csBest->cost, uiBestPUMode))
          {
            break;
          }
        }

        // 决定ISP哪些划分模式和预测模式可以组合使用进行完整的RD Cost测试
        xGetNextISPMode(rdModeList[mode], (mode > 0 ? &rdModeList[mode - 1] : nullptr), Size(width, height));
        if (rdModeList[mode].ispMod == INTRA_SUBPARTITIONS_RESERVED)
        {
          continue;
        }
        cu.lfnstIdx = m_curIspLfnstIdx; // 当前使用lfnst的变换核
        uiOrgMode   = rdModeList[mode];
      }

      cu.mipFlag                     = uiOrgMode.mipFlg; // 是否 MIP模式
      pu.mipTransposedFlag           = uiOrgMode.mipTrFlg; // 候选模式是否MIP变换后的
      cu.ispMode                     = uiOrgMode.ispMod; // 是否ISP模式
      pu.multiRefIdx                 = uiOrgMode.mRefId; // 使用MRL的索引
      pu.intraDir[CHANNEL_TYPE_LUMA] = uiOrgMode.modeId; // 当前候选模式放入intraDir

      CHECK(cu.mipFlag && pu.multiRefIdx, "Error: combination of MIP and MRL not supported");
      CHECK(pu.multiRefIdx && (pu.intraDir[0] == PLANAR_IDX), "Error: combination of MRL and Planar mode not supported");
      CHECK(cu.ispMode && cu.mipFlag, "Error: combination of ISP and MIP not supported");
      CHECK(cu.ispMode && pu.multiRefIdx, "Error: combination of ISP and MRL not supported");
      CHECK(cu.ispMode&& cu.colorTransform, "Error: combination of ISP and ACT not supported");

      pu.intraDir[CHANNEL_TYPE_CHROMA] = cu.colorTransform ? DM_CHROMA_IDX : pu.intraDir[CHANNEL_TYPE_CHROMA];

      // set context models 设置上下文
      m_CABACEstimator->getCtx() = ctxStart;

      // determine residual for partition 划分残差
      cs.initSubStructure( *csTemp, partitioner.chType, cs.area, true );

      bool tmpValidReturn = false;


      // 模式已经选择好了，接下来进入变换部分。
      // ISP模式在选择好的划分模式（水平或垂直）和预测模式以后，正式进入ISP的划分、预测、变换等等。
      if( cu.ispMode )
      {
        if ( m_pcEncCfg->getUseFastISP() )
        {
          m_modeCtrl->setISPWasTested(true);
        }

        // xIntraCodingLumaISP函数主要是对CU进行划分，预测并进行变换
        // 进行变换、量化等一系列操作。计算所得失真、码率，从而计算RD Cost，最后再求出当前ISP模式的总的RD Cost。
        // 在进行变换时候会判断是否是隐式的MTS
        tmpValidReturn = xIntraCodingLumaISP(*csTemp, subTuPartitioner, bestCurrentCost);
        if (csTemp->tus.size() == 0)
        {
          // no TUs were coded
          csTemp->cost = MAX_DOUBLE;
          continue;
        }
        // we save the data for future tests
        // 我们保存数据以备将来的测试
        m_ispTestedModes[m_curIspLfnstIdx].setModeResults((ISPType)cu.ispMode, (int)uiOrgMode.modeId, (int)csTemp->tus.size(), csTemp->cus[0]->firstTU->cbf[COMPONENT_Y] ? csTemp->cost : MAX_DOUBLE, csBest->cost);
        csTemp->cost = !tmpValidReturn ? MAX_DOUBLE : csTemp->cost;
      }
      else
      {
        if (cu.colorTransform) // ACT变换
        {
          tmpValidReturn = xRecurIntraCodingACTQT(*csTemp, partitioner, mtsCheckRangeFlag, mtsFirstCheckId, mtsLastCheckId, moreProbMTSIdxFirst);
        }
        else
        {

          // 使用上层函数选中的预测模式，遍历上层函数传来的变换模式，通过调用xIntraCodingTUBlock
          // 进行变换、量化等操作，计算相应的失真，从而计算RD Cost，从而选出最佳变换。
          // 在进行变换时候会判断是否是隐式的MTS
          tmpValidReturn = xRecurIntraCodingLumaQT(*csTemp, partitioner, mtsCheckRangeFlag, mtsFirstCheckId, mtsLastCheckId, moreProbMTSIdxFirst);
        }
      }
      
      // m_regIntraRDListWithCosts主要包含的是第三轮RD Cost细选所包含的常规帧内角度模式。
      if (!cu.ispMode && !cu.mtsFlag && !cu.lfnstIdx && !cu.bdpcmMode && !pu.multiRefIdx && !cu.mipFlag && testISP)
      {
        m_regIntraRDListWithCosts.push_back( ModeInfoWithCost( cu.mipFlag, pu.mipTransposedFlag, pu.multiRefIdx, cu.ispMode, uiOrgMode.modeId, csTemp->cost ) );
      }

      if( cu.ispMode && !csTemp->cus[0]->firstTU->cbf[COMPONENT_Y] )
      {
        csTemp->cost = MAX_DOUBLE;
        csTemp->costDbOffset = 0;
        tmpValidReturn = false;
      }
      validReturn |= tmpValidReturn;
      // 如果SPS层开启LFNST且可以使用MTS并正在检查DCT-2，将当前RD Cost保存下来以在检查MTS时使用快速算法
      if( sps.getUseLFNST() && mtsUsageFlag == 1 && !cu.ispMode && mode >= 0 )
      {
        m_modeCostStore[lfnstIdx][mode] = tmpValidReturn ? csTemp->cost : (MAX_DOUBLE / 2.0);
      }

      DTRACE(g_trace_ctx, D_INTRA_COST, "IntraCost T [x=%d,y=%d,w=%d,h=%d] %f (%d,%d,%d,%d,%d,%d) \n", cu.blocks[0].x,
             cu.blocks[0].y, (int) width, (int) height, csTemp->cost, uiOrgMode.modeId, uiOrgMode.ispMod,
             pu.multiRefIdx, cu.mipFlag, cu.lfnstIdx, cu.mtsFlag);

      if( tmpValidReturn )//如果是有效返回值，比较最优模式代价和当前模式代价
      {
        if (isFirstColorSpace)
        {
          if (m_pcEncCfg->getRGBFormatFlag() || !cu.ispMode)
          {
            sortRdModeListFirstColorSpace(uiOrgMode, csTemp->cost, cu.bdpcmMode, m_savedRdModeFirstColorSpace[m_savedRdModeIdx], m_savedRdCostFirstColorSpace[m_savedRdModeIdx], m_savedBDPCMModeFirstColorSpace[m_savedRdModeIdx], m_numSavedRdModeFirstColorSpace[m_savedRdModeIdx]);
          }
        }
        // 检查如果是最优的代价就更新模式
        // check r-d cost
        if( csTemp->cost < csBest->cost ) //如果当前模式的cost小于目前最优的模式的cost
        {
          std::swap( csTemp, csBest );//设置当前模式为最优模式

          uiBestPUMode  = uiOrgMode;
          bestBDPCMMode = cu.bdpcmMode;
          if( sps.getUseLFNST() && mtsUsageFlag == 1 && !cu.ispMode )
          {
            m_bestModeCostStore[ lfnstIdx ] = csBest->cost; //cs.cost;
            m_bestModeCostValid[ lfnstIdx ] = true;
          }
          if( csBest->cost < bestCurrentCost )
          {
            bestCurrentCost = csBest->cost;
          }
          if ( cu.ispMode )//如果当前模式时ISP模式，保存最优的ISP模式和相应的lfnstIdx
          {
            m_modeCtrl->setIspCost(csBest->cost);
            bestLfnstIdx = cu.lfnstIdx;
          }
          else if ( testISP ) //如果需要测试ISP，则保存MTS的第一个流程中非ISP模式的Cost
          {
            m_modeCtrl->setMtsFirstPassNoIspCost(csBest->cost);
          }
        }
        if( !cu.ispMode && !cu.bdpcmMode && csBest->cost < bestCostNonBDPCM )
        {
          bestCostNonBDPCM = csBest->cost;
        }
      }

      csTemp->releaseIntermediateData();
      if( m_pcEncCfg->getFastLocalDualTreeMode() )
      {
        if( cu.isConsIntra() && !cu.slice->isIntra() && csBest->cost != MAX_DOUBLE && costInterCU != COST_UNKNOWN && mode >= 0 )
        {
          if( m_pcEncCfg->getFastLocalDualTreeMode() == 2 )
          {
            //Note: only try one intra mode, which is especially useful to reduce EncT for LDB case (around 4%)
            // 注意：仅尝试一种帧内模式，这对于减少LDB情况下的编码时间尤其有用（大约4％）
            break;
          }
          else
          {
            if( csBest->cost > costInterCU * 1.5 )
            {
              break;
            }
          }
        }
      }
      if (sps.getUseColorTrans() && !CS::isDualITree(cs))
      {
        if ((m_pcEncCfg->getRGBFormatFlag() && !cu.colorTransform) && csBest->cost != MAX_DOUBLE && bestCS->cost != MAX_DOUBLE && mode >= 0)
        {
          if (csBest->cost > bestCS->cost)
          {
            break;
          }
        }
      }
    } // Mode loop RD COST 模式循环
    cu.ispMode = uiBestPUMode.ispMod;
    cu.lfnstIdx = bestLfnstIdx; // 选出了最佳的lfnst索引

    if( validReturn )
    {
      if (cu.colorTransform)
      {
        cs.useSubStructure(*csBest, partitioner.chType, pu, true, true, KEEP_PRED_AND_RESI_SIGNALS, KEEP_PRED_AND_RESI_SIGNALS, true);
      }
      else
      {
        cs.useSubStructure(*csBest, partitioner.chType, pu.singleChan(CHANNEL_TYPE_LUMA), true, true, KEEP_PRED_AND_RESI_SIGNALS,
                           KEEP_PRED_AND_RESI_SIGNALS, true);
      }
    }
    csBest->releaseIntermediateData();

    if( validReturn )
    {
      // 更新PU数据
      //=== update PU data ====
      cu.mipFlag = uiBestPUMode.mipFlg;
      pu.mipTransposedFlag             = uiBestPUMode.mipTrFlg;
      pu.multiRefIdx = uiBestPUMode.mRefId;
      pu.intraDir[ CHANNEL_TYPE_LUMA ] = uiBestPUMode.modeId;
      cu.bdpcmMode = bestBDPCMMode;
      if (cu.colorTransform)
      {
        CHECK(pu.intraDir[CHANNEL_TYPE_CHROMA] != DM_CHROMA_IDX, "chroma should use DM mode for adaptive color transform");
      }
    }
  }

  //===== reset context models =====
  m_CABACEstimator->getCtx() = ctxStart;

  return validReturn;
}

/*
* 色度预测有DC，PLANAR，VER，HOR，DM_CHROMA，LM_CHROMA，MDLM_L，MDLM_T八种模式。
* LM_CHROMA，MDLM_L和MDLM_T分别表示CCLM模式，DM_CHROMA为从亮度分量获取到的模式。
1. 利用getIntraChromaCandModes函数获取色度模式列表，并初始化PU、TU
2. 利用SATD对DC、Ver、Hor、LM_L、LM_T五种模式进行粗选，禁用色度模式列表中SATD最大的两个模式
3. 利用RD Cost对剩余的六种模式进行细选，主要是通过xRecurIntraChromaCodingQT进行变换、量化后计算RD Cost，选出最低的RD Cost
4. 保存最佳色度模式。
*/
void IntraSearch::estIntraPredChromaQT( CodingUnit &cu, Partitioner &partitioner, const double maxCostAllowed )
{
  const ChromaFormat format   = cu.chromaFormat;
  const uint32_t    numberValidComponents = getNumberValidComponents(format);
  CodingStructure &cs = *cu.cs;
  const TempCtx ctxStart  ( m_CtxCache, m_CABACEstimator->getCtx() );

  cs.setDecomp( cs.area.Cb(), false );

  double    bestCostSoFar = maxCostAllowed;
  bool      lumaUsesISP   = !cu.isSepTree() && cu.ispMode;
  // ISP类型
  PartSplit ispType       = lumaUsesISP ? CU::getISPType( cu, COMPONENT_Y ) : TU_NO_ISP;
  CHECK( cu.ispMode && bestCostSoFar < 0, "bestCostSoFar must be positive!" );

  auto &pu = *cu.firstPU;

  {
    uint32_t       uiBestMode = 0;
    Distortion uiBestDist = 0;
    double     dBestCost = MAX_DOUBLE;
    int32_t bestBDPCMMode = 0;

    //----- init mode list ----
    {
      int32_t  uiMinMode = 0;
      int32_t  uiMaxMode = NUM_CHROMA_MODE; // 8
      //----- check chroma modes -----
      // 
      uint32_t chromaCandModes[ NUM_CHROMA_MODE ];
      // 初始化色度模式列表为PLANAR，VER，HOR，DC，LM_CHROMA，MDLM_L，MDLM_T，DM_CHROMA。
      PU::getIntraChromaCandModes( pu, chromaCandModes );

      // create a temporary CS
      CodingStructure &saveCS = *m_pSaveCS[0];
      saveCS.pcv      = cs.pcv;
      saveCS.picture  = cs.picture;
      saveCS.area.repositionTo( cs.area );
      saveCS.clearTUs();

      if( !cu.isSepTree() && cu.ispMode )
      {
        saveCS.clearCUs();
        saveCS.clearPUs();
      }
      // 如果是色度单独划分的话，将色度进行划分，并把TU存好
      if( cu.isSepTree() )
      {
        if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
        {
          partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );

          do
          {
            cs.addTU( CS::getArea( cs, partitioner.currArea(), partitioner.chType ), partitioner.chType ).depth = partitioner.currTrDepth;
          } while( partitioner.nextPart( cs ) );

          partitioner.exitCurrSplit();
        }
        else
        cs.addTU( CS::getArea( cs, partitioner.currArea(), partitioner.chType ), partitioner.chType );
      }

      std::vector<TransformUnit*> orgTUs;

      if( lumaUsesISP )// 如果亮度是ISP，记录当前cu，pu信息
      {
        CodingUnit& auxCU = saveCS.addCU( cu, partitioner.chType );
        auxCU.ispMode = cu.ispMode;
        saveCS.sps = cu.cs->sps;
        saveCS.addPU( *cu.firstPU, partitioner.chType );
      }


      // create a store for the TUs
      // 保存tu进orgTUs中以备后续处理
      for( const auto &ptu : cs.tus )
      {
        // for split TUs in HEVC, add the TUs without Chroma parts for correct setting of Cbfs
        if( lumaUsesISP || pu.contains( *ptu, CHANNEL_TYPE_CHROMA ) )
        {
          saveCS.addTU( *ptu, partitioner.chType );
          orgTUs.push_back( ptu );
        }
      }
      if( lumaUsesISP )
      {
        saveCS.clearCUs();
      }

      //进行第一次选择，按DC、Ver、Hor、LM_L、LM_T进行预测，按SATD排序，去掉两个SATD最大的模式
      // SATD pre-selecting.
      int satdModeList[NUM_CHROMA_MODE]; // 模式列表
      int64_t satdSortedCost[NUM_CHROMA_MODE];// SATD列表
      for (int i = 0; i < NUM_CHROMA_MODE; i++)
      {
        // 对于不是由SATD预先选择的模式，默认情况下执行RDO，因此设置初始值0。
        satdSortedCost[i] = 0; // for the mode not pre-select by SATD, do RDO by default, so set the initial value 0.
        satdModeList[i] = 0;
      }
      // 使用帧内模式idx检查是否启用
      bool modeIsEnable[NUM_INTRA_MODE + 1]; // use intra mode idx to check whether enable
      for (int i = 0; i < NUM_INTRA_MODE + 1; i++)
      {
        modeIsEnable[i] = 1;
      }
      DistParam distParamSad;
      DistParam distParamSatd;
      // 暂时pu模式设置为MDLM模式，为了在下面对luma重建信号进行下采样
      pu.intraDir[1] = MDLM_L_IDX; // temporary assigned, just to indicate this is a MDLM mode. for luma down-sampling operation.

      // 获取色度原始参考像素并进行滤波
      initIntraPatternChType(cu, pu.Cb());
      initIntraPatternChType(cu, pu.Cr());
      // CCLM获取亮度的重建像素
      xGetLumaRecPixels(pu, pu.Cb());
      // 初始化色度模式列表为PLANAR，VER，HOR，DC，LM_CHROMA，MDLM_L，MDLM_T，DM_CHROMA。
      // 遍历所有8种模式，第一轮SATD粗选VER,HOR,DC,LM_L,LM_T共5种模式
      for (int idx = uiMinMode; idx <= uiMaxMode - 1; idx++)
      {
        int mode = chromaCandModes[idx];
        satdModeList[idx] = mode;
        if (PU::isLMCMode(mode) && !PU::isLMCModeEnabled(pu, mode))
        {
          continue;
        }
        // 不是这5种的跳过
        if ((mode == LM_CHROMA_IDX) || (mode == PLANAR_IDX) || (mode == DM_CHROMA_IDX)) // only pre-check regular modes and MDLM modes, not including DM ,Planar, and LM
        {
          continue;
        }
        pu.intraDir[1] = mode; // temporary assigned, for SATD checking.

        int64_t sad = 0;
        int64_t sadCb = 0;
        int64_t satdCb = 0;
        int64_t sadCr = 0;
        int64_t satdCr = 0;
        CodingStructure& cs = *(pu.cs);
        // Cb分量的SATD
        CompArea areaCb = pu.Cb();
        PelBuf orgCb = cs.getOrgBuf(areaCb);
        PelBuf predCb = cs.getPredBuf(areaCb);
        m_pcRdCost->setDistParam(distParamSad, orgCb, predCb, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cb, false);
        m_pcRdCost->setDistParam(distParamSatd, orgCb, predCb, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cb, true);
        distParamSad.applyWeight = false;
        distParamSatd.applyWeight = false;
        if (PU::isLMCMode(mode))// CCLM，LM_T，LM_L模式，进行跨分量预测
        {
          predIntraChromaLM(COMPONENT_Cb, predCb, pu, areaCb, mode);
        }
        else // 传统角度预测
        {
          initPredIntraParams(pu, pu.Cb(), *pu.cs->sps);
          predIntraAng(COMPONENT_Cb, predCb, pu);
        }
        sadCb = distParamSad.distFunc(distParamSad) * 2; // SAD
        satdCb = distParamSatd.distFunc(distParamSatd); // SATD
        sad += std::min(sadCb, satdCb);

        // Cr分量的失真SAD和SATD
        CompArea areaCr = pu.Cr();
        PelBuf orgCr = cs.getOrgBuf(areaCr);
        PelBuf predCr = cs.getPredBuf(areaCr);
        m_pcRdCost->setDistParam(distParamSad, orgCr, predCr, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cr, false);
        m_pcRdCost->setDistParam(distParamSatd, orgCr, predCr, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cr, true);
        distParamSad.applyWeight = false;
        distParamSatd.applyWeight = false;
        if (PU::isLMCMode(mode))
        {
          predIntraChromaLM(COMPONENT_Cr, predCr, pu, areaCr, mode);
        }
        else
        {
          initPredIntraParams(pu, pu.Cr(), *pu.cs->sps);
          predIntraAng(COMPONENT_Cr, predCr, pu);
        }
        sadCr = distParamSad.distFunc(distParamSad) * 2;
        satdCr = distParamSatd.distFunc(distParamSatd);
        sad += std::min(sadCr, satdCr);
        satdSortedCost[idx] = sad; // 保存Cb和Cr总的失真
      }
      // sort the mode based on the cost from small to large.
      // 冒泡排序对代价和模式进行排序，去除失真最大的两个
      int tempIdx = 0;
      int64_t tempCost = 0;
      for (int i = uiMinMode; i <= uiMaxMode - 1; i++)
      {
        for (int j = i + 1; j <= uiMaxMode - 1; j++)
        {
          if (satdSortedCost[j] < satdSortedCost[i])
          {
            tempIdx = satdModeList[i];
            satdModeList[i] = satdModeList[j];
            satdModeList[j] = tempIdx;

            tempCost = satdSortedCost[i];
            satdSortedCost[i] = satdSortedCost[j];
            satdSortedCost[j] = tempCost;

          }
        }
      }
      // 减少两个色度模式
      int reducedModeNumber = 2; // reduce the number of chroma modes
      for (int i = 0; i < reducedModeNumber; i++)
      {
        // 禁用最后reducedModelNumber个模式，即将SATD最大的两个模式排除
        modeIsEnable[satdModeList[uiMaxMode - 1 - i]] = 0; // disable the last reducedModeNumber modes
      }

      // save the dist
      // 保存失真
      Distortion baseDist = cs.dist;
      bool testBDPCM = true;
      testBDPCM = testBDPCM && CU::bdpcmAllowed(cu, COMPONENT_Cb) && cu.ispMode == 0 && cu.mtsFlag == 0 && cu.lfnstIdx == 0;
      // 第二次选择，SATD小的3种和剩下未筛选的3种模式进行编码后计算RD cost
      for (int32_t mode = uiMinMode - (2 * int(testBDPCM)); mode < uiMaxMode; mode++)
      {
        int chromaIntraMode;

        if (mode < 0) // BDPCM模式
        {
          cu.bdpcmModeChroma = -mode;
          chromaIntraMode    = cu.bdpcmModeChroma == 2 ? chromaCandModes[1] : chromaCandModes[2];
        }
        else
        {
          chromaIntraMode = chromaCandModes[mode];

          cu.bdpcmModeChroma = 0;
          if( PU::isLMCMode( chromaIntraMode ) && ! PU::isLMCModeEnabled( pu, chromaIntraMode ) )
          {
            continue;
          }
          // 如果当前的是CCLM模式但是CCLM模式被禁用，不进行SATD筛选
          if (!modeIsEnable[chromaIntraMode] && PU::isLMCModeEnabled(pu, chromaIntraMode)) // when CCLM is disable, then MDLM is disable. not use satd checking
          {
            continue;
          }
        }
        cs.setDecomp( pu.Cb(), false );
        cs.dist = baseDist;
        //----- restore context models -----
        m_CABACEstimator->getCtx() = ctxStart;

        //----- chroma coding -----
        pu.intraDir[1] = chromaIntraMode;
        // 分别进行Cb Cr单独编码和JointCbCr联合编码，计算变换量化、反变换反量化
        // 计算代价，选出色度最佳编码模式
        xRecurIntraChromaCodingQT( cs, partitioner, bestCostSoFar, ispType );
        if( lumaUsesISP && cs.dist == MAX_UINT )
        {
          continue;
        }

        if (cs.sps->getTransformSkipEnabledFlag())
        {
          m_CABACEstimator->getCtx() = ctxStart;
        }
        // 计算当前模式的RD cost代价
        uint64_t fracBits   = xGetIntraFracBitsQT( cs, partitioner, false, true, -1, ispType );
        Distortion uiDist = cs.dist;
        double    dCost   = m_pcRdCost->calcRdCost( fracBits, uiDist - baseDist );

        //----- compare -----
#if GDR_ENABLED
        bool allOk = (dCost < dBestCost);
        if (m_pcEncCfg->getGdrEnabled())
          allOk = allOk && dBestCost && isValidIntraPredChroma(pu, (int)PU::getCoLocatedIntraLumaMode(pu), chromaIntraMode);

        if (allOk)
#else
        // 比较最优的模式
        if( dCost < dBestCost )
#endif
        {
          if( lumaUsesISP && dCost < bestCostSoFar )
          {
            bestCostSoFar = dCost;
          }
          for( uint32_t i = getFirstComponentOfChannel( CHANNEL_TYPE_CHROMA ); i < numberValidComponents; i++ )
          {
            const CompArea &area = pu.blocks[i];

            saveCS.getRecoBuf     ( area ).copyFrom( cs.getRecoBuf   ( area ) );
#if KEEP_PRED_AND_RESI_SIGNALS
            saveCS.getPredBuf     ( area ).copyFrom( cs.getPredBuf   ( area ) );
            saveCS.getResiBuf     ( area ).copyFrom( cs.getResiBuf   ( area ) );
#endif
            saveCS.getPredBuf     ( area ).copyFrom( cs.getPredBuf   (area ) );
            cs.picture->getPredBuf( area ).copyFrom( cs.getPredBuf   (area ) );
            cs.picture->getRecoBuf( area ).copyFrom( cs.getRecoBuf( area ) );

            for( uint32_t j = 0; j < saveCS.tus.size(); j++ )
            {
              saveCS.tus[j]->copyComponentFrom( *orgTUs[j], area.compID );
            }
          }

          dBestCost  = dCost;
          uiBestDist = uiDist;
          uiBestMode = chromaIntraMode;
          bestBDPCMMode = cu.bdpcmModeChroma;
        }
      }//第二次选择

      // 将最优信号，更新在picture buffer
      for( uint32_t i = getFirstComponentOfChannel( CHANNEL_TYPE_CHROMA ); i < numberValidComponents; i++ )
      {
        const CompArea &area = pu.blocks[i];

        cs.getRecoBuf         ( area ).copyFrom( saveCS.getRecoBuf( area ) );
#if KEEP_PRED_AND_RESI_SIGNALS
        cs.getPredBuf         ( area ).copyFrom( saveCS.getPredBuf( area ) );
        cs.getResiBuf         ( area ).copyFrom( saveCS.getResiBuf( area ) );
#endif
        cs.getPredBuf         ( area ).copyFrom( saveCS.getPredBuf( area ) );
        cs.picture->getPredBuf( area ).copyFrom( cs.getPredBuf    ( area ) );

        cs.picture->getRecoBuf( area ).copyFrom( cs.    getRecoBuf( area ) );

        for( uint32_t j = 0; j < saveCS.tus.size(); j++ )
        {
          orgTUs[ j ]->copyComponentFrom( *saveCS.tus[ j ], area.compID );
        }
      }
    }

    pu.intraDir[1] = uiBestMode;// 保存最佳模式
    cs.dist        = uiBestDist;// 保存最佳失真
    cu.bdpcmModeChroma = bestBDPCMMode;
  } // 代码块

  // 重置上下文模式
  //----- restore context models -----
  m_CABACEstimator->getCtx() = ctxStart;
  if( lumaUsesISP && bestCostSoFar >= maxCostAllowed )
  {
    cu.ispMode = 0;
  }
}


void IntraSearch::saveCuAreaCostInSCIPU( Area area, double cost )
{
  if( m_numCuInSCIPU < NUM_INTER_CU_INFO_SAVE )
  {
    m_cuAreaInSCIPU[m_numCuInSCIPU] = area;
    m_cuCostInSCIPU[m_numCuInSCIPU] = cost;
    m_numCuInSCIPU++;
  }
}

void IntraSearch::initCuAreaCostInSCIPU()
{
  for( int i = 0; i < NUM_INTER_CU_INFO_SAVE; i++ )
  {
    m_cuAreaInSCIPU[i] = Area();
    m_cuCostInSCIPU[i] = 0;
  }
  m_numCuInSCIPU = 0;
}
void IntraSearch::PLTSearch(CodingStructure &cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp)
{
  CodingUnit    &cu = *cs.getCU(partitioner.chType);
  TransformUnit &tu = *cs.getTU(partitioner.chType);
  uint32_t height = cu.block(compBegin).height;
  uint32_t width = cu.block(compBegin).width;
  if (m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
  {
    cs.getPredBuf().copyFrom(cs.getOrgBuf());
    cs.getPredBuf().Y().rspSignal(m_pcReshape->getFwdLUT());
  }
  if( cu.isLocalSepTree() )
  {
    cs.prevPLT.curPLTSize[compBegin] = cs.prevPLT.curPLTSize[COMPONENT_Y];
  }
  cu.lastPLTSize[compBegin] = cs.prevPLT.curPLTSize[compBegin];
  //derive palette
  derivePLTLossy(cs, partitioner, compBegin, numComp);//获得调色板
  reorderPLT(cs, partitioner, compBegin, numComp);//把可复用的放在前面 不可放在后面

  //下面做扫描优化 把部分用的少的项去除
  bool idxExist[MAXPLTSIZE + 1] = { false };
  preCalcPLTIndexRD(cs, partitioner, compBegin, numComp); // Pre-calculate distortions for each pixel
  double rdCost = MAX_DOUBLE;
  deriveIndexMap(cs, partitioner, compBegin, numComp, PLT_SCAN_HORTRAV, rdCost, idxExist); // Optimize palette index map (horizontal scan)
  if ((cu.curPLTSize[compBegin] + cu.useEscape[compBegin]) > 1)
  {
    deriveIndexMap(cs, partitioner, compBegin, numComp, PLT_SCAN_VERTRAV, rdCost, idxExist); // Optimize palette index map (vertical scan)
  }
  // Remove unused palette entries
  uint8_t newPLTSize = 0;
  int idxMapping[MAXPLTSIZE + 1];
  memset(idxMapping, -1, sizeof(int) * (MAXPLTSIZE + 1));
  for (int i = 0; i < cu.curPLTSize[compBegin]; i++)
  {
    if (idxExist[i])
    {
      idxMapping[i] = newPLTSize;
      newPLTSize++;
    }
  }
  idxMapping[cu.curPLTSize[compBegin]] = cu.useEscape[compBegin]? newPLTSize: -1;
  if (newPLTSize != cu.curPLTSize[compBegin]) // there exist unused palette entries
  { // update palette table and reuseflag
    Pel curPLTtmp[MAX_NUM_COMPONENT][MAXPLTSIZE];
    int reuseFlagIdx = 0, curPLTtmpIdx = 0, reuseEntrySize = 0;
    memset(cu.reuseflag[compBegin], false, sizeof(bool) * MAXPLTPREDSIZE);
    int compBeginTmp = compBegin;
    int numCompTmp   = numComp;
    if( cu.isLocalSepTree() )
    {
      memset(cu.reuseflag[COMPONENT_Y], false, sizeof(bool) * MAXPLTPREDSIZE);
      compBeginTmp = COMPONENT_Y;
      numCompTmp   = (cu.chromaFormat != CHROMA_400) ? 3 : 1;
    }
    for (int curIdx = 0; curIdx < cu.curPLTSize[compBegin]; curIdx++)
    {
      if (idxExist[curIdx])
      {
        for (int comp = compBeginTmp; comp < (compBeginTmp + numCompTmp); comp++)
          curPLTtmp[comp][curPLTtmpIdx] = cu.curPLT[comp][curIdx];

        // Update reuse flags
        if (curIdx < cu.reusePLTSize[compBegin])
        {
          bool match = false;
          for (; reuseFlagIdx < cs.prevPLT.curPLTSize[compBegin]; reuseFlagIdx++)
          {
            bool matchTmp = true;
            for (int comp = compBegin; comp < (compBegin + numComp); comp++)
            {
              matchTmp = matchTmp && (curPLTtmp[comp][curPLTtmpIdx] == cs.prevPLT.curPLT[comp][reuseFlagIdx]);
            }
            if (matchTmp)
            {
              match = true;
              break;
            }
          }
          if (match)
          {
            cu.reuseflag[compBegin][reuseFlagIdx] = true;
            if( cu.isLocalSepTree() )
            {
              cu.reuseflag[COMPONENT_Y][reuseFlagIdx] = true;
            }
            reuseEntrySize++;
          }
        }
        curPLTtmpIdx++;
      }
    }
    cu.reusePLTSize[compBegin] = reuseEntrySize;
    // update palette table
    cu.curPLTSize[compBegin] = newPLTSize;
    if( cu.isLocalSepTree() )
    {
      cu.curPLTSize[COMPONENT_Y] = newPLTSize;
    }
    for (int comp = compBeginTmp; comp < (compBeginTmp + numCompTmp); comp++)
    {
      memcpy( cu.curPLT[comp], curPLTtmp[comp], sizeof(Pel)*cu.curPLTSize[compBegin]);
    }
  }
  cu.useRotation[compBegin] = m_bestScanRotationMode;
  int indexMaxSize = cu.useEscape[compBegin] ? (cu.curPLTSize[compBegin] + 1) : cu.curPLTSize[compBegin];
  if (indexMaxSize <= 1)
  {
    cu.useRotation[compBegin] = false;
  }
  //reconstruct pixel
  PelBuf    curPLTIdx = tu.getcurPLTIdx(compBegin);
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      curPLTIdx.at(x, y) = idxMapping[curPLTIdx.at(x, y)];
      if (curPLTIdx.at(x, y) == cu.curPLTSize[compBegin])
      {
        calcPixelPred(cs, partitioner, y, x, compBegin, numComp);
      }
      else
      {
        for (uint32_t compID = compBegin; compID < (compBegin + numComp); compID++)
        {
          CompArea area = cu.blocks[compID];
          PelBuf   recBuf = cs.getRecoBuf(area);
          uint32_t scaleX = getComponentScaleX((ComponentID)COMPONENT_Cb, cs.sps->getChromaFormatIdc());
          uint32_t scaleY = getComponentScaleY((ComponentID)COMPONENT_Cb, cs.sps->getChromaFormatIdc());
          if (compBegin != COMPONENT_Y || compID == COMPONENT_Y)
          {
            recBuf.at(x, y) = cu.curPLT[compID][curPLTIdx.at(x, y)];
          }
          else if (compBegin == COMPONENT_Y && compID != COMPONENT_Y && y % (1 << scaleY) == 0 && x % (1 << scaleX) == 0)
          {
            recBuf.at(x >> scaleX, y >> scaleY) = cu.curPLT[compID][curPLTIdx.at(x, y)];
          }
        }
      }
    }
  }

  cs.getPredBuf().fill(0);
  cs.getResiBuf().fill(0);
  cs.getOrgResiBuf().fill(0);

  cs.fracBits = MAX_UINT;
  cs.cost = MAX_DOUBLE;
  Distortion distortion = 0;
  for (uint32_t comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    const ComponentID compID = ComponentID(comp);
    CPelBuf reco = cs.getRecoBuf(compID);
    CPelBuf org = cs.getOrgBuf(compID);
#if WCG_EXT
    if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (
      m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())))
    {
      const CPelBuf orgLuma = cs.getOrgBuf(cs.area.blocks[COMPONENT_Y]);

      if (compID == COMPONENT_Y && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
      {
        const CompArea &areaY = cu.Y();
        CompArea tmpArea1(COMPONENT_Y, areaY.chromaFormat, Position(0, 0), areaY.size());
        PelBuf   tmpRecLuma = m_tmpStorageLCU.getBuf(tmpArea1);
        tmpRecLuma.copyFrom(reco);
        tmpRecLuma.rspSignal(m_pcReshape->getInvLUT());
        distortion += m_pcRdCost->getDistPart(org, tmpRecLuma, cs.sps->getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
      }
      else
      {
        distortion += m_pcRdCost->getDistPart(org, reco, cs.sps->getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
      }
    }
    else
#endif
    {
      distortion += m_pcRdCost->getDistPart(org, reco, cs.sps->getBitDepth(toChannelType(compID)), compID, DF_SSE);
    }
  }

  cs.dist += distortion;
  const CompArea &area = cu.blocks[compBegin];
  cs.setDecomp(area);
  cs.picture->getRecoBuf(area).copyFrom(cs.getRecoBuf(area));
}
void IntraSearch::calcPixelPredRD(CodingStructure& cs, Partitioner& partitioner, Pel* orgBuf, Pel* paPixelValue, Pel* paRecoValue, ComponentID compBegin, uint32_t numComp)
{
  CodingUnit &cu = *cs.getCU(partitioner.chType);
  TransformUnit &tu = *cs.getTU(partitioner.chType);

  int qp[3];
  int qpRem[3];
  int qpPer[3];
  int quantiserScale[3];
  int quantiserRightShift[3];
  int rightShiftOffset[3];
  int invquantiserRightShift[3];
  int add[3];
  for (uint32_t ch = compBegin; ch < (compBegin + numComp); ch++)
  {
    QpParam cQP(tu, ComponentID(ch));
    qp[ch] = cQP.Qp(true);
    qpRem[ch] = qp[ch] % 6;
    qpPer[ch] = qp[ch] / 6;
    quantiserScale[ch] = g_quantScales[0][qpRem[ch]];
    quantiserRightShift[ch] = QUANT_SHIFT + qpPer[ch];
    rightShiftOffset[ch] = 1 << (quantiserRightShift[ch] - 1);
    invquantiserRightShift[ch] = IQUANT_SHIFT;
    add[ch] = 1 << (invquantiserRightShift[ch] - 1);
  }

  for (uint32_t ch = compBegin; ch < (compBegin + numComp); ch++)
  {
    const int  channelBitDepth = cu.cs->sps->getBitDepth(toChannelType((ComponentID)ch));
    paPixelValue[ch] = Pel(std::max<int>(0, ((orgBuf[ch] * quantiserScale[ch] + rightShiftOffset[ch]) >> quantiserRightShift[ch])));
    assert(paPixelValue[ch] < (1 << (channelBitDepth + 1)));
    paRecoValue[ch] = (((paPixelValue[ch] * g_invQuantScales[0][qpRem[ch]]) << qpPer[ch]) + add[ch]) >> invquantiserRightShift[ch];
    paRecoValue[ch] = Pel(ClipBD<int>(paRecoValue[ch], channelBitDepth));//to be checked
  }
}

void IntraSearch::preCalcPLTIndexRD(CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp)
{
  CodingUnit &cu = *cs.getCU(partitioner.chType);
  uint32_t height = cu.block(compBegin).height;
  uint32_t width = cu.block(compBegin).width;
  bool lossless = (m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && cs.slice->isLossless());

  CPelBuf   orgBuf[3];
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    CompArea  area = cu.blocks[comp];
    if (m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
    {
      orgBuf[comp] = cs.getPredBuf(area);
    }
    else
    {
      orgBuf[comp] = cs.getOrgBuf(area);
    }
  }

  int rasPos;
  uint32_t scaleX = getComponentScaleX(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  uint32_t scaleY = getComponentScaleY(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      rasPos = y * width + x;;
      // chroma discard
      bool discardChroma = (compBegin == COMPONENT_Y) && (y&scaleY || x&scaleX);
      Pel curPel[3];
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        uint32_t pX1 = (comp > 0 && compBegin == COMPONENT_Y) ? (x >> scaleX) : x;
        uint32_t pY1 = (comp > 0 && compBegin == COMPONENT_Y) ? (y >> scaleY) : y;
        curPel[comp] = orgBuf[comp].at(pX1, pY1);
      }

      uint8_t  pltIdx = 0;
      double minError = MAX_DOUBLE;
      uint8_t  bestIdx = 0;
      for (uint8_t z = 0; z < cu.curPLTSize[compBegin]; z++)
      {
        m_indexError[z][rasPos] = minError;
      }
      while (pltIdx < cu.curPLTSize[compBegin])
      {
        uint64_t sqrtError = 0;
        if (lossless)
        {
          for (int comp = compBegin; comp < (discardChroma ? 1 : (compBegin + numComp)); comp++)
          {
            sqrtError += int64_t(abs(curPel[comp] - cu.curPLT[comp][pltIdx]));
          }
          if (sqrtError == 0)
          {
            m_indexError[pltIdx][rasPos] = (double) sqrtError;
            minError                     = (double) sqrtError;
            bestIdx                      = pltIdx;
            break;
          }
        }
        else
        {
          for (int comp = compBegin; comp < (discardChroma ? 1 : (compBegin + numComp)); comp++)
          {
            int64_t tmpErr = int64_t(curPel[comp] - cu.curPLT[comp][pltIdx]);
            if (isChroma((ComponentID) comp))
            {
              sqrtError += uint64_t(tmpErr * tmpErr * ENC_CHROMA_WEIGHTING);
            }
            else
            {
              sqrtError += tmpErr * tmpErr;
            }
          }
          m_indexError[pltIdx][rasPos] = (double) sqrtError;
          if (sqrtError < minError)
          {
            minError = (double) sqrtError;
            bestIdx  = pltIdx;
          }
        }
        pltIdx++;
      }

      Pel paPixelValue[3], paRecoValue[3];
      if (!lossless)
      {
        calcPixelPredRD(cs, partitioner, curPel, paPixelValue, paRecoValue, compBegin, numComp);
      }
      uint64_t error = 0, rate = 0;
      for (int comp = compBegin; comp < (discardChroma ? 1 : (compBegin + numComp)); comp++)
      {
        if (lossless)
        {
          rate += getEpExGolombNumBins(curPel[comp], 5);
        }
        else
        {
          int64_t tmpErr = int64_t(curPel[comp] - paRecoValue[comp]);
          if (isChroma((ComponentID) comp))
          {
            error += uint64_t(tmpErr * tmpErr * ENC_CHROMA_WEIGHTING);
          }
          else
          {
            error += tmpErr * tmpErr;
          }
          rate += getEpExGolombNumBins(paPixelValue[comp], 5);   // encode quantized escape color
        }
      }
      double rdCost = (double)error + m_pcRdCost->getLambda()*(double)rate;
      m_indexError[cu.curPLTSize[compBegin]][rasPos] = rdCost;
      if (rdCost < minError)
      {
        minError = rdCost;
        bestIdx = (uint8_t)cu.curPLTSize[compBegin];
      }
      m_minErrorIndexMap[rasPos] = bestIdx; // save the optimal index of the current pixel
    }
  }
}

void IntraSearch::deriveIndexMap(CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp, PLTScanMode pltScanMode, double& dMinCost, bool* idxExist)
{
  CodingUnit    &cu = *cs.getCU(partitioner.chType);
  TransformUnit &tu = *cs.getTU(partitioner.chType);
  uint32_t      height = cu.block(compBegin).height;
  uint32_t      width = cu.block(compBegin).width;

  int   total     = height*width;
  Pel  *runIndex = tu.getPLTIndex(compBegin);
  bool *runType  = tu.getRunTypes(compBegin);
  m_scanOrder = g_scanOrder[SCAN_UNGROUPED][pltScanMode ? SCAN_TRAV_VER : SCAN_TRAV_HOR][gp_sizeIdxInfo->idxFrom(width)][gp_sizeIdxInfo->idxFrom(height)];
// Trellis initialization
  for (int i = 0; i < 2; i++)
  {
    memset(m_prevRunTypeRDOQ[i], 0, sizeof(Pel)*NUM_TRELLIS_STATE);
    memset(m_prevRunPosRDOQ[i],  0, sizeof(int)*NUM_TRELLIS_STATE);
    memset(m_stateCostRDOQ[i],  0, sizeof (double)*NUM_TRELLIS_STATE);
  }
  for (int state = 0; state < NUM_TRELLIS_STATE; state++)
  {
    m_statePtRDOQ[state][0] = 0;
  }
// Context modeling
  const FracBitsAccess& fracBits = m_CABACEstimator->getCtx().getFracBitsAcess();
  BinFracBits fracBitsPltCopyFlagIndex[RUN_IDX_THRE + 1];
  for (int dist = 0; dist <= RUN_IDX_THRE; dist++)
  {
    const unsigned  ctxId = DeriveCtx::CtxPltCopyFlag(PLT_RUN_INDEX, dist);
    fracBitsPltCopyFlagIndex[dist] = fracBits.getFracBitsArray(Ctx::IdxRunModel( ctxId ) );
  }
  BinFracBits fracBitsPltCopyFlagAbove[RUN_IDX_THRE + 1];
  for (int dist = 0; dist <= RUN_IDX_THRE; dist++)
  {
    const unsigned  ctxId = DeriveCtx::CtxPltCopyFlag(PLT_RUN_COPY, dist);
    fracBitsPltCopyFlagAbove[dist] = fracBits.getFracBitsArray(Ctx::CopyRunModel( ctxId ) );
  }
  const BinFracBits fracBitsPltRunType = fracBits.getFracBitsArray( Ctx::RunTypeFlag() );

// Trellis RDO per CG
  bool contTrellisRD = true;
  for (int subSetId = 0; ( subSetId <= (total - 1) >> LOG2_PALETTE_CG_SIZE ) && contTrellisRD; subSetId++)
  {
    int minSubPos = subSetId << LOG2_PALETTE_CG_SIZE;
    int maxSubPos = minSubPos + (1 << LOG2_PALETTE_CG_SIZE);
    maxSubPos = (maxSubPos > total) ? total : maxSubPos; // if last position is out of the current CU size
    contTrellisRD = deriveSubblockIndexMap(cs, partitioner, compBegin, pltScanMode, minSubPos, maxSubPos, fracBitsPltRunType, fracBitsPltCopyFlagIndex, fracBitsPltCopyFlagAbove, dMinCost, (bool)pltScanMode);
  }
  if (!contTrellisRD)
  {
    return;
  }


// best state at the last scan position
  double  sumRdCost = MAX_DOUBLE;
  uint8_t bestState = 0;
  for (uint8_t state = 0; state < NUM_TRELLIS_STATE; state++)
  {
    if (m_stateCostRDOQ[0][state] < sumRdCost)
    {
      sumRdCost = m_stateCostRDOQ[0][state];
      bestState = state;
    }
  }

     bool checkRunTable  [MAX_CU_BLKSIZE_PLT*MAX_CU_BLKSIZE_PLT];
  uint8_t checkIndexTable[MAX_CU_BLKSIZE_PLT*MAX_CU_BLKSIZE_PLT];
  uint8_t bestStateTable [MAX_CU_BLKSIZE_PLT*MAX_CU_BLKSIZE_PLT];
  uint8_t nextState = bestState;
// best trellis path
  for (int i = (width*height - 1); i >= 0; i--)
  {
    bestStateTable[i] = nextState;
    int rasterPos = m_scanOrder[i].idx;
    nextState = m_statePtRDOQ[nextState][rasterPos];
  }
// reconstruct index and runs based on the state pointers
  for (int i = 0; i < (width*height); i++)
  {
    int rasterPos = m_scanOrder[i].idx;
    int  abovePos = (pltScanMode == PLT_SCAN_HORTRAV) ? m_scanOrder[i].idx - width : m_scanOrder[i].idx - 1;
        nextState = bestStateTable[i];
    if ( nextState == 0 ) // same as the previous
    {
      checkRunTable[rasterPos] = checkRunTable[ m_scanOrder[i - 1].idx ];
      if ( checkRunTable[rasterPos] == PLT_RUN_INDEX )
      {
        checkIndexTable[rasterPos] = checkIndexTable[m_scanOrder[i - 1].idx];
      }
      else
      {
        checkIndexTable[rasterPos] = checkIndexTable[ abovePos ];
      }
    }
    else if (nextState == 1) // CopyAbove mode
    {
      checkRunTable[rasterPos] = PLT_RUN_COPY;
      checkIndexTable[rasterPos] = checkIndexTable[abovePos];
    }
    else if (nextState == 2) // Index mode
    {
      checkRunTable[rasterPos] = PLT_RUN_INDEX;
      checkIndexTable[rasterPos] = m_minErrorIndexMap[rasterPos];
    }
  }

// Escape flag
  m_bestEscape = false;
  for (int pos = 0; pos < (width*height); pos++)
  {
    uint8_t index = checkIndexTable[pos];
    if (index == cu.curPLTSize[compBegin])
    {
      m_bestEscape = true;
      break;
    }
  }

// Horizontal scan v.s vertical scan
  if (sumRdCost < dMinCost)
  {
    cu.useEscape[compBegin] = m_bestEscape;
    m_bestScanRotationMode = pltScanMode;
    memset(idxExist, false, sizeof(bool) * (MAXPLTSIZE + 1));
    for (int pos = 0; pos < (width*height); pos++)
    {
      runIndex[pos] = checkIndexTable[pos];
      runType[pos] = checkRunTable[pos];
      idxExist[checkIndexTable[pos]] = true;
    }
    dMinCost = sumRdCost;
  }
}

bool IntraSearch::deriveSubblockIndexMap(
  CodingStructure& cs,
  Partitioner&  partitioner,
  ComponentID   compBegin,
  PLTScanMode   pltScanMode,
  int           minSubPos,
  int           maxSubPos,
  const BinFracBits& fracBitsPltRunType,
  const BinFracBits* fracBitsPltIndexINDEX,
  const BinFracBits* fracBitsPltIndexCOPY,
  const double minCost,
  bool         useRotate
)
{
  CodingUnit &cu    = *cs.getCU(partitioner.chType);
  uint32_t   height = cu.block(compBegin).height;
  uint32_t   width  = cu.block(compBegin).width;
  int indexMaxValue = cu.curPLTSize[compBegin];

  int refId = 0;
  int currRasterPos, currScanPos, prevScanPos, aboveScanPos, roffset;
  int log2Width = (pltScanMode == PLT_SCAN_HORTRAV) ? floorLog2(width): floorLog2(height);
  int buffersize = (pltScanMode == PLT_SCAN_HORTRAV) ? 2*width: 2*height;
  for (int curPos = minSubPos; curPos < maxSubPos; curPos++)
  {
    currRasterPos = m_scanOrder[curPos].idx;
    prevScanPos = (curPos == 0) ? 0 : (curPos - 1) % buffersize;
    roffset = (curPos >> log2Width) << log2Width;
    aboveScanPos = roffset - (curPos - roffset + 1);
    aboveScanPos %= buffersize;
    currScanPos = curPos % buffersize;
    if ((pltScanMode == PLT_SCAN_HORTRAV && curPos < width) || (pltScanMode == PLT_SCAN_VERTRAV && curPos < height))
    {
      aboveScanPos = -1; // first column/row: above row is not valid
    }

// Trellis stats:
// 1st state: same as previous scanned sample
// 2nd state: Copy_Above mode
// 3rd state: Index mode
// Loop of current state
    for ( int curState = 0; curState < NUM_TRELLIS_STATE; curState++ )
    {
      double    minRdCost          = MAX_DOUBLE;
      int       minState           = 0; // best prevState
      uint8_t   bestRunIndex       = 0;
      bool      bestRunType        = 0;
      bool      bestPrevCodedType  = 0;
      int       bestPrevCodedPos   = 0;
      if ( ( curState == 0 && curPos == 0 ) || ( curState == 1 && aboveScanPos < 0 ) ) // state not available
      {
        m_stateCostRDOQ[1 - refId][curState] = MAX_DOUBLE;
        continue;
      }

      bool    runType  = 0;
      uint8_t runIndex = 0;
      if ( curState == 1 ) // 2nd state: Copy_Above mode
      {
        runType = PLT_RUN_COPY;
      }
      else if ( curState == 2 ) // 3rd state: Index mode
      {
        runType = PLT_RUN_INDEX;
        runIndex = m_minErrorIndexMap[currRasterPos];
      }

// Loop of previous state
      for ( int stateID = 0; stateID < NUM_TRELLIS_STATE; stateID++ )
      {
        if ( m_stateCostRDOQ[refId][stateID] == MAX_DOUBLE )
        {
          continue;
        }
        if ( curState == 0 ) // 1st state: same as previous scanned sample
        {
          runType = m_runMapRDOQ[refId][stateID][prevScanPos];
          runIndex = ( runType == PLT_RUN_INDEX ) ? m_indexMapRDOQ[refId][stateID][ prevScanPos ] : m_indexMapRDOQ[refId][stateID][ aboveScanPos ];
        }
        else if ( curState == 1 ) // 2nd state: Copy_Above mode
        {
          runIndex = m_indexMapRDOQ[refId][stateID][aboveScanPos];
        }
        bool    prevRunType   = m_runMapRDOQ[refId][stateID][prevScanPos];
        uint8_t prevRunIndex  = m_indexMapRDOQ[refId][stateID][prevScanPos];
        uint8_t aboveRunIndex = (aboveScanPos >= 0) ? m_indexMapRDOQ[refId][stateID][aboveScanPos] : 0;
        int      dist = curPos - m_prevRunPosRDOQ[refId][stateID] - 1;
        double rdCost = m_stateCostRDOQ[refId][stateID];
        if ( rdCost >= minRdCost ) continue;

// Calculate Rd cost
        bool prevCodedRunType = m_prevRunTypeRDOQ[refId][stateID];
        int  prevCodedPos     = m_prevRunPosRDOQ [refId][stateID];
        const BinFracBits* fracBitsPt = (m_prevRunTypeRDOQ[refId][stateID] == PLT_RUN_INDEX) ? fracBitsPltIndexINDEX : fracBitsPltIndexCOPY;
        rdCost += rateDistOptPLT(runType, runIndex, prevRunType, prevRunIndex, aboveRunIndex, prevCodedRunType, prevCodedPos, curPos, (pltScanMode == PLT_SCAN_HORTRAV) ? width : height, dist, indexMaxValue, fracBitsPt, fracBitsPltRunType);
        if (rdCost < minRdCost) // update minState ( minRdCost )
        {
          minRdCost    = rdCost;
          minState     = stateID;
          bestRunType  = runType;
          bestRunIndex = runIndex;
          bestPrevCodedType = prevCodedRunType;
          bestPrevCodedPos  = prevCodedPos;
        }
      }
// Update trellis info of current state
      m_stateCostRDOQ  [1 - refId][curState]  = minRdCost;
      m_prevRunTypeRDOQ[1 - refId][curState]  = bestPrevCodedType;
      m_prevRunPosRDOQ [1 - refId][curState]  = bestPrevCodedPos;
      m_statePtRDOQ[curState][currRasterPos] = minState;
      int buffer2update = std::min(buffersize, curPos);
      memcpy(m_indexMapRDOQ[1 - refId][curState], m_indexMapRDOQ[refId][minState], sizeof(uint8_t)*buffer2update);
      memcpy(m_runMapRDOQ[1 - refId][curState], m_runMapRDOQ[refId][minState], sizeof(bool)*buffer2update);
      m_indexMapRDOQ[1 - refId][curState][currScanPos] = bestRunIndex;
      m_runMapRDOQ  [1 - refId][curState][currScanPos] = bestRunType;
    }

    if (useRotate) // early terminate: Rd cost >= min cost in horizontal scan
    {
      if ((m_stateCostRDOQ[1 - refId][0] >= minCost) &&
         (m_stateCostRDOQ[1 - refId][1] >= minCost) &&
         (m_stateCostRDOQ[1 - refId][2] >= minCost) )
      {
        return 0;
      }
    }
    refId = 1 - refId;
  }
  return 1;
}

double IntraSearch::rateDistOptPLT(
  bool      runType,
  uint8_t   runIndex,
  bool      prevRunType,
  uint8_t   prevRunIndex,
  uint8_t   aboveRunIndex,
  bool&     prevCodedRunType,
  int&      prevCodedPos,
  int       scanPos,
  uint32_t  width,
  int       dist,
  int       indexMaxValue,
  const BinFracBits* IndexfracBits,
  const BinFracBits& TypefracBits)
{
  double rdCost = 0.0;
  bool identityFlag = !( (runType != prevRunType) || ( (runType == PLT_RUN_INDEX) && (runIndex != prevRunIndex) ) );

  if ( ( !identityFlag && runType == PLT_RUN_INDEX ) || scanPos == 0 ) // encode index value
  {
    uint8_t refIndex = (prevRunType == PLT_RUN_INDEX) ? prevRunIndex : aboveRunIndex;
    refIndex = (scanPos == 0) ? ( indexMaxValue + 1) : refIndex;
    if ( runIndex == refIndex )
    {
      rdCost = MAX_DOUBLE;
      return rdCost;
    }
    rdCost += m_pcRdCost->getLambda()*(getTruncBinBits((runIndex > refIndex) ? runIndex - 1 : runIndex, (scanPos == 0) ? (indexMaxValue + 1) : indexMaxValue)  << SCALE_BITS);
  }
  rdCost += m_indexError[runIndex][m_scanOrder[scanPos].idx] * (1 << SCALE_BITS);
  if (scanPos > 0)
  {
    rdCost += m_pcRdCost->getLambda()*( identityFlag ? (IndexfracBits[(dist < RUN_IDX_THRE) ? dist : RUN_IDX_THRE].intBits[1]) : (IndexfracBits[(dist < RUN_IDX_THRE) ? dist : RUN_IDX_THRE].intBits[0] ) );
  }
  if ( !identityFlag && scanPos >= width && prevRunType != PLT_RUN_COPY )
  {
    rdCost += m_pcRdCost->getLambda()*TypefracBits.intBits[runType];
  }
  if (!identityFlag || scanPos == 0)
  {
    prevCodedRunType = runType;
    prevCodedPos = scanPos;
  }
  return rdCost;
}

uint32_t IntraSearch::getEpExGolombNumBins(uint32_t symbol, uint32_t count)
{
  uint32_t numBins = 0;
  while (symbol >= (uint32_t)(1 << count))
  {
    numBins++;
    symbol -= 1 << count;
    count++;
  }
  numBins++;
  numBins += count;
  assert(numBins <= 32);
  return numBins;
}

uint32_t IntraSearch::getTruncBinBits(uint32_t symbol, uint32_t maxSymbol)
{
  uint32_t idxCodeBit = 0;
  uint32_t thresh;
  if (maxSymbol > 256)
  {
    uint32_t threshVal = 1 << 8;
    thresh = 8;
    while (threshVal <= maxSymbol)
    {
      thresh++;
      threshVal <<= 1;
    }
    thresh--;
  }
  else
  {
    thresh = g_tbMax[maxSymbol];
  }
  uint32_t uiVal = 1 << thresh;
  assert(uiVal <= maxSymbol);
  assert((uiVal << 1) > maxSymbol);
  assert(symbol < maxSymbol);
  uint32_t b = maxSymbol - uiVal;
  assert(b < uiVal);
  if (symbol < uiVal - b)
  {
    idxCodeBit = thresh;
  }
  else
  {
    idxCodeBit = thresh + 1;
  }
  return idxCodeBit;
}

void IntraSearch::calcPixelPred(CodingStructure& cs, Partitioner& partitioner, uint32_t yPos, uint32_t xPos, ComponentID compBegin, uint32_t numComp)
{
  CodingUnit    &cu = *cs.getCU(partitioner.chType);
  TransformUnit &tu = *cs.getTU(partitioner.chType);
  bool lossless = (m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && cs.slice->isLossless());

  CPelBuf   orgBuf[3];
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    CompArea  area = cu.blocks[comp];
    if (m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
    {
      orgBuf[comp] = cs.getPredBuf(area);
    }
    else
    {
      orgBuf[comp] = cs.getOrgBuf(area);
    }
  }

  int qp[3];
  int qpRem[3];
  int qpPer[3];
  int quantiserScale[3];
  int quantiserRightShift[3];
  int rightShiftOffset[3];
  int invquantiserRightShift[3];
  int add[3];
  if (!lossless)
  {
    for (uint32_t ch = compBegin; ch < (compBegin + numComp); ch++)
    {
      QpParam cQP(tu, ComponentID(ch));
      qp[ch]                     = cQP.Qp(true);
      qpRem[ch]                  = qp[ch] % 6;
      qpPer[ch]                  = qp[ch] / 6;
      quantiserScale[ch]         = g_quantScales[0][qpRem[ch]];
      quantiserRightShift[ch]    = QUANT_SHIFT + qpPer[ch];
      rightShiftOffset[ch]       = 1 << (quantiserRightShift[ch] - 1);
      invquantiserRightShift[ch] = IQUANT_SHIFT;
      add[ch]                    = 1 << (invquantiserRightShift[ch] - 1);
    }
  }

  uint32_t scaleX = getComponentScaleX(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  uint32_t scaleY = getComponentScaleY(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  for (uint32_t ch = compBegin; ch < (compBegin + numComp); ch++)
  {
    const int channelBitDepth = cu.cs->sps->getBitDepth(toChannelType((ComponentID)ch));
    CompArea  area = cu.blocks[ch];
    PelBuf    recBuf = cs.getRecoBuf(area);
    PLTescapeBuf escapeValue = tu.getescapeValue((ComponentID)ch);
    if (compBegin != COMPONENT_Y || ch == 0)
    {
      if (lossless)
      {
        escapeValue.at(xPos, yPos) = orgBuf[ch].at(xPos, yPos);
        recBuf.at(xPos, yPos)      = orgBuf[ch].at(xPos, yPos);
      }
      else
      {
      escapeValue.at(xPos, yPos) = std::max<TCoeff>(0, ((orgBuf[ch].at(xPos, yPos) * quantiserScale[ch] + rightShiftOffset[ch]) >> quantiserRightShift[ch]));
      assert(escapeValue.at(xPos, yPos) < (TCoeff(1) << (channelBitDepth + 1)));
      TCoeff value = (((escapeValue.at(xPos, yPos)*g_invQuantScales[0][qpRem[ch]]) << qpPer[ch]) + add[ch]) >> invquantiserRightShift[ch];
      recBuf.at(xPos, yPos) = Pel(ClipBD<TCoeff>(value, channelBitDepth));//to be checked
      }
    }
    else if (compBegin == COMPONENT_Y && ch > 0 && yPos % (1 << scaleY) == 0 && xPos % (1 << scaleX) == 0)
    {
      uint32_t yPosC = yPos >> scaleY;
      uint32_t xPosC = xPos >> scaleX;
      if (lossless)
      {
        escapeValue.at(xPosC, yPosC) = orgBuf[ch].at(xPosC, yPosC);
        recBuf.at(xPosC, yPosC)      = orgBuf[ch].at(xPosC, yPosC);
      }
      else
      {
        escapeValue.at(xPosC, yPosC) = std::max<TCoeff>(
          0, ((orgBuf[ch].at(xPosC, yPosC) * quantiserScale[ch] + rightShiftOffset[ch]) >> quantiserRightShift[ch]));
        assert(escapeValue.at(xPosC, yPosC) < (TCoeff(1) << (channelBitDepth + 1)));
        TCoeff value = (((escapeValue.at(xPosC, yPosC) * g_invQuantScales[0][qpRem[ch]]) << qpPer[ch]) + add[ch])
                       >> invquantiserRightShift[ch];
        recBuf.at(xPosC, yPosC) = Pel(ClipBD<TCoeff>(value, channelBitDepth));   // to be checked
      }
    }
  }
}

void IntraSearch::derivePLTLossy(CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp)
{
  CodingUnit &cu = *cs.getCU(partitioner.chType);
  const int channelBitDepth_L = cs.sps->getBitDepth(CHANNEL_TYPE_LUMA);
  const int channelBitDepth_C = cs.sps->getBitDepth(CHANNEL_TYPE_CHROMA);
  bool lossless        = (m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && cs.slice->isLossless());
  int  pcmShiftRight_L = (channelBitDepth_L - PLT_ENCBITDEPTH);
  int  pcmShiftRight_C = (channelBitDepth_C - PLT_ENCBITDEPTH);
  if (lossless)
  {
    pcmShiftRight_L = 0;
    pcmShiftRight_C = 0;
  }

  int maxPltSize = cu.isSepTree() ? MAXPLTSIZE_DUALTREE : MAXPLTSIZE;

  uint32_t height = cu.block(compBegin).height;
  uint32_t width = cu.block(compBegin).width;

  CPelBuf   orgBuf[3];
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    CompArea  area = cu.blocks[comp];
    if (m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
    {
      orgBuf[comp] = cs.getPredBuf(area);
    }
    else
    {
      orgBuf[comp] = cs.getOrgBuf(area);
    }
  }

  TransformUnit &tu = *cs.getTU(partitioner.chType);
  QpParam cQP(tu, compBegin);
  int qp = cQP.Qp(true) - 6*(channelBitDepth_L - 8);
  qp = (qp < 0) ? 0 : ((qp > 56) ? 56 : qp);
  int errorLimit = g_paletteQuant[qp];//根据QP得到颜色项误差上限

  if (lossless)
  {
    errorLimit = 0;
  }
  uint32_t totalSize = height*width;
  SortingElement *pelList = new SortingElement[totalSize];
  SortingElement  element;
  SortingElement *pelListSort = new SortingElement[MAXPLTSIZE + 1];//最大变成了32 HEVC是64
  uint32_t dictMaxSize = maxPltSize;
  uint32_t idx = 0;
  int last = -1;

  uint32_t scaleX = getComponentScaleX(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  uint32_t scaleY = getComponentScaleY(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      uint32_t org[3], pX, pY;
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        pX = (comp > 0 && compBegin == COMPONENT_Y) ? (x >> scaleX) : x;
        pY = (comp > 0 && compBegin == COMPONENT_Y) ? (y >> scaleY) : y;
        org[comp] = orgBuf[comp].at(pX, pY);
      }
      //设置当前像素的element
      element.setAll(org, compBegin, numComp);

      ComponentID tmpCompBegin = compBegin;
      int tmpNumComp = numComp;
      if( cs.sps->getChromaFormatIdc() != CHROMA_444 &&
          numComp == 3 &&
         (x != ((x >> scaleX) << scaleX) || (y != ((y >> scaleY) << scaleY))) )
      {
        tmpCompBegin = COMPONENT_Y;
        tmpNumComp   = 1;
      }
      //last==-1说明是第一项 所以bestSAD设置为max
      //首先计算当前像素和上一个像素的SAD情况
      int besti = last, bestSAD = (last == -1) ? MAX_UINT : pelList[last].getSAD(element, cs.sps->getBitDepths(), tmpCompBegin, tmpNumComp, lossless);
      //无损情况下
      if (lossless)
      {
        if (bestSAD)
        {
          for (int i = idx - 1; i >= 0; i--)
          {
            uint32_t sad = pelList[i].getSAD(element, cs.sps->getBitDepths(), tmpCompBegin, tmpNumComp, lossless);
            if (sad == 0)
            {
              bestSAD = sad;
              besti   = i;
              break;
            }
          }
        }
      }
      else
      {
        if (bestSAD)
        {
          //idx=0时不进行 
          //比较当前项和调色板中所有项的SAD
          for (int i = idx - 1; i >= 0; i--)
          {
            uint32_t sad = pelList[i].getSAD(element, cs.sps->getBitDepths(), tmpCompBegin, tmpNumComp, lossless);
            if (sad < bestSAD)
            {
              bestSAD = sad;
              besti   = i;
              if (!sad)
              {
                break;
              }
            }
          }
        }
      }
      //把当前项加入到调色板中某一项中
      if (besti >= 0 && pelList[besti].almostEqualData(element, errorLimit, cs.sps->getBitDepths(), tmpCompBegin, tmpNumComp, lossless))
      {
        pelList[besti].addElement(element, tmpCompBegin, tmpNumComp);
        last = besti;
      }
      else
      {
        //第一个像素时，直接把当前像素加入调色板
        pelList[idx].copyDataFrom(element, tmpCompBegin, tmpNumComp);
        for (int comp = tmpCompBegin; comp < (tmpCompBegin + tmpNumComp); comp++)
        {
          pelList[idx].setCnt(1, comp);
        }
        last = idx;
        idx++;
      }
    }
  }

  if( cs.sps->getChromaFormatIdc() != CHROMA_444 && numComp == 3 )
  {
    for( int i = 0; i < idx; i++ )
    {
      pelList[i].setCnt( pelList[i].getCnt(COMPONENT_Y) + (pelList[i].getCnt(COMPONENT_Cb) >> 2), MAX_NUM_COMPONENT);
    }
  }
  else
  {
    if( compBegin == 0 )
    {
      for( int i = 0; i < idx; i++ )
      {
        pelList[i].setCnt(pelList[i].getCnt(COMPONENT_Y), COMPONENT_Cb);
        pelList[i].setCnt(pelList[i].getCnt(COMPONENT_Y), COMPONENT_Cr);
        pelList[i].setCnt(pelList[i].getCnt(COMPONENT_Y), MAX_NUM_COMPONENT);
      }
    }
    else
    {
      for( int i = 0; i < idx; i++ )
      {
        pelList[i].setCnt(pelList[i].getCnt(COMPONENT_Cb), COMPONENT_Y);
        pelList[i].setCnt(pelList[i].getCnt(COMPONENT_Cb), MAX_NUM_COMPONENT);
      }
    }
  }

  for (int i = 0; i < dictMaxSize; i++)
  {
    pelListSort[i].setCnt(0, COMPONENT_Y);
    pelListSort[i].setCnt(0, COMPONENT_Cb);
    pelListSort[i].setCnt(0, COMPONENT_Cr);
    pelListSort[i].setCnt(0, MAX_NUM_COMPONENT);
    pelListSort[i].resetAll(compBegin, numComp);
  }

  //bubble sorting
  //按照每一项的频次 冒泡排序
  dictMaxSize = 1;
  for (int i = 0; i < idx; i++)
  {
    if( pelList[i].getCnt(MAX_NUM_COMPONENT) > pelListSort[dictMaxSize - 1].getCnt(MAX_NUM_COMPONENT) )
    {
      int j;
      for (j = dictMaxSize; j > 0; j--)
      {
        if (pelList[i].getCnt(MAX_NUM_COMPONENT) > pelListSort[j - 1].getCnt(MAX_NUM_COMPONENT))
        {
          pelListSort[j].copyAllFrom(pelListSort[j - 1], compBegin, numComp);
          dictMaxSize = std::min(dictMaxSize + 1, (uint32_t)maxPltSize);
        }
        else
        {
          break;
        }
      }
      pelListSort[j].copyAllFrom(pelList[i], compBegin, numComp);
    }
  }

  uint32_t paletteSize = 0;
  uint64_t numColorBits = 0;
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    numColorBits += (comp > 0) ? channelBitDepth_C : channelBitDepth_L;
  }
  const int plt_lambda_shift = (compBegin > 0) ? pcmShiftRight_C : pcmShiftRight_L;
  double    bitCost          = m_pcRdCost->getLambda() / (double) (1 << (2 * plt_lambda_shift)) * numColorBits;
  bool   reuseflag[MAXPLTPREDSIZE] = { false };
  int    run;//游程编码的长度
  double reuseflagCost;
  for (int i = 0; i < maxPltSize; i++)
  {
    if( pelListSort[i].getCnt(MAX_NUM_COMPONENT) )
    {
      ComponentID tmpCompBegin = compBegin;
      int tmpNumComp = numComp;
      if( cs.sps->getChromaFormatIdc() != CHROMA_444 && numComp == 3 && pelListSort[i].getCnt(COMPONENT_Cb) == 0 )
      {
        tmpCompBegin = COMPONENT_Y;
        tmpNumComp   = 1;
      }

      for( int comp = tmpCompBegin; comp < (tmpCompBegin + tmpNumComp); comp++ )
      {
        int half = pelListSort[i].getCnt(comp) >> 1;
        cu.curPLT[comp][paletteSize] = (pelListSort[i].getSumData(comp) + half) / pelListSort[i].getCnt(comp);
      }

      int best = -1;
      if( errorLimit )
      {
        //计算均方差
        double pal[MAX_NUM_COMPONENT], err = 0.0, bestCost = 0.0;
        for( int comp = tmpCompBegin; comp < (tmpCompBegin + tmpNumComp); comp++ )
        {
          pal[comp] = pelListSort[i].getSumData(comp) / (double)pelListSort[i].getCnt(comp);
          err = pal[comp] - cu.curPLT[comp][paletteSize];
          if( isChroma((ComponentID) comp) )
          {
            bestCost += (err * err * PLT_CHROMA_WEIGHTING) / (1 << (2 * pcmShiftRight_C)) * pelListSort[i].getCnt(comp);
          }
          else
          {
            bestCost += (err * err) / (1 << (2 * pcmShiftRight_L)) * pelListSort[i].getCnt(comp);
          }
        }
        bestCost += bitCost;
        //根据codingstructure中保存的之前的PLT对前面生成的PLT进行优化 
        //使用reuseflag标识对之前PLT的复用
        //在Pre中找curElement相近或相同的项
        for( int t = 0; t < cs.prevPLT.curPLTSize[compBegin]; t++ )
        {
          double cost = 0.0;
          for( int comp = tmpCompBegin; comp < (tmpCompBegin + tmpNumComp); comp++ )
          {
            err = pal[comp] - cs.prevPLT.curPLT[comp][t];
            if( isChroma((ComponentID) comp) )
            {
              cost += (err * err * PLT_CHROMA_WEIGHTING) / (1 << (2 * pcmShiftRight_C)) * pelListSort[i].getCnt(comp);
            }
            else
            {
              cost += (err * err) / (1 << (2 * pcmShiftRight_L)) * pelListSort[i].getCnt(comp);
            }
          }
          run = 0;
          //寻找能否复用
          for (int t2 = t; t2 >= 0; t2--)
          {
            if (!reuseflag[t2])
            {
              run++;
            }
            else
            {
              break;
            }
          }
          //不能复用的标志 需要花费的bit （run个0）
          reuseflagCost = m_pcRdCost->getLambda() / (double)(1 << (2 * plt_lambda_shift)) * getEpExGolombNumBins(run ? run + 1 : run, 0);
          cost += reuseflagCost;

          if( cost < bestCost )
          {
            best = t;
            bestCost = cost;
          }
        }
        if( best != -1 )
        {
          for( int comp = tmpCompBegin; comp < (tmpCompBegin + tmpNumComp); comp++ )
          {
            cu.curPLT[comp][paletteSize] = cs.prevPLT.curPLT[comp][best];
          }
          reuseflag[best] = true;
        }
      }
      //去重
      bool duplicate = false;
      if( pelListSort[i].getCnt(MAX_NUM_COMPONENT) == 1 && best == -1 )
      {
        duplicate = true;
      }
      else
      {
        for( int t = 0; t < paletteSize; t++ )
        {
          bool duplicateTmp = true;
          for( int comp = tmpCompBegin; comp < (tmpCompBegin + tmpNumComp); comp++ )
          {
            duplicateTmp = duplicateTmp && (cu.curPLT[comp][paletteSize] == cu.curPLT[comp][t]);
          }
          if( duplicateTmp )
          {
            duplicate = true;
            break;
          }
        }
      }
      if( !duplicate )
      {
        if( cs.sps->getChromaFormatIdc() != CHROMA_444 && numComp == 3 && pelListSort[i].getCnt(COMPONENT_Cb) == 0 )
        {
          if( best != -1 )
          {
            cu.curPLT[COMPONENT_Cb][paletteSize] = cs.prevPLT.curPLT[COMPONENT_Cb][best];
            cu.curPLT[COMPONENT_Cr][paletteSize] = cs.prevPLT.curPLT[COMPONENT_Cr][best];
          }
          else
          {
            cu.curPLT[COMPONENT_Cb][paletteSize] = 1 << (channelBitDepth_C - 1);
            cu.curPLT[COMPONENT_Cr][paletteSize] = 1 << (channelBitDepth_C - 1);
          }
        }
        paletteSize++;
      }
    }
    else
    {
      break;
    }
  }
  cu.curPLTSize[compBegin] = paletteSize;
  if( cu.isLocalSepTree() )
  {
    cu.curPLTSize[COMPONENT_Y] = paletteSize;
  }

  delete[] pelList;
  delete[] pelListSort;
}
// -------------------------------------------------------------------------------------------------------------------
// Intra search
// -------------------------------------------------------------------------------------------------------------------

void IntraSearch::xEncIntraHeader( CodingStructure &cs, Partitioner &partitioner, const bool &bLuma, const bool &bChroma, const int subTuIdx )
{
  CodingUnit &cu = *cs.getCU( partitioner.chType );

  if (bLuma)
  {
    bool isFirst = cu.ispMode ? subTuIdx == 0 : partitioner.currArea().lumaPos() == cs.area.lumaPos();

    // CU header
    if( isFirst )
    {
      if ((!cs.slice->isIntra() || cs.slice->getSPS()->getIBCFlag() || cs.slice->getSPS()->getPLTMode())
          && cu.Y().valid())
      {
        m_CABACEstimator->cu_skip_flag( cu );
        m_CABACEstimator->pred_mode   ( cu );
      }
      if (CU::isPLT(cu))
      {
        return;
      }
    }

    PredictionUnit &pu = *cs.getPU(partitioner.currArea().lumaPos(), partitioner.chType);

    // luma prediction mode
    if (isFirst)
    {
      if ( !cu.Y().valid())
      {
        m_CABACEstimator->pred_mode( cu );
      }
      m_CABACEstimator->bdpcm_mode( cu, COMPONENT_Y );
      m_CABACEstimator->intra_luma_pred_mode( pu );
    }
  }

  if (bChroma)
  {
    bool isFirst = partitioner.currArea().Cb().valid() && partitioner.currArea().chromaPos() == cs.area.chromaPos();

    PredictionUnit &pu = *cs.getPU( partitioner.currArea().chromaPos(), CHANNEL_TYPE_CHROMA );

    if( isFirst )
    {
      m_CABACEstimator->bdpcm_mode( cu, ComponentID(CHANNEL_TYPE_CHROMA) );
      m_CABACEstimator->intra_chroma_pred_mode( pu );
    }
  }
}

void IntraSearch::xEncSubdivCbfQT( CodingStructure &cs, Partitioner &partitioner, const bool &bLuma, const bool &bChroma, const int subTuIdx, const PartSplit ispType )
{
  const UnitArea &currArea = partitioner.currArea();
          int subTuCounter = subTuIdx;
  TransformUnit &currTU = *cs.getTU( currArea.blocks[partitioner.chType], partitioner.chType, subTuCounter );
  CodingUnit    &currCU = *currTU.cu;
  uint32_t currDepth           = partitioner.currTrDepth;

  const bool subdiv        = currTU.depth > currDepth;
  ComponentID compID = partitioner.chType == CHANNEL_TYPE_LUMA ? COMPONENT_Y : COMPONENT_Cb;

  if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
  {
    CHECK( !subdiv, "TU split implied" );
  }
  else
  {
    CHECK( subdiv && !currCU.ispMode && isLuma( compID ), "No TU subdivision is allowed with QTBT" );
  }

  if (bChroma)
  {
    const bool chromaCbfISP = currArea.blocks[COMPONENT_Cb].valid() && currCU.ispMode && !subdiv;
    if ( !currCU.ispMode || chromaCbfISP )
    {
      const uint32_t numberValidComponents = getNumberValidComponents(currArea.chromaFormat);
      const uint32_t cbfDepth              = (chromaCbfISP ? currDepth - 1 : currDepth);

      for (uint32_t ch = COMPONENT_Cb; ch < numberValidComponents; ch++)
      {
        const ComponentID compID = ComponentID(ch);

        if (currDepth == 0 || TU::getCbfAtDepth(currTU, compID, currDepth - 1) || chromaCbfISP)
        {
          const bool prevCbf = (compID == COMPONENT_Cr ? TU::getCbfAtDepth(currTU, COMPONENT_Cb, currDepth) : false);
          m_CABACEstimator->cbf_comp(cs, TU::getCbfAtDepth(currTU, compID, currDepth), currArea.blocks[compID],
                                     cbfDepth, prevCbf);
        }
      }
    }
  }

  if (subdiv)
  {
    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }
    else if( currCU.ispMode && isLuma( compID ) )
    {
      partitioner.splitCurrArea( ispType, cs );
    }
    else
    {
      THROW("Cannot perform an implicit split!");
    }

    do
    {
      xEncSubdivCbfQT( cs, partitioner, bLuma, bChroma, subTuCounter, ispType );
      subTuCounter += subTuCounter != -1 ? 1 : 0;
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
  }
  else
  {
    //===== Cbfs =====
    if (bLuma)
    {
      bool previousCbf       = false;
      bool lastCbfIsInferred = false;
      if( ispType != TU_NO_ISP )
      {
        bool rootCbfSoFar = false;
        uint32_t nTus = currCU.ispMode == HOR_INTRA_SUBPARTITIONS ? currCU.lheight() >> floorLog2(currTU.lheight()) : currCU.lwidth() >> floorLog2(currTU.lwidth());
        if( subTuCounter == nTus - 1 )
        {
          TransformUnit* tuPointer = currCU.firstTU;
          for( int tuIdx = 0; tuIdx < nTus - 1; tuIdx++ )
          {
            rootCbfSoFar |= TU::getCbfAtDepth( *tuPointer, COMPONENT_Y, currDepth );
            tuPointer = tuPointer->next;
          }
          if( !rootCbfSoFar )
          {
            lastCbfIsInferred = true;
          }
        }
        if( !lastCbfIsInferred )
        {
          previousCbf = TU::getPrevTuCbfAtDepth( currTU, COMPONENT_Y, partitioner.currTrDepth );
        }
      }
      if( !lastCbfIsInferred )
      {
        m_CABACEstimator->cbf_comp( cs, TU::getCbfAtDepth( currTU, COMPONENT_Y, currDepth ), currTU.Y(), currTU.depth, previousCbf, currCU.ispMode );
      }
    }
  }
}

void IntraSearch::xEncCoeffQT( CodingStructure &cs, Partitioner &partitioner, const ComponentID compID, const int subTuIdx, const PartSplit ispType, CUCtx* cuCtx )
{
  const UnitArea &currArea  = partitioner.currArea();

       int subTuCounter     = subTuIdx;
  TransformUnit &currTU     = *cs.getTU( currArea.blocks[partitioner.chType], partitioner.chType, subTuIdx );
  uint32_t      currDepth       = partitioner.currTrDepth;
  const bool subdiv         = currTU.depth > currDepth;

  if (subdiv)
  {
    if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs))
    {
      partitioner.splitCurrArea(TU_MAX_TR_SPLIT, cs);
    }
    else if( currTU.cu->ispMode )
    {
      partitioner.splitCurrArea( ispType, cs );
    }
    else
    {
      THROW("Implicit TU split not available!");
    }

    do
    {
      xEncCoeffQT( cs, partitioner, compID, subTuCounter, ispType, cuCtx );
      subTuCounter += subTuCounter != -1 ? 1 : 0;
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
  }
  else
  {
    if (currArea.blocks[compID].valid())
    {
      if (compID == COMPONENT_Cr)
      {
        const int cbfMask = (TU::getCbf(currTU, COMPONENT_Cb) ? 2 : 0) + (TU::getCbf(currTU, COMPONENT_Cr) ? 1 : 0);
        m_CABACEstimator->joint_cb_cr(currTU, cbfMask);
      }
      if (TU::getCbf(currTU, compID))
      {
        if (isLuma(compID))
        {
          m_CABACEstimator->residual_coding(currTU, compID, cuCtx);
          m_CABACEstimator->mts_idx(*currTU.cu, cuCtx);
        }
        else
        {
          m_CABACEstimator->residual_coding(currTU, compID);
        }
      }
    }
  }
}

uint64_t IntraSearch::xGetIntraFracBitsQT( CodingStructure &cs, Partitioner &partitioner, const bool &bLuma, const bool &bChroma, const int subTuIdx, const PartSplit ispType, CUCtx* cuCtx )
{
  m_CABACEstimator->resetBits();

  xEncIntraHeader( cs, partitioner, bLuma, bChroma, subTuIdx );
  xEncSubdivCbfQT( cs, partitioner, bLuma, bChroma, subTuIdx, ispType );

  if( bLuma )
  {
    xEncCoeffQT( cs, partitioner, COMPONENT_Y, subTuIdx, ispType, cuCtx );
  }
  if( bChroma )
  {
    xEncCoeffQT( cs, partitioner, COMPONENT_Cb, subTuIdx, ispType );
    xEncCoeffQT( cs, partitioner, COMPONENT_Cr, subTuIdx, ispType );
  }

  CodingUnit& cu = *cs.getCU(partitioner.chType);
  if ( cuCtx && bLuma && cu.isSepTree() && ( !cu.ispMode || ( cu.lfnstIdx && subTuIdx == 0 ) || ( !cu.lfnstIdx && subTuIdx == m_ispTestedModes[cu.lfnstIdx].numTotalParts[cu.ispMode - 1] - 1 ) ) )
  {
    m_CABACEstimator->residual_lfnst_mode(cu, *cuCtx);
  }

  uint64_t fracBits = m_CABACEstimator->getEstFracBits();
  return fracBits;
}

uint64_t IntraSearch::xGetIntraFracBitsQTSingleChromaComponent( CodingStructure &cs, Partitioner &partitioner, const ComponentID compID )
{
  m_CABACEstimator->resetBits();

  if( compID == COMPONENT_Cb )
  {
    //intra mode coding
    PredictionUnit &pu = *cs.getPU( partitioner.currArea().lumaPos(), partitioner.chType );
    m_CABACEstimator->intra_chroma_pred_mode( pu );
    //xEncIntraHeader(cs, partitioner, false, true);
  }
  CHECK( partitioner.currTrDepth != 1, "error in the depth!" );
  const UnitArea &currArea = partitioner.currArea();

  TransformUnit &currTU = *cs.getTU( currArea.blocks[partitioner.chType], partitioner.chType );

  //cbf coding
  const bool prevCbf = ( compID == COMPONENT_Cr ? TU::getCbfAtDepth( currTU, COMPONENT_Cb, partitioner.currTrDepth ) : false );
  m_CABACEstimator->cbf_comp( cs, TU::getCbfAtDepth( currTU, compID, partitioner.currTrDepth ), currArea.blocks[compID], partitioner.currTrDepth - 1, prevCbf );
  //coeffs coding and cross comp coding
  if( TU::getCbf( currTU, compID ) )
  {
    m_CABACEstimator->residual_coding( currTU, compID );
  }

  uint64_t fracBits = m_CABACEstimator->getEstFracBits();
  return fracBits;
}

uint64_t IntraSearch::xGetIntraFracBitsQTChroma(TransformUnit& currTU, const ComponentID &compID)
{
  m_CABACEstimator->resetBits();
  // Include Cbf and jointCbCr flags here as we make decisions across components
  CodingStructure &cs = *currTU.cs;

  if ( currTU.jointCbCr )
  {
    const int cbfMask = ( TU::getCbf( currTU, COMPONENT_Cb ) ? 2 : 0 ) + ( TU::getCbf( currTU, COMPONENT_Cr ) ? 1 : 0 );
    m_CABACEstimator->cbf_comp( cs, cbfMask>>1, currTU.blocks[ COMPONENT_Cb ], currTU.depth, false );
    m_CABACEstimator->cbf_comp( cs, cbfMask &1, currTU.blocks[ COMPONENT_Cr ], currTU.depth, cbfMask>>1 );
    if( cbfMask )
    {
      m_CABACEstimator->joint_cb_cr( currTU, cbfMask );
    }
    if( cbfMask >> 1 )
    {
      m_CABACEstimator->residual_coding( currTU, COMPONENT_Cb );
    }
    if( cbfMask & 1 )
    {
      m_CABACEstimator->residual_coding( currTU, COMPONENT_Cr );
    }
  }
  else
  {
    if ( compID == COMPONENT_Cb )
    {
      m_CABACEstimator->cbf_comp( cs, TU::getCbf( currTU, compID ), currTU.blocks[ compID ], currTU.depth, false );
    }
    else
    {
      const bool cbCbf    = TU::getCbf( currTU, COMPONENT_Cb );
      const bool crCbf    = TU::getCbf( currTU, compID );
      const int  cbfMask  = ( cbCbf ? 2 : 0 ) + ( crCbf ? 1 : 0 );
      m_CABACEstimator->cbf_comp( cs, crCbf, currTU.blocks[ compID ], currTU.depth, cbCbf );
      m_CABACEstimator->joint_cb_cr( currTU, cbfMask );
    }
  }

  if( !currTU.jointCbCr && TU::getCbf( currTU, compID ) )
  {
    m_CABACEstimator->residual_coding( currTU, compID );
  }

  uint64_t fracBits = m_CABACEstimator->getEstFracBits();
  return fracBits;
}


/*
* 进行帧内编码并计算相应的失真，用于上层函数对最佳预测模式和最佳变换模式的选择。
初始化使用模式
进行预测并保存预测信号
由预测信号和原始信号获得残差信号
进行变换和量化
进行反变换
重建
更新失真distortion
*/
void IntraSearch::xIntraCodingTUBlock(TransformUnit &tu, const ComponentID &compID, Distortion& ruiDist, const int &default0Save1Load2, uint32_t* numSig, std::vector<TrMode>* trModes, const bool loadTr)
{
  if (!tu.blocks[compID].valid())
  {
    return;
  }

  CodingStructure &cs                       = *tu.cs;
  m_pcRdCost->setChromaFormat(cs.sps->getChromaFormatIdc());

  const CompArea      &area                 = tu.blocks[compID];
  const SPS           &sps                  = *cs.sps;

  const ChannelType    chType               = toChannelType(compID);
  const int            bitDepth             = sps.getBitDepth(chType);

  PelBuf         piOrg                      = cs.getOrgBuf    (area);
  PelBuf         piPred                     = cs.getPredBuf   (area);
  PelBuf         piResi                     = cs.getResiBuf   (area);
  PelBuf         piReco                     = cs.getRecoBuf   (area);

  const PredictionUnit &pu                  = *cs.getPU(area.pos(), chType);
  const uint32_t           uiChFinalMode        = PU::getFinalIntraMode(pu, chType);

  //===== init availability pattern =====
  CHECK( tu.jointCbCr && compID == COMPONENT_Cr, "wrong combination of compID and jointCbCr" );
  bool jointCbCr = tu.jointCbCr && compID == COMPONENT_Cb;

  if (compID == COMPONENT_Y)
  {
    PelBuf sharedPredTS( m_pSharedPredTransformSkip[compID], area );
    if( default0Save1Load2 != 2 )
    {
      bool predRegDiffFromTB = CU::isPredRegDiffFromTB(*tu.cu, compID);
      bool firstTBInPredReg = CU::isFirstTBInPredReg(*tu.cu, compID, area);
      CompArea areaPredReg(COMPONENT_Y, tu.chromaFormat, area);
      if (tu.cu->ispMode && isLuma(compID))
      {
        if (predRegDiffFromTB)
        {
          if (firstTBInPredReg)
          {
            CU::adjustPredArea(areaPredReg);
            initIntraPatternChTypeISP(*tu.cu, areaPredReg, piReco);
          }
        }
        else
        {
          initIntraPatternChTypeISP(*tu.cu, area, piReco);
        }
      }
      else
      {
        initIntraPatternChType(*tu.cu, area);
      }

      //===== get prediction signal =====
      if(compID != COMPONENT_Y && !tu.cu->bdpcmModeChroma && PU::isLMCMode(uiChFinalMode))
      {
        xGetLumaRecPixels( pu, area );
        predIntraChromaLM( compID, piPred, pu, area, uiChFinalMode );
      }
      else
      {
        if( PU::isMIP( pu, chType ) )
        {
          initIntraMip( pu, area );
          predIntraMip( compID, piPred, pu );
        }
        else
        {
          if (predRegDiffFromTB)
          {
            if (firstTBInPredReg)
            {
              PelBuf piPredReg = cs.getPredBuf(areaPredReg);
              predIntraAng(compID, piPredReg, pu);
            }
          }
          else
          {
            predIntraAng(compID, piPred, pu);
          }
        }
      }

      // save prediction
      if( default0Save1Load2 == 1 )
      {
        sharedPredTS.copyFrom( piPred );
      }
    }
    else
    {
      // load prediction
      piPred.copyFrom( sharedPredTS );
    }
  }


  DTRACE( g_trace_ctx, D_PRED, "@(%4d,%4d) [%2dx%2d] IMode=%d\n", tu.lx(), tu.ly(), tu.lwidth(), tu.lheight(), uiChFinalMode );
  //DTRACE_PEL_BUF( D_PRED, piPred, tu, tu.cu->predMode, COMPONENT_Y );

  const Slice           &slice = *cs.slice;
  bool flag = slice.getLmcsEnabledFlag() && (slice.isIntra() || (!slice.isIntra() && m_pcReshape->getCTUFlag()));
  if (isLuma(compID))
  {
    //===== get residual signal =====
    piResi.copyFrom( piOrg  );
    if (slice.getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && compID == COMPONENT_Y)
    {
      CompArea      tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
      PelBuf tmpPred = m_tmpStorageLCU.getBuf(tmpArea);
      tmpPred.copyFrom(piPred);
      piResi.rspSignal(m_pcReshape->getFwdLUT());
      piResi.subtract(tmpPred);
    }
    else
    {
      piResi.subtract( piPred );
    }
  }

  //===== transform and quantization =====
  //--- init rate estimation arrays for RDOQ ---
  //--- transform and quantization           ---
  TCoeff uiAbsSum = 0;

  const QpParam cQP(tu, compID);

#if RDOQ_CHROMA_LAMBDA
  m_pcTrQuant->selectLambda(compID);
#endif

  flag =flag && (tu.blocks[compID].width*tu.blocks[compID].height > 4);
  if (flag && isChroma(compID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() )
  {
    int cResScaleInv = tu.getChromaAdj();
    double cResScale = (double)(1 << CSCALE_FP_PREC) / (double)cResScaleInv;
    m_pcTrQuant->setLambda(m_pcTrQuant->getLambda() / (cResScale*cResScale));
  }

  PelBuf          crOrg;
  PelBuf          crPred;
  PelBuf          crResi;
  PelBuf          crReco;

  if (isChroma(compID))
  {
    const CompArea &crArea = tu.blocks[ COMPONENT_Cr ];
    crOrg  = cs.getOrgBuf  ( crArea );
    crPred = cs.getPredBuf ( crArea );
    crResi = cs.getResiBuf ( crArea );
    crReco = cs.getRecoBuf ( crArea );
  }

  if ( jointCbCr )
  {
    // Lambda is loosened for the joint mode with respect to single modes as the same residual is used for both chroma blocks
    const int    absIct = abs( TU::getICTMode(tu) );
    const double lfact  = ( absIct == 1 || absIct == 3 ? 0.8 : 0.5 );
    m_pcTrQuant->setLambda( lfact * m_pcTrQuant->getLambda() );
  }
  if ( sps.getJointCbCrEnabledFlag() && isChroma(compID) && (tu.cu->cs->slice->getSliceQp() > 18) )
  {
    m_pcTrQuant->setLambda( 1.3 * m_pcTrQuant->getLambda() );
  }

  if( isLuma(compID) )
  {
    if (trModes)
    {
      m_pcTrQuant->transformNxN(tu, compID, cQP, trModes, m_pcEncCfg->getMTSIntraMaxCand());
      tu.mtsIdx[compID] = trModes->at(0).first;
    }
    if (!(m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless() && tu.mtsIdx[compID] == 0) || tu.cu->bdpcmMode != 0)
    {
      m_pcTrQuant->transformNxN(tu, compID, cQP, uiAbsSum, m_CABACEstimator->getCtx(), loadTr);
    }

    DTRACE(g_trace_ctx, D_TU_ABS_SUM, "%d: comp=%d, abssum=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_TU_ABS_SUM), compID,
           uiAbsSum);

    if (tu.cu->ispMode && isLuma(compID) && CU::isISPLast(*tu.cu, area, area.compID) && CU::allLumaCBFsAreZero(*tu.cu))
    {
      // ISP has to have at least one non-zero CBF
      ruiDist = MAX_INT;
      return;
    }
    if ((m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless() && tu.mtsIdx[compID] == 0)
        && 0 == tu.cu->bdpcmMode)
    {
      uiAbsSum = 0;
      tu.getCoeffs(compID).fill(0);
      TU::setCbfAtDepth(tu, compID, tu.depth, 0);
    }

    //--- inverse transform ---
    if (uiAbsSum > 0)
    {
      m_pcTrQuant->invTransformNxN(tu, compID, piResi, cQP);
    }
    else
    {
      piResi.fill(0);
    }
  }
  else // chroma
  {
    int         codedCbfMask  = 0;
    ComponentID codeCompId    = (tu.jointCbCr ? (tu.jointCbCr >> 1 ? COMPONENT_Cb : COMPONENT_Cr) : compID);
    const QpParam qpCbCr(tu, codeCompId);

    if( tu.jointCbCr )
    {
      ComponentID otherCompId = ( codeCompId==COMPONENT_Cr ? COMPONENT_Cb : COMPONENT_Cr );
      tu.getCoeffs( otherCompId ).fill(0); // do we need that?
      TU::setCbfAtDepth (tu, otherCompId, tu.depth, false );
    }
    PelBuf& codeResi = ( codeCompId == COMPONENT_Cr ? crResi : piResi );
    uiAbsSum = 0;

    if (trModes)
    {
      m_pcTrQuant->transformNxN(tu, codeCompId, qpCbCr, trModes, m_pcEncCfg->getMTSIntraMaxCand());
      tu.mtsIdx[codeCompId] = trModes->at(0).first;
      if (tu.jointCbCr)
      {
        tu.mtsIdx[(codeCompId == COMPONENT_Cr) ? COMPONENT_Cb : COMPONENT_Cr] = MTS_DCT2_DCT2;
      }
    }
    // encoder bugfix: Set loadTr to aovid redundant transform process
    if (!(m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless() && tu.mtsIdx[compID] == 0) || tu.cu->bdpcmModeChroma != 0)
    {
        m_pcTrQuant->transformNxN(tu, codeCompId, qpCbCr, uiAbsSum, m_CABACEstimator->getCtx(), loadTr);
    }
    if ((m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless() && tu.mtsIdx[compID] == 0) && 0 == tu.cu->bdpcmModeChroma)
    {
        uiAbsSum = 0;
        tu.getCoeffs(compID).fill(0);
        TU::setCbfAtDepth(tu, compID, tu.depth, 0);
    }

    DTRACE( g_trace_ctx, D_TU_ABS_SUM, "%d: comp=%d, abssum=%d\n", DTRACE_GET_COUNTER( g_trace_ctx, D_TU_ABS_SUM ), codeCompId, uiAbsSum );
    if( uiAbsSum > 0 )
    {
      m_pcTrQuant->invTransformNxN(tu, codeCompId, codeResi, qpCbCr);
      codedCbfMask += ( codeCompId == COMPONENT_Cb ? 2 : 1 );
    }
    else
    {
      codeResi.fill(0);
    }

    if( tu.jointCbCr )
    {
      if( tu.jointCbCr == 3 && codedCbfMask == 2 )
      {
        codedCbfMask = 3;
        TU::setCbfAtDepth (tu, COMPONENT_Cr, tu.depth, true );
      }
      if( tu.jointCbCr != codedCbfMask )
      {
        ruiDist = std::numeric_limits<Distortion>::max();
        return;
      }
      m_pcTrQuant->invTransformICT( tu, piResi, crResi );
      uiAbsSum = codedCbfMask;
    }
  }

  //===== reconstruction =====
  if ( flag && uiAbsSum > 0 && isChroma(compID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() )
  {
    piResi.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(compID));
    if( jointCbCr )
    {
      crResi.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(COMPONENT_Cr));
    }
  }

  if (slice.getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && compID == COMPONENT_Y)
  {
    CompArea      tmpArea(COMPONENT_Y, area.chromaFormat, Position(0,0), area.size());
    PelBuf tmpPred = m_tmpStorageLCU.getBuf(tmpArea);
    tmpPred.copyFrom(piPred);
    piReco.reconstruct(tmpPred, piResi, cs.slice->clpRng(compID));
  }
  else
  {
    piReco.reconstruct(piPred, piResi, cs.slice->clpRng( compID ));
    if( jointCbCr )
    {
      crReco.reconstruct(crPred, crResi, cs.slice->clpRng( COMPONENT_Cr ));
    }
  }


  //===== update distortion =====
#if WCG_EXT
  if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (m_pcEncCfg->getLmcs()
    && slice.getLmcsEnabledFlag() && (m_pcReshape->getCTUFlag() || (isChroma(compID) && m_pcEncCfg->getReshapeIntraCMD()))))
  {
    const CPelBuf orgLuma = cs.getOrgBuf( cs.area.blocks[COMPONENT_Y] );
    if (compID == COMPONENT_Y  && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
    {
      CompArea      tmpArea1(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
      PelBuf tmpRecLuma = m_tmpStorageLCU.getBuf(tmpArea1);
      tmpRecLuma.copyFrom(piReco);
      tmpRecLuma.rspSignal(m_pcReshape->getInvLUT());
      ruiDist += m_pcRdCost->getDistPart(piOrg, tmpRecLuma, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
    }
    else
    {
      ruiDist += m_pcRdCost->getDistPart(piOrg, piReco, bitDepth, compID, DF_SSE_WTD, &orgLuma);
      if( jointCbCr )
      {
        ruiDist += m_pcRdCost->getDistPart(crOrg, crReco, bitDepth, COMPONENT_Cr, DF_SSE_WTD, &orgLuma);
      }
    }
  }
  else
#endif
  {
    ruiDist += m_pcRdCost->getDistPart( piOrg, piReco, bitDepth, compID, DF_SSE );
    if( jointCbCr )
    {
      ruiDist += m_pcRdCost->getDistPart( crOrg, crReco, bitDepth, COMPONENT_Cr, DF_SSE );
    }
  }
}

void IntraSearch::xIntraCodingACTTUBlock(TransformUnit &tu, const ComponentID &compID, Distortion& ruiDist, std::vector<TrMode>* trModes, const bool loadTr)
{
  if (!tu.blocks[compID].valid())
  {
    THROW("tu does not exist");
  }

  CodingStructure     &cs = *tu.cs;
  const SPS           &sps = *cs.sps;
  const Slice         &slice = *cs.slice;
  const CompArea      &area = tu.blocks[compID];
  const CompArea &crArea = tu.blocks[COMPONENT_Cr];

  PelBuf              piOrgResi = cs.getOrgResiBuf(area);
  PelBuf              piResi = cs.getResiBuf(area);
  PelBuf              crOrgResi = cs.getOrgResiBuf(crArea);
  PelBuf              crResi = cs.getResiBuf(crArea);
  TCoeff              uiAbsSum = 0;

  CHECK(tu.jointCbCr && compID == COMPONENT_Cr, "wrong combination of compID and jointCbCr");
  bool jointCbCr = tu.jointCbCr && compID == COMPONENT_Cb;

  m_pcRdCost->setChromaFormat(cs.sps->getChromaFormatIdc());
  if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
  m_pcTrQuant->lambdaAdjustColorTrans(true);

  if (jointCbCr)
  {
    ComponentID compIdCode = (tu.jointCbCr >> 1 ? COMPONENT_Cb : COMPONENT_Cr);
    m_pcTrQuant->selectLambda(compIdCode);
  }
  else
  {
    m_pcTrQuant->selectLambda(compID);
  }

  bool flag = slice.getLmcsEnabledFlag() && (slice.isIntra() || (!slice.isIntra() && m_pcReshape->getCTUFlag())) && (tu.blocks[compID].width*tu.blocks[compID].height > 4);
  if (flag && isChroma(compID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag())
  {
    int    cResScaleInv = tu.getChromaAdj();
    double cResScale = (double)(1 << CSCALE_FP_PREC) / (double)cResScaleInv;
    m_pcTrQuant->setLambda(m_pcTrQuant->getLambda() / (cResScale*cResScale));
  }

  if (jointCbCr)
  {
    // Lambda is loosened for the joint mode with respect to single modes as the same residual is used for both chroma blocks
    const int    absIct = abs(TU::getICTMode(tu));
    const double lfact = (absIct == 1 || absIct == 3 ? 0.8 : 0.5);
    m_pcTrQuant->setLambda(lfact * m_pcTrQuant->getLambda());
  }
  if (sps.getJointCbCrEnabledFlag() && isChroma(compID) && (slice.getSliceQp() > 18))
  {
    m_pcTrQuant->setLambda(1.3 * m_pcTrQuant->getLambda());
  }

  if (isLuma(compID))
  {
    QpParam cQP(tu, compID);

    if (trModes)
    {
      m_pcTrQuant->transformNxN(tu, compID, cQP, trModes, m_pcEncCfg->getMTSIntraMaxCand());
      tu.mtsIdx[compID] = trModes->at(0).first;
    }
    if (!(m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless() && tu.mtsIdx[compID] == 0) || tu.cu->bdpcmMode != 0)
    {
      m_pcTrQuant->transformNxN(tu, compID, cQP, uiAbsSum, m_CABACEstimator->getCtx(), loadTr);
    }
    if ((m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless() && tu.mtsIdx[compID] == 0) && tu.cu->bdpcmMode == 0)
    {
      uiAbsSum = 0;
      tu.getCoeffs(compID).fill(0);
      TU::setCbfAtDepth(tu, compID, tu.depth, 0);
    }

    if (uiAbsSum > 0)
    {
      m_pcTrQuant->invTransformNxN(tu, compID, piResi, cQP);
    }
    else
    {
      piResi.fill(0);
    }
  }
  else
  {
    int         codedCbfMask = 0;
    ComponentID codeCompId = (tu.jointCbCr ? (tu.jointCbCr >> 1 ? COMPONENT_Cb : COMPONENT_Cr) : compID);
    QpParam qpCbCr(tu, codeCompId);

    if (tu.jointCbCr)
    {
      ComponentID otherCompId = (codeCompId == COMPONENT_Cr ? COMPONENT_Cb : COMPONENT_Cr);
      tu.getCoeffs(otherCompId).fill(0);
      TU::setCbfAtDepth(tu, otherCompId, tu.depth, false);
    }

    PelBuf& codeResi = (codeCompId == COMPONENT_Cr ? crResi : piResi);
    uiAbsSum = 0;
    if (trModes)
    {
      m_pcTrQuant->transformNxN(tu, codeCompId, qpCbCr, trModes, m_pcEncCfg->getMTSIntraMaxCand());
      tu.mtsIdx[codeCompId] = trModes->at(0).first;
      if (tu.jointCbCr)
      {
        tu.mtsIdx[(codeCompId == COMPONENT_Cr) ? COMPONENT_Cb : COMPONENT_Cr] = MTS_DCT2_DCT2;
      }
    }
    if (!(m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless() && tu.mtsIdx[codeCompId] == 0) || tu.cu->bdpcmModeChroma != 0)
    {
      m_pcTrQuant->transformNxN(tu, codeCompId, qpCbCr, uiAbsSum, m_CABACEstimator->getCtx(), loadTr);
    }
    if (uiAbsSum > 0)
    {
      m_pcTrQuant->invTransformNxN(tu, codeCompId, codeResi, qpCbCr);
      codedCbfMask += (codeCompId == COMPONENT_Cb ? 2 : 1);
    }
    else
    {
      codeResi.fill(0);
    }

    if (tu.jointCbCr)
    {
      if (tu.jointCbCr == 3 && codedCbfMask == 2)
      {
        codedCbfMask = 3;
        TU::setCbfAtDepth(tu, COMPONENT_Cr, tu.depth, true);
      }
      if (tu.jointCbCr != codedCbfMask)
      {
        ruiDist = std::numeric_limits<Distortion>::max();
        if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
        m_pcTrQuant->lambdaAdjustColorTrans(false);
        return;
      }
      m_pcTrQuant->invTransformICT(tu, piResi, crResi);
      uiAbsSum = codedCbfMask;
    }
  }

  if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
  m_pcTrQuant->lambdaAdjustColorTrans(false);

  ruiDist += m_pcRdCost->getDistPart(piOrgResi, piResi, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE);
  if (jointCbCr)
  {
    ruiDist += m_pcRdCost->getDistPart(crOrgResi, crResi, sps.getBitDepth(toChannelType(COMPONENT_Cr)), COMPONENT_Cr, DF_SSE);
  }
}

bool IntraSearch::xIntraCodingLumaISP(CodingStructure& cs, Partitioner& partitioner, const double bestCostSoFar)
{
  int               subTuCounter = 0; // TU计数器
  const CodingUnit& cu = *cs.getCU(partitioner.currArea().lumaPos(), partitioner.chType);
  bool              earlySkipISP = false;//提前跳过ISP
  bool              splitCbfLuma = false;
  const PartSplit   ispType = CU::getISPType(cu, COMPONENT_Y);// ISP划分类型

  cs.cost = 0;

  partitioner.splitCurrArea(ispType, cs);

  CUCtx cuCtx;
  cuCtx.isDQPCoded = true;
  cuCtx.isChromaQpAdjCoded = true;

  do   // subpartitions loop
  {
    uint32_t   numSig = 0;
    Distortion singleDistTmpLuma = 0; // 失真
    uint64_t   singleTmpFracBits = 0; // 码率
    double     singleCostTmp     = 0; // RD COST
    // 获得划分后的TU
    TransformUnit& tu = cs.addTU(CS::getArea(cs, partitioner.currArea(), partitioner.chType), partitioner.chType);
    tu.depth = partitioner.currTrDepth;

    // Encode TU
    xIntraCodingTUBlock(tu, COMPONENT_Y, singleDistTmpLuma, 0, &numSig);
    if (singleDistTmpLuma == MAX_INT)   // all zero CBF skip
    {
      earlySkipISP = true;
      partitioner.exitCurrSplit();
      cs.cost = MAX_DOUBLE;
      return false;
    }

    if (m_pcRdCost->calcRdCost(cs.fracBits, cs.dist + singleDistTmpLuma) > bestCostSoFar)
    {
      // The accumulated cost + distortion is already larger than the best cost so far, so it is not necessary to
      // calculate the rate
      // 累计成本 + 失真已经大于目前为止的最佳成本，因此不必计算码率
      earlySkipISP = true;
    }
    else
    {
      // 计算码率
      singleTmpFracBits = xGetIntraFracBitsQT(cs, partitioner, true, false, subTuCounter, ispType, &cuCtx);
    }
    // 计算失真
    singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma);

    cs.cost += singleCostTmp; // 总代价
    cs.dist += singleDistTmpLuma; // 总失真
    cs.fracBits += singleTmpFracBits; //总码率

    subTuCounter++;

    splitCbfLuma |= TU::getCbfAtDepth(*cs.getTU(partitioner.currArea().lumaPos(), partitioner.chType, subTuCounter - 1), COMPONENT_Y, partitioner.currTrDepth);//当前的TU数目小于分区总数目
    int nSubPartitions = m_ispTestedModes[cu.lfnstIdx].numTotalParts[cu.ispMode - 1];
    if (subTuCounter < nSubPartitions)
    {
      // exit condition if the accumulated cost is already larger than the best cost so far (no impact in RD performance) 
      // 如果累计成本已经大于目前为止的最佳成本（对研发绩效没有影响），则退出条件
      if (cs.cost > bestCostSoFar)
      {
        earlySkipISP = true;
        break;
      }
      else if (subTuCounter < nSubPartitions)
      {
        // more restrictive exit condition
        double threshold = nSubPartitions == 2 ? 0.95 : subTuCounter == 1 ? 0.83 : 0.91;
        if (subTuCounter < nSubPartitions && cs.cost > bestCostSoFar * threshold)
        {
          earlySkipISP = true;
          break;
        }
      }
    }
  } while (partitioner.nextPart(cs));   // subpartitions loop

  partitioner.exitCurrSplit();
  const UnitArea& currArea = partitioner.currArea();
  const uint32_t  currDepth = partitioner.currTrDepth;
  // 如果提前退出ISP，则cost设置为MAXDOUBLE
  if (earlySkipISP)
  {
    cs.cost = MAX_DOUBLE;
  }
  else
  {
    cs.cost = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);
    // The cost check is necessary here again to avoid superfluous operations if the maximum number of coded subpartitions was reached and yet ISP did not win
    // 如果达到了最大编码子分区数，但ISP没有获胜，则必须再次进行成本检查，以避免多余的操作
    if (cs.cost < bestCostSoFar)
    {
      cs.setDecomp(cu.Y());
      cs.picture->getRecoBuf(currArea.Y()).copyFrom(cs.getRecoBuf(currArea.Y()));

      for (auto& ptu : cs.tus)
      {
        if (currArea.Y().contains(ptu->Y()))
        {
          TU::setCbfAtDepth(*ptu, COMPONENT_Y, currDepth, splitCbfLuma ? 1 : 0);
        }
      }
    }
    else
    {
      earlySkipISP = true;
    }
  }
  return !earlySkipISP;
}

/*
* 
xRecurIntraCodingLumaQT函数主要是使用上层函数选中的预测模式，遍历上层函数传来的变换模式，通过调用xIntraCodingTUBlock计算相应的失真，从而计算RD
Cost，从而选出最佳变换。主要流程如下：

确定当前分区是否可以继续划分，如果需要划分则递归调用自身。
确定候选模式变换集
遍历候选模式遍历集，调用xIntraCodingTUBlock函数进行预测计算残差、进行变换、量化、重建，并计算失真。
调用xGetIntraFracBitsQT函数进行编码计算比特数和调用calcRdCost函数计算RD Cost
确定候选模式变换集分为两种情况：

（1）SPS层开启LFNST、当前块允许TransformSkip、使用DCT-2变换且lfnstIdx==0
         此时，需要检查DCT-2变换和TransformSkip，从二者之间选出最佳值

（2）SPS层关闭LFNST
        如果此时允许TransformSkip和MTS，则待测候选变换模式为DCT-2、TS以及MTS的四种组合

TrModes变量包含两个值，第一个是int值，表示变换模式，第二个值是bool值，表示该模式是否可用

如果TrModes变量的size大于1时，则会通过xIntraCodingTUBlock函数调用第一种TransformNxN函数，该函数比较TrModes中的几种变换后的绝对残差系数和，来修改TrModes中的bool值，然后在xRecurIntraCodingLumaQT函数中决定是否需要跳过该模式。

具体代码注释如下：
*/
bool IntraSearch::xRecurIntraCodingLumaQT( CodingStructure &cs, Partitioner &partitioner, bool mtsCheckRangeFlag, int mtsFirstCheckId, int mtsLastCheckId, bool moreProbMTSIdxFirst )
{
  // mtsCheckRangeFlag是MTS使能标志0/1
  // mtsFirstCheckId和mtsLastCheckId是MTS变换核的索引
  // moreProbMTSIdxFirst是MTS变换核的索引是否>0
  const UnitArea &currArea = partitioner.currArea();
  const CodingUnit     &cu = *cs.getCU( currArea.lumaPos(), partitioner.chType );
  uint32_t currDepth       = partitioner.currTrDepth;
  const SPS &sps           = *cs.sps;
  
 // 检查是否需要继续划分，即判断当前TU宽度和高度是否大于最大支持变换尺寸
  bool bCheckFull          = true;
  bool bCheckSplit         = false;
  bCheckFull               = !partitioner.canSplit( TU_MAX_TR_SPLIT, cs );
  bCheckSplit              = partitioner.canSplit( TU_MAX_TR_SPLIT, cs );
  const Slice           &slice = *cs.slice;

  CHECK(cu.ispMode != NOT_INTRA_SUBPARTITIONS, "Use the function xIntraCodingLumaISP for ISP cases.");

  uint32_t    numSig           = 0;

  double     dSingleCost                        = MAX_DOUBLE; 
  Distortion uiSingleDistLuma                   = 0;
  uint64_t   singleFracBits                     = 0;
  // 检查变换跳过标志
  bool       checkTransformSkip                 = sps.getTransformSkipEnabledFlag();
  int        bestModeId[ MAX_NUM_COMPONENT ]    = { 0, 0, 0 };
  // cu.mtsflag在xCheckRDCostIntra函数里控制，如果为0表示一次变换为DCT-2，否则为DST7与DCT8的四种组合
  uint8_t    nNumTransformCands                 = cu.mtsFlag ? 4 : 1;
  uint8_t    numTransformIndexCands             = nNumTransformCands;

  const TempCtx ctxStart  ( m_CtxCache, m_CABACEstimator->getCtx() );
  TempCtx       ctxBest   ( m_CtxCache );

  CodingStructure *csSplit = nullptr;
  CodingStructure *csFull  = nullptr;

  CUCtx cuCtx;
  cuCtx.isDQPCoded = true;
  cuCtx.isChromaQpAdjCoded = true;

  if( bCheckSplit )
  {
    csSplit = &cs;
  }
  else if( bCheckFull )
  {
    csFull = &cs;
  }

  bool validReturnFull = false;

  if( bCheckFull )//不需要进一步划分
  {
    csFull->cost = 0.0;

    TransformUnit &tu = csFull->addTU( CS::getArea( *csFull, currArea, partitioner.chType ), partitioner.chType );
    tu.depth = currDepth;

    const bool tsAllowed  = TU::isTSAllowed( tu, COMPONENT_Y );//当前TU是否允许TransformSkip
    const bool mtsAllowed = CU::isMTSAllowed( cu, COMPONENT_Y );//当前CU是否允许MTS

    std::vector<TrMode> trModes;//trModes的第一个值是变换模式，第二个值表示是否使用该模式

    // 使用LFNST
    if( sps.getUseLFNST() )
    {
      // 判断是否需要检查TransformSkip
      checkTransformSkip &= tsAllowed;
      checkTransformSkip &= !cu.mtsFlag;
      checkTransformSkip &= !cu.lfnstIdx;

      // 不使用MTS且检查变换跳过
      if( !cu.mtsFlag && checkTransformSkip )
      {
        trModes.push_back( TrMode( 0, true ) ); //DCT2
        trModes.push_back( TrMode( 1, true ) ); //TS
      }
    }
    else // SPS关闭LFNST
    {
      // 此时如果需要检查变换跳过和MTS，则总共需要检查6种变换模式
      // DCT-2+transform skip + 4 MTS
      nNumTransformCands = 1 + ( tsAllowed ? 1 : 0 ) + ( mtsAllowed ? 4 : 0 ); // DCT + TS + 4 MTS = 6 tests
      if (m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless())
      {
        nNumTransformCands = 1;
        CHECK(!tsAllowed && !cu.bdpcmMode, "transform skip should be enabled for LS");
        if (cu.bdpcmMode)
        {
          trModes.push_back(TrMode(0, true));
        }
        else
        {
          trModes.push_back(TrMode(1, true));
        }
      }
      else//将6种变换模式保存在trModes中
      {
        trModes.push_back(TrMode(0, true));   // DCT2
        if (tsAllowed)
        {
          trModes.push_back(TrMode(1, true)); // 变换跳过
        }
        if (mtsAllowed) //MTS模式
        {
          for (int i = 2; i < 6; i++)
          {
            trModes.push_back(TrMode(i, true));
          }
        }
      }
    }

    CHECK( !tu.Y().valid(), "Invalid TU" );

    CodingStructure &saveCS = *m_pSaveCS[0];

    TransformUnit *tmpTU = nullptr;

    Distortion singleDistTmpLuma = 0;
    uint64_t     singleTmpFracBits = 0;
    double     singleCostTmp     = 0;

    // mtsFlag=1和mtsCheckRangeFlag=1说明正在检查的是MTS二次变换，这时候选择MTS变换核的索引
    int        firstCheckId      = ( sps.getUseLFNST() && mtsCheckRangeFlag && cu.mtsFlag ) ? mtsFirstCheckId : 0;

    //we add the MTS candidates to the loop. TransformSkip will still be the last one to be checked (when modeId == lastCheckId) as long as checkTransformSkip is true

    // 我们将MTS候选添加到循环中。只要checkTransformSkip为true，
    // TransformSkip仍然是最后一个要检查的（当modeId ==lastCheckId时）
    int        lastCheckId       = sps.getUseLFNST() ? ( ( mtsCheckRangeFlag && cu.mtsFlag ) ? ( mtsLastCheckId + ( int ) checkTransformSkip ) : ( numTransformIndexCands - ( firstCheckId + 1 ) + ( int ) checkTransformSkip ) ) :
                                   trModes[ nNumTransformCands - 1 ].first;
    // 不只检查一种模式
    bool isNotOnlyOneMode        = sps.getUseLFNST() ? lastCheckId != firstCheckId : nNumTransformCands != 1;

    if( isNotOnlyOneMode )
    {
      saveCS.pcv     = cs.pcv;
      saveCS.picture = cs.picture;
      saveCS.area.repositionTo(cs.area);
      saveCS.clearTUs();
      tmpTU = &saveCS.addTU(currArea, partitioner.chType);
    }

    bool    cbfBestMode      = false;//最佳的CBF值
    bool    cbfBestModeValid = false;
    bool    cbfDCT2  = true;

    // 遍历所有可能的变换模式
    for( int modeId = firstCheckId; modeId <= ( sps.getUseLFNST() ? lastCheckId : ( nNumTransformCands - 1 ) ); modeId++ )
    {
      uint8_t transformIndex = modeId;

      if( sps.getUseLFNST() )//SPS层开启LFNST
      {
        // 如果当前检查的模式不是最后一种模式或者当前检查的是最后一种模式且该模式不是变换跳过模式
        if( ( transformIndex < lastCheckId ) || ( ( transformIndex == lastCheckId ) && !checkTransformSkip ) ) //we avoid this if the mode is transformSkip
        {
          // Skip checking other transform candidates if zero CBF is encountered and it is the best transform so far
          // 如果遇到零CBF，则跳过检查其他候选变换，这是迄今为止最好的转换
          if( m_pcEncCfg->getUseFastLFNST() && transformIndex && !cbfBestMode && cbfBestModeValid )
          {
            continue;
          }
        }
      }
      else//SPS层关闭LFNST
      {
        if (!(m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless()))
        {
          if (!cbfDCT2 || (m_pcEncCfg->getUseTransformSkipFast() && bestModeId[COMPONENT_Y] == MTS_SKIP))
          {
            break;
          }
          if (!trModes[modeId].second)
          {
            continue;
          }
        }
        tu.mtsIdx[COMPONENT_Y] = trModes[modeId].first;
      }


      if ((modeId != firstCheckId) && isNotOnlyOneMode)
      {
        m_CABACEstimator->getCtx() = ctxStart;
      }

      int default0Save1Load2 = 0;
      singleDistTmpLuma = 0;

      // 如果当前检查的是第一种变换模式
      // 且当SPS层开启LFNST时，检查的不是最后一种模式
      // 且当SPS层关闭LFNST时，待检测模式大于1
      // default0Save1Load2该参数为1时在xIntraCodingTUBlock函数中进行预测时将预测信号保存
      // default0Save1Load2该参数为2时在xIntraCodingTUBlock函数中之间加载预测信号
      if( modeId == firstCheckId && ( sps.getUseLFNST() ? ( modeId != lastCheckId ) : ( nNumTransformCands > 1 ) ) )
      {
        default0Save1Load2 = 1;
      }
      else if (modeId != firstCheckId)//当前检查的不是第一个模式
      {
        if( sps.getUseLFNST() && !cbfBestModeValid )
        {
          default0Save1Load2 = 1;
        }
        else
        {
          default0Save1Load2 = 2;
        }
      }
      //LFNST模式
      if( sps.getUseLFNST() )
      {
        // 使用MTS
        if( cu.mtsFlag )
        {
          // 根据模式选择不同的MTS变换核，由trGrpIdx>0控制
          if( moreProbMTSIdxFirst )
          {
            const ChannelType     chType      = toChannelType( COMPONENT_Y );
            const CompArea&       area        = tu.blocks[ COMPONENT_Y ];
            const PredictionUnit& pu          = *cs.getPU( area.pos(), chType );
            uint32_t              uiIntraMode = pu.intraDir[ chType ];

            if( transformIndex == 1 )
            {
              tu.mtsIdx[COMPONENT_Y] = (uiIntraMode < 34) ? MTS_DST7_DCT8 : MTS_DCT8_DST7;
            }
            else if( transformIndex == 2 )
            {
              tu.mtsIdx[COMPONENT_Y] = (uiIntraMode < 34) ? MTS_DCT8_DST7 : MTS_DST7_DCT8;
            }
            else
            {
              tu.mtsIdx[COMPONENT_Y] = MTS_DST7_DST7 + transformIndex;
            }
          }
          else
          {
            tu.mtsIdx[COMPONENT_Y] = MTS_DST7_DST7 + transformIndex;
          }
        }
        else
        {
          tu.mtsIdx[COMPONENT_Y] = transformIndex;
        }

        // 如果当前检查的不是MTS（即DCT-2）且需要检查TransformSkip
        // xIntraCodingTUBlock进行预测、变换之后再进行反变换后获得重建值，然后计算相应的失真
        if( !cu.mtsFlag && checkTransformSkip )
        {
          xIntraCodingTUBlock( tu, COMPONENT_Y, singleDistTmpLuma, default0Save1Load2, &numSig, modeId == 0 ? &trModes : nullptr, true );
          if( modeId == 0 )
          {
            // 根据DCT-2变换和TS变换后的绝对残差系数和决定是否需要跳过TS变换
            for( int i = 0; i < 2; i++ )
            {
              if( trModes[ i ].second )
              {
                lastCheckId = trModes[ i ].first;
              }
            }
          }
        }
        else
        {
          xIntraCodingTUBlock( tu, COMPONENT_Y, singleDistTmpLuma, default0Save1Load2, &numSig );
        }
      }
      else
      {
        if( nNumTransformCands > 1 )
        {
          xIntraCodingTUBlock( tu, COMPONENT_Y, singleDistTmpLuma, default0Save1Load2, &numSig, modeId == 0 ? &trModes : nullptr, true );
          if( modeId == 0 )
          {
            for( int i = 0; i < nNumTransformCands; i++ )
            {
              if( trModes[ i ].second )
              {
                lastCheckId = trModes[ i ].first;
              }
            }
          }
        }
        else
        {
          xIntraCodingTUBlock( tu, COMPONENT_Y, singleDistTmpLuma, default0Save1Load2, &numSig );
        }
      }

      cuCtx.mtsLastScanPos = false;
      cuCtx.violatesMtsCoeffConstraint = false;
      //----- determine rate and r-d cost -----
      // SPS层开启LFNST时判断当前模式是否是最后一个测试模式且模式号不等于0且需要检查TransformSkip
      // SPS层关闭LFNST时，判断当前模式不是DCT-2变换
      if( ( sps.getUseLFNST() ? ( modeId == lastCheckId && modeId != 0 && checkTransformSkip ) : ( trModes[ modeId ].first != 0 ) ) && !TU::getCbfAtDepth( tu, COMPONENT_Y, currDepth ) )
      {
        //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
        // 为了在cbf为零时不编码TS标志，禁止cbf为零的TS情况。
        if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
        {
          singleCostTmp = MAX_DOUBLE;
        }
        else
        {
          singleTmpFracBits = xGetIntraFracBitsQT(*csFull, partitioner, true, false, -1, TU_NO_ISP, &cuCtx);
          singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma);
        }
      }
      else
      {
        // xGetIntraFracBitsQT函数对当前变换模式及残差进行编码，获取所需编码比特数
        singleTmpFracBits = xGetIntraFracBitsQT(*csFull, partitioner, true, false, -1, TU_NO_ISP, &cuCtx);
        if (tu.mtsIdx[COMPONENT_Y] > MTS_SKIP)//如果当前变换模式是MTS的四种组合
        {
          if (!cuCtx.mtsLastScanPos)//如果没有编码MTS的最后非零系数位置
          {
            singleCostTmp = MAX_DOUBLE;
          }
          else
          {
            singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma);
          }
        }
        else
        {
          singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma);
        }
      }


      if (singleCostTmp < dSingleCost)
      {
        dSingleCost       = singleCostTmp;
        uiSingleDistLuma  = singleDistTmpLuma;
        singleFracBits    = singleTmpFracBits;

        if( sps.getUseLFNST() )
        {
          // 设置最佳模式和最佳CBF
          bestModeId[ COMPONENT_Y ] = modeId;
          cbfBestMode = TU::getCbfAtDepth( tu, COMPONENT_Y, currDepth );
          cbfBestModeValid = true;
          validReturnFull = true;
        }
        else
        {
          bestModeId[ COMPONENT_Y ] = trModes[ modeId ].first;
          if( trModes[ modeId ].first == 0 )
          {
            cbfDCT2 = TU::getCbfAtDepth( tu, COMPONENT_Y, currDepth );
          }
        }

        if( bestModeId[COMPONENT_Y] != lastCheckId )
        {
          // 将当前的预测和重建信号保存下来
          saveCS.getPredBuf( tu.Y() ).copyFrom( csFull->getPredBuf( tu.Y() ) );
          saveCS.getRecoBuf( tu.Y() ).copyFrom( csFull->getRecoBuf( tu.Y() ) );

          if( KEEP_PRED_AND_RESI_SIGNALS )
          {
            saveCS.getResiBuf   ( tu.Y() ).copyFrom( csFull->getResiBuf   ( tu.Y() ) );
            saveCS.getOrgResiBuf( tu.Y() ).copyFrom( csFull->getOrgResiBuf( tu.Y() ) );
          }

          tmpTU->copyComponentFrom( tu, COMPONENT_Y );

          ctxBest = m_CABACEstimator->getCtx();
        }
      }
    }

    if( sps.getUseLFNST() && !validReturnFull )
    {
      csFull->cost = MAX_DOUBLE;

      if( bCheckSplit )
      {
        ctxBest = m_CABACEstimator->getCtx();
      }
    }
    else
    {
      //最佳的模式不是最后一个检查的模式（说明提前截止）
      if( bestModeId[COMPONENT_Y] != lastCheckId )
      {
        csFull->getPredBuf( tu.Y() ).copyFrom( saveCS.getPredBuf( tu.Y() ) );
        csFull->getRecoBuf( tu.Y() ).copyFrom( saveCS.getRecoBuf( tu.Y() ) );

        if( KEEP_PRED_AND_RESI_SIGNALS )
        {
          csFull->getResiBuf   ( tu.Y() ).copyFrom( saveCS.getResiBuf   ( tu.Y() ) );
          csFull->getOrgResiBuf( tu.Y() ).copyFrom( saveCS.getOrgResiBuf( tu.Y() ) );
        }

        tu.copyComponentFrom( *tmpTU, COMPONENT_Y );

        if( !bCheckSplit )
        {
          m_CABACEstimator->getCtx() = ctxBest;
        }
      }
      else if( bCheckSplit )
      {
        ctxBest = m_CABACEstimator->getCtx();
      }

      csFull->cost     += dSingleCost;
      csFull->dist     += uiSingleDistLuma;
      csFull->fracBits += singleFracBits;
    }
  }

  bool validReturnSplit = false;
  if( bCheckSplit )
  {
    //----- store full entropy coding status, load original entropy coding status -----
    if( bCheckFull )
    {
      m_CABACEstimator->getCtx() = ctxStart;
    }
    //----- code splitted block -----
    csSplit->cost = 0;

    bool uiSplitCbfLuma  = false;
    bool splitIsSelected = true;
    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }

    do
    {
      bool tmpValidReturnSplit = xRecurIntraCodingLumaQT( *csSplit, partitioner, false, mtsCheckRangeFlag, mtsFirstCheckId, mtsLastCheckId );
      if( sps.getUseLFNST() && !tmpValidReturnSplit )
      {
        splitIsSelected = false;
        break;
      }

      csSplit->setDecomp(partitioner.currArea().Y());

      uiSplitCbfLuma |= TU::getCbfAtDepth(*csSplit->getTU(partitioner.currArea().lumaPos(), partitioner.chType, -1), COMPONENT_Y, partitioner.currTrDepth);

    } while( partitioner.nextPart( *csSplit ) );

    partitioner.exitCurrSplit();

    if( splitIsSelected )
    {
      for( auto &ptu : csSplit->tus )
      {
        if( currArea.Y().contains( ptu->Y() ) )
        {
          TU::setCbfAtDepth( *ptu, COMPONENT_Y, currDepth, uiSplitCbfLuma ? 1 : 0 );
        }
      }

      //----- restore context states -----
      m_CABACEstimator->getCtx() = ctxStart;

      cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_LUMA] = false;
      cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] = false;
      cuCtx.lfnstLastScanPos = false;
      cuCtx.violatesMtsCoeffConstraint = false;
      cuCtx.mtsLastScanPos = false;

      //----- determine rate and r-d cost -----
      csSplit->fracBits = xGetIntraFracBitsQT( *csSplit, partitioner, true, false, -1, TU_NO_ISP, &cuCtx );

      //--- update cost ---
      csSplit->cost     = m_pcRdCost->calcRdCost(csSplit->fracBits, csSplit->dist);

      validReturnSplit = true;
    }
  }

  bool retVal = false;
  if( csFull || csSplit )
  {
    if( !sps.getUseLFNST() || validReturnFull || validReturnSplit )//如果存在合理的返回值
    {
      // otherwise this would've happened in useSubStructure
      cs.picture->getRecoBuf(currArea.Y()).copyFrom(cs.getRecoBuf(currArea.Y()));
      cs.picture->getPredBuf(currArea.Y()).copyFrom(cs.getPredBuf(currArea.Y()));
      cs.cost = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);
      retVal = true;
    }
  }
  return retVal;
}

bool IntraSearch::xRecurIntraCodingACTQT(CodingStructure &cs, Partitioner &partitioner, bool mtsCheckRangeFlag, int mtsFirstCheckId, int mtsLastCheckId, bool moreProbMTSIdxFirst)
{
  const UnitArea &currArea = partitioner.currArea();
  uint32_t       currDepth = partitioner.currTrDepth;
  const Slice    &slice = *cs.slice;
  const SPS      &sps = *cs.sps;

  bool bCheckFull = !partitioner.canSplit(TU_MAX_TR_SPLIT, cs);
  bool bCheckSplit = !bCheckFull;

  TempCtx ctxStart(m_CtxCache, m_CABACEstimator->getCtx());
  TempCtx ctxBest(m_CtxCache);

  CodingStructure *csSplit = nullptr;
  CodingStructure *csFull = nullptr;
  if (bCheckSplit)
  {
    csSplit = &cs;
  }
  else if (bCheckFull)
  {
    csFull = &cs;
  }

  bool validReturnFull = false;

  if (bCheckFull)
  {
    TransformUnit        &tu = csFull->addTU(CS::getArea(*csFull, currArea, partitioner.chType), partitioner.chType);
    tu.depth = currDepth;
    const CodingUnit     &cu = *csFull->getCU(tu.Y().pos(), CHANNEL_TYPE_LUMA);
    const PredictionUnit &pu = *csFull->getPU(tu.Y().pos(), CHANNEL_TYPE_LUMA);
    CHECK(!tu.Y().valid() || !tu.Cb().valid() || !tu.Cr().valid(), "Invalid TU");
    CHECK(tu.cu != &cu, "wrong CU fetch");
    CHECK(cu.ispMode, "adaptive color transform cannot be applied to ISP");
    CHECK(pu.intraDir[CHANNEL_TYPE_CHROMA] != DM_CHROMA_IDX, "chroma should use DM mode for adaptive color transform");

    // 1. intra prediction and forward color transform

    PelUnitBuf orgBuf = csFull->getOrgBuf(tu);
    PelUnitBuf predBuf = csFull->getPredBuf(tu);
    PelUnitBuf resiBuf = csFull->getResiBuf(tu);
    PelUnitBuf orgResiBuf = csFull->getOrgResiBuf(tu);
    bool doReshaping = (slice.getLmcsEnabledFlag() && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && (slice.isIntra() || m_pcReshape->getCTUFlag()) && (tu.blocks[COMPONENT_Cb].width * tu.blocks[COMPONENT_Cb].height > 4));
    if (doReshaping)
    {
      const Area      area = tu.Y().valid() ? tu.Y() : Area(recalcPosition(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.blocks[tu.chType].pos()), recalcSize(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.blocks[tu.chType].size()));
      const CompArea &areaY = CompArea(COMPONENT_Y, tu.chromaFormat, area);
      int             adj = m_pcReshape->calculateChromaAdjVpduNei(tu, areaY);
      tu.setChromaAdj(adj);
    }

    for (int i = 0; i < getNumberValidComponents(tu.chromaFormat); i++)
    {
      ComponentID          compID = (ComponentID)i;
      const CompArea       &area = tu.blocks[compID];
      const ChannelType    chType = toChannelType(compID);

      PelBuf         piOrg = orgBuf.bufs[compID];
      PelBuf         piPred = predBuf.bufs[compID];
      PelBuf         piResi = resiBuf.bufs[compID];

      initIntraPatternChType(*tu.cu, area);
      if (PU::isMIP(pu, chType))
      {
        initIntraMip(pu, area);
        predIntraMip(compID, piPred, pu);
      }
      else
      {
        predIntraAng(compID, piPred, pu);
      }

      piResi.copyFrom(piOrg);
      if (slice.getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && compID == COMPONENT_Y)
      {
        CompArea tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
        PelBuf   tmpPred = m_tmpStorageLCU.getBuf(tmpArea);
        tmpPred.copyFrom(piPred);
        piResi.rspSignal(m_pcReshape->getFwdLUT());
        piResi.subtract(tmpPred);
      }
      else if (doReshaping && (compID != COMPONENT_Y))
      {
        piResi.subtract(piPred);
        int cResScaleInv = tu.getChromaAdj();
        piResi.scaleSignal(cResScaleInv, 1, slice.clpRng(compID));
      }
      else
      {
        piResi.subtract(piPred);
      }
    }

    resiBuf.colorSpaceConvert(orgResiBuf, true, cs.slice->clpRng(COMPONENT_Y));

    // 2. luma residual optimization
    double     dSingleCostLuma = MAX_DOUBLE;
    bool       checkTransformSkip = sps.getTransformSkipEnabledFlag();
    int        bestLumaModeId = 0;
    uint8_t    nNumTransformCands = cu.mtsFlag ? 4 : 1;
    uint8_t    numTransformIndexCands = nNumTransformCands;

    const bool tsAllowed = TU::isTSAllowed(tu, COMPONENT_Y);
    const bool mtsAllowed = CU::isMTSAllowed(cu, COMPONENT_Y);
    std::vector<TrMode> trModes;

    if (sps.getUseLFNST())
    {
      checkTransformSkip &= tsAllowed;
      checkTransformSkip &= !cu.mtsFlag;
      checkTransformSkip &= !cu.lfnstIdx;

      if (!cu.mtsFlag && checkTransformSkip)
      {
        trModes.push_back(TrMode(0, true)); //DCT2
        trModes.push_back(TrMode(1, true)); //TS
      }
    }
    else
    {
      if (m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless())
      {
        nNumTransformCands = 1;
        CHECK(!tsAllowed && !cu.bdpcmMode, "transform skip should be enabled for LS");
        if (cu.bdpcmMode)
        {
          trModes.push_back(TrMode(0, true));
        }
        else
        {
          trModes.push_back(TrMode(1, true));
        }
      }
      else
      {
        nNumTransformCands = 1 + (tsAllowed ? 1 : 0) + (mtsAllowed ? 4 : 0);   // DCT + TS + 4 MTS = 6 tests

        trModes.push_back(TrMode(0, true));   // DCT2
        if (tsAllowed)
        {
          trModes.push_back(TrMode(1, true));
        }
        if (mtsAllowed)
        {
          for (int i = 2; i < 6; i++)
          {
            trModes.push_back(TrMode(i, true));
          }
        }
      }
    }

    CodingStructure &saveLumaCS = *m_pSaveCS[0];
    TransformUnit   *tmpTU = nullptr;
    Distortion      singleDistTmpLuma = 0;
    uint64_t        singleTmpFracBits = 0;
    double          singleCostTmp = 0;
    int             firstCheckId = (sps.getUseLFNST() && mtsCheckRangeFlag && cu.mtsFlag) ? mtsFirstCheckId : 0;
    int             lastCheckId = sps.getUseLFNST() ? ((mtsCheckRangeFlag && cu.mtsFlag) ? (mtsLastCheckId + (int)checkTransformSkip) : (numTransformIndexCands - (firstCheckId + 1) + (int)checkTransformSkip)) : trModes[nNumTransformCands - 1].first;
    bool            isNotOnlyOneMode = sps.getUseLFNST() ? lastCheckId != firstCheckId : nNumTransformCands != 1;

    if (isNotOnlyOneMode)
    {
      saveLumaCS.pcv = csFull->pcv;
      saveLumaCS.picture = csFull->picture;
      saveLumaCS.area.repositionTo(csFull->area);
      saveLumaCS.clearTUs();
      tmpTU = &saveLumaCS.addTU(currArea, partitioner.chType);
    }

    bool    cbfBestMode = false;
    bool    cbfBestModeValid = false;
    bool    cbfDCT2 = true;
    if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
    m_pcRdCost->lambdaAdjustColorTrans(true, COMPONENT_Y);
    for (int modeId = firstCheckId; modeId <= ((m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless()) ? (nNumTransformCands - 1) : lastCheckId); modeId++)
    {
      uint8_t transformIndex = modeId;
      csFull->getResiBuf(tu.Y()).copyFrom(csFull->getOrgResiBuf(tu.Y()));

      m_CABACEstimator->getCtx() = ctxStart;
      m_CABACEstimator->resetBits();

      if (sps.getUseLFNST())
      {
        if ((transformIndex < lastCheckId) || ((transformIndex == lastCheckId) && !checkTransformSkip)) //we avoid this if the mode is transformSkip
        {
          // Skip checking other transform candidates if zero CBF is encountered and it is the best transform so far
          if (m_pcEncCfg->getUseFastLFNST() && transformIndex && !cbfBestMode && cbfBestModeValid)
          {
            continue;
          }
        }
      }
      else
      {
        if (!(m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless()))
        {
          if (!cbfDCT2 || (m_pcEncCfg->getUseTransformSkipFast() && bestLumaModeId == 1))
          {
            break;
          }
          if (!trModes[modeId].second)
          {
            continue;
          }
        }
        tu.mtsIdx[COMPONENT_Y] = trModes[modeId].first;
      }

      singleDistTmpLuma = 0;
      if (sps.getUseLFNST())
      {
        if (cu.mtsFlag)
        {
          if (moreProbMTSIdxFirst)
          {
            uint32_t uiIntraMode = pu.intraDir[CHANNEL_TYPE_LUMA];

            if (transformIndex == 1)
            {
              tu.mtsIdx[COMPONENT_Y] = (uiIntraMode < 34) ? MTS_DST7_DCT8 : MTS_DCT8_DST7;
            }
            else if (transformIndex == 2)
            {
              tu.mtsIdx[COMPONENT_Y] = (uiIntraMode < 34) ? MTS_DCT8_DST7 : MTS_DST7_DCT8;
            }
            else
            {
              tu.mtsIdx[COMPONENT_Y] = MTS_DST7_DST7 + transformIndex;
            }
          }
          else
          {
            tu.mtsIdx[COMPONENT_Y] = MTS_DST7_DST7 + transformIndex;
          }
        }
        else
        {
          tu.mtsIdx[COMPONENT_Y] = transformIndex;
        }

        if (!cu.mtsFlag && checkTransformSkip)
        {
          xIntraCodingACTTUBlock(tu, COMPONENT_Y, singleDistTmpLuma, modeId == 0 ? &trModes : nullptr, true);
          if (modeId == 0)
          {
            for (int i = 0; i < 2; i++)
            {
              if (trModes[i].second)
              {
                lastCheckId = trModes[i].first;
              }
            }
          }
        }
        else
        {
          xIntraCodingACTTUBlock(tu, COMPONENT_Y, singleDistTmpLuma);
        }
      }
      else
      {
        if (nNumTransformCands > 1)
        {
          xIntraCodingACTTUBlock(tu, COMPONENT_Y, singleDistTmpLuma, modeId == 0 ? &trModes : nullptr, true);
          if (modeId == 0)
          {
            for (int i = 0; i < nNumTransformCands; i++)
            {
              if (trModes[i].second)
              {
                lastCheckId = trModes[i].first;
              }
            }
          }
        }
        else
        {
          xIntraCodingACTTUBlock(tu, COMPONENT_Y, singleDistTmpLuma);
        }
      }

      CUCtx cuCtx;
      cuCtx.isDQPCoded = true;
      cuCtx.isChromaQpAdjCoded = true;
      //----- determine rate and r-d cost -----
      if ((sps.getUseLFNST() ? (modeId == lastCheckId && modeId != 0 && checkTransformSkip) : (trModes[modeId].first != 0)) && !TU::getCbfAtDepth(tu, COMPONENT_Y, currDepth))
      {
        //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
        if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
        singleCostTmp = MAX_DOUBLE;
        else
        {
          singleTmpFracBits = xGetIntraFracBitsQT(*csFull, partitioner, true, false, -1, TU_NO_ISP);
          singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma, false);
        }
      }
      else
      {
        singleTmpFracBits = xGetIntraFracBitsQT(*csFull, partitioner, true, false, -1, TU_NO_ISP, &cuCtx);

        if (tu.mtsIdx[COMPONENT_Y] > MTS_SKIP)
        {
          if (!cuCtx.mtsLastScanPos)
          {
            singleCostTmp = MAX_DOUBLE;
          }
          else
          {
            singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma, false);
          }
        }
        else
        {
          singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma, false);
        }
      }

      if (singleCostTmp < dSingleCostLuma)
      {
        dSingleCostLuma = singleCostTmp;
        validReturnFull = true;

        if (sps.getUseLFNST())
        {
          bestLumaModeId = modeId;
          cbfBestMode = TU::getCbfAtDepth(tu, COMPONENT_Y, currDepth);
          cbfBestModeValid = true;
        }
        else
        {
          bestLumaModeId = trModes[modeId].first;
          if (trModes[modeId].first == 0)
          {
            cbfDCT2 = TU::getCbfAtDepth(tu, COMPONENT_Y, currDepth);
          }
        }

        if (bestLumaModeId != lastCheckId)
        {
          saveLumaCS.getResiBuf(tu.Y()).copyFrom(csFull->getResiBuf(tu.Y()));
          tmpTU->copyComponentFrom(tu, COMPONENT_Y);
          ctxBest = m_CABACEstimator->getCtx();
        }
      }
    }
    if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
    m_pcRdCost->lambdaAdjustColorTrans(false, COMPONENT_Y);

    if (sps.getUseLFNST())
    {
      if (!validReturnFull)
      {
        csFull->cost = MAX_DOUBLE;
        return false;
      }
    }
    else
    {
      CHECK(!validReturnFull, "no transform mode was tested for luma");
    }

    csFull->setDecomp(currArea.Y(), true);
    csFull->setDecomp(currArea.Cb(), true);

    if (bestLumaModeId != lastCheckId)
    {
      csFull->getResiBuf(tu.Y()).copyFrom(saveLumaCS.getResiBuf(tu.Y()));
      tu.copyComponentFrom(*tmpTU, COMPONENT_Y);
      m_CABACEstimator->getCtx() = ctxBest;
    }

    // 3 chroma residual optimization
    CodingStructure &saveChromaCS = *m_pSaveCS[1];
    saveChromaCS.pcv = csFull->pcv;
    saveChromaCS.picture = csFull->picture;
    saveChromaCS.area.repositionTo(csFull->area);
    saveChromaCS.initStructData(MAX_INT, true);
    tmpTU = &saveChromaCS.addTU(currArea, partitioner.chType);

    CompArea&  cbArea = tu.blocks[COMPONENT_Cb];
    CompArea&  crArea = tu.blocks[COMPONENT_Cr];

    tu.jointCbCr = 0;


    CompStorage  orgResiCb[5], orgResiCr[5]; // 0:std, 1-3:jointCbCr (placeholder at this stage), 4:crossComp
    orgResiCb[0].create(cbArea);
    orgResiCr[0].create(crArea);
    orgResiCb[0].copyFrom(csFull->getOrgResiBuf(cbArea));
    orgResiCr[0].copyFrom(csFull->getOrgResiBuf(crArea));

    // 3.1 regular chroma residual coding
    csFull->getResiBuf(cbArea).copyFrom(orgResiCb[0]);
    csFull->getResiBuf(crArea).copyFrom(orgResiCr[0]);

    for (uint32_t c = COMPONENT_Cb; c < ::getNumberValidTBlocks(*csFull->pcv); c++)
    {
      const ComponentID compID = ComponentID(c);
      double  dSingleBestCostChroma = MAX_DOUBLE;
      int     bestModeId = -1;
      bool    tsAllowed = TU::isTSAllowed(tu, compID) && (m_pcEncCfg->getUseChromaTS()) && !cu.lfnstIdx;
      uint8_t numTransformCands = 1 + (tsAllowed ? 1 : 0);  // DCT + TS = 2 tests
      bool        cbfDCT2 = true;

      trModes.clear();
      if (m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless())
      {
        numTransformCands = 1;
        CHECK(!tsAllowed && !cu.bdpcmModeChroma, "transform skip should be enabled for LS");
        if (cu.bdpcmModeChroma)
        {
          trModes.push_back(TrMode(0, true));
        }
        else
        {
          trModes.push_back(TrMode(1, true));
        }
      }
      else
      {
        trModes.push_back(TrMode(0, true));                    // DCT
        if (tsAllowed)
        {
          trModes.push_back(TrMode(1, true));                  // TS
        }
      }
      if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
      {
        if (doReshaping)
        {
          int cResScaleInv = tu.getChromaAdj();
          m_pcRdCost->lambdaAdjustColorTrans(true, compID, true, &cResScaleInv);
        }
        else
        {
          m_pcRdCost->lambdaAdjustColorTrans(true, compID);
        }
      }

      TempCtx ctxBegin(m_CtxCache);
      ctxBegin = m_CABACEstimator->getCtx();

      for (int modeId = 0; modeId < numTransformCands; modeId++)
      {
        if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
        {
          if (modeId && !cbfDCT2)
          {
            continue;
          }
          if (!trModes[modeId].second)
          {
            continue;
          }
        }

        if (modeId > 0)
        {
          m_CABACEstimator->getCtx() = ctxBegin;
        }

        tu.mtsIdx[compID] = trModes[modeId].first;
        Distortion singleDistChroma = 0;
        if (numTransformCands > 1)
        {
          xIntraCodingACTTUBlock(tu, compID, singleDistChroma, modeId == 0 ? &trModes : nullptr, true);
        }
        else
        {
          xIntraCodingACTTUBlock(tu, compID, singleDistChroma);
        }
        if (!tu.mtsIdx[compID])
        {
          cbfDCT2 = TU::getCbfAtDepth(tu, compID, currDepth);
        }
        uint64_t fracBitChroma     = xGetIntraFracBitsQTChroma(tu, compID);
        double   dSingleCostChroma = m_pcRdCost->calcRdCost(fracBitChroma, singleDistChroma, false);
        if (dSingleCostChroma < dSingleBestCostChroma)
        {
          dSingleBestCostChroma = dSingleCostChroma;
          bestModeId            = modeId;
          if (bestModeId != (numTransformCands - 1))
          {
            saveChromaCS.getResiBuf(tu.blocks[compID]).copyFrom(csFull->getResiBuf(tu.blocks[compID]));
            tmpTU->copyComponentFrom(tu, compID);
            ctxBest = m_CABACEstimator->getCtx();
          }
        }
      }

      if (bestModeId != (numTransformCands - 1))
      {
        csFull->getResiBuf(tu.blocks[compID]).copyFrom(saveChromaCS.getResiBuf(tu.blocks[compID]));
        tu.copyComponentFrom(*tmpTU, compID);
        m_CABACEstimator->getCtx() = ctxBest;
      }
      if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
      {
        m_pcRdCost->lambdaAdjustColorTrans(false, compID);
      }
    }

    Position tuPos = tu.Y();
    tuPos.relativeTo(cu.Y());
    const UnitArea relativeUnitArea(tu.chromaFormat, Area(tuPos, tu.Y().size()));
    PelUnitBuf     invColorTransResidual = m_colorTransResiBuf.getBuf(relativeUnitArea);
    csFull->getResiBuf(tu).colorSpaceConvert(invColorTransResidual, false, cs.slice->clpRng(COMPONENT_Y));

    Distortion totalDist = 0;
    for (uint32_t c = COMPONENT_Y; c < ::getNumberValidTBlocks(*csFull->pcv); c++)
    {
      const ComponentID compID = ComponentID(c);
      const CompArea&   area = tu.blocks[compID];
      PelBuf            piOrg = csFull->getOrgBuf(area);
      PelBuf            piReco = csFull->getRecoBuf(area);
      PelBuf            piPred = csFull->getPredBuf(area);
      PelBuf            piResi = invColorTransResidual.bufs[compID];

      if (doReshaping && (compID != COMPONENT_Y))
      {
        piResi.scaleSignal(tu.getChromaAdj(), 0, slice.clpRng(compID));
      }
      piReco.reconstruct(piPred, piResi, cs.slice->clpRng(compID));

      if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (m_pcEncCfg->getLmcs()
        & slice.getLmcsEnabledFlag() && (m_pcReshape->getCTUFlag() || (isChroma(compID) && m_pcEncCfg->getReshapeIntraCMD()))))
      {
        const CPelBuf orgLuma = csFull->getOrgBuf(csFull->area.blocks[COMPONENT_Y]);
        if (compID == COMPONENT_Y && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
        {
          CompArea      tmpArea1(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
          PelBuf tmpRecLuma = m_tmpStorageLCU.getBuf(tmpArea1);
          tmpRecLuma.copyFrom(piReco);
          tmpRecLuma.rspSignal(m_pcReshape->getInvLUT());
          totalDist += m_pcRdCost->getDistPart(piOrg, tmpRecLuma, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
        }
        else
        {
          totalDist += m_pcRdCost->getDistPart(piOrg, piReco, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
        }
      }
      else
      {
        totalDist += m_pcRdCost->getDistPart(piOrg, piReco, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE);
      }
    }

    m_CABACEstimator->getCtx() = ctxStart;
    uint64_t totalBits = xGetIntraFracBitsQT(*csFull, partitioner, true, true, -1, TU_NO_ISP);
    double   totalCost = m_pcRdCost->calcRdCost(totalBits, totalDist);

    saveChromaCS.getResiBuf(cbArea).copyFrom(csFull->getResiBuf(cbArea));
    saveChromaCS.getResiBuf(crArea).copyFrom(csFull->getResiBuf(crArea));
    saveChromaCS.getRecoBuf(tu).copyFrom(csFull->getRecoBuf(tu));
    tmpTU->copyComponentFrom(tu, COMPONENT_Cb);
    tmpTU->copyComponentFrom(tu, COMPONENT_Cr);
    ctxBest = m_CABACEstimator->getCtx();

    // 3.2 jointCbCr
    double     bestCostJointCbCr = totalCost;
    Distortion bestDistJointCbCr = totalDist;
    uint64_t   bestBitsJointCbCr = totalBits;
    int        bestJointCbCr = tu.jointCbCr; assert(!bestJointCbCr);

    bool       lastIsBest = false;
    std::vector<int>  jointCbfMasksToTest;
    if (sps.getJointCbCrEnabledFlag() && (TU::getCbf(tu, COMPONENT_Cb) || TU::getCbf(tu, COMPONENT_Cr)))
    {
      jointCbfMasksToTest = m_pcTrQuant->selectICTCandidates(tu, orgResiCb, orgResiCr);
    }

    for (int cbfMask : jointCbfMasksToTest)
    {
      tu.jointCbCr = (uint8_t)cbfMask;

      ComponentID codeCompId = ((cbfMask >> 1) ? COMPONENT_Cb : COMPONENT_Cr);
      ComponentID otherCompId = ((codeCompId == COMPONENT_Cb) ? COMPONENT_Cr : COMPONENT_Cb);
      bool        tsAllowed = TU::isTSAllowed(tu, codeCompId) && (m_pcEncCfg->getUseChromaTS()) && !cu.lfnstIdx;
      uint8_t     numTransformCands = 1 + (tsAllowed ? 1 : 0); // DCT + TS = 2 tests
      bool        cbfDCT2 = true;

      trModes.clear();
      trModes.push_back(TrMode(0, true)); // DCT2
      if (tsAllowed)
      {
        trModes.push_back(TrMode(1, true));//TS
      }

      for (int modeId = 0; modeId < numTransformCands; modeId++)
      {
        if (modeId && !cbfDCT2)
        {
          continue;
        }
        if (!trModes[modeId].second)
        {
          continue;
        }
        Distortion distTmp = 0;
        tu.mtsIdx[codeCompId] = trModes[modeId].first;
        tu.mtsIdx[otherCompId] = MTS_DCT2_DCT2;
        m_CABACEstimator->getCtx() = ctxStart;
        csFull->getResiBuf(cbArea).copyFrom(orgResiCb[cbfMask]);
        csFull->getResiBuf(crArea).copyFrom(orgResiCr[cbfMask]);
        if (nNumTransformCands > 1)
        {
          xIntraCodingACTTUBlock(tu, COMPONENT_Cb, distTmp, modeId == 0 ? &trModes : nullptr, true);
        }
        else
        {
          xIntraCodingACTTUBlock(tu, COMPONENT_Cb, distTmp);
        }

        double   costTmp = std::numeric_limits<double>::max();
        uint64_t bitsTmp = 0;
        if (distTmp < std::numeric_limits<Distortion>::max())
        {
          if (!tu.mtsIdx[codeCompId])
          {
            cbfDCT2 = true;
          }
          csFull->getResiBuf(tu).colorSpaceConvert(invColorTransResidual, false, csFull->slice->clpRng(COMPONENT_Y));
          distTmp = 0;
          for (uint32_t c = COMPONENT_Y; c < ::getNumberValidTBlocks(*csFull->pcv); c++)
          {
            const ComponentID compID = ComponentID(c);
            const CompArea &  area   = tu.blocks[compID];
            PelBuf            piOrg  = csFull->getOrgBuf(area);
            PelBuf            piReco = csFull->getRecoBuf(area);
            PelBuf            piPred = csFull->getPredBuf(area);
            PelBuf            piResi = invColorTransResidual.bufs[compID];

            if (doReshaping && (compID != COMPONENT_Y))
            {
              piResi.scaleSignal(tu.getChromaAdj(), 0, slice.clpRng(compID));
            }
            piReco.reconstruct(piPred, piResi, cs.slice->clpRng(compID));
            if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()
                || (m_pcEncCfg->getLmcs() & slice.getLmcsEnabledFlag()
                    && (m_pcReshape->getCTUFlag() || (isChroma(compID) && m_pcEncCfg->getReshapeIntraCMD()))))
            {
              const CPelBuf orgLuma = csFull->getOrgBuf(csFull->area.blocks[COMPONENT_Y]);
              if (compID == COMPONENT_Y && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
              {
                CompArea tmpArea1(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
                PelBuf   tmpRecLuma = m_tmpStorageLCU.getBuf(tmpArea1);
                tmpRecLuma.copyFrom(piReco);
                tmpRecLuma.rspSignal(m_pcReshape->getInvLUT());
                distTmp += m_pcRdCost->getDistPart(piOrg, tmpRecLuma, sps.getBitDepth(toChannelType(compID)), compID,
                                                   DF_SSE_WTD, &orgLuma);
              }
              else
              {
                distTmp += m_pcRdCost->getDistPart(piOrg, piReco, sps.getBitDepth(toChannelType(compID)), compID,
                                                   DF_SSE_WTD, &orgLuma);
              }
            }
            else
            {
              distTmp += m_pcRdCost->getDistPart(piOrg, piReco, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE);
            }
          }

          bitsTmp = xGetIntraFracBitsQT(*csFull, partitioner, true, true, -1, TU_NO_ISP);
          costTmp = m_pcRdCost->calcRdCost(bitsTmp, distTmp);
        }
        else if (!tu.mtsIdx[codeCompId])
        {
          cbfDCT2 = false;
        }

        if (costTmp < bestCostJointCbCr)
        {
          bestCostJointCbCr = costTmp;
          bestDistJointCbCr = distTmp;
          bestBitsJointCbCr = bitsTmp;
          bestJointCbCr     = tu.jointCbCr;
          lastIsBest        = (cbfMask == jointCbfMasksToTest.back() && modeId == (numTransformCands - 1));

          // store data
          if (!lastIsBest)
          {
            saveChromaCS.getResiBuf(cbArea).copyFrom(csFull->getResiBuf(cbArea));
            saveChromaCS.getResiBuf(crArea).copyFrom(csFull->getResiBuf(crArea));
            saveChromaCS.getRecoBuf(tu).copyFrom(csFull->getRecoBuf(tu));
            tmpTU->copyComponentFrom(tu, COMPONENT_Cb);
            tmpTU->copyComponentFrom(tu, COMPONENT_Cr);

            ctxBest = m_CABACEstimator->getCtx();
          }
        }
      }
    }

    if (!lastIsBest)
    {
      csFull->getResiBuf(cbArea).copyFrom(saveChromaCS.getResiBuf(cbArea));
      csFull->getResiBuf(crArea).copyFrom(saveChromaCS.getResiBuf(crArea));
      csFull->getRecoBuf(tu).copyFrom(saveChromaCS.getRecoBuf(tu));
      tu.copyComponentFrom(*tmpTU, COMPONENT_Cb);
      tu.copyComponentFrom(*tmpTU, COMPONENT_Cr);

      m_CABACEstimator->getCtx() = ctxBest;
    }
    tu.jointCbCr = bestJointCbCr;
    csFull->picture->getRecoBuf(tu).copyFrom(csFull->getRecoBuf(tu));

    csFull->dist += bestDistJointCbCr;
    csFull->fracBits += bestBitsJointCbCr;
    csFull->cost = m_pcRdCost->calcRdCost(csFull->fracBits, csFull->dist);
  }

  bool validReturnSplit = false;
  if (bCheckSplit)
  {
    if (partitioner.canSplit(TU_MAX_TR_SPLIT, *csSplit))
    {
      partitioner.splitCurrArea(TU_MAX_TR_SPLIT, *csSplit);
    }

    bool splitIsSelected = true;
    do
    {
      bool tmpValidReturnSplit = xRecurIntraCodingACTQT(*csSplit, partitioner, mtsCheckRangeFlag, mtsFirstCheckId, mtsLastCheckId, moreProbMTSIdxFirst);
      if (sps.getUseLFNST())
      {
        if (!tmpValidReturnSplit)
        {
          splitIsSelected = false;
          break;
        }
      }
      else
      {
        CHECK(!tmpValidReturnSplit, "invalid RD of sub-TU partitions for ACT");
      }
    } while (partitioner.nextPart(*csSplit));

    partitioner.exitCurrSplit();

    if (splitIsSelected)
    {
      unsigned compCbf[3] = { 0, 0, 0 };
      for (auto &currTU : csSplit->traverseTUs(currArea, partitioner.chType))
      {
        for (unsigned ch = 0; ch < getNumberValidTBlocks(*csSplit->pcv); ch++)
        {
          compCbf[ch] |= (TU::getCbfAtDepth(currTU, ComponentID(ch), currDepth + 1) ? 1 : 0);
        }
      }

      for (auto &currTU : csSplit->traverseTUs(currArea, partitioner.chType))
      {
        TU::setCbfAtDepth(currTU, COMPONENT_Y, currDepth, compCbf[COMPONENT_Y]);
        TU::setCbfAtDepth(currTU, COMPONENT_Cb, currDepth, compCbf[COMPONENT_Cb]);
        TU::setCbfAtDepth(currTU, COMPONENT_Cr, currDepth, compCbf[COMPONENT_Cr]);
      }

      m_CABACEstimator->getCtx() = ctxStart;
      csSplit->fracBits = xGetIntraFracBitsQT(*csSplit, partitioner, true, true, -1, TU_NO_ISP);
      csSplit->cost = m_pcRdCost->calcRdCost(csSplit->fracBits, csSplit->dist);

      validReturnSplit = true;
    }
  }

  bool retVal = false;
  if (csFull || csSplit)
  {
    if (sps.getUseLFNST())
    {
      if (validReturnFull || validReturnSplit)
      {
        retVal = true;
      }
    }
    else
    {
      CHECK(!validReturnFull && !validReturnSplit, "illegal TU optimization");
      retVal = true;
    }
  }
  return retVal;
}

ChromaCbfs IntraSearch::xRecurIntraChromaCodingQT( CodingStructure &cs, Partitioner& partitioner, const double bestCostSoFar, const PartSplit ispType )
{
  UnitArea currArea                   = partitioner.currArea();
  const bool keepResi                 = cs.sps->getUseLMChroma() || KEEP_PRED_AND_RESI_SIGNALS;
  if( !currArea.Cb().valid() ) return ChromaCbfs( false );
  const Slice           &slice = *cs.slice;

  // 当前TU、PU
  TransformUnit &currTU               = *cs.getTU( currArea.chromaPos(), CHANNEL_TYPE_CHROMA );
  const PredictionUnit &pu            = *cs.getPU( currArea.chromaPos(), CHANNEL_TYPE_CHROMA );

  bool lumaUsesISP                    = false;
  uint32_t     currDepth                  = partitioner.currTrDepth;
  ChromaCbfs cbfs                     ( false );

  if (currDepth == currTU.depth) // 当前划分深度
  {
    if (!currArea.Cb().valid() || !currArea.Cr().valid())
    {
      return cbfs;
    }

    CodingStructure &saveCS = *m_pSaveCS[1];
    saveCS.pcv      = cs.pcv;
    saveCS.picture  = cs.picture;
    saveCS.area.repositionTo( cs.area ); // 临时存储CS的area
    saveCS.initStructData( MAX_INT, true );

    if( !currTU.cu->isSepTree() && currTU.cu->ispMode )
    {
      saveCS.clearCUs();
      CodingUnit& auxCU = saveCS.addCU( *currTU.cu, partitioner.chType );
      auxCU.ispMode = currTU.cu->ispMode;
      saveCS.sps = currTU.cs->sps;
      saveCS.clearPUs();
      saveCS.addPU( *currTU.cu->firstPU, partitioner.chType );
    }

    TransformUnit &tmpTU = saveCS.addTU(currArea, partitioner.chType);

    cs.setDecomp(currArea.Cb(), true); // set in advance (required for Cb2/Cr2 in 4:2:2 video)

    const unsigned      numTBlocks  = ::getNumberValidTBlocks( *cs.pcv );

    CompArea&  cbArea         = currTU.blocks[COMPONENT_Cb];
    CompArea&  crArea         = currTU.blocks[COMPONENT_Cr];
    double     bestCostCb     = MAX_DOUBLE;
    double     bestCostCr     = MAX_DOUBLE;
    Distortion bestDistCb     = 0;
    Distortion bestDistCr     = 0;
    int        maxModesTested = 0;
    bool       earlyExitISP   = false;

    TempCtx ctxStartTU( m_CtxCache );
    TempCtx ctxStart  ( m_CtxCache );
    TempCtx ctxBest   ( m_CtxCache );

    ctxStartTU       = m_CABACEstimator->getCtx();
    currTU.jointCbCr = 0;

    // Do predictions here to avoid repeating the "default0Save1Load2" stuff
    // 在这进行预测，之后就不需要传输并修改default0Save1Load2值，不需要在xIntraCodingTUBlock函数中计算并保存预测值
    int  predMode   = pu.cu->bdpcmModeChroma ? BDPCM_IDX : PU::getFinalIntraMode(pu, CHANNEL_TYPE_CHROMA);

    // 色度预测值
    PelBuf piPredCb = cs.getPredBuf(cbArea);
    PelBuf piPredCr = cs.getPredBuf(crArea);

    initIntraPatternChType( *currTU.cu, cbArea);
    initIntraPatternChType( *currTU.cu, crArea);

    // 计算CCLM预测值 
    if( PU::isLMCMode( predMode ) )
    {
      xGetLumaRecPixels( pu, cbArea );
      predIntraChromaLM( COMPONENT_Cb, piPredCb, pu, cbArea, predMode );
      predIntraChromaLM( COMPONENT_Cr, piPredCr, pu, crArea, predMode );
    }
    else if (PU::isMIP(pu, CHANNEL_TYPE_CHROMA)) //YUV444格式色度采用MIP
    {
      initIntraMip(pu, cbArea);
      predIntraMip(COMPONENT_Cb, piPredCb, pu);

      initIntraMip(pu, crArea);
      predIntraMip(COMPONENT_Cr, piPredCr, pu);
    }
    else // 角度模式
    {
      predIntraAng( COMPONENT_Cb, piPredCb, pu);
      predIntraAng( COMPONENT_Cr, piPredCr, pu);
    }

    // determination of chroma residuals including reshaping and cross-component prediction
    //----- get chroma residuals -----
    PelBuf resiCb  = cs.getResiBuf(cbArea);
    PelBuf resiCr  = cs.getResiBuf(crArea);
    resiCb.copyFrom( cs.getOrgBuf (cbArea) );
    resiCr.copyFrom( cs.getOrgBuf (crArea) );
    // 计算原始残差(原始-预测)
    resiCb.subtract( piPredCb );
    resiCr.subtract( piPredCr );

    //----- get reshape parameter ----
    bool doReshaping = ( cs.slice->getLmcsEnabledFlag() && cs.picHeader->getLmcsChromaResidualScaleFlag()
                         && (cs.slice->isIntra() || m_pcReshape->getCTUFlag()) && (cbArea.width * cbArea.height > 4) );
    if( doReshaping )
    {
      const Area area = currTU.Y().valid() ? currTU.Y() : Area(recalcPosition(currTU.chromaFormat, currTU.chType, CHANNEL_TYPE_LUMA, currTU.blocks[currTU.chType].pos()), recalcSize(currTU.chromaFormat, currTU.chType, CHANNEL_TYPE_LUMA, currTU.blocks[currTU.chType].size()));
      const CompArea &areaY = CompArea(COMPONENT_Y, currTU.chromaFormat, area);
      int adj = m_pcReshape->calculateChromaAdjVpduNei(currTU, areaY);
      currTU.setChromaAdj(adj);
    }

    //----- get cross component prediction parameters -----
    //===== store original residual signals =====
    // 保存原始残差信号
    CompStorage  orgResiCb[4], orgResiCr[4]; // 0:std, 1-3:jointCbCr (placeholder at this stage)
    orgResiCb[0].create( cbArea );
    orgResiCr[0].create( crArea );
    orgResiCb[0].copyFrom( resiCb );
    orgResiCr[0].copyFrom( resiCr );
    if( doReshaping )
    {
      int cResScaleInv = currTU.getChromaAdj();
      orgResiCb[0].scaleSignal( cResScaleInv, 1, currTU.cu->cs->slice->clpRng(COMPONENT_Cb) );
      orgResiCr[0].scaleSignal( cResScaleInv, 1, currTU.cu->cs->slice->clpRng(COMPONENT_Cr) );
    }
    // 遍历Cb Cr，分别对Cb Cr单独进行编码
    for( uint32_t c = COMPONENT_Cb; c < numTBlocks; c++)
    {
      const ComponentID compID  = ComponentID(c);
      const CompArea&   area    = currTU.blocks[compID];

      double     dSingleCost    = MAX_DOUBLE;
      int        bestModeId     = 0;
      Distortion singleDistCTmp = 0;
      double     singleCostTmp  = 0;
      // 是否允许色度进行变换跳过
      const bool tsAllowed = TU::isTSAllowed(currTU, compID) && m_pcEncCfg->getUseChromaTS() && !currTU.cu->lfnstIdx;
      uint8_t nNumTransformCands = 1 + (tsAllowed ? 1 : 0); // DCT + TS = 2 tests
      std::vector<TrMode> trModes;
      // 无损编码
      if (m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless())
      {
        nNumTransformCands = 1;
        CHECK(!tsAllowed && !currTU.cu->bdpcmModeChroma, "transform skip should be enabled for LS");
        if (currTU.cu->bdpcmModeChroma)
        {
          trModes.push_back(TrMode(0, true));
        }
        else
        {
          trModes.push_back(TrMode(1, true));
        }
      }
      else
      {
        trModes.push_back(TrMode(0, true));   // DCT2加入模式测试列表

        if (tsAllowed)
        {
          trModes.push_back(TrMode(1, true));   // TS变换跳过加入模式测试列表
        }
      }
      CHECK(!currTU.Cb().valid(), "Invalid TU");

      const int  totalModesToTest            = nNumTransformCands;
      bool cbfDCT2 = true;
      const bool isOneMode                   = false;
      maxModesTested                         = totalModesToTest > maxModesTested ? totalModesToTest : maxModesTested;

      int currModeId = 0;
      int default0Save1Load2 = 0;

      if (!isOneMode)
      {
        ctxStart = m_CABACEstimator->getCtx();
      }
      // 遍历变换类型 只进行DCT-2或者TS变换跳过
      for (int modeId = 0; modeId < nNumTransformCands; modeId++)
      {
        // 复制原始残差信号
        resiCb.copyFrom(orgResiCb[0]);
        resiCr.copyFrom(orgResiCr[0]);
        // 设置变换类型
        currTU.mtsIdx[compID] = currTU.cu->bdpcmModeChroma ? MTS_SKIP : trModes[modeId].first;

        currModeId++;

        const bool isFirstMode = (currModeId == 1); //是否是第一个测试的模式
        // 始终将输出存储到saveCS和tmpTU
        const bool isLastMode  = false;   // Always store output to saveCS and tmpTU
        if (!(m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless()))
        {
          // if DCT2's cbf==0, skip ts search
          // 如果DCT - 2变换的cbf为0，则跳过变换跳过模式
          if (!cbfDCT2 && trModes[modeId].first == MTS_SKIP)
          {
              break;
          }
          if (!trModes[modeId].second)
          {
              continue;
          }
        }

        if (!isFirstMode)   // if not first mode to be tested
        {
          m_CABACEstimator->getCtx() = ctxStart;
        }

        singleDistCTmp = 0;
        // 进行变换量化，并计算原始值和重建值的失真D，存在singleDistCTmp中
        if (nNumTransformCands > 1)
        {
          xIntraCodingTUBlock(currTU, compID, singleDistCTmp, default0Save1Load2, nullptr,
                              modeId == 0 ? &trModes : nullptr, true);
        }
        else
        {
          xIntraCodingTUBlock(currTU, compID, singleDistCTmp, default0Save1Load2);
        }

        // 为了在cbf为零时不编码TS标志，禁止cbf为零的TS的情况。
        if (((currTU.mtsIdx[compID] == MTS_SKIP && !currTU.cu->bdpcmModeChroma)
             && !TU::getCbf(currTU, compID)))   // In order not to code TS flag when cbf is zero, the case for TS with
                                                // cbf being zero is forbidden.
        {
          if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
          {
            singleCostTmp = MAX_DOUBLE;
          }
          else
          {
            // 获得残差系数编码比特数
            uint64_t fracBitsTmp = xGetIntraFracBitsQTChroma(currTU, compID);
            singleCostTmp        = m_pcRdCost->calcRdCost(fracBitsTmp, singleDistCTmp);  // RD cost
          }
        }
        else if (lumaUsesISP && bestCostSoFar != MAX_DOUBLE && c == COMPONENT_Cb)
        {
          uint64_t fracBitsTmp = xGetIntraFracBitsQTSingleChromaComponent(cs, partitioner, ComponentID(c));
          singleCostTmp        = m_pcRdCost->calcRdCost(fracBitsTmp, singleDistCTmp);
          if (isOneMode || (!isOneMode && !isLastMode))
          {
            m_CABACEstimator->getCtx() = ctxStart;
          }
        }
        else if (!isOneMode)
        {
          uint64_t fracBitsTmp = xGetIntraFracBitsQTChroma(currTU, compID);
          singleCostTmp        = m_pcRdCost->calcRdCost(fracBitsTmp, singleDistCTmp);
        }

        if (singleCostTmp < dSingleCost)
        {
          dSingleCost = singleCostTmp; //单个分量的最佳Cost
          bestModeId  = currModeId;

          if (c == COMPONENT_Cb)
          {
            bestCostCb = singleCostTmp; //最佳Cb变换模式对应的Cost
            bestDistCb = singleDistCTmp; //原始值和重建值的失真D
          }
          else
          {
            bestCostCr = singleCostTmp;
            bestDistCr = singleDistCTmp;
          }

          if (currTU.mtsIdx[compID] == MTS_DCT2_DCT2)
          {
            cbfDCT2 = TU::getCbfAtDepth(currTU, compID, currDepth);
          }

          if (!isLastMode)
          {
#if KEEP_PRED_AND_RESI_SIGNALS
            saveCS.getPredBuf(area).copyFrom(cs.getPredBuf(area));
            saveCS.getOrgResiBuf(area).copyFrom(cs.getOrgResiBuf(area));
#endif
            saveCS.getPredBuf(area).copyFrom(cs.getPredBuf(area));
            if (keepResi)
            {
              saveCS.getResiBuf(area).copyFrom(cs.getResiBuf(area));
            }
            saveCS.getRecoBuf(area).copyFrom(cs.getRecoBuf(area));

            tmpTU.copyComponentFrom(currTU, compID);

            ctxBest = m_CABACEstimator->getCtx();
          }
        }
      }

      if( lumaUsesISP && dSingleCost > bestCostSoFar && c == COMPONENT_Cb )
      {
        //Luma + Cb cost is already larger than the best cost, so we don't need to test Cr
        //  Luma+Cb成本已经大于最佳成本，所以我们不需要测试Cr
        cs.dist = MAX_UINT;
        m_CABACEstimator->getCtx() = ctxStart;
        earlyExitISP               = true;
        break;
        //return cbfs;
      }

      // Done with one component of separate coding of Cr and Cb, just switch to the best Cb contexts if Cr coding is still to be done 
      // 使用Cr和Cb单独编码的一个分量完成，如果Cr编码仍然需要完成，只需切换到最佳Cb上下文即可
      if ((c == COMPONENT_Cb && bestModeId < totalModesToTest) || (c == COMPONENT_Cb && m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless()))
      {
        m_CABACEstimator->getCtx() = ctxBest;

        currTU.copyComponentFrom(tmpTU, COMPONENT_Cb); // Cbf of Cb is needed to estimate cost for Cr Cbf
      }
    }
    // 测试JointCbCr模式
    if ( !earlyExitISP )
    {
      // Test using joint chroma residual coding
      // 联合色度残差编码测试,CbCr的最佳失真和cost
      double     bestCostCbCr   = bestCostCb + bestCostCr;
      Distortion bestDistCbCr   = bestDistCb + bestDistCr;
      int        bestJointCbCr  = 0;
      std::vector<int>  jointCbfMasksToTest;
      if ( cs.sps->getJointCbCrEnabledFlag() && (TU::getCbf(tmpTU, COMPONENT_Cb) || TU::getCbf(tmpTU, COMPONENT_Cr)))
      {
        jointCbfMasksToTest = m_pcTrQuant->selectICTCandidates(currTU, orgResiCb, orgResiCr);
      }
      // 只使用DCT-2
      bool checkDCTOnly = (TU::getCbf(tmpTU, COMPONENT_Cb) && tmpTU.mtsIdx[COMPONENT_Cb] == MTS_DCT2_DCT2 && !TU::getCbf(tmpTU, COMPONENT_Cr)) ||
                          (TU::getCbf(tmpTU, COMPONENT_Cr) && tmpTU.mtsIdx[COMPONENT_Cr] == MTS_DCT2_DCT2 && !TU::getCbf(tmpTU, COMPONENT_Cb)) ||
                          (TU::getCbf(tmpTU, COMPONENT_Cb) && tmpTU.mtsIdx[COMPONENT_Cb] == MTS_DCT2_DCT2 && TU::getCbf(tmpTU, COMPONENT_Cr) && tmpTU.mtsIdx[COMPONENT_Cr] == MTS_DCT2_DCT2);

      // 只是用TS变换跳过
      bool checkTSOnly = (TU::getCbf(tmpTU, COMPONENT_Cb) && tmpTU.mtsIdx[COMPONENT_Cb] == MTS_SKIP && !TU::getCbf(tmpTU, COMPONENT_Cr)) ||
                         (TU::getCbf(tmpTU, COMPONENT_Cr) && tmpTU.mtsIdx[COMPONENT_Cr] == MTS_SKIP && !TU::getCbf(tmpTU, COMPONENT_Cb)) ||
                         (TU::getCbf(tmpTU, COMPONENT_Cb) && tmpTU.mtsIdx[COMPONENT_Cb] == MTS_SKIP && TU::getCbf(tmpTU, COMPONENT_Cr) && tmpTU.mtsIdx[COMPONENT_Cr] == MTS_SKIP);

      if (jointCbfMasksToTest.size() && currTU.cu->bdpcmModeChroma)
      {
        CHECK(!checkTSOnly || checkDCTOnly, "bdpcm only allows transform skip");
      }
      // 遍历需要测试的JointCbCr模式
      for( int cbfMask : jointCbfMasksToTest )
      {

        currTU.jointCbCr               = (uint8_t)cbfMask;
        ComponentID codeCompId = ((currTU.jointCbCr >> 1) ? COMPONENT_Cb : COMPONENT_Cr);
        ComponentID otherCompId = ((codeCompId == COMPONENT_Cb) ? COMPONENT_Cr : COMPONENT_Cb);
        bool        tsAllowed = TU::isTSAllowed(currTU, codeCompId) && (m_pcEncCfg->getUseChromaTS()) && !currTU.cu->lfnstIdx;
        uint8_t     numTransformCands = 1 + (tsAllowed ? 1 : 0); // DCT + TS = 2 tests
        bool        cbfDCT2 = true;

        std::vector<TrMode> trModes;
        if (checkDCTOnly || checkTSOnly)
        {
          numTransformCands = 1;
        }

        if (!checkTSOnly || currTU.cu->bdpcmModeChroma)
        {
          trModes.push_back(TrMode(0, true)); // DCT2
        }
        if (tsAllowed && !checkDCTOnly)
        {
          trModes.push_back(TrMode(1, true));//TS
        }
        // 遍历两种模式
        for (int modeId = 0; modeId < numTransformCands; modeId++)
        {
          if (modeId && !cbfDCT2)
          {
            continue;
          }
          if (!trModes[modeId].second)
          {
            continue;
          }
          Distortion distTmp = 0;
          currTU.mtsIdx[codeCompId] = currTU.cu->bdpcmModeChroma ? MTS_SKIP : trModes[modeId].first;
          currTU.mtsIdx[otherCompId] = MTS_DCT2_DCT2;
          m_CABACEstimator->getCtx() = ctxStartTU;

          resiCb.copyFrom(orgResiCb[cbfMask]);
          resiCr.copyFrom(orgResiCr[cbfMask]);
          if (numTransformCands > 1)
          {
            xIntraCodingTUBlock(currTU, COMPONENT_Cb, distTmp, 0, nullptr, modeId == 0 ? &trModes : nullptr, true);
          }
          else
          {
            xIntraCodingTUBlock(currTU, COMPONENT_Cb, distTmp, 0);
          }
          double costTmp = std::numeric_limits<double>::max();
          if (distTmp < std::numeric_limits<Distortion>::max())
          {
            uint64_t bits = xGetIntraFracBitsQTChroma(currTU, COMPONENT_Cb);
            costTmp       = m_pcRdCost->calcRdCost(bits, distTmp);
            if (!currTU.mtsIdx[codeCompId])
            {
              cbfDCT2 = true;
            }
          }
          else if (!currTU.mtsIdx[codeCompId])
          {
            cbfDCT2 = false;
          }

          if (costTmp < bestCostCbCr)
          {
            bestCostCbCr  = costTmp;
            bestDistCbCr  = distTmp;
            bestJointCbCr = currTU.jointCbCr;

            // store data
            {
#if KEEP_PRED_AND_RESI_SIGNALS
              saveCS.getOrgResiBuf(cbArea).copyFrom(cs.getOrgResiBuf(cbArea));
              saveCS.getOrgResiBuf(crArea).copyFrom(cs.getOrgResiBuf(crArea));
#endif
              saveCS.getPredBuf(cbArea).copyFrom(cs.getPredBuf(cbArea));
              saveCS.getPredBuf(crArea).copyFrom(cs.getPredBuf(crArea));
              if (keepResi)
              {
                saveCS.getResiBuf(cbArea).copyFrom(cs.getResiBuf(cbArea));
                saveCS.getResiBuf(crArea).copyFrom(cs.getResiBuf(crArea));
              }
              saveCS.getRecoBuf(cbArea).copyFrom(cs.getRecoBuf(cbArea));
              saveCS.getRecoBuf(crArea).copyFrom(cs.getRecoBuf(crArea));

              tmpTU.copyComponentFrom(currTU, COMPONENT_Cb);
              tmpTU.copyComponentFrom(currTU, COMPONENT_Cr);

              ctxBest = m_CABACEstimator->getCtx();
            }
          }
        }
      }
      // 检索最佳CU数据（除非它是最后一个测试的数据）
      // Retrieve the best CU data (unless it was the very last one tested)
      {
#if KEEP_PRED_AND_RESI_SIGNALS
        cs.getPredBuf   (cbArea).copyFrom(saveCS.getPredBuf   (cbArea));
        cs.getOrgResiBuf(cbArea).copyFrom(saveCS.getOrgResiBuf(cbArea));
        cs.getPredBuf   (crArea).copyFrom(saveCS.getPredBuf   (crArea));
        cs.getOrgResiBuf(crArea).copyFrom(saveCS.getOrgResiBuf(crArea));
#endif
        cs.getPredBuf   (cbArea).copyFrom(saveCS.getPredBuf   (cbArea));
        cs.getPredBuf   (crArea).copyFrom(saveCS.getPredBuf   (crArea));

        if( keepResi )
        {
          cs.getResiBuf (cbArea).copyFrom(saveCS.getResiBuf   (cbArea));
          cs.getResiBuf (crArea).copyFrom(saveCS.getResiBuf   (crArea));
        }
        cs.getRecoBuf   (cbArea).copyFrom(saveCS.getRecoBuf   (cbArea));
        cs.getRecoBuf   (crArea).copyFrom(saveCS.getRecoBuf   (crArea));

        currTU.copyComponentFrom(tmpTU, COMPONENT_Cb);
        currTU.copyComponentFrom(tmpTU, COMPONENT_Cr);

        m_CABACEstimator->getCtx() = ctxBest;
      }

      // Copy results to the picture structures
      cs.picture->getRecoBuf(cbArea).copyFrom(cs.getRecoBuf(cbArea));
      cs.picture->getRecoBuf(crArea).copyFrom(cs.getRecoBuf(crArea));
      cs.picture->getPredBuf(cbArea).copyFrom(cs.getPredBuf(cbArea));
      cs.picture->getPredBuf(crArea).copyFrom(cs.getPredBuf(crArea));

      cbfs.cbf(COMPONENT_Cb) = TU::getCbf(currTU, COMPONENT_Cb);
      cbfs.cbf(COMPONENT_Cr) = TU::getCbf(currTU, COMPONENT_Cr);

      currTU.jointCbCr = ( (cbfs.cbf(COMPONENT_Cb) + cbfs.cbf(COMPONENT_Cr)) ? bestJointCbCr : 0 );
      cs.dist         += bestDistCbCr;
    }
  }
  else
  {
    // 仅ISP模式（仅单树下才会进入，双树亮度色度单独划分）或者CU尺寸大于最大变换尺寸时，会进入该部分，对TU进行进一步划分
    unsigned    numValidTBlocks   = ::getNumberValidTBlocks( *cs.pcv );
    ChromaCbfs  SplitCbfs         ( false );

    // 超过最大TU尺寸要进一步划分
    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }
    else if( currTU.cu->ispMode ) // ISP模式仅单树才会进一步划分
    {
      partitioner.splitCurrArea( ispType, cs );
    }
    else
    {
      THROW( "Implicit TU split not available" );
    }

    do
    {
      ChromaCbfs subCbfs = xRecurIntraChromaCodingQT( cs, partitioner, bestCostSoFar, ispType );

      for( uint32_t ch = COMPONENT_Cb; ch < numValidTBlocks; ch++ )
      {
        const ComponentID compID = ComponentID( ch );
        SplitCbfs.cbf( compID ) |= subCbfs.cbf( compID );
      }
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();

    if( lumaUsesISP && cs.dist == MAX_UINT )
    {
      return cbfs;
    }
    cbfs.Cb |= SplitCbfs.Cb;
    cbfs.Cr |= SplitCbfs.Cr;

    if (!lumaUsesISP)
    {
      for (auto &ptu: cs.tus)
      {
        if (currArea.Cb().contains(ptu->Cb()) || (!ptu->Cb().valid() && currArea.Y().contains(ptu->Y())))
        {
          TU::setCbfAtDepth(*ptu, COMPONENT_Cb, currDepth, SplitCbfs.Cb);
          TU::setCbfAtDepth(*ptu, COMPONENT_Cr, currDepth, SplitCbfs.Cr);
        }
      }
    }
  }

  return cbfs;
}

uint64_t IntraSearch::xFracModeBitsIntra(PredictionUnit &pu, const uint32_t &mode, const ChannelType &chType)
{
  uint32_t orgMode = mode;

  if (!pu.ciipFlag)
  std::swap(orgMode, pu.intraDir[chType]);

  m_CABACEstimator->resetBits();

  if( isLuma( chType ) )
  {
    if (!pu.ciipFlag)
    {
      m_CABACEstimator->intra_luma_pred_mode(pu);
    }
  }
  else
  {
    m_CABACEstimator->intra_chroma_pred_mode( pu );
  }

  if ( !pu.ciipFlag )
  std::swap(orgMode, pu.intraDir[chType]);

  return m_CABACEstimator->getEstFracBits();
}

void IntraSearch::sortRdModeListFirstColorSpace(ModeInfo mode, double cost, char bdpcmMode, ModeInfo* rdModeList, double* rdCostList, char* bdpcmModeList, int& candNum)
{
  if (candNum == 0)
  {
    rdModeList[0] = mode;
    rdCostList[0] = cost;
    bdpcmModeList[0] = bdpcmMode;
    candNum++;
    return;
  }

  int insertPos = -1;
  for (int pos = candNum - 1; pos >= 0; pos--)
  {
    if (cost < rdCostList[pos])
    {
      insertPos = pos;
    }
  }

  if (insertPos >= 0)
  {
    for (int i = candNum - 1; i >= insertPos; i--)
    {
      rdModeList[i + 1] = rdModeList[i];
      rdCostList[i + 1] = rdCostList[i];
      bdpcmModeList[i + 1] = bdpcmModeList[i];
    }
    rdModeList[insertPos] = mode;
    rdCostList[insertPos] = cost;
    bdpcmModeList[insertPos] = bdpcmMode;
    candNum++;
  }
  else
  {
    rdModeList[candNum] = mode;
    rdCostList[candNum] = cost;
    bdpcmModeList[candNum] = bdpcmMode;
    candNum++;
  }

  CHECK(candNum > FAST_UDI_MAX_RDMODE_NUM, "exceed intra mode candidate list capacity");

  return;
}

void IntraSearch::invalidateBestRdModeFirstColorSpace()
{
  int numSaveRdClass = 4 * NUM_LFNST_NUM_PER_SET * 2;
  int savedRdModeListSize = FAST_UDI_MAX_RDMODE_NUM;

  for (int i = 0; i < numSaveRdClass; i++)
  {
    m_numSavedRdModeFirstColorSpace[i] = 0;
    for (int j = 0; j < savedRdModeListSize; j++)
    {
      m_savedRdModeFirstColorSpace[i][j] = ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, 0);
      m_savedBDPCMModeFirstColorSpace[i][j] = 0;
      m_savedRdCostFirstColorSpace[i][j] = MAX_DOUBLE;
    }
  }
}

template<typename T, size_t N>
void IntraSearch::reduceHadCandList(static_vector<T, N>& candModeList, static_vector<double, N>& candCostList, int& numModesForFullRD, const double thresholdHadCost, const double* mipHadCost, const PredictionUnit &pu, const bool fastMip)
{
  // 每种类型的模式，最多的候选数目 mip/traditional
  const int maxCandPerType = numModesForFullRD >> 1; 
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> tempRdModeList;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> tempCandCostList; // 临时候选模式和代价列表
  const double minCost = candCostList[0]; // 最小代价
  bool keepOneMip = candModeList.size() > numModesForFullRD;// 保留一个MIP模式

  int numConv = 0;// 传统模式数目
  int numMip = 0; // MIP模式数目

  // 缩减传统候选模式数和MIP候选模式数
  // keepOneMip（无论之前TRUE or FALSE，最终都会删除最后一种MIP模式）
  for (int idx = 0; idx < candModeList.size() - (keepOneMip?0:1); idx++)
  {
    bool addMode = false;
    const ModeInfo& orgMode = candModeList[idx];

    if (!orgMode.mipFlg) // 传统模式
    {
      // 如果传统模式数目小于 3，则将其加入临时候选模式列表中，否则，不加入候选模式列表中
      addMode = (numConv < 3);  
      numConv += addMode ? 1:0;
    }
    else
    {
      // 如果MIP模式的候选数目小于maxCandPerType 或者 MIP模式的Cost小于
      // thresholdHadCost * minCost或者保留第一个MIP模式
      // 将MIP模式加入到临时候选模式列表中
      addMode = ( numMip < maxCandPerType || (candCostList[idx] < thresholdHadCost * minCost) || keepOneMip );
      keepOneMip = false;
      numMip += addMode ? 1:0;
    }
    if( addMode )
    {
      tempRdModeList.push_back(orgMode);
      tempCandCostList.push_back(candCostList[idx]);
    }
  }

  // 如果PU大于 8x8，判断前三种MIP模式是否在临时的候选模式列表中，如果不在，则将其加入进去
  if ((pu.lwidth() > 8 && pu.lheight() > 8))
  {
    // Sort MIP candidates by Hadamard cost
    const int transpOff = getNumModesMip( pu.Y() );
    static_vector<uint8_t, FAST_UDI_MAX_RDMODE_NUM> sortedMipModes(0);
    static_vector<double, FAST_UDI_MAX_RDMODE_NUM> sortedMipCost(0);
    // 将前三种MIP模式矩阵排序
    for( uint8_t mode : { 0, 1, 2 } )
    {
      // 判断转置后的MIP模式的HAD和转置前的MIP模式的HAD，将HAD较小的MIP模式加入到sortedMipModes中
      uint8_t candMode = mode + uint8_t((mipHadCost[mode + transpOff] < mipHadCost[mode]) ? transpOff : 0);
      updateCandList(candMode, mipHadCost[candMode], sortedMipModes, sortedMipCost, 3);
    }

    // Append MIP mode to RD mode list 将MIP模式加入到RD模式列表
    const int modeListSize = int(tempRdModeList.size()); // 临时模式列表的尺寸
    for (int idx = 0; idx < 3; idx++)
    {
      const bool     isTransposed = (sortedMipModes[idx] >= transpOff ? true : false);
      // MIP模式号
      const uint32_t mipIdx       = (isTransposed ? sortedMipModes[idx] - transpOff : sortedMipModes[idx]);
      const ModeInfo mipMode( true, isTransposed, 0, NOT_INTRA_SUBPARTITIONS, mipIdx );

      bool alreadyIncluded = false; // 是否在模式列表种
      for (int modeListIdx = 0; modeListIdx < modeListSize; modeListIdx++)
      {
        if (tempRdModeList[modeListIdx] == mipMode)
        {
          alreadyIncluded = true;
          break;
        }
      }

      if (!alreadyIncluded)// 如果没有在候选列表中，则将其加入
      {
        tempRdModeList.push_back(mipMode);
        tempCandCostList.push_back(0);
        if( fastMip ) break;// 如果使用fastMIP，则仅加入一种模式
      }
    }
  }
  // 更新模式列表和代价列表
  candModeList = tempRdModeList;
  candCostList = tempCandCostList;
  numModesForFullRD = int(candModeList.size());
}

// It decides which modes from the ISP lists can be full RD tested
void IntraSearch::xGetNextISPMode(ModeInfo& modeInfo, const ModeInfo* lastMode, const Size cuSize)
{
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM>* rdModeLists[2] = { &m_ispCandListHor, &m_ispCandListVer };

  const int curIspLfnstIdx = m_curIspLfnstIdx;
  if (curIspLfnstIdx >= NUM_LFNST_NUM_PER_SET)
  {
    //All lfnst indices have been checked
    return;
  }

  ISPType nextISPcandSplitType;
  auto& ispTestedModes = m_ispTestedModes[curIspLfnstIdx];
  // 是否已经完成水平和垂直划分
  const bool horSplitIsTerminated = ispTestedModes.splitIsFinished[HOR_INTRA_SUBPARTITIONS - 1];
  const bool verSplitIsTerminated = ispTestedModes.splitIsFinished[VER_INTRA_SUBPARTITIONS - 1];
  if (!horSplitIsTerminated && !verSplitIsTerminated) 
  {
    nextISPcandSplitType = !lastMode ? HOR_INTRA_SUBPARTITIONS : lastMode->ispMod == HOR_INTRA_SUBPARTITIONS ? VER_INTRA_SUBPARTITIONS : HOR_INTRA_SUBPARTITIONS;
  }
  else if (!horSplitIsTerminated && verSplitIsTerminated)
  {
    nextISPcandSplitType = HOR_INTRA_SUBPARTITIONS;
  }
  else if (horSplitIsTerminated && !verSplitIsTerminated)
  {
    nextISPcandSplitType = VER_INTRA_SUBPARTITIONS;
  }
  else
  {
    xFinishISPModes();
    return;   // no more modes will be tested
  }
  // 最大划分数
  int maxNumSubPartitions = ispTestedModes.numTotalParts[nextISPcandSplitType - 1];

  // We try to break the split here for lfnst > 0 according to the first mode
  // 根据第一种模式，我们尝试在这里打破lfnst>0的划分
  if (curIspLfnstIdx > 0 && ispTestedModes.numTestedModes[nextISPcandSplitType - 1] == 1)
  {
    int firstModeThisSplit = ispTestedModes.getTestedIntraMode(nextISPcandSplitType, 0);
    int numSubPartsFirstModeThisSplit = ispTestedModes.getNumCompletedSubParts(nextISPcandSplitType, firstModeThisSplit);
    CHECK(numSubPartsFirstModeThisSplit < 0, "wrong number of subpartitions!");
    bool stopThisSplit = false;
    bool stopThisSplitAllLfnsts = false;
    if (numSubPartsFirstModeThisSplit < maxNumSubPartitions)
    {
      stopThisSplit = true;
      if (m_pcEncCfg->getUseFastISP() && curIspLfnstIdx == 1 && numSubPartsFirstModeThisSplit < maxNumSubPartitions - 1)
      {
        stopThisSplitAllLfnsts = true;
      }
    }

    if (stopThisSplit)
    {
      ispTestedModes.splitIsFinished[nextISPcandSplitType - 1] = true;
      if (curIspLfnstIdx == 1 && stopThisSplitAllLfnsts)
      {
        m_ispTestedModes[2].splitIsFinished[nextISPcandSplitType - 1] = true;
      }
      return;
    }
  }

  // We try to break the split here for lfnst = 0 or all lfnst indices according to the first two modes
  if (curIspLfnstIdx == 0 && ispTestedModes.numTestedModes[nextISPcandSplitType - 1] == 2)
  {
    // Split stop criteria after checking the performance of previously tested intra modes
    const int thresholdSplit1 = maxNumSubPartitions;
    bool stopThisSplit = false;
    bool stopThisSplitForAllLFNSTs = false;
    const int thresholdSplit1ForAllLFNSTs = maxNumSubPartitions - 1;

    int mode1 = ispTestedModes.getTestedIntraMode((ISPType)nextISPcandSplitType, 0);
    mode1 = mode1 == DC_IDX ? -1 : mode1;
    int numSubPartsBestMode1 = mode1 != -1 ? ispTestedModes.getNumCompletedSubParts((ISPType)nextISPcandSplitType, mode1) : -1;
    int mode2 = ispTestedModes.getTestedIntraMode((ISPType)nextISPcandSplitType, 1);
    mode2 = mode2 == DC_IDX ? -1 : mode2;
    int numSubPartsBestMode2 = mode2 != -1 ? ispTestedModes.getNumCompletedSubParts((ISPType)nextISPcandSplitType, mode2) : -1;

    // 1) The 2 most promising modes do not reach a certain number of sub-partitions
    if (numSubPartsBestMode1 != -1 && numSubPartsBestMode2 != -1)
    {
      if (numSubPartsBestMode1 < thresholdSplit1 && numSubPartsBestMode2 < thresholdSplit1)
      {
        stopThisSplit = true;
        if (curIspLfnstIdx == 0 && numSubPartsBestMode1 < thresholdSplit1ForAllLFNSTs && numSubPartsBestMode2 < thresholdSplit1ForAllLFNSTs)
        {
          stopThisSplitForAllLFNSTs = true;
        }
      }
      else
      {
        //we stop also if the cost is MAX_DOUBLE for both modes
        double mode1Cost = ispTestedModes.getRDCost(nextISPcandSplitType, mode1);
        double mode2Cost = ispTestedModes.getRDCost(nextISPcandSplitType, mode2);
        if (!(mode1Cost < MAX_DOUBLE || mode2Cost < MAX_DOUBLE))
        {
          stopThisSplit = true;
        }
      }
    }

    if (!stopThisSplit)
    {
      // 2) One split type may be discarded by comparing the number of sub-partitions of the best angle modes of both splits
      ISPType otherSplit = nextISPcandSplitType == HOR_INTRA_SUBPARTITIONS ? VER_INTRA_SUBPARTITIONS : HOR_INTRA_SUBPARTITIONS;
      int  numSubPartsBestMode2OtherSplit = mode2 != -1 ? ispTestedModes.getNumCompletedSubParts(otherSplit, mode2) : -1;
      if (numSubPartsBestMode2OtherSplit != -1 && numSubPartsBestMode2 != -1 && ispTestedModes.bestSplitSoFar != nextISPcandSplitType)
      {
        if (numSubPartsBestMode2OtherSplit > numSubPartsBestMode2)
        {
          stopThisSplit = true;
        }
        // both have the same number of subpartitions
        else if (numSubPartsBestMode2OtherSplit == numSubPartsBestMode2)
        {
          // both have the maximum number of subpartitions, so it compares RD costs to decide
          if (numSubPartsBestMode2OtherSplit == maxNumSubPartitions)
          {
            double rdCostBestMode2ThisSplit = ispTestedModes.getRDCost(nextISPcandSplitType, mode2);
            double rdCostBestMode2OtherSplit = ispTestedModes.getRDCost(otherSplit, mode2);
            double threshold = 1.3;
            if (rdCostBestMode2ThisSplit == MAX_DOUBLE || rdCostBestMode2OtherSplit < rdCostBestMode2ThisSplit * threshold)
            {
              stopThisSplit = true;
            }
          }
          else // none of them reached the maximum number of subpartitions with the best angle modes, so it compares the results with the the planar mode
          {
            int  numSubPartsBestMode1OtherSplit = mode1 != -1 ? ispTestedModes.getNumCompletedSubParts(otherSplit, mode1) : -1;
            if (numSubPartsBestMode1OtherSplit != -1 && numSubPartsBestMode1 != -1 && numSubPartsBestMode1OtherSplit > numSubPartsBestMode1)
            {
              stopThisSplit = true;
            }
          }
        }
      }
    }
    if (stopThisSplit)
    {
      ispTestedModes.splitIsFinished[nextISPcandSplitType - 1] = true;
      if (stopThisSplitForAllLFNSTs)
      {
        for (int lfnstIdx = 1; lfnstIdx < NUM_LFNST_NUM_PER_SET; lfnstIdx++)
        {
          m_ispTestedModes[lfnstIdx].splitIsFinished[nextISPcandSplitType - 1] = true;
        }
      }
      return;
    }
  }

  // Now a new mode is retrieved from the list and it has to be decided whether it should be tested or not
  if (ispTestedModes.candIndexInList[nextISPcandSplitType - 1] < rdModeLists[nextISPcandSplitType - 1]->size())
  {
    ModeInfo candidate = rdModeLists[nextISPcandSplitType - 1]->at(ispTestedModes.candIndexInList[nextISPcandSplitType - 1]);
    ispTestedModes.candIndexInList[nextISPcandSplitType - 1]++;

    // extra modes are only tested if ISP has won so far
    if (ispTestedModes.candIndexInList[nextISPcandSplitType - 1] > ispTestedModes.numOrigModesToTest)
    {
      if (ispTestedModes.bestSplitSoFar != candidate.ispMod || ispTestedModes.bestModeSoFar == PLANAR_IDX)
      {
        ispTestedModes.splitIsFinished[nextISPcandSplitType - 1] = true;
        return;
      }
    }

    bool testCandidate = true;

    // we look for a reference mode that has already been tested within the window and decide to test the new one according to the reference mode costs
    if (maxNumSubPartitions > 2 && (curIspLfnstIdx > 0 || (candidate.modeId >= DC_IDX && ispTestedModes.numTestedModes[nextISPcandSplitType - 1] >= 2)))
    {
      int       refLfnstIdx = -1;
      const int angWindowSize = 5;
      int       numSubPartsLeftMode, numSubPartsRightMode, numSubPartsRefMode, leftIntraMode = -1, rightIntraMode = -1;
      int       windowSize = candidate.modeId > DC_IDX ? angWindowSize : 1;
      int       numSamples = cuSize.width << floorLog2(cuSize.height);
      int       numSubPartsLimit = numSamples >= 256 ? maxNumSubPartitions - 1 : 2;

      xFindAlreadyTestedNearbyIntraModes(curIspLfnstIdx, (int)candidate.modeId, &refLfnstIdx, &leftIntraMode, &rightIntraMode, (ISPType)candidate.ispMod, windowSize);

      if (refLfnstIdx != -1 && refLfnstIdx != curIspLfnstIdx)
      {
        CHECK(leftIntraMode != candidate.modeId || rightIntraMode != candidate.modeId, "wrong intra mode and lfnstIdx values!");
        numSubPartsRefMode = m_ispTestedModes[refLfnstIdx].getNumCompletedSubParts((ISPType)candidate.ispMod, candidate.modeId);
        CHECK(numSubPartsRefMode <= 0, "Wrong value of the number of subpartitions completed!");
      }
      else
      {
        numSubPartsLeftMode = leftIntraMode != -1 ? ispTestedModes.getNumCompletedSubParts((ISPType)candidate.ispMod, leftIntraMode) : -1;
        numSubPartsRightMode = rightIntraMode != -1 ? ispTestedModes.getNumCompletedSubParts((ISPType)candidate.ispMod, rightIntraMode) : -1;

        numSubPartsRefMode = std::max(numSubPartsLeftMode, numSubPartsRightMode);
      }

      if (numSubPartsRefMode > 0)
      {
        // The mode was found. Now we check the condition
        testCandidate = numSubPartsRefMode > numSubPartsLimit;
      }
    }

    if (testCandidate)
    {
      modeInfo = candidate;
    }
  }
  else
  {
    //the end of the list was reached, so the split is invalidated
    ispTestedModes.splitIsFinished[nextISPcandSplitType - 1] = true;
  }
}

void IntraSearch::xFindAlreadyTestedNearbyIntraModes(int lfnstIdx, int currentIntraMode, int* refLfnstIdx, int* leftIntraMode, int* rightIntraMode, ISPType ispOption, int windowSize)
{
  bool leftModeFound = false, rightModeFound = false;
  *leftIntraMode = -1;
  *rightIntraMode = -1;
  *refLfnstIdx = -1;
  const unsigned st = ispOption - 1;

  //first we check if the exact intra mode was already tested for another lfnstIdx value
  if (lfnstIdx > 0)
  {
    bool sameIntraModeFound = false;
    if (lfnstIdx == 2 && m_ispTestedModes[1].modeHasBeenTested[currentIntraMode][st])
    {
      sameIntraModeFound = true;
      *refLfnstIdx = 1;
    }
    else if (m_ispTestedModes[0].modeHasBeenTested[currentIntraMode][st])
    {
      sameIntraModeFound = true;
      *refLfnstIdx = 0;
    }

    if (sameIntraModeFound)
    {
      *leftIntraMode = currentIntraMode;
      *rightIntraMode = currentIntraMode;
      return;
    }
  }

  //The mode has not been checked for another lfnstIdx value, so now we look for a similar mode within a window using the same lfnstIdx
  for (int k = 1; k <= windowSize; k++)
  {
    int off = currentIntraMode - 2 - k;
    int leftMode = (off < 0) ? NUM_LUMA_MODE + off : currentIntraMode - k;
    int rightMode = currentIntraMode > DC_IDX ? (((int)currentIntraMode - 2 + k) % 65) + 2 : PLANAR_IDX;

    leftModeFound  = leftMode  != (int)currentIntraMode ? m_ispTestedModes[lfnstIdx].modeHasBeenTested[leftMode][st]  : false;
    rightModeFound = rightMode != (int)currentIntraMode ? m_ispTestedModes[lfnstIdx].modeHasBeenTested[rightMode][st] : false;
    if (leftModeFound || rightModeFound)
    {
      *leftIntraMode = leftModeFound ? leftMode : -1;
      *rightIntraMode = rightModeFound ? rightMode : -1;
      *refLfnstIdx = lfnstIdx;
      break;
    }
  }
}

//It prepares the list of potential intra modes candidates that will be tested using RD costs
//它准备将使用 RD 成本进行测试的潜在帧内模式候选列表
bool IntraSearch::xSortISPCandList(double bestCostSoFar, double bestNonISPCost, ModeInfo bestNonISPMode)
{
  int bestISPModeInRelCU = -1;
  m_modeCtrl->setStopNonDCT2Transforms(false);

  // ISP快速算法
  if (m_pcEncCfg->getUseFastISP())
  {
    //we check if the ISP tests can be cancelled 检查ISP测试是否可以取消
    double thSkipISP = 1.4;
    if (bestNonISPCost > bestCostSoFar * thSkipISP)
    {
      // 如果bestNonISPCost > bestCostSoFar * thSkipISP则不再对ISP进行测试
      for (int splitIdx = 0; splitIdx < NUM_INTRA_SUBPARTITIONS_MODES - 1; splitIdx++)
      {
        for (int j = 0; j < NUM_LFNST_NUM_PER_SET; j++)
        {
          m_ispTestedModes[j].splitIsFinished[splitIdx] = true;
        }
      }
      return false;
    }
    if (!updateISPStatusFromRelCU(bestNonISPCost, bestNonISPMode, bestISPModeInRelCU))
    {
      return false; //根据相关CU的信息更新当前ISP状态
    }
  }

  for (int k = 0; k < m_ispCandListHor.size(); k++)
  {
    //设置正确的ISP划分类型
    m_ispCandListHor.at(k).ispMod = HOR_INTRA_SUBPARTITIONS; //we set the correct ISP split type value
  }

  //保存常规帧内模式的原始hadamard列表
  auto origHadList = m_ispCandListHor;   // save the original hadamard list of regular intra
  bool modeIsInList[NUM_LUMA_MODE] = { false }; // 67种模式开关

  m_ispCandListHor.clear(); // 水平的列表
  m_ispCandListVer.clear(); // 垂直的列表

  // we sort the normal intra modes according to their full RD costs
  // 根据RD Costs对常规帧内模式进行排序
  std::stable_sort(m_regIntraRDListWithCosts.begin(), m_regIntraRDListWithCosts.end(), ModeInfoWithCost::compareModeInfoWithCost);

  // we get the best angle from the regular intra list
  // 从常规帧内模式列表中得到最佳预测角度模式
  int bestNormalIntraAngle = -1;
  for (int modeIdx = 0; modeIdx < m_regIntraRDListWithCosts.size(); modeIdx++)
  {
    if (bestNormalIntraAngle == -1 && m_regIntraRDListWithCosts.at(modeIdx).modeId > DC_IDX)
    {
      bestNormalIntraAngle = m_regIntraRDListWithCosts.at(modeIdx).modeId;
      break;
    }
  }

  int mode1 = PLANAR_IDX;
  int mode2 = bestNormalIntraAngle;

  ModeInfo refMode = origHadList.at(0);
  auto* destListPtr = &m_ispCandListHor; // ISP测试的模式候选列表
  //List creation

  if (m_pcEncCfg->getUseFastISP() && bestISPModeInRelCU != -1) //RelCU intra mode
  {
    // 将相关CU的最佳ISP模式加入到候选列表中
    destListPtr->push_back(
      ModeInfo(refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, bestISPModeInRelCU));
    modeIsInList[bestISPModeInRelCU] = true;
  }

  // Planar 将Planar模式加入到候选列表中
  if (!modeIsInList[mode1])
  {
    destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, mode1));
    modeIsInList[mode1] = true;
  }
  // Best angle in regular intra  将最佳的角度模式加入到候选列表中
  if (mode2 != -1 && !modeIsInList[mode2])
  {
    destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, mode2));
    modeIsInList[mode2] = true;
  }
  // Remaining regular intra modes that were full RD tested (except DC, which is added after the angles from regular intra)
  // 其余的进行全RD测试的常规帧内模式（除了DC，它是在常规帧内角度模式之后添加的）
  int dcModeIndex = -1;
  for (int remModeIdx = 0; remModeIdx < m_regIntraRDListWithCosts.size(); remModeIdx++)
  {
    int currentMode = m_regIntraRDListWithCosts.at(remModeIdx).modeId;
    if (currentMode != mode1 && currentMode != mode2 && !modeIsInList[currentMode])
    {
      // 如果当前模式不是Planar模式且不是最佳角度模式且不在模式列表中
      if (currentMode > DC_IDX)
      {
        destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, currentMode));
        modeIsInList[currentMode] = true;
      }
      else if (currentMode == DC_IDX)
      {
        dcModeIndex = remModeIdx;
      }
    }
  }

  // DC is added after the angles from regular intra 将DC模式加入到候选模式列表中
  if (dcModeIndex != -1 && !modeIsInList[DC_IDX])
  {
    destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, DC_IDX));
    modeIsInList[DC_IDX] = true;
  }

  // We add extra candidates to the list that will only be tested if ISP is likely to win
  //将额外的候选添加到列表中，只有在 ISP 可能获胜时？？才会进行测试（测试LFNST的两种变换核）

  for (int j = 0; j < NUM_LFNST_NUM_PER_SET; j++)
  {
    m_ispTestedModes[j].numOrigModesToTest = (int)destListPtr->size();
  }
  const int addedModesFromHadList = 3;
  int       newModesAdded = 0;

  // 再从常规模式的哈达玛列表中加入额外的三种模式
  for (int k = 0; k < origHadList.size(); k++)
  {
    if (newModesAdded == addedModesFromHadList)
    {
      break;
    }
    if (!modeIsInList[origHadList.at(k).modeId])
    {
      destListPtr->push_back( ModeInfo( refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, origHadList.at(k).modeId ) );
      newModesAdded++;
    }
  }

  if (m_pcEncCfg->getUseFastISP() && bestISPModeInRelCU != -1)
  {
    destListPtr->resize(1);
  }

  // 刚才将模式添加到水平划分的候选模式列表中，将这些模式再复制到垂直划分的模式列表中
  // Copy modes to other split-type list
  m_ispCandListVer = m_ispCandListHor;
  for (int i = 0; i < m_ispCandListVer.size(); i++)
  {
    m_ispCandListVer[i].ispMod = VER_INTRA_SUBPARTITIONS;
  }

  // Reset the tested modes information to 0
  for (int j = 0; j < NUM_LFNST_NUM_PER_SET; j++)
  {
    for (int i = 0; i < m_ispCandListHor.size(); i++)
    {
      m_ispTestedModes[j].clearISPModeInfo(m_ispCandListHor[i].modeId);
    }
  }
  return true;
}

void IntraSearch::xSortISPCandListLFNST()
{
  //It resorts the list of intra mode candidates for lfnstIdx > 0 by checking the RD costs for lfnstIdx = 0
  ISPTestedModesInfo& ispTestedModesRef = m_ispTestedModes[0];
  for (int splitIdx = 0; splitIdx < NUM_INTRA_SUBPARTITIONS_MODES - 1; splitIdx++)
  {
    ISPType ispMode = splitIdx ? VER_INTRA_SUBPARTITIONS : HOR_INTRA_SUBPARTITIONS;
    if (!m_ispTestedModes[m_curIspLfnstIdx].splitIsFinished[splitIdx] && ispTestedModesRef.testedModes[splitIdx].size() > 1)
    {
      auto& candList   = ispMode == HOR_INTRA_SUBPARTITIONS ? m_ispCandListHor : m_ispCandListVer;
      int bestModeId   = candList[1].modeId > DC_IDX ? candList[1].modeId : -1;
      int bestSubParts = candList[1].modeId > DC_IDX ? ispTestedModesRef.getNumCompletedSubParts(ispMode, bestModeId) : -1;
      double bestCost  = candList[1].modeId > DC_IDX ? ispTestedModesRef.getRDCost(ispMode, bestModeId) : MAX_DOUBLE;
      for (int i = 0; i < candList.size(); i++)
      {
        const int candSubParts = ispTestedModesRef.getNumCompletedSubParts(ispMode, candList[i].modeId);
        const double candCost = ispTestedModesRef.getRDCost(ispMode, candList[i].modeId);
        if (candSubParts > bestSubParts || candCost < bestCost)
        {
          bestModeId = candList[i].modeId;
          bestCost = candCost;
          bestSubParts = candSubParts;
        }
      }

      if (bestModeId != -1)
      {
        if (bestModeId != candList[0].modeId)
        {
          auto prevMode = candList[0];
          candList[0].modeId = bestModeId;
          for (int i = 1; i < candList.size(); i++)
          {
            auto nextMode = candList[i];
            candList[i] = prevMode;
            if (nextMode.modeId == bestModeId)
            {
              break;
            }
            prevMode = nextMode;
          }
        }
      }
    }
  }
}

bool IntraSearch::updateISPStatusFromRelCU( double bestNonISPCostCurrCu, ModeInfo bestNonISPModeCurrCu, int& bestISPModeInRelCU )
{
  //It compares the data of a related CU with the current CU to cancel or reduce the ISP tests
  bestISPModeInRelCU = -1;
  if (m_modeCtrl->getRelatedCuIsValid())
  {
    double bestNonISPCostRelCU = m_modeCtrl->getBestDCT2NonISPCostRelCU();
    double costRatio           = bestNonISPCostCurrCu / bestNonISPCostRelCU;
    bool   bestModeRelCuIsMip  = (m_modeCtrl->getIspPredModeValRelCU() >> 5) & 0x1;
    bool   bestModeCurrCuIsMip = bestNonISPModeCurrCu.mipFlg;
    int    relatedCuIntraMode  = m_modeCtrl->getIspPredModeValRelCU() >> 9;
    bool   isSameTypeOfMode    = (bestModeRelCuIsMip && bestModeCurrCuIsMip) || (!bestModeRelCuIsMip && !bestModeCurrCuIsMip);
    bool   bothModesAreAngular = bestNonISPModeCurrCu.modeId > DC_IDX && relatedCuIntraMode > DC_IDX;
    bool   modesAreComparable  = isSameTypeOfMode && (bestModeCurrCuIsMip || bestNonISPModeCurrCu.modeId == relatedCuIntraMode || (bothModesAreAngular && abs(relatedCuIntraMode - (int)bestNonISPModeCurrCu.modeId) <= 5));
    int    status              = m_modeCtrl->getIspPredModeValRelCU();

    if ((status & 0x3) == 0x3) //ISP was not selected in the relCU
    {
      double bestNonDCT2Cost = m_modeCtrl->getBestNonDCT2Cost();
      double ratioWithNonDCT2 = bestNonDCT2Cost / bestNonISPCostRelCU;
      double margin = ratioWithNonDCT2 < 0.95 ? 0.2 : 0.1;

      if (costRatio > 1 - margin && costRatio < 1 + margin && modesAreComparable)
      {
        for (int lfnstVal = 0; lfnstVal < NUM_LFNST_NUM_PER_SET; lfnstVal++)
        {
          m_ispTestedModes[lfnstVal].splitIsFinished[HOR_INTRA_SUBPARTITIONS - 1] = true;
          m_ispTestedModes[lfnstVal].splitIsFinished[VER_INTRA_SUBPARTITIONS - 1] = true;
        }
        return false;
      }
    }
    else if ((status & 0x3) == 0x1) //ISP was selected in the relCU
    {
      double margin = 0.05;

      if (costRatio > 1 - margin && costRatio < 1 + margin && modesAreComparable)
      {
        int  ispSplitIdx = (m_modeCtrl->getIspPredModeValRelCU() >> 2) & 0x1;
        bool lfnstIdxIsNot0 = (bool)((m_modeCtrl->getIspPredModeValRelCU() >> 3) & 0x1);
        bool lfnstIdxIs2 = (bool)((m_modeCtrl->getIspPredModeValRelCU() >> 4) & 0x1);
        int  lfnstIdx = !lfnstIdxIsNot0 ? 0 : lfnstIdxIs2 ? 2 : 1;
        bestISPModeInRelCU = (int)m_modeCtrl->getBestISPIntraModeRelCU();

        for (int splitIdx = 0; splitIdx < NUM_INTRA_SUBPARTITIONS_MODES - 1; splitIdx++)
        {
          for (int lfnstVal = 0; lfnstVal < NUM_LFNST_NUM_PER_SET; lfnstVal++)
          {
            if (lfnstVal == lfnstIdx && splitIdx == ispSplitIdx)
            {
              continue;
            }
            m_ispTestedModes[lfnstVal].splitIsFinished[splitIdx] = true;
          }
        }

        bool stopNonDCT2Transforms = (bool)((m_modeCtrl->getIspPredModeValRelCU() >> 6) & 0x1);
        m_modeCtrl->setStopNonDCT2Transforms(stopNonDCT2Transforms);
      }
    }
    else
    {
      THROW("Wrong ISP relCU status");
    }
  }

  return true;
}

void IntraSearch::xFinishISPModes()
{
  //Continue to the next lfnst index
  m_curIspLfnstIdx++;

  if (m_curIspLfnstIdx < NUM_LFNST_NUM_PER_SET)
  {
    //Check if LFNST is applicable
    if (m_curIspLfnstIdx == 1)
    {
      bool canTestLFNST = false;
      for (int lfnstIdx = 1; lfnstIdx < NUM_LFNST_NUM_PER_SET; lfnstIdx++)
      {
        canTestLFNST |= !m_ispTestedModes[lfnstIdx].splitIsFinished[HOR_INTRA_SUBPARTITIONS - 1] || !m_ispTestedModes[lfnstIdx].splitIsFinished[VER_INTRA_SUBPARTITIONS - 1];
      }
      if (canTestLFNST)
      {
        //Construct the intra modes candidates list for the lfnst > 0 cases
        xSortISPCandListLFNST();
      }
    }
  }
}

