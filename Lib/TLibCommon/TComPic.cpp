/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2015, ITU/ISO/IEC
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

/** \file     TComPic.cpp
    \brief    picture class
*/

#include "TComPic.h"
#include "SEI.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TComPic::TComPic()
: m_uiTLayer                              (0)
, m_bUsedByCurr                           (false)
, m_bIsLongTerm                           (false)
, m_pcPicYuvPred                          (NULL)
, m_pcPicYuvResi                          (NULL)
, m_bReconstructed                        (false)
, m_bNeededForOutput                      (false)
, m_uiCurrSliceIdx                        (0)
, m_bCheckLTMSB                           (false)
#if SCM_U0181_STORAGE_BOTH_VERSIONS_CURR_DEC_PIC
, m_bCurPic                               (false)
, m_bInDPB                                (false)
#endif

{
  for(UInt i=0; i<NUM_PIC_YUV; i++)
  {
    m_apcPicYuv[i]      = NULL;
  }
  m_apcPicYuvCSC = NULL;
}

TComPic::~TComPic()
{
  destroy();
}

#if SCM_U0181_STORAGE_BOTH_VERSIONS_CURR_DEC_PIC
Void TComPic::copyPicInfo( const TComPic& sComPic )
{
  UInt i = 0;

  m_uiTLayer = sComPic.m_uiTLayer;

  m_bNeededForOutput = sComPic.m_bNeededForOutput;
  m_bReconstructed = sComPic.m_bReconstructed;

  m_uiCurrSliceIdx = sComPic.m_uiCurrSliceIdx;
  m_bCheckLTMSB = sComPic.m_bCheckLTMSB;

  m_isTop = sComPic.m_isTop;
  m_isField = sComPic.m_isField;

  for ( i = 0; i < NUM_PIC_YUV; i++ )
  {
    if ( sComPic.m_apcPicYuv[i] != NULL )
    {
      *m_apcPicYuv[i] = *(sComPic.m_apcPicYuv[i]);
    }
  }
}
#endif

Void TComPic::create( const TComSPS &sps, const TComPPS &pps,
                      UInt uiPLTMaxSize, UInt uiPLTMaxPredSize, const Bool bIsVirtual )
{
  destroy();

  const ChromaFormat chromaFormatIDC = sps.getChromaFormatIdc();
  const Int          iWidth          = sps.getPicWidthInLumaSamples();
  const Int          iHeight         = sps.getPicHeightInLumaSamples();
  const UInt         uiMaxCuWidth    = sps.getMaxCUWidth();
  const UInt         uiMaxCuHeight   = sps.getMaxCUHeight();
  const UInt         uiMaxDepth      = sps.getMaxTotalCUDepth();

  m_picSym.create( sps, pps, uiMaxDepth, uiPLTMaxSize, uiPLTMaxPredSize );
  if (!bIsVirtual)
  {
    m_apcPicYuv[PIC_YUV_ORG    ]   = new TComPicYuv;  m_apcPicYuv[PIC_YUV_ORG     ]->create( iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true );
    m_apcPicYuv[PIC_YUV_TRUE_ORG]  = new TComPicYuv;  m_apcPicYuv[PIC_YUV_TRUE_ORG]->create( iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true );
  }
  m_apcPicYuv[PIC_YUV_REC]  = new TComPicYuv;  m_apcPicYuv[PIC_YUV_REC]->create( iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true );

  // there are no SEI messages associated with this picture initially
  if (m_SEIs.size() > 0)
  {
    deleteSEIs (m_SEIs);
  }
  m_bUsedByCurr = false;
  m_hashMap.clearAll();
}

Void TComPic::destroy()
{
  m_picSym.destroy();

  for(UInt i=0; i<NUM_PIC_YUV; i++)
  {
    if (m_apcPicYuv[i])
    {
      m_apcPicYuv[i]->destroy();
      delete m_apcPicYuv[i];
      m_apcPicYuv[i]  = NULL;
    }
  }

  m_hashMap.clearAll();
  if (m_apcPicYuvCSC)
  {
    m_apcPicYuvCSC->destroy();
    delete m_apcPicYuvCSC;
    m_apcPicYuvCSC = NULL;
  }

  deleteSEIs(m_SEIs);
}

Void TComPic::compressMotion()
{
  TComPicSym* pPicSym = getPicSym();
  for ( UInt uiCUAddr = 0; uiCUAddr < pPicSym->getNumberOfCtusInFrame(); uiCUAddr++ )
  {
    TComDataCU* pCtu = pPicSym->getCtu(uiCUAddr);
    pCtu->compressMV();
  }
}

Bool  TComPic::getSAOMergeAvailability(Int currAddr, Int mergeAddr)
{
  Bool mergeCtbInSliceSeg = (mergeAddr >= getPicSym()->getCtuTsToRsAddrMap(getCtu(currAddr)->getSlice()->getSliceCurStartCtuTsAddr()));
  Bool mergeCtbInTile     = (getPicSym()->getTileIdxMap(mergeAddr) == getPicSym()->getTileIdxMap(currAddr));
  return (mergeCtbInSliceSeg && mergeCtbInTile);
}

UInt TComPic::getSubstreamForCtuAddr(const UInt ctuAddr, const Bool bAddressInRaster, TComSlice *pcSlice)
{
  UInt subStrm;
  const bool bWPPEnabled=pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag();
  const TComPicSym &picSym            = *(getPicSym());

  if ((bWPPEnabled && picSym.getFrameHeightInCtus()>1) || (picSym.getNumTiles()>1)) // wavefronts, and possibly tiles being used.
  {
    if (bWPPEnabled)
    {
      const UInt ctuRsAddr                = bAddressInRaster?ctuAddr : picSym.getCtuTsToRsAddrMap(ctuAddr);
      const UInt frameWidthInCtus         = picSym.getFrameWidthInCtus();
      const UInt tileIndex                = picSym.getTileIdxMap(ctuRsAddr);
      const UInt numTileColumns           = (picSym.getNumTileColumnsMinus1()+1);
      const TComTile *pTile               = picSym.getTComTile(tileIndex);
      const UInt firstCtuRsAddrOfTile     = pTile->getFirstCtuRsAddr();
      const UInt tileYInCtus              = firstCtuRsAddrOfTile / frameWidthInCtus;
      // independent tiles => substreams are "per tile"
      const UInt ctuLine                  = ctuRsAddr / frameWidthInCtus;
      const UInt startingSubstreamForTile =(tileYInCtus*numTileColumns) + (pTile->getTileHeightInCtus()*(tileIndex%numTileColumns));
      subStrm = startingSubstreamForTile + (ctuLine - tileYInCtus);
    }
    else
    {
      const UInt ctuRsAddr                = bAddressInRaster?ctuAddr : picSym.getCtuTsToRsAddrMap(ctuAddr);
      const UInt tileIndex                = picSym.getTileIdxMap(ctuRsAddr);
      subStrm=tileIndex;
    }
  }
  else
  {
    // dependent tiles => substreams are "per frame".
    subStrm = 0;
  }
  return subStrm;
}

Void TComPic::addPictureToHashMapForInter()
{
  Int picWidth = getSlice( 0 )->getSPS()->getPicWidthInLumaSamples();
  Int picHeight = getSlice( 0 )->getSPS()->getPicHeightInLumaSamples();
#if SCM_W0078_HASH_BOTTOM_UP // add hash values to hash table
  UInt* uiBlockHashValues[2][2];
  Bool* bIsBlockSame[2][3];

  for (Int i = 0; i < 2; i++)
  {
    for (Int j = 0; j < 2; j++)
    {
      uiBlockHashValues[i][j] = new UInt[picWidth*picHeight];
    }

    for (Int j = 0; j < 3; j++)
    {
      bIsBlockSame[i][j] = new Bool[picWidth*picHeight];
    }
  }

  m_hashMap.create();
  m_hashMap.generateBlock2x2HashValue(getPicYuvOrg(), picWidth, picHeight, getSlice(0)->getSPS()->getBitDepths(), uiBlockHashValues[0], bIsBlockSame[0]);//2x2
  m_hashMap.generateBlockHashValue(getPicYuvOrg(), picWidth, picHeight, 4, 4, getSlice(0)->getSPS()->getBitDepths(), uiBlockHashValues[0], uiBlockHashValues[1], bIsBlockSame[0], bIsBlockSame[1]);//4x4

  m_hashMap.generateBlockHashValue(getPicYuvOrg(), picWidth, picHeight, 8, 8, getSlice(0)->getSPS()->getBitDepths(), uiBlockHashValues[1], uiBlockHashValues[0], bIsBlockSame[1], bIsBlockSame[0]);//8x8
  m_hashMap.addToHashMapByRowWithPrecalData(uiBlockHashValues[0], bIsBlockSame[0][2], picWidth, picHeight, 8, 8, getSlice(0)->getSPS()->getBitDepths());

  m_hashMap.generateBlockHashValue(getPicYuvOrg(), picWidth, picHeight, 16, 16, getSlice(0)->getSPS()->getBitDepths(), uiBlockHashValues[0], uiBlockHashValues[1], bIsBlockSame[0], bIsBlockSame[1]);//16x16
  m_hashMap.addToHashMapByRowWithPrecalData(uiBlockHashValues[1], bIsBlockSame[1][2], picWidth, picHeight, 16, 16, getSlice(0)->getSPS()->getBitDepths());

  m_hashMap.generateBlockHashValue(getPicYuvOrg(), picWidth, picHeight, 32, 32, getSlice(0)->getSPS()->getBitDepths(), uiBlockHashValues[1], uiBlockHashValues[0], bIsBlockSame[1], bIsBlockSame[0]);//32x32
  m_hashMap.addToHashMapByRowWithPrecalData(uiBlockHashValues[0], bIsBlockSame[0][2], picWidth, picHeight, 32, 32, getSlice(0)->getSPS()->getBitDepths());

  m_hashMap.generateBlockHashValue(getPicYuvOrg(), picWidth, picHeight, 64, 64, getSlice(0)->getSPS()->getBitDepths(), uiBlockHashValues[0], uiBlockHashValues[1], bIsBlockSame[0], bIsBlockSame[1]);//64x64
  m_hashMap.addToHashMapByRowWithPrecalData(uiBlockHashValues[1], bIsBlockSame[1][2], picWidth, picHeight, 64, 64, getSlice(0)->getSPS()->getBitDepths());

  for (Int i = 0; i < 2; i++)
  {
    for (Int j = 0; j < 2; j++)
    {
      delete[] uiBlockHashValues[i][j];
    }

    for (Int j = 0; j<3; j++)
    {
      delete[] bIsBlockSame[i][j];
    }
  }
#else
  m_hashMap.create();
  m_hashMap.addToHashMapByRow( getPicYuvOrg(), picWidth, picHeight, 8, 8, getSlice( 0 )->getSPS()->getBitDepths() );
  m_hashMap.addToHashMapByRow( getPicYuvOrg(), picWidth, picHeight, 16, 16, getSlice( 0 )->getSPS()->getBitDepths() );
  m_hashMap.addToHashMapByRow( getPicYuvOrg(), picWidth, picHeight, 32, 32, getSlice( 0 )->getSPS()->getBitDepths() );
  m_hashMap.addToHashMapByRow( getPicYuvOrg(), picWidth, picHeight, 64, 64, getSlice( 0 )->getSPS()->getBitDepths() );
#endif
}

//! \}
