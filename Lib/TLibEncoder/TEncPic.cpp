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

/** \file     TEncPic.cpp
    \brief    class of picture which includes side information for encoder
*/

#include "TEncPic.h"

//! \ingroup TLibEncoder
//! \{

/** Constructor
 */
TEncQPAdaptationUnit::TEncQPAdaptationUnit()
: m_dActivity(0.0)
{
}

/** Destructor
 */
TEncQPAdaptationUnit::~TEncQPAdaptationUnit()
{
}

/** Constructor
 */
TEncQPAdaptationSaliencyUnit::TEncQPAdaptationSaliencyUnit()
        : m_dActivity(0.0)
{
}

/** Destructor
 */
TEncQPAdaptationSaliencyUnit::~TEncQPAdaptationSaliencyUnit()
{
}

/** Constructor
 */
TEncPicQPAdaptationLayer::TEncPicQPAdaptationLayer()
: m_uiAQPartWidth(0)
, m_uiAQPartHeight(0)
, m_uiNumAQPartInWidth(0)
, m_uiNumAQPartInHeight(0)
, m_acTEncAQU(NULL)
, m_dAvgActivity(0.0)
{
}

/** Destructor
 */
TEncPicQPAdaptationLayer::~TEncPicQPAdaptationLayer()
{
  destroy();
}

/** Initialize member variables
 * \param iWidth Picture width
 * \param iHeight Picture height
 * \param uiAQPartWidth Width of unit block for analyzing local image characteristics
 * \param uiAQPartHeight Height of unit block for analyzing local image characteristics
 * \return Void
 */
Void TEncPicQPAdaptationLayer::create( Int iWidth, Int iHeight, UInt uiAQPartWidth, UInt uiAQPartHeight )
{
  m_uiAQPartWidth = uiAQPartWidth;
  m_uiAQPartHeight = uiAQPartHeight;
  m_uiNumAQPartInWidth = (iWidth + m_uiAQPartWidth-1) / m_uiAQPartWidth;
  m_uiNumAQPartInHeight = (iHeight + m_uiAQPartHeight-1) / m_uiAQPartHeight;
  m_acTEncAQU = new TEncQPAdaptationUnit[ m_uiNumAQPartInWidth * m_uiNumAQPartInHeight ];
}

/** Clean up
 * \return Void
 */
Void TEncPicQPAdaptationLayer::destroy()
{
  if (m_acTEncAQU)
  {
    delete[] m_acTEncAQU;
    m_acTEncAQU = NULL;
  }
}

/** Constructor
 */
TEncPicQPAdaptationSaliencyLayer::TEncPicQPAdaptationSaliencyLayer()
        : m_uiAQPartWidth(0)
        , m_uiAQPartHeight(0)
        , m_uiNumAQPartInWidth(0)
        , m_uiNumAQPartInHeight(0)
        , m_acTEncAQSU(NULL)
        , m_dAvgActivity(0.0)
{
}

/** Destructor
 */
TEncPicQPAdaptationSaliencyLayer::~TEncPicQPAdaptationSaliencyLayer()
{
  destroy();
}

/** Initialize member variables
 * \param iWidth Picture width
 * \param iHeight Picture height
 * \param uiAQPartWidth Width of unit block for analyzing local image characteristics
 * \param uiAQPartHeight Height of unit block for analyzing local image characteristics
 * \return Void
 */
Void TEncPicQPAdaptationSaliencyLayer::create( Int iWidth, Int iHeight, UInt uiAQPartWidth, UInt uiAQPartHeight )
{
  m_uiAQPartWidth = uiAQPartWidth;
  m_uiAQPartHeight = uiAQPartHeight;
  m_uiNumAQPartInWidth = (iWidth + m_uiAQPartWidth-1) / m_uiAQPartWidth;
  m_uiNumAQPartInHeight = (iHeight + m_uiAQPartHeight-1) / m_uiAQPartHeight;
  m_acTEncAQSU = new TEncQPAdaptationSaliencyUnit[ m_uiNumAQPartInWidth * m_uiNumAQPartInHeight ];
}

/** Clean up
 * \return Void
 */
Void TEncPicQPAdaptationSaliencyLayer::destroy()
{
  if (m_acTEncAQSU)
  {
    delete[] m_acTEncAQSU;
    m_acTEncAQSU = NULL;
  }
}

/** Constructor
 */
TEncPic::TEncPic()
: m_acAQLayer(NULL)
, m_uiMaxAQDepth(0)
{
}

/** Destructor
 */
TEncPic::~TEncPic()
{
  destroy();
}

/** Initialize member variables
 * \param sps reference to used SPS
 * \param pps reference to used PPS
 * \param uiMaxAdaptiveQPDepth Maximum depth of unit block for assigning QP adaptive to local image characteristics
 * \param bIsVirtual
 */
Void TEncPic::create( const TComSPS &sps, const TComPPS &pps, UInt uiMaxAdaptiveQPDepth,
                      UInt uiPLTMaxSize, UInt uiPLTMaxPredSize,
                      Bool bIsVirtual, Bool bIsAQ, Bool bIsAQS )
{
  TComPic::create( sps, pps, uiPLTMaxSize, uiPLTMaxPredSize, bIsVirtual );
  const Int  iWidth      = sps.getPicWidthInLumaSamples();
  const Int  iHeight     = sps.getPicHeightInLumaSamples();
  const UInt uiMaxWidth  = sps.getMaxCUWidth();
  const UInt uiMaxHeight = sps.getMaxCUHeight();
  m_uiMaxAQDepth = uiMaxAdaptiveQPDepth;
  if ( uiMaxAdaptiveQPDepth > 0 )
  {
    if (bIsAQ) {
      m_acAQLayer = new TEncPicQPAdaptationLayer[ m_uiMaxAQDepth ];
      for (UInt d = 0; d < m_uiMaxAQDepth; d++)
      {
        m_acAQLayer[d].create( iWidth, iHeight, uiMaxWidth>>d, uiMaxHeight>>d );
      }
    }
    if (bIsAQS) {
      m_acAQSLayer = new TEncPicQPAdaptationSaliencyLayer[ m_uiMaxAQDepth ];
      for (UInt d = 0; d < m_uiMaxAQDepth; d++)
      {
        m_acAQSLayer[d].create( iWidth, iHeight, uiMaxWidth>>d, uiMaxHeight>>d );
      }
    }

  }
}

//! Clean up
Void TEncPic::destroy()
{
  if (m_acAQLayer)
  {
    delete[] m_acAQLayer;
    m_acAQLayer = NULL;
  }
  TComPic::destroy();
}
//! \}


Double TEncPic::getSaliencyFactor(Int x, Int y, Int width, Int height) const{
  auto meanblk = cv::mean(m_saliency_map(cv::Rect(x/4, y/4, width/4, height/4))).val[0];
  return (2*meanblk + m_saliency_mean_val) / (2*m_saliency_mean_val + meanblk);
}

