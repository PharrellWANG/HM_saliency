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

/** \file     TEncPreanalyzer.cpp
    \brief    source picture analyzer class
*/

#include <cfloat>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "TEncPreanalyzer.h"
#include "SaliencyNet.h"

using namespace std;

//! \ingroup TLibEncoder
//! \{

/** Constructor
 */
TEncPreanalyzer::TEncPreanalyzer()
{
    //m_psaliecy_net = new SaliencyNet();
}

/** Destructor
 */
TEncPreanalyzer::~TEncPreanalyzer()
{
//    delete( m_psaliecy_net );
}

/** Analyze source picture and compute local image characteristics used for QP adaptation
 * \param pcEPic Picture object to be analyzed
 * \return Void
 */

Void TEncPreanalyzer::xPreanalyze( TEncPic* pcEPic )
{
  TComPicYuv* pcPicYuv = pcEPic->getPicYuvOrg();
  const Int iWidth = pcPicYuv->getWidth(COMPONENT_Y);
  const Int iHeight = pcPicYuv->getHeight(COMPONENT_Y);
  const Int iStride = pcPicYuv->getStride(COMPONENT_Y);

  for ( UInt d = 0; d < pcEPic->getMaxAQDepth(); d++ )
  {
    const Pel* pLineY = pcPicYuv->getAddr(COMPONENT_Y);
    TEncPicQPAdaptationLayer* pcAQLayer = pcEPic->getAQLayer(d);
    const UInt uiAQPartWidth = pcAQLayer->getAQPartWidth();
    const UInt uiAQPartHeight = pcAQLayer->getAQPartHeight();
    TEncQPAdaptationUnit* pcAQU = pcAQLayer->getQPAdaptationUnit();

    Double dSumAct = 0.0;
    for ( UInt y = 0; y < iHeight; y += uiAQPartHeight )
    {
      const UInt uiCurrAQPartHeight = min(uiAQPartHeight, iHeight-y);
      for ( UInt x = 0; x < iWidth; x += uiAQPartWidth, pcAQU++ )
      {
        const UInt uiCurrAQPartWidth = min(uiAQPartWidth, iWidth-x);
        const Pel* pBlkY = &pLineY[x];
        UInt64 uiSum[4] = {0, 0, 0, 0};
        UInt64 uiSumSq[4] = {0, 0, 0, 0};
        UInt by = 0;
        for ( ; by < uiCurrAQPartHeight>>1; by++ )
        {
          UInt bx = 0;
          for ( ; bx < uiCurrAQPartWidth>>1; bx++ )
          {
            uiSum  [0] += pBlkY[bx];
            uiSumSq[0] += pBlkY[bx] * pBlkY[bx];
          }
          for ( ; bx < uiCurrAQPartWidth; bx++ )
          {
            uiSum  [1] += pBlkY[bx];
            uiSumSq[1] += pBlkY[bx] * pBlkY[bx];
          }
          pBlkY += iStride;
        }
        for ( ; by < uiCurrAQPartHeight; by++ )
        {
          UInt bx = 0;
          for ( ; bx < uiCurrAQPartWidth>>1; bx++ )
          {
            uiSum  [2] += pBlkY[bx];
            uiSumSq[2] += pBlkY[bx] * pBlkY[bx];
          }
          for ( ; bx < uiCurrAQPartWidth; bx++ )
          {
            uiSum  [3] += pBlkY[bx];
            uiSumSq[3] += pBlkY[bx] * pBlkY[bx];
          }
          pBlkY += iStride;
        }

        assert ((uiCurrAQPartWidth&1)==0);
        assert ((uiCurrAQPartHeight&1)==0);
        const UInt pixelWidthOfQuadrants  = uiCurrAQPartWidth >>1;
        const UInt pixelHeightOfQuadrants = uiCurrAQPartHeight>>1;
        const UInt numPixInAQPart         = pixelWidthOfQuadrants * pixelHeightOfQuadrants;

        Double dMinVar = DBL_MAX;
        if (numPixInAQPart!=0)
        {
          for ( Int i=0; i<4; i++)
          {
            const Double dAverage = Double(uiSum[i]) / numPixInAQPart;
            const Double dVariance = Double(uiSumSq[i]) / numPixInAQPart - dAverage * dAverage;
            dMinVar = min(dMinVar, dVariance);
          }
        }
        else
        {
          dMinVar = 0.0;
        }
        const Double dActivity = 1.0 + dMinVar;
        pcAQU->setActivity( dActivity );
        dSumAct += dActivity;
      }
      pLineY += iStride * uiCurrAQPartHeight;
    }

    const Double dAvgAct = dSumAct / (pcAQLayer->getNumAQPartInWidth() * pcAQLayer->getNumAQPartInHeight());
    pcAQLayer->setAvgActivity( dAvgAct );
  }
}
//! \}

void TEncPreanalyzer::xComputeSaliency(TEncPic *pcEPic)
{
    TComPicYuv* pcPicYuv = pcEPic->getPicYuvOrg();
    //pcEPic.get
    const Int iWidth = pcPicYuv->getWidth(COMPONENT_Y);
    const Int iHeight = pcPicYuv->getHeight(COMPONENT_Y);
    const Int iStride = pcPicYuv->getStride(COMPONENT_Y);

    std::vector<cv::Mat> srcs;
    srcs.push_back(cv::Mat(iHeight, iWidth, CV_16UC1, pcPicYuv->getAddr(COMPONENT_Y), iStride*2));
    srcs.push_back(cv::Mat(pcPicYuv->getHeight(COMPONENT_Cr),
                           pcPicYuv->getWidth(COMPONENT_Cr),
                           CV_16UC1,
                           pcPicYuv->getAddr(COMPONENT_Cr),
                           pcPicYuv->getStride(COMPONENT_Cr) * 2
    ));
    srcs.push_back(cv::Mat(pcPicYuv->getHeight(COMPONENT_Cb),
                           pcPicYuv->getWidth(COMPONENT_Cb),
                           CV_16UC1,
                           pcPicYuv->getAddr(COMPONENT_Cb),
                           pcPicYuv->getStride(COMPONENT_Cb) * 2
    ));
    std::for_each(srcs.begin(), srcs.end(), [](cv::Mat& src){ src.convertTo(src, CV_8UC1); } );
    cv::resize(srcs[1], srcs[1], srcs[0].size());
    cv::resize(srcs[2], srcs[2], srcs[0].size());
    cv::Mat pic_orig, saliency_map;
    cv::merge(srcs, pic_orig);
    cv::cvtColor(pic_orig, pic_orig, CV_YCrCb2BGR);
    m_psaliecy_net->get_saliency_map(pic_orig, saliency_map);
    cv::resize(saliency_map, saliency_map, cv::Size(iWidth, iHeight));


    for ( UInt d = 0; d < pcEPic->getMaxAQDepth(); d++ ) {
        //const Pel *pLineY = pcPicYuv->getAddr(COMPONENT_Y);
        TEncPicQPAdaptationSaliencyLayer *pcAQSLayer = pcEPic->getAQSLayer(d);
        const UInt uiAQPartWidth = pcAQSLayer->getAQPartWidth();
        const UInt uiAQPartHeight = pcAQSLayer->getAQPartHeight();
        TEncQPAdaptationSaliencyUnit *pcAQU = pcAQSLayer->getQPAdaptationSaliencyUnit();

        Double dSumAct = 0.0;
        for (UInt y = 0; y < iHeight; y += uiAQPartHeight) {

            const UInt uiCurrAQPartHeight = min(uiAQPartHeight, iHeight - y);
            for (UInt x = 0; x < iWidth; x += uiAQPartWidth, pcAQU++) {
                const UInt uiCurrAQPartWidth = min(uiAQPartWidth, iWidth - x);
                //const Pel *pBlkY = &pLineY[x];

                UInt64 uiSaliencySum = 0;
                UInt64 uiSum[4] = {0, 0, 0, 0};
                UInt64 uiSumSq[4] = {0, 0, 0, 0};
                UInt by = 0;

                for (; by < uiCurrAQPartHeight; by++) {
                    const char* pLineY = saliency_map.ptr<char>(y + by);
                    const char *pBlkY = &pLineY[x];
                    UInt bx = 0;
                    for (; bx < uiCurrAQPartWidth; bx++) {
                        uiSaliencySum += pBlkY[bx];
                    }
                }

                assert ((uiCurrAQPartWidth & 1) == 0);
                assert ((uiCurrAQPartHeight & 1) == 0);

                const UInt numPixInAQPart = uiAQPartWidth * uiAQPartHeight;


                const Double dActivity = uiSaliencySum / numPixInAQPart;
                pcAQU->setActivity( dActivity);
                dSumAct += dActivity;
            }
        }

        const Double dAvgAct = dSumAct / (pcAQSLayer->getNumAQPartInWidth() * pcAQSLayer->getNumAQPartInHeight());
        pcAQSLayer->setAvgActivity(dAvgAct);
    }
}


void TEncPreanalyzer::xComputeSaliency(TEncPic *pcEPic, const std::string video_filename) {
    static const std::string sal_map_path = "/data/saliency_maps/";
    std::string sal_map_filename = sal_map_path + video_filename + "/" + std::to_string(pcEPic->getPOC()) + ".png";
    cv::Mat sal_map;
    sal_map = cv::imread(sal_map_filename, cv::IMREAD_GRAYSCALE);
    if (!sal_map.data) {
        std::cerr << "Cannot load saliency data!" << std::endl;
        assert(0);
        return;
    }
    cv::resize(sal_map, *pcEPic->getSaliencyMap(), sal_map.size() / 4);
    pcEPic->updateSaliencyMeanVal();

    TComPicYuv* pcPicYuv = pcEPic->getPicYuvOrg();
    const Int iWidth = pcPicYuv->getWidth(COMPONENT_Y);
    const Int iHeight = pcPicYuv->getHeight(COMPONENT_Y);
    cv::resize(sal_map, sal_map, cv::Size(iWidth, iHeight));
    for ( UInt d = 0; d < pcEPic->getMaxAQDepth(); d++ ) {
        TEncPicQPAdaptationSaliencyLayer *pcAQSLayer = pcEPic->getAQSLayer(d);
        const UInt uiAQPartWidth = pcAQSLayer->getAQPartWidth();
        const UInt uiAQPartHeight = pcAQSLayer->getAQPartHeight();
        TEncQPAdaptationSaliencyUnit *pcAQU = pcAQSLayer->getQPAdaptationSaliencyUnit();

        Double dSumAct = 0.0;
        for (UInt y = 0; y < iHeight; y += uiAQPartHeight) {
            const UInt uiCurrAQPartHeight = min(uiAQPartHeight, iHeight - y);
            for (UInt x = 0; x < iWidth; x += uiAQPartWidth, pcAQU++) {
                const UInt uiCurrAQPartWidth = min(uiAQPartWidth, iWidth - x);
                assert ((uiCurrAQPartWidth & 1) == 0);
                assert ((uiCurrAQPartHeight & 1) == 0);
                const Double dActivity = cv::mean(sal_map(cv::Rect(x, y, uiCurrAQPartWidth, uiCurrAQPartHeight))).val[0];
                pcAQU->setActivity( dActivity);
                dSumAct += dActivity;
            }
        }

        const Double dAvgAct = dSumAct / (pcAQSLayer->getNumAQPartInWidth() * pcAQSLayer->getNumAQPartInHeight());
        pcAQSLayer->setAvgActivity(dAvgAct);
    }

//    cv::imshow("show", sal_map);
//    cv::waitKey(0);
}

