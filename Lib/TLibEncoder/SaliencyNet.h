//
// Created by anserw on 16-8-26.
//

#ifndef CAFFE_BASED_DEMO_SALIENCY_NET_H
#define CAFFE_BASED_DEMO_SALIENCY_NET_H

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "google/protobuf/text_format.h"

#include "caffe/blob.hpp"
#include "caffe/common.hpp"
#include "caffe/net.hpp"
#include "caffe/layer.hpp"
#include "caffe/proto/caffe.pb.h"
#include "caffe/util/db.hpp"
#include "caffe/util/format.hpp"
#include "caffe/util/io.hpp"


class SaliencyNet {

public:
    SaliencyNet(
            std::string caffe_model_name="/home/anserw/git/saliency-2016-cvpr/deep_net_model.caffemodel",
            std::string net_proto_name="/home/anserw/git/saliency-2016-cvpr/deep/deep_net_deploy.prototxt",
            std::string input_blob_name="data1",
            std::string output_blob_name="deconv1"
    );
    virtual ~SaliencyNet();

    void get_saliency_map(const cv::Mat& src, cv::Mat& saliecy_map);
    void get_saliency_map(caffe::Blob<float> &input_blob);

    void get_saliency_map_to_opencv(const std::string input_filename, cv::Mat & saliency_map);
    void get_saliency_map_to_file(const std::string input_filename, std::string saliency_map_filename);

private:
    void load_to_blob(const std::string input_filename, caffe::Blob<float> &input_blob);
    void convert_cv_mat_to_blob(const cv::Mat& src, caffe::Blob<float> &input_blob);
    void get_feature_blob_to_opencv(cv::Mat & saliency_map);

private:
    std::string m_caffe_model;
    std::string m_feature_extraction_proto;
    std::string m_input_blob_name;
    std::string m_output_blob_name;
    boost::shared_ptr<caffe::Net<float> > m_saliency_net;
};


#endif //CAFFE_BASED_DEMO_SALIENCY_NET_H
