//
// Created by anserw on 16-8-26.
//

#include "SaliencyNet.h"
#include <glog/logging.h>

SaliencyNet::SaliencyNet(std::string caffe_model_name,
                                std::string net_proto_name,
                                std::string input_blob_name,
                                std::string output_blob_name):
m_caffe_model(caffe_model_name),
m_feature_extraction_proto(net_proto_name),
m_input_blob_name(input_blob_name),
m_output_blob_name(output_blob_name)
{

    caffe::Caffe::set_mode(caffe::Caffe::GPU);
    LOG(ERROR)<<"Net proto: " << m_feature_extraction_proto;
    m_saliency_net = boost::shared_ptr<caffe::Net<float>>(new caffe::Net<float>(m_feature_extraction_proto, caffe::TEST));
    LOG(ERROR)<<"Load pretrained caffe model: " << m_caffe_model;
    m_saliency_net->CopyTrainedLayersFrom(m_caffe_model);
    CHECK(m_saliency_net->has_blob(m_input_blob_name))<< "Unknown input blob name " << m_input_blob_name;
    CHECK(m_saliency_net->has_blob(m_output_blob_name))<< "Unknown feature blob name " << m_output_blob_name;

}

SaliencyNet::~SaliencyNet()
{

}


void SaliencyNet::get_saliency_map_to_opencv(const std::string input_filename, cv::Mat &saliency_map)
{
    caffe::Blob<float> input_blob;
    LOG(ERROR) << "Load input image file " << input_filename << " to blob";
    load_to_blob(input_filename, input_blob);

    get_saliency_map(input_blob);

    get_feature_blob_to_opencv(saliency_map);
    LOG(ERROR) << "Successfully extracted the saliency map";
}

void SaliencyNet::get_saliency_map(caffe::Blob<float> &input_blob)
{
    std::vector<caffe::Blob<float>* > input_blob_vec;
    input_blob_vec.push_back(&input_blob);
    boost::shared_ptr<caffe::Blob<float> > bottom_blob = m_saliency_net->blob_by_name(m_input_blob_name);
    bottom_blob->ReshapeLike(input_blob);

    LOG(ERROR) << "Start extracting saliency map";
    m_saliency_net->Forward(input_blob_vec);
}

void SaliencyNet::get_saliency_map_to_file(const std::string input_filename,
                                                  std::string saliency_map_filename)
{
    cv::Mat saliency_map;
    get_saliency_map_to_opencv(input_filename, saliency_map);
    cv::imwrite(saliency_map_filename, saliency_map);
    LOG(ERROR) << "Write the saliency map to file " << saliency_map_filename;
}


void SaliencyNet::load_to_blob(const std::string input_filename, caffe::Blob<float> &input_blob)
{
    cv::Mat src;
    src = cv::imread(input_filename);
    convert_cv_mat_to_blob(src, input_blob);
}


void SaliencyNet::get_feature_blob_to_opencv(cv::Mat &saliency_map)
{
    const boost::shared_ptr<caffe::Blob<float> > feature_blob = m_saliency_net->blob_by_name(m_output_blob_name);
    auto new_shape = feature_blob->shape();
    int cols = new_shape[3];
    int rows = new_shape[2];
    const float* feature_blob_data;
    feature_blob_data = feature_blob->cpu_data();

    float* ptr_cv_mat = new float[cols*rows];
    memcpy(ptr_cv_mat, feature_blob_data, cols*rows*sizeof(float));

    cv::Mat img(rows, cols, CV_32FC1, ptr_cv_mat);
    saliency_map = img + cv::Scalar((float)1.0);
    saliency_map.convertTo(saliency_map, CV_8UC1, 128);
    delete[] ptr_cv_mat;
}

void SaliencyNet::convert_cv_mat_to_blob(const cv::Mat &src, caffe::Blob<float> &input_blob)
{
    cv::Mat temp_mat;
    float scale = (float) std::min(320.0 / src.cols, 320.0 / src.rows);
    cv::resize(src, temp_mat, cv::Size(), scale, scale);
    temp_mat = temp_mat - cv::Scalar(100, 110, 118);
    temp_mat.convertTo(temp_mat, CV_32FC3, 1.0/128);
    std::vector<cv::Mat> channels;
    cv::split(temp_mat, channels);
    input_blob.Reshape(1, 3, temp_mat.rows, temp_mat.cols);
    float* data_ptr = input_blob.mutable_cpu_data();
    for (int i=0; i<3; i++) {
        memcpy(data_ptr + i*temp_mat.cols*temp_mat.rows, channels[i].data, temp_mat.cols*temp_mat.rows*sizeof(float));
    }
}

void SaliencyNet::get_saliency_map(const cv::Mat &src, cv::Mat &saliecy_map)
{
    caffe::Blob<float> input_blob;
    convert_cv_mat_to_blob(src, input_blob);
    get_saliency_map(input_blob);
    get_feature_blob_to_opencv(saliecy_map);
}