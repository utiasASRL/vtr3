#include <algorithm>
#include <memory>
#include <numeric>
#include <vector>
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/cudastereo.hpp>

#include <vtr_vision/features/extractor/learned_feature_extractor.hpp>

namespace vtr {
namespace vision {

using LFE = LearnedFeatureExtractor;

std::mutex LFE::gpu_mutex_;

torch::Tensor normalizeKeypointCoordinates(torch::Tensor keypoints, 
                                                     int width, 
                                                     int height) {
  torch::Tensor u_norm = 
      (2 * keypoints.index({0, 0, "..."}).reshape({1, -1}) / (width - 1)) - 1;
  torch::Tensor v_norm = 
      (2 * keypoints.index({0, 1, "..."}).reshape({1, -1}) / (height - 1)) - 1;

  // grid_sample expects the normalized coordinates as (u, v), 1 x 1 x N x 2.
  return torch::stack({u_norm, v_norm}, 2).unsqueeze_(1);
}

torch::Tensor getKeypointDisparities(torch::Tensor disparity, 
                                     torch::Tensor keypoints) {
  int width = disparity.size(3);
  int height = disparity.size(2);

  torch::Tensor keypoints_norm = normalizeKeypointCoordinates(keypoints, 
                                                              width, height); 

  namespace F = torch::nn::functional;
  auto options = F::GridSampleFuncOptions().mode(
                     torch::kNearest).align_corners(false);

  return F::grid_sample(disparity, keypoints_norm, options).reshape({-1});

}

torch::Tensor getKeypointScores(torch::Tensor scores, torch::Tensor keypoints) {
  int width = scores.size(3);
  int height = scores.size(2);

  torch::Tensor keypoints_norm = normalizeKeypointCoordinates(keypoints, 
                                                              width, height); 

  namespace F = torch::nn::functional;
  auto options = F::GridSampleFuncOptions().mode(
                     torch::kBilinear).align_corners(false);

  return F::grid_sample(scores, keypoints_norm, options).reshape({-1});

}

torch::Tensor getKeypointDescriptors(torch::Tensor descriptors, 
                                     torch::Tensor keypoints) {
  int width = descriptors.size(3);
  int height = descriptors.size(2);
  int channels = descriptors.size(1);

  torch::Tensor keypoints_norm = normalizeKeypointCoordinates(keypoints, 
                                                              width, height); 

  namespace F = torch::nn::functional;
  auto options = F::GridSampleFuncOptions().mode(
                     torch::kBilinear).align_corners(false);
  
  torch::Tensor point_descriptors = F::grid_sample(descriptors, keypoints_norm, 
                                                   options);

  point_descriptors = point_descriptors.reshape({channels, -1});

  return point_descriptors.permute({(1), (0)}); // N x C

}

////////////////////////////////////////////////////////////////////////////////
void LFE::initialize(LearnedFeatureConfiguration &config) {
  config_ = config;

  // Load the pytorch model.
  try {
    // Deserialize the ScriptModule from a file using torch::jit::load().
    LOG(INFO) << config_.model_path;
    detector_ = torch::jit::load(config_.model_path);
    std::unique_lock<std::mutex> lock(gpu_mutex_);
    detector_.to(at::kCUDA);
    lock.unlock();
  }
  catch (const c10::Error& err) {
    LOG(ERROR) << "Error loading the Pytorch learned feature model.";
    LOG(ERROR) << "Model path: " << config_.model_path;
    throw std::runtime_error{"Error loading Pytorch learned feature model."};
  }
}

////////////////////////////////////////////////////////////////////////////////
void LFE::initialize(LearnedFeatureStereoConfiguration &stereo_config) {
  stereo_config_ = stereo_config; 
}

////////////////////////////////////////////////////////////////////////////////
torch::Tensor LFE::getDisparity(const cv::Mat& left, const cv::Mat& right, 
                               const LearnedFeatureStereoConfiguration config) {

  int minDisparity = 0;
  int numDisparities = 128;
  int P1 = 10;
  int P2 = 120;
  int uniquenessRatio = 5;
  int mode = cv::cuda::StereoSGM::MODE_HH4;

  std::unique_lock<std::mutex> lock(gpu_mutex_);    

  // Upload images to gpu.
  cv::cuda::GpuMat left_gpu;
  cv::cuda::GpuMat right_gpu;
  left_gpu.upload(left);
  right_gpu.upload(right);
  
  cv::Ptr<cv::cuda::StereoSGM> sgm;
  sgm = cv::cuda::createStereoSGM(minDisparity, 
                                  numDisparities, 
                                  P1,
                                  P2,
                                  uniquenessRatio, 
                                  mode);

  cv::cuda::GpuMat disp_gpu;
  sgm->compute(left_gpu, right_gpu, disp_gpu);

  cv::Mat disp;
  disp_gpu.download(disp);

  lock.unlock();

  // int minDisparity = 0;
  // int numDisparities = 48;
  // int blockSize = 5;
  // int preFilterCap = 30;
  // int uniquenessRatio = 20;
  // int P1 = 200;
  // int P2 = 800;
  // int speckleWindowSize = 200;
  // int speckleRange = 1;
  // int disp12MaxDiff = -1;
  // bool fullDP = false;
   
  // cv::Ptr<cv::StereoSGBM> sbm;
  // sbm = cv::StereoSGBM::create(minDisparity, 
  //                              numDisparities, 
  //                              blockSize,
  //                              P1,
  //                              P2,
  //                              disp12MaxDiff, 
  //                              preFilterCap, 
  //                              uniquenessRatio, 
  //                              speckleWindowSize, 
  //                              speckleRange, 
  //                              fullDP ? cv::StereoSGBM::MODE_HH : 
  //                                       cv::StereoSGBM::MODE_SGBM);

  // // cv::cuda::GpuMat disp_gpu;
  // cv::Mat disp;
  // sbm->compute(left.data, right.data, disp);
 
  // // Check for left-right consistency.
  // cv::Mat left_flip;
  // cv::Mat right_flip;
  // cv::flip(left, left_flip, 1);
  // cv::flip(right, right_flip, 1);

  // cv::Mat disp_lr_flip;
  // sgbm->compute(right_flip, left_flip, disp_lr_flip);

  // cv::Mat disp_lr;
  // cv::flip(disp_lr_flip, disp_lr, 1);

  // cv::Mat disp_diff;
  // cv::absdiff(disp, disp_lr, disp_diff);
  // cv::Mat disp_consistent;
  // cv::Mat disp_consistent_mask = disp_diff <= 0.01;
  // disp.copyTo(disp_consistent, disp_consistent_mask);
 
  float disparity_multiplier = 1.0f;
  if (disp.type() == CV_16S) {
    disparity_multiplier = 16.0f;
  }
  cv::Mat floatDisp;
  disp.convertTo(floatDisp, CV_32F, 1.0f / disparity_multiplier);

  // //Crop the image
  cv::Mat disp_cropped;
  floatDisp(cv::Rect(48, 0, 464, 384)).copyTo(disp_cropped);

  // Convert the cv image to a tensor
  torch::Tensor disp_tensor = torch::from_blob(disp_cropped.data, 
                                              {disp_cropped.rows, 
                                               disp_cropped.cols, 1},  
                                               torch::kFloat); 

  // torch::Tensor disp_tensor = torch::from_blob(disp.data, 
  //                                             {disp.rows, 
  //                                              disp.cols, 1},  
  //                                              torch::kFloat); 

  disp_tensor = disp_tensor.permute({(2), (0), (1)});
  disp_tensor.unsqueeze_(0);

  return disp_tensor;

}


////////////////////////////////////////////////////////////////////////////////
torch::Tensor LFE::getDisparityTensor(const cv::Mat& disp) {

  float disparity_multiplier = 1.0f;
  if (disp.type() == CV_16S) {
    disparity_multiplier = 16.0f;
  }
  cv::Mat floatDisp;
  disp.convertTo(floatDisp, CV_32F, 1.0f / disparity_multiplier);

  //Crop the image
  cv::Mat disp_cropped;
  floatDisp(cv::Rect(48, 0, 464, 384)).copyTo(disp_cropped);

  // Convert the cv image to a tensor
  torch::Tensor disp_tensor = torch::from_blob(disp_cropped.data, 
                                              {disp_cropped.rows, 
                                               disp_cropped.cols, 1},  
                                               torch::kFloat); 

  // torch::Tensor disp_tensor = torch::from_blob(floatDisp.data, 
  //                                             {floatDisp.rows, 
  //                                              floatDisp.cols, 1},  
  //                                              torch::kFloat); 

  disp_tensor = disp_tensor.permute({(2), (0), (1)});
  disp_tensor.unsqueeze_(0);

  return disp_tensor;
}

////////////////////////////////////////////////////////////////////////////////
std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> 
                     LFE::extractLearnedFeaturesDense(const cv::Mat &image) {

  // Crop the image
  cv::Mat image_cropped;
  image(cv::Rect(48, 0, 464, 384)).copyTo(image_cropped);

  // Make sure image memory is contigious before converting it to a tensor.
  if (!image_cropped.isContinuous()) {  
    LOG(INFO) << "Converting cv::Mat to torch::Tensor, memory isn't contigious";
    image_cropped = image_cropped.clone();
  }

  // we're about to use the gpu, lock
  std::unique_lock<std::mutex> lock(gpu_mutex_);

  // torch::NoGradGuard no_grad;
   
  // Convert the cv image to a tensor
  torch::Tensor image_tensor = torch::from_blob(image_cropped.data, 
                                               {image_cropped.rows, 
                                                image_cropped.cols, 3}, 
                                                torch::kByte).to(at::kCUDA); 

  // Convert the tensor into float and scale it (TODO: Normalize it)
  image_tensor = image_tensor.toType(c10::kFloat).div(255.0);

  // Transpose  and unsqueese to get tensor of dims [1, 3, height, width]
  image_tensor = image_tensor.permute({(2), (0), (1)});
  image_tensor.unsqueeze_(0);

  // Convert to input type expected by the model and pass to the forward method.
  auto inputs = std::vector<torch::jit::IValue>{image_tensor.to(at::kCUDA)};
  auto outputs = detector_.forward(inputs).toTuple();

  // Keypoints, dense descriptors, and dense scores (one per pixel).
  torch::Tensor keypoints = outputs->elements()[0].toTensor(); 
  torch::Tensor descriptors = outputs->elements()[1].toTensor();
  torch::Tensor scores = outputs->elements()[2].toTensor();

  keypoints = keypoints.detach().cpu();
  scores = scores.detach().cpu();
  descriptors = descriptors.detach().cpu();

  // Normalize each descriptor so we can use ZNCC when matching them.
  torch::Tensor desc_mean = torch::mean(descriptors, 1, true);
  torch::Tensor desc_std = torch::std(descriptors, 1, true, true);// Nx1
  torch::Tensor descriptors_norm = (descriptors - desc_mean) / desc_std; // NxC

  // We're done with the gpu, unlock
  lock.unlock();

  return std::make_tuple(keypoints, descriptors_norm, scores);

}


////////////////////////////////////////////////////////////////////////////////
std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> 
                     LFE::extractLearnedFeaturesSparse(const cv::Mat &image) {

  //Crop the image
  cv::Mat image_cropped;
  image(cv::Rect(48, 0, 464, 384)).copyTo(image_cropped);

  // Make sure image memory is contigious before converting it to a tensor.
  if (!image_cropped.isContinuous()) {  
    LOG(INFO) << "Converting cv::Mat to torch::Tensor, memory isn't contigious";
    image_cropped = image_cropped.clone();
  }             

  // we're about to use the gpu, lock
  std::unique_lock<std::mutex> lock(gpu_mutex_);
  
  // Convert the cv image to a tensor
  torch::Tensor image_tensor = torch::from_blob(image_cropped.data, 
                                               {image_cropped.rows, 
                                                image_cropped.cols, 3}, 
                                                torch::kByte).to(at::kCUDA); 

  // Convert the tensor into float and scale it (TODO: Normalize it)
  image_tensor = image_tensor.toType(c10::kFloat).div(255.0);

  // Transpose  and unsqueese to get tensor of dims [1, 3, height, width]
  image_tensor = image_tensor.permute({(2), (0), (1)});
  image_tensor.unsqueeze_(0);

  // Convert to input type expected by the model and pass to the forward method.
  auto inputs = std::vector<torch::jit::IValue>{image_tensor.to(at::kCUDA)};
  auto outputs = detector_.forward(inputs).toTuple();

  // Keypoints, dense descriptors, and dense scores (one per pixel).
  torch::Tensor keypoints = outputs->elements()[0].toTensor(); 
  torch::Tensor descriptors = outputs->elements()[1].toTensor();
  torch::Tensor scores = outputs->elements()[2].toTensor();

  // Get a score and descriptor for each keypoint
  torch::Tensor point_scores = getKeypointScores(scores, keypoints);
  torch::Tensor point_descriptors = getKeypointDescriptors(descriptors, 
                                                          keypoints); // NxC

  // Normalize each descriptor so we can use ZNCC when matching them.
  torch::Tensor desc_mean = torch::mean(point_descriptors, 1, true);
  torch::Tensor desc_std = torch::std(point_descriptors, 1, true, true);// Nx1
  torch::Tensor point_descriptors_norm = 
                           (point_descriptors - desc_mean) / desc_std; // NxC

  keypoints = keypoints.detach().cpu();
  point_scores = point_scores.detach().cpu();
  point_descriptors_norm = point_descriptors_norm.detach().cpu();
  
  // We're done with the gpu, unlock
  lock.unlock();

  return std::make_tuple(keypoints, point_descriptors_norm, point_scores);

}

////////////////////////////////////////////////////////////////////////////////
Features LFE::learnedFeatureToFrame(const torch::Tensor &keypoints,
                                    const torch::Tensor &point_descriptors,
                                    const torch::Tensor &point_scores) {
  // make a frame to return
  unsigned num_keypoints = keypoints.size(2);

  Features features;
  // we need to convert the tensor keypoints to opencv keypoints
  features.keypoints.reserve(num_keypoints);
  features.feat_infos.reserve(num_keypoints);
  std::vector<bool> valid_keypoint;
  valid_keypoint.resize(num_keypoints, true);
  
  for (unsigned i = 0; i < num_keypoints; i++) {
    features.keypoints.emplace_back();
    auto &kpt = features.keypoints.back();
    kpt.pt.x = keypoints[0][0][i].item<float>();
    kpt.pt.y = keypoints[0][1][i].item<float>();
    kpt.angle = -1;
    kpt.octave = 0;
    kpt.response = point_scores[i].item<float>();
    kpt.size = -1;  // TODO: What to put?
    
    // augment with data that isn't in the cv::KeyPoint struct
    double sigma = (1 << static_cast<int>(kpt.octave));
    bool laplacian_bit = static_cast<int>(kpt.response) & 0x1;
    auto precision = 1.0 / std::pow(sigma, 2.0);
    features.feat_infos.emplace_back(laplacian_bit, precision);
    auto &info = features.feat_infos.back();
    info.covariance(0, 0) = (1 << kpt.octave)*(1 << kpt.octave);
    info.covariance(1, 1) = (1 << kpt.octave)*(1 << kpt.octave);
    info.covariance(0, 1) = 0.0;
    info.covariance(1, 0) = 0.0;
  }

  // get the descriptor size
  unsigned descriptor_size = point_descriptors.size(0);

  // set the upright flag
  features.feat_type.upright = false;

  // set the feature type
  features.feat_type.impl = FeatureImpl::LEARNED_FEATURE;

  // copy into the Frame's cv::Mat
  features.descriptors = cv::Mat(num_keypoints, descriptor_size, 
                         CV_32F, point_descriptors.data_ptr<float>());

  // set the dimensions
  features.feat_type.dims = descriptor_size;

  // set bytes per descriptor
  features.feat_type.bytes_per_desc = features.feat_type.dims * sizeof(float);

  return features;
}

ChannelFeatures LFE::learnedFeaturesToStereoKeypoints(
                                       const torch::Tensor &keypoints, 
                                       const torch::Tensor &point_descriptors,
                                       const torch::Tensor &point_scores,
                                       const torch::Tensor &point_disparities) {

  unsigned num_keypoints = keypoints.size(2);
  unsigned descriptor_size = point_descriptors.size(1);

  // set up the stereo feature output
  ChannelFeatures channel;
  channel.cameras.resize(2);
  auto &left_feat = channel.cameras[0];
  auto &right_feat = channel.cameras[1];
  auto cam_rng = {0, 1};
  for (auto &i : cam_rng) {
    channel.cameras[i].keypoints.reserve(num_keypoints);
    channel.cameras[i].feat_infos.reserve(num_keypoints);
    channel.cameras[i].feat_type.upright = false;
    channel.cameras[i].feat_type.impl = FeatureImpl::LEARNED_FEATURE;
    channel.cameras[i].feat_type.dims = descriptor_size;
    channel.cameras[i].feat_type.bytes_per_desc =
        channel.cameras[i].feat_type.dims * sizeof(float);
  }

  std::vector<long> valid_keypoints = {};
  int num_valid = 0;
  for (unsigned i = 0; i < num_keypoints; i++) {
    
    // Check disparity to see if point is valid
    if ((point_disparities[i].item<float>() > 
         stereo_config_.stereoDisparityMinimum) && 
        (point_disparities[i].item<float>() < 
         stereo_config_.stereoDisparityMaximum)) {
    
      //&&
        // (point_scores[i].item<float>() >= 5.0e-3)
         
      valid_keypoints.emplace_back(i);
      num_valid++;
       
      // set up the left keypoint
      left_feat.keypoints.emplace_back();
      auto &kpt_l = left_feat.keypoints.back();
      kpt_l.pt.x = keypoints[0][0][i].item<float>() + 48.0;
      kpt_l.pt.y = keypoints[0][1][i].item<float>();
      kpt_l.angle = -1;
      kpt_l.octave = 0;
      kpt_l.response = point_scores[i].item<float>();
      kpt_l.size = -1; 

      // augment with data that isn't in the cv::KeyPoint struct
      double sigma_l = (1 << static_cast<int>(kpt_l.octave));
      bool laplacian_bit_l = static_cast<int>(kpt_l.response) & 0x1;
      left_feat.feat_infos.emplace_back(laplacian_bit_l,
                                        1.0 / std::pow(sigma_l, 2.0));

      auto &left_info = left_feat.feat_infos.back();
      left_info.covariance(0, 0) = 
                          (1 << kpt_l.octave)*(1 << kpt_l.octave);
      left_info.covariance(1, 1) = 
                          (1 << kpt_l.octave)*(1 << kpt_l.octave);
      left_info.covariance(0, 1) = 0.0;
      left_info.covariance(1, 0) = 0.0;

      // set up the matching right keypoint
      right_feat.keypoints.emplace_back();
      auto &kpt_r = right_feat.keypoints.back();
      kpt_r.pt.x = (keypoints[0][0][i].item<float>() + 48.0) - 
                    point_disparities[i].item<float>();
      // kpt_r.pt.x = keypoints[0][0][i].item<float>() - 
      //              point_disparities[i].item<float>();
      kpt_r.pt.y = keypoints[0][1][i].item<float>();
      kpt_r.angle = -1;
      kpt_r.octave = 0;
      kpt_r.response = point_scores[i].item<float>();
      kpt_r.size = -1; 

      // augment with data that isn't in the cv::KeyPoint struct
      double sigma_r = (1 << static_cast<int>(kpt_r.octave));
      bool laplacian_bit_r = static_cast<int>(kpt_r.response) & 0x1;
      right_feat.feat_infos.emplace_back(laplacian_bit_r,
                                        1.0 / std::pow(sigma_r, 2.0));

      auto &right_info = right_feat.feat_infos.back();
      right_info.covariance(0, 0) = 
                          (1 << kpt_r.octave)*(1 << kpt_r.octave);
      right_info.covariance(1, 1) = 
                          (1 << kpt_r.octave)*(1 << kpt_r.octave);
      right_info.covariance(0, 1) = 0.0;
      right_info.covariance(1, 0) = 0.0;
    }  
  }

  // LOG(INFO) << "num_valid: " << num_valid;

  auto options = torch::TensorOptions().dtype(torch::kLong);
  torch::Tensor valid = torch::from_blob(valid_keypoints.data(), {num_valid}, 
                                         options);  

  // Make sure memory for the tensor is contigious before converting to cv::Mat.
  // Otherwise elements in the matrix may be ordered incorrectly.
  torch::Tensor point_desc_valid = point_descriptors.index({valid, "..."});
  auto point_desc_tensor_ptr = point_desc_valid.contiguous().data_ptr<float>();

  left_feat.descriptors = cv::Mat(num_valid, descriptor_size, CV_32F, 
                                  point_desc_tensor_ptr);

  return channel;
}

////////////////////////////////////////////////////////////////////////////////
ChannelExtra LFE::extractFeaturesExtra(const cv::Mat &image) {
  
  auto outputs = extractLearnedFeaturesDense(image);
  torch::Tensor keypoints_ret = std::get<0>(outputs);  // 1x2xn
  torch::Tensor descriptors_ret = std::get<1>(outputs);
  torch::Tensor scores = std::get<2>(outputs);

  torch::Tensor keypoints = keypoints_ret.index({"..."});
  torch::Tensor descriptors = descriptors_ret.index({"..."});
 
  int width = scores.size(3);
  int height = scores.size(2);
  int num_channels = descriptors.size(1); // Tensor channels
  int num_points = keypoints.size(2);

  ChannelExtra channel;
  channel.cameras.resize(2);
  auto &left_extra = channel.cameras[0];

  keypoints.squeeze_(0);
  descriptors.squeeze_(0);
  scores.squeeze_(0);

  // Make sure memory for the tensor is contigious before converting to cv::Mat.
  // Otherwise elements in the matrix may be ordered incorrectly.
  auto keypoints_tensor_ptr = keypoints.contiguous().data_ptr<float>();
  auto descriptors_tensor_ptr = descriptors.contiguous().data_ptr<float>();
  auto scores_tensor_ptr = scores.contiguous().data_ptr<float>();

  std::vector<int> sz_kpt = {2, num_points};
  std::vector<int> sz_desc = {num_channels, height, width};
  std::vector<int> sz_score = {1, height, width};

  cv::Mat keypoints_mat(sz_kpt, CV_32F, keypoints_tensor_ptr);
  cv::Mat descriptors_mat(sz_desc, CV_32F, descriptors_tensor_ptr);
  cv::Mat scores_mat(sz_score, CV_32F, scores_tensor_ptr);

  left_extra.keypoints = keypoints_mat.clone();
  left_extra.descriptors = descriptors_mat.clone();
  left_extra.scores = scores_mat.clone(); 

  return channel;
}

////////////////////////////////////////////////////////////////////////////////
Features LFE::extractFeatures(const cv::Mat &image) {
  
  auto outputs = extractLearnedFeaturesSparse(image);

  torch::Tensor keypoints = std::get<0>(outputs);
  torch::Tensor point_descriptors = std::get<1>(outputs);
  torch::Tensor point_scores = std::get<2>(outputs);

  // Convert outputs from tensors to types expected by vtr.
  return learnedFeatureToFrame(keypoints, 
                               point_descriptors, 
                               point_scores);
}

////////////////////////////////////////////////////////////////////////////////
ChannelFeatures LFE::extractStereoFeatures(const cv::Mat &left_img,
                                            const cv::Mat &right_img) {
 
  auto outputs = extractLearnedFeaturesSparse(left_img);
  torch::Tensor keypoints = std::get<0>(outputs);
  torch::Tensor point_descriptors = std::get<1>(outputs);
  torch::Tensor point_scores = std::get<2>(outputs);

  // Get disparity
  torch::Tensor disparity = getDisparity(left_img, right_img, stereo_config_);
  torch::Tensor point_disparities = getKeypointDisparities(disparity, 
                                                          keypoints);

  return learnedFeaturesToStereoKeypoints(keypoints, point_descriptors, 
                                          point_scores, point_disparities);
}

////////////////////////////////////////////////////////////////////////////////
ChannelFeatures LFE::extractStereoFeaturesDisp(const cv::Mat &left_img,
                                               const cv::Mat &disp) {
  
  auto outputs = extractLearnedFeaturesSparse(left_img);
  torch::Tensor keypoints = std::get<0>(outputs);
  torch::Tensor point_descriptors = std::get<1>(outputs);
  torch::Tensor point_scores = std::get<2>(outputs);

  // Get disparity for each keypoint
  torch::Tensor disparity = getDisparityTensor(disp);
  torch::Tensor point_disparities = getKeypointDisparities(disparity, 
                                                           keypoints);
  // return channel;
  return learnedFeaturesToStereoKeypoints(keypoints, point_descriptors, 
                                          point_scores, point_disparities);
}

////////////////////////////////////////////////////////////////////////////////
ChannelFeatures LFE::extractStereoFeaturesDispExtra(const cv::Mat &left_img,
                                                    const cv::Mat &disp,
                                                    const cv::Mat &keypoints,
                                                    const cv::Mat &descriptors,
                                                    const cv::Mat &scores) {
  
  LOG(INFO) << "Kpt: " << keypoints.size;
  LOG(INFO) << "desc: " << descriptors.size;
  LOG(INFO) << "scores: " << scores.size;

  int height = descriptors.size[1];
  int width = descriptors.size[2];
  int num_channels = descriptors.size[0];
  int num_points = keypoints.size[1];
  
  torch::Tensor keypoints_t = torch::from_blob(keypoints.data, {2, num_points}, 
                                               torch::kFloat).unsqueeze_(0); 
  
  torch::Tensor descriptors_t = torch::from_blob(descriptors.data, 
                                                 {num_channels, height, width}, 
                                                 torch::kFloat).unsqueeze_(0);

  torch::Tensor scores_t = torch::from_blob(scores.data, {1, height, width}, 
                                            torch::kFloat).unsqueeze_(0);


  LOG(INFO) << "Before";

  LOG(INFO) << "Kpt: " << keypoints_t.sizes();
  LOG(INFO) << "desc: " << descriptors_t.sizes();
  LOG(INFO) << "scores: " << scores_t.sizes();

  // Get a score and descriptor for each keypoint
  torch::Tensor point_scores = getKeypointScores(scores_t, keypoints_t);
  
  LOG(INFO) << "point scores: " << point_scores.sizes();

  torch::Tensor point_descriptors = getKeypointDescriptors(descriptors_t, 
                                                           keypoints_t);
  LOG(INFO) << "point descriptors: " << point_descriptors.sizes();

  // Get disparity for each keypoint
  torch::Tensor disparity = getDisparityTensor(disp);
  torch::Tensor point_disparities = getKeypointDisparities(disparity, 
                                                           keypoints_t);

  LOG(INFO) << "point disparity: " << point_disparities.sizes();

  LOG(INFO) << "After";

  return learnedFeaturesToStereoKeypoints(keypoints_t, point_descriptors, 
                                          point_scores, point_disparities);
}

}  // namespace vision
}  // namespace vtr
