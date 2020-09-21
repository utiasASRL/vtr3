#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <asrl/common/logging.hpp>

#pragma GCC diagnostic ignored "-pedantic"
#include <ros/ros.h>
#pragma GCC diagnostic pop

#include <robochunk/base/DataStream.hpp>
#include <robochunk_msgs/StereoImage.pb.h>
#include <robochunk_msgs/StereoCameraInfo.pb.h>
#include <asrl/messages/conversion_tools.hpp>

#include <asrl/vision/features/extractor/FeatureExtractorFactory.hpp>
#include <asrl/vision/features/matcher/BRIEFFeatureMatcher.hpp>
#include <asrl/vision/matching/matching.h>

// ** FOLLOWING LINE SHOULD BE USED ONCE AND ONLY ONCE IN WHOLE APPLICATION **
// ** THE BEST PLACE TO PUT THIS LINE IS IN main.cpp RIGHT AFTER INCLUDING easylogging++.h **
INITIALIZE_EASYLOGGINGPP

////////////////////////////////////////////////////////////////////
/// @brief Stereo Calibration information.
////////////////////////////////////////////////////////////////////
struct StereoCalibration {

    ////////////////////////////////////////////////////////////////////
    /// @brief Intrinsic camera matrix.
    ////////////////////////////////////////////////////////////////////
    Eigen::Matrix3d intrinsics;

    ////////////////////////////////////////////////////////////////////
    /// @brief Projection matrix of both left and right cameras.
    ////////////////////////////////////////////////////////////////////
    Eigen::Matrix<double,3,4> projection_l, projection_r;

    ////////////////////////////////////////////////////////////////////
    /// @brief Camera baseline.
    ////////////////////////////////////////////////////////////////////
    double baseline;
};

Eigen::Vector3d triangulatePoint(const StereoCalibration calibration, const cv::Point & left_keypoint,const cv::Point &right_keypoint) {

    double f = calibration.intrinsics(0,0);
    double disparity = left_keypoint.x - right_keypoint.x;
    double xl = left_keypoint.x - calibration.intrinsics(0,2);
    double yl = left_keypoint.y - calibration.intrinsics(1,2);
    double z = (calibration.baseline*f)/disparity;
    double x = xl*z/f;
    double y = yl*z/f;
    return Eigen::Vector3d(x,y,z);
}


cv::Mat CreateMatchView(cv::Mat &left_image,
                        cv::Mat &right_image) {
    // Create the image
    cv::Mat matchView = cv::Mat(left_image.rows,
                                left_image.cols*2,
                                CV_8UC3);

    int offset1 = (matchView.cols*3)/2; 
    for(int row = 0; row < left_image.rows; row++)
            {
            for(int col = 0; col < left_image.cols; col++)
                {
                int pixelIdx = row*matchView.cols*3+col*3;
                if(left_image.rows > 0) {
                    //Top Left
                    matchView.data[pixelIdx] =   left_image.data[row*left_image.cols+col];
                    matchView.data[pixelIdx+1] = matchView.data[pixelIdx];
                    matchView.data[pixelIdx+2] = matchView.data[pixelIdx];

                    //Top Right
                    matchView.data[pixelIdx+offset1] = right_image.data[row*left_image.cols+col];
                    matchView.data[pixelIdx+offset1+1] = matchView.data[pixelIdx+offset1];
                    matchView.data[pixelIdx+offset1+2] = matchView.data[pixelIdx+offset1];
                }
                }
            }
    // Fill in matches
    return matchView;
}

void visualizeMatches(cv::Mat &left_image,
                      cv::Mat &right_image,
                      asrl::vision::StereoFrame &frame) {

    cv::Mat keyPointView = CreateMatchView(left_image,right_image);
    cv::Mat matchView = CreateMatchView(left_image,right_image);
    cv::Mat matchView2 = CreateMatchView(left_image,right_image);
    cv::Scalar trackColor(203,201,40);

    //auto keypoints = asrl::messages::copyKeypoints(frame);

    // draw the matches
    for(uint32_t idx = 0; idx < frame.match.size(); ++idx) {

        cv::Point left = frame.left.keypoints[frame.match[idx].first].pt;
        cv::Point right = frame.right.keypoints[frame.match[idx].second].pt;
        right.x += left_image.size().width;
        cv::line(matchView, left, right, trackColor, 1);

        cv::Point left2 = frame.left.keypoints[frame.match[idx].first].pt;
        cv::Point right2 = frame.right.keypoints[frame.match[idx].second].pt;
        cv::line(matchView2, left2, right2, trackColor, 1);

        right2.x += left_image.size().width;
        left2.x += left_image.size().width;
        cv::line(matchView2, left2, right2, trackColor, 1);
    }


    // fill in the left keypoints
    for(uint32_t idx = 0; idx < frame.left.keypoints.size(); ++idx) {
        int blue = 0;
        int red = std::max(0,255-20*(int)frame.points_3D(2,idx));
        int green = std::min(255,20*(int)frame.points_3D(2,idx));
        cv::Scalar kpColor(blue,green,red);
        cv::KeyPoint kp = frame.left.keypoints[idx];
        cv::circle(keyPointView, kp.pt, 4, kpColor);
        cv::circle(matchView, kp.pt, 4, kpColor);
        cv::circle(matchView2, kp.pt, 4, kpColor);
    }

    // fill in the right keypoints
    for(uint32_t idx = 0; idx < frame.right.keypoints.size(); ++idx) {
        int blue = 0;
        int red = std::max(0,255-20*(int)frame.points_3D(2,idx));
        int green = std::min(255,20*(int)frame.points_3D(2,idx));
        cv::Scalar kpColor(blue,green,red);
        cv::KeyPoint kp = frame.right.keypoints[idx];
        kp.pt.x += left_image.cols;
        cv::circle(keyPointView, kp.pt, 4, kpColor);
        cv::circle(matchView, kp.pt, 4, kpColor);
        cv::circle(matchView2, kp.pt, 4, kpColor);

    }

    // print the number of matches
    std::ostringstream tostr;
    tostr << frame.match.size();
    cv::putText(matchView2, std::string("Matches: " +tostr.str()).c_str(), cv::Point(25,25),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,0),1);

    // show the image
    //cv::imshow("Keypoints ", keyPointView);

    //cv::imshow("Matches ", matchView);
    cv::imshow("Matches 2 ", matchView2);
    cv::waitKey(10);
}
/*
void visualizeMatches(cv::Mat &prev_image,
                      cv::Mat &curr_image,
                      asrl::vision::MultiFrame &frame) {

    if(frame.frames.size() < 2) {
        std::cerr << "Error: Less than two frames sent to visualizeMatches!" << std::endl;
        return;
    }
    cv::Mat keyPointView = CreateMatchView(prev_image,curr_image);
    cv::Mat matchView = CreateMatchView(prev_image,curr_image);
    cv::Mat matchView2 = CreateMatchView(prev_image,curr_image);
    cv::Scalar trackColor(203,201,40);

    // draw the matches
    for(unsigned n = 0; n < 1; n++) { // Only stereo right now
      for(uint32_t idx = 0; idx < frame.matches[n].size(); ++idx) {

        // matches
        cv::Point prev = frame.frames[0].keypoints[frame.matches[n][idx].first].pt;
        cv::Point curr = frame.frames[1].keypoints[frame.matches[n][idx].second].pt;
        curr.x += prev_image.size().width;
        cv::line(matchView, prev, curr, trackColor, 1);

        cv::Point prev2 = frame.frames[0].keypoints[frame.matches[n][idx].first].pt;
        cv::Point curr2 = frame.frames[1].keypoints[frame.matches[n][idx].second].pt;
        cv::line(matchView2, prev2, curr2, trackColor, 1);

        curr2.x += prev_image.size().width;
        prev2.x += prev_image.size().width;
        cv::line(matchView2, prev2, curr2, trackColor, 1);


        int blue = 0;
        int red = std::max(0,255-20*(int)frame.points_3D(2,frame.matches[n][idx].second));
        int green = std::min(255,20*(int)frame.points_3D(2,frame.matches[n][idx].second));
        cv::Scalar kpColor(blue,green,red);

        // left keypoint
        cv::KeyPoint kp = frame.frames[0].keypoints[frame.matches[n][idx].first];
        cv::circle(keyPointView, kp.pt, 4, kpColor);
        cv::circle(matchView, kp.pt, 4, kpColor);
        cv::circle(matchView2, kp.pt, 4, kpColor);

        // right keypoint
        cv::KeyPoint kp2 = frame.frames[1].keypoints[frame.matches[n][idx].second];
        kp2.pt.x += prev_image.cols;
        cv::circle(keyPointView, kp2.pt, 4, kpColor);
        cv::circle(matchView, kp2.pt, 4, kpColor);
        cv::circle(matchView2, kp2.pt, 4, kpColor);
      }

    }

    // print the number of matches
    std::ostringstream tostr;
    tostr << frame.matches[0].size();
    cv::putText(matchView2, std::string("Matches: " +tostr.str()).c_str(), cv::Point(25,25),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,0),1);

    // show the image
    //cv::imshow("Keypoints Temporal", keyPointView);

    //cv::imshow("Matches Temporal", matchView);
    cv::imshow("Matches 2 Temporal", matchView2);
    cv::waitKey(10);
}
*/

void ManageFeatures(std::shared_ptr<asrl::vision::BaseFeatureExtractor> extractor, std::shared_ptr<std::queue<std::vector<cv::Mat>>> images) {
    unsigned count = 0;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    while(ros::ok()) {
        auto frame = extractor->getOldestStereoFrame();
        while(images.get()->empty()) {};
        visualizeMatches(images.get()->front()[0],images.get()->front()[1],frame);
        images.get()->pop();
        if(count == 100) {
            end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end-start;
            double avg_speed = elapsed_seconds.count()/count;
            LOG(INFO) << "Speed " << 1.0/avg_speed << "hz. Queue size: " << images.get()->size();
            count = 0;
            start = end;
        }
        count++;
    }
}

////////////////////////////////////////////////////////////////////////////////////
/// @brief Demonstration of reading and streaming image data from a chunk folder.
///        As well as synchronizing two image streams according to time.
////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char ** argv) {
    // ros publishers for visualisations
    ros::init(argc, argv, "extractor_demo");
    ros::NodeHandle nh("~");

    // Seek to a specified index
    uint32_t seekIdx;
    std::string data_directory;
    std::string feature_type;

    nh.param<std::string>("feature_type",feature_type,"");
    // robochunk
    nh.param<std::string>("data_directory",data_directory,"");
    int int_param;
    nh.param<int>("seek_idx",int_param,0);
    
    bool asynchronous = false;
    nh.param<bool>("asynchronous",asynchronous,false);
    seekIdx = int_param;

    // Set up the data streams
    std::string greyscale_stream_name("/front_xb3/greyscale/stereo_images");
    robochunk::base::ChunkStream greyscale_stream(data_directory,greyscale_stream_name);

    // extract the calibration
    robochunk::msgs::RobochunkMessage calibration_msg;
    robochunk::sensor_msgs::StereoCameraInfo * camera_info;
    // Check out the calibration
    if(greyscale_stream.fetchCalibration(calibration_msg) == true) {
        printf("received camera calibration!\n");
        // Extract the intrinsic params out of the bsae message
        camera_info = calibration_msg.extractPayload<robochunk::sensor_msgs::StereoCameraInfo>();
        if(camera_info != nullptr) {
            for(int idx = 0; idx < camera_info->left_camera_info().p().size(); idx++) {
                std::cout << idx << ": " << camera_info->left_camera_info().p(idx) << std::endl;
            }
        } else {
            printf("ERROR: intrinsic params is not the correc type: (actual: %s \n",calibration_msg.header().type_name().c_str());
            return -1;
        }
    } else {
        printf("ERROR: Could not read calibration message!\n");
    }

    StereoCalibration stereo_calibration;

    // get the intrinsics
    stereo_calibration.intrinsics = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> >(camera_info->left_camera_info().k().data());

    // fix the (potentially) missing 1.0 in the dataset's calibration
    stereo_calibration.intrinsics(2,2) = 1.0;
    std::cout << "Intrinsics " << std::endl << stereo_calibration.intrinsics << std::endl;
    // get the projection matrices
    stereo_calibration.projection_l = Eigen::Map<const Eigen::Matrix<double,3,4,Eigen::RowMajor> >(camera_info->left_camera_info().p().data());
    stereo_calibration.projection_r = Eigen::Map<const Eigen::Matrix<double,3,4,Eigen::RowMajor> >(camera_info->right_camera_info().p().data());

    // extract the baseline
    stereo_calibration.baseline = -camera_info->right_camera_info().p(3)/stereo_calibration.intrinsics(0,0);
    std::cout << "Baseline: " << stereo_calibration.baseline << std::endl;

    // Initialize the extractor
    std::shared_ptr<asrl::vision::BaseFeatureExtractor> extractor_;
    extractor_ = asrl::vision::FeatureExtractorFactory::createExtractor(feature_type);
    asrl::vision::ExtractorConfig feature_config;
    // TODO: Make parameters configurable
    auto &gpu_surf_config = feature_config.gpu_surf_stereo_params;
    gpu_surf_config.threshold = 1e-7;
    gpu_surf_config.nOctaves = 4;
    gpu_surf_config.nIntervals = 4;
    gpu_surf_config.initialScale = 1.5;
    gpu_surf_config.edgeScale = 1.5;
    gpu_surf_config.l1 = 3.f/1.5f;
    gpu_surf_config.l2 = 5.f/1.5f;
    gpu_surf_config.l3 = 3.f/1.5f;
    gpu_surf_config.l4 = 1.f/1.5f;
    gpu_surf_config.initialStep = 1;
    gpu_surf_config.targetFeatures = 800;
    gpu_surf_config.detector_threads_x = 16;
    gpu_surf_config.detector_threads_y = 4;
    gpu_surf_config.nonmax_threads_x = 16;
    gpu_surf_config.nonmax_threads_y = 16;
    gpu_surf_config.regions_horizontal = 8;
    gpu_surf_config.regions_vertical = 6;
    gpu_surf_config.regions_target = 800;
    gpu_surf_config.stereoDisparityMinimum = 0.0f;
    gpu_surf_config.stereoDisparityMaximum = 120.0f;
    gpu_surf_config.stereoCorrelationThreshold = 0.79f;
    gpu_surf_config.stereoYTolerance = 1.0f;
    gpu_surf_config.stereoScaleTolerance = 0.8f;

    auto &opencv_orb_config = feature_config.opencv_orb_params;
    opencv_orb_config.num_detector_features_ = 15000;
    opencv_orb_config.num_binned_features_ = 800;
    opencv_orb_config.scaleFactor_ = 1.8f;
    opencv_orb_config.nlevels_ = 8;
    opencv_orb_config.edgeThreshold_ = 8;
    opencv_orb_config.firstLevel_ = 0;
    opencv_orb_config.WTA_K_ = 2;
    opencv_orb_config.scoreType_ = cv::ORB::HARRIS_SCORE;
    opencv_orb_config.patchSize_ = 64;
    opencv_orb_config.x_bins_ = 1;
    opencv_orb_config.y_bins_ = 1;
    opencv_orb_config.stereo_matcher_config_.descriptor_match_thresh_ = 0.55;
    opencv_orb_config.stereo_matcher_config_.stereo_descriptor_match_thresh_ = 0.55;
    opencv_orb_config.stereo_matcher_config_.stereo_y_tolerance_ = 1.0;
    opencv_orb_config.stereo_matcher_config_.stereo_x_tolerance_min_ = 0;
    opencv_orb_config.stereo_matcher_config_.stereo_x_tolerance_max_ = 250;
    opencv_orb_config.stereo_matcher_config_.check_octave_ = true;
    opencv_orb_config.stereo_matcher_config_.check_response_ = true;//1.0;
    opencv_orb_config.stereo_matcher_config_.min_response_ratio_ = 0.2;//0;
    opencv_orb_config.stereo_matcher_config_.scale_x_tolerance_by_y_ = true;
    opencv_orb_config.stereo_matcher_config_.x_tolerance_scale_ = 768;
    extractor_->initialize(feature_config);

    std::shared_ptr<std::queue<std::vector<cv::Mat>>> images;
    images.reset(new std::queue<std::vector<cv::Mat>>);
    auto output_thread = std::thread(ManageFeatures,extractor_, images);
    output_thread.detach();

    // Initialize the messages
    robochunk::msgs::RobochunkMessage greyscale_msg;

    // Seek to the absolute index
    greyscale_stream.seek(seekIdx);

    // Set a flag to continue the stream
    bool continue_stream = true;

    // Grab the next greyscale image.
    continue_stream &= greyscale_stream.next(greyscale_msg);
    uint32_t idx = 0;
    uint32_t count = 0;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    cv::Mat old_left;
    asrl::vision::StereoFrame old_frame;
    while(continue_stream && ros::ok()) {
        // Convert to OpenCV and display
        auto stereo_image = greyscale_msg.extractSharedPayload<robochunk::sensor_msgs::StereoImage>();
        if(stereo_image != nullptr) {
            // extract the left/right images.
            std::vector<cv::Mat> left_right_pair;
            left_right_pair.resize(2);
            left_right_pair[0] = asrl::messages::wrapImage(*stereo_image->mutable_left_image()).clone();
            left_right_pair[1] = asrl::messages::wrapImage(*stereo_image->mutable_right_image()).clone();
            images.get()->push(left_right_pair);
            // Run asynchronous processing if requested.
            if(asynchronous) {
                // send to the feature extractor
                try{
                    extractor_->addExtractionTask(left_right_pair[0], left_right_pair[1]);
                    //extractor_->addExtractionTask(left_right_pair);
                } catch(...) {
                    LOG(ERROR) << "we caught something...";
                }
            } else {
                asrl::vision::StereoFrame frame = extractor_->extractFeatures(left_right_pair[0], left_right_pair[1]);
                frame.points_3D.resize(3,frame.match.size());
                for(unsigned i = 0; i < frame.match.size(); i++) {
                    frame.points_3D.col(i) = triangulatePoint(stereo_calibration,frame.left.keypoints[frame.match[i].first].pt, frame.right.keypoints[frame.match[i].second].pt);
                }
                visualizeMatches(left_right_pair[0],left_right_pair[1],frame);
                if(count > 0) {
                    asrl::vision::BRIEFFeatureMatcher matcher;
                    matcher.initialize(opencv_orb_config.stereo_matcher_config_);
                    // TODO restore this visualization
                    /*
                    asrl::vision::MultiFrame multiframe;
                    multiframe.frames.push_back(old_frame.left);
                    multiframe.frames.push_back(frame.left);
                    //multiframe.match =  matcher.matchFeatures(old_frame.left, frame.left, 50);
                    multiframe.matches.push_back(matcher.matchFeatures(old_frame, frame, 300, 0.1));
                    multiframe.points_3D = frame.points_3D;
                    visualizeMatches(old_left,left_right_pair[0],multiframe);
                    */

                    asrl::vision::MatchList inliers;

                    // Create a feature mask for close features (to be favoured in RANSAC sampling)
                    std::vector<bool> mask(frame.points_3D.cols());

                    // Set up the model
                    asrl::vision::matching::StereoTransformModel::Ptr model = std::make_shared<asrl::vision::matching::StereoTransformModel>();
                    //model->setMeasurementVariance(keyframe_features->inv_r_matrix);
                    model->setPoints(&old_frame.points_3D, &frame.points_3D);
                    //model->setCalibration(stereo_calibration_.projection_l, stereo_calibration_.projection_r);
                    model->setCalibration(stereo_calibration.intrinsics, stereo_calibration.baseline);

                    // Storage for solution
                    Eigen::Matrix4d solution;

                    /*
                    // Verify points are sane
                    if (!model->verifyMatches(multiframe.matches[0])) {
                        std::cout << "Model has been given bad points..." << std::endl;
                    }
                    */

                    auto verifier = std::make_shared<asrl::vision::matching::VerifySampleSubsetMask>(0,mask); // Need 1 close feature
                    auto sampler = std::make_shared<asrl::vision::matching::BasicSampler>(verifier);
                    asrl::vision::matching::VanillaRansac<Eigen::Matrix4d> ransac(sampler,3.5,5.0,1000,1.0,800);

                    // Register the callback with RANSAC
                    ransac.setCallback(model);

                    // Perform RANSAC

                    unsigned num = 0; /* = ransac.run(multiframe.matches[0], &solution, &inliers); */
                    std::cout << "Returned RANSAC inliers: " << inliers.size() << std::endl;
                    std::cout << "Returned RANSAC solution: " << std::endl << solution << std::endl;

                    if(num == 0) {
                        LOG(ERROR) << "RANSAC returned 0 inliers!";
                    } else {
                        //multiframe.matches[0] =  inliers;
                       // visualizeMatches(old_left,left_right_pair[0],multiframe);
                    }
                }

                if(count == 100) {
                    end = std::chrono::system_clock::now();
                    std::chrono::duration<double> elapsed_seconds = end-start;
                    double avg_speed = elapsed_seconds.count()/count;
                    LOG(INFO) << "Speed " << 1.0/avg_speed << "hz";
                    count = 0;
                    start = end;
                }
                count++;
                old_left = left_right_pair[0].clone();
                old_frame = frame;
            }
            idx++;
        }
        greyscale_msg.Clear();
        continue_stream &= greyscale_stream.next(greyscale_msg);
        // If this is asynchronous, then throttle the thread.
        if(asynchronous) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    } 
}
