// External
#include <future>
#include <list>
#include <opencv2/opencv.hpp>
#include <vtr_vision/image_conversions.hpp>

namespace vtr {
namespace vision {

// this is important for OpenCV and SURF GPU multithreaded operations
std::mutex __gpu_mutex__;

std::string ImageConversionToString(const ImageConversion &conversion) {
  switch (conversion) {
    case ImageConversion::RGB_TO_GRAYSCALE:
      return "RGB_TO_GRAYSCALE";
    case ImageConversion::RGB_TO_COLOR_CONSTANT:
      return "RGB_TO_COLOR_CONSTANT";
    case ImageConversion::GRAY_TO_UNDISTORTED:
      return "GRAY_TO_UNDISTORTED";
    case ImageConversion::UNKNOWN:
      return "UNKNOWN";
    default:
      return "UNKNOWN";
  }
}

ImageConversion StringToImageConversion(const std::string &conversion) {
  if (conversion == "RGB_TO_GRAYSCALE") {
    return ImageConversion::RGB_TO_GRAYSCALE;
  } else if (conversion == "RGB_TO_COLOR_CONSTANT") {
    return ImageConversion::RGB_TO_COLOR_CONSTANT;
  } else if (conversion == "GRAY_TO_UNDISTORTED") {
    return ImageConversion::GRAY_TO_UNDISTORTED;
  } else {
    return ImageConversion::UNKNOWN;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Converts an RGB image into a grayscale image.
/// @param src The RGB source image.
/// @return A grayscale image.
////////////////////////////////////////////////////////////////////////////////
Image RGB2Grayscale(const Image &src) {
  Image dst;
  cv::cvtColor(src.data, dst.data,
               cv::COLOR_BGR2GRAY);  // vtr3 change: opencv 4+
  dst.name = src.name;
  return dst;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Converts a channel of RGB images into a channel of grayscale images.
/// @param src The RGB source channel.
/// @return A grayscale image channel.
////////////////////////////////////////////////////////////////////////////////
ChannelImages RGB2Grayscale(const ChannelImages &src) {
  // set up the new channel
  ChannelImages dst;
  dst.name = "grayscale";

  // start the conversion job in parallel for each camera in the channel.
  std::list<std::future<Image>> futures;
  auto func = static_cast<Image (*)(const Image &)>(&RGB2Grayscale);
  for (const auto &image : src.cameras) {
    futures.emplace_back(std::async(std::launch::async, func, image));
  }

  // Wait until completion and add to the destination channel.
  for (auto &future : futures) {
    dst.cameras.emplace_back(future.get());
  }

  // return the new channel.
  return dst;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Converts an RGB image into a color constant grayscale image.
/// @param src The RGB source image.
/// @param alpha The color constant weight.
/// @param histogram_equalization Histogram Equalization flag.
/// @return A color constant grayscale image.
////////////////////////////////////////////////////////////////////////////////
Image RGB2ColorConstant(const Image &src, float &alpha,
                        bool histogram_equalization) {
  // Create a log lookup table
  // TODO: (old) Hardcode for speedup?
  std::vector<float> log_table(256, 0);
  log_table[0] = 0;
  for (int idx = 1; idx < 256; ++idx) {
    log_table[idx] = log(static_cast<float>(idx));
  }

  // set up a temporary floating point image.
  cv::Mat dst = cv::Mat(src.data.rows, src.data.cols, CV_32FC1);
  auto *gray = (float *)&dst.data[0];

  // compute the beta weight.
  double beta = 1 - alpha;

  // Perform per-pixel transformation.
  //#pragma omp parallel for num_threads(6)
  for (int idx = 0; idx < dst.cols * dst.rows; idx++) {
    auto data_idx = idx * 3;
    gray[idx] = log_table[src.data.data[data_idx + 1]] -
                alpha * log_table[src.data.data[data_idx]] -
                beta * log_table[src.data.data[data_idx + 2]];
  }

  // normalize the floating point image to the range [0,1].
  cv::normalize(dst, dst, 0, 1, cv::NORM_MINMAX, CV_32FC1);

  // set up the new image.
  Image image;
  image.data = cv::Mat(src.data.rows, src.data.cols, CV_8UC1);
  image.name = src.name;

  // convert to byte image.
  dst.convertTo(image.data, CV_8UC1, 255.0);

  // Perform histogram equalization if requsted.
  if (histogram_equalization) {
    cv::equalizeHist(image.data, image.data);
  }

  // return the image.
  return image;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Converts a channel of RGB images into a channel of color constant
/// grayscale images.
/// @param src The RGB source channel.
/// @param alpha The color constant weight.
/// @param histogram_equalization Histogram Equalization flag.
/// @return A color constant grayscale image channel.
////////////////////////////////////////////////////////////////////////////////
ChannelImages RGB2ColorConstant(const ChannelImages &src, float alpha,
                                bool histogram_equalization) {
  // set up the channel.
  ChannelImages dst;
  dst.name = "cc_" + std::to_string(alpha);

  // start the conversion job in parallel for each camera in the channel.
  std::list<std::future<Image>> futures;
  auto func =
      static_cast<Image (*)(const Image &, float &, bool)>(&RGB2ColorConstant);
  for (const auto &image : src.cameras) {
    futures.emplace_back(std::async(std::launch::async, func, std::ref(image),
                                    std::ref(alpha), histogram_equalization));
  }

  // Wait until completion and add to the destination channel.
  for (auto &future : futures) {
    dst.cameras.emplace_back(future.get());
  }
  return dst;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Converts a channel of grayscale images into a channel of undistorted
/// grayscale images.
/// @param src The grayscale source channel.
/// @param intrinsics A vector of the intrinsics for each camera
/// @param dists A vector of the five elements of distortion for each camera
/// @return A color constant grayscale image channel.
////////////////////////////////////////////////////////////////////////////////
ChannelImages Gray2Undistorted(const ChannelImages &src,
                               const CameraIntrinsics &intrinsics,
                               const CameraDistortions &dists) {
  // set up the channel.
  ChannelImages dst;
  dst.name = "undistorted";

  // start the conversion job in parallel for each camera in the channel.

  // ...well, that was the plan. It turns out cv::undistort is not thread-safe
  // and corrupts images if called in parallel. Therefore the images are
  // undistorted here in a non parallel fashion. The code to undistort in
  // parallel is left here for posterity and such time as the OpenCV library
  // function is fixed.
  // std::list<std::future<void>> futures;
  // auto func = static_cast<void(*)(cv::InputArray&, cv::OutputArray&,
  // cv::InputArray&, cv::InputArray&, cv::InputArray&)>(&cv::undistort);

  unsigned ii = 0;
  for (const auto &image : src.cameras) {
    dst.cameras.emplace_back(Image());
    vision::Image &dstimg = dst.cameras.back();
    dstimg.name = image.name;
    dstimg.stamp = image.stamp;

    cv::Matx33d intrinsic(
        intrinsics[ii](0, 0), intrinsics[ii](0, 1), intrinsics[ii](0, 2),
        intrinsics[ii](1, 0), intrinsics[ii](1, 1), intrinsics[ii](1, 2),
        intrinsics[ii](2, 0), intrinsics[ii](2, 1), intrinsics[ii](2, 2));
    cv::Vec<double, 5> dist(dists[ii](0), dists[ii](1), dists[ii](2),
                            dists[ii](3), dists[ii](4));

    // replace the parallel method with a direct call
    // futures.emplace_back(std::async(func,image.data,dstimg.data, intrinsic,
    // dist, cv::noArray()));
    cv::undistort(image.data, dstimg.data, intrinsic, dist, cv::noArray());
    ii++;
  }

  // Wait until completion (or not, since there's nothing to wait for)
#if 0
  for(auto & future : futures) {
    future.get();
  }
#endif
  return dst;
}

}  // namespace vision
}  // namespace vtr
