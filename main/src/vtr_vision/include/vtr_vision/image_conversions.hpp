////////////////////////////////////////////////////////////////////////////////
/// @brief image_conversions.h header file
/// @details Image conversion functions including from RGB to grayscale or
///          colour constant
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <vtr_vision/types.hpp>

namespace vtr {
namespace vision {

enum class ImageConversion : uint8_t {
  UNKNOWN = 0,
  RGB_TO_GRAYSCALE = 1,
  RGB_TO_COLOR_CONSTANT = 2,
  GRAY_TO_UNDISTORTED = 3
};

std::string ImageConversionToString(const ImageConversion& conversion);
ImageConversion StringToImageConversion(const std::string &conversion);

////////////////////////////////////////////////////////////////////////////////
/// @brief Converts an RGB image into a grayscale image.
/// @param src The RGB source image.
/// @return A grayscale image.
////////////////////////////////////////////////////////////////////////////////
Image RGB2Grayscale(const Image &src);

////////////////////////////////////////////////////////////////////////////////
/// @brief Converts a channel of RGB images into a channel of grayscale images.
/// @param src The RGB source channel.
/// @return A grayscale image channel.
////////////////////////////////////////////////////////////////////////////////
ChannelImages RGB2Grayscale(const ChannelImages &src);

////////////////////////////////////////////////////////////////////////////////
/// @brief Converts an RGB image into a color constant grayscale image.
/// @param src The RGB source image.
/// @param alpha The color constant weight.
/// @param histogram_equalization Histogram Equalization flag.
/// @return A color constant grayscale image.
////////////////////////////////////////////////////////////////////////////////
Image RGB2ColorConstant(const Image &src,float &alpha, bool histogram_equalization);

////////////////////////////////////////////////////////////////////////////////
/// @brief Converts a channel of RGB images into a channel of color constant grayscale images.
/// @param src The RGB source channel.
/// @param alpha The color constant weight.
/// @param histogram_equalization Histogram Equalization flag.
/// @return A color constant grayscale image channel.
////////////////////////////////////////////////////////////////////////////////
ChannelImages RGB2ColorConstant(const ChannelImages & src, float alpha, bool histogram_equalization);

////////////////////////////////////////////////////////////////////////////////
/// @brief Converts a channel of grayscale images into a channel of undistorted grayscale images.
/// @param src The grayscale source channel.
/// @param intrinsics A vector of the intrinsics for each camera
/// @param dists A vector of the five elements of distortion for each camera
/// @return A color constant grayscale image channel.
////////////////////////////////////////////////////////////////////////////////
ChannelImages Gray2Undistorted(const ChannelImages & src, const CameraIntrinsics & intrinsics, const CameraDistortions & dists);

}
}
