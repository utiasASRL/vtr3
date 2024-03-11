/*
Copyright (c) 2010, Paul Furgale and Chi Hay Tong
All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are 
met:

* Redistributions of source code must retain the above copyright notice, 
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
  notice, this list of conditions and the following disclaimer in the 
  documentation and/or other materials provided with the distribution.
* The names of its contributors may not be used to endorse or promote 
  products derived from this software without specific prior written 
  permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ASRL_GPU_INTEGRAL_IMAGE_PROCESSOR_HPP
#define ASRL_GPU_INTEGRAL_IMAGE_PROCESSOR_HPP


#include <opencv2/core/core.hpp>

#include <memory>
#include <builtin_types.h>

// Forward declarations.
struct cudaArray;
typedef size_t CUDPPHandle;


namespace asrl {

  // Forward declaration
  class GpuIntegralImage;

  /**
   * @class GpuIntegralImageProcessor
   * @brief A class that reserves memory on the GPU for creating integral images.
   *
   * This class sets up the necessary functions and memory for GpuIntegralImage_kernel.cu to compute the integral image.
   */
  class GpuIntegralImageProcessor {
  public:
    /** 
     * The constructor reserves memory for integral image processing given that
     * the source image is width\f$\times\f$height
     * 
     * @param width The width of the expected input image
     * @param height The height of the expected input image
     */
    GpuIntegralImageProcessor(int width, int height);

    /** 
     * Destructor
     * 
     */
    ~GpuIntegralImageProcessor();

    /** 
     * Uploads the image to the device.
     * 
     * @param img The input image. The image must be tightly packed, of type CV_U8C1, and width()\f$\times\f$height()
     */
    void upload(const cv::Mat & img);
    
    /** 
     * Uploads the image to the device then creates the integral image. This is the equivalent of calling
     * \code
     * upload(img);
     * process(outImage,stream);
     * \endcode
     * 
     * @param img The input image. The image must be tightly packed, of type CV_U8C1, and width()\f$\times\f$height()
     * @param outImage The output integral image
     * @param stream An optional cuda stream for performing the processing
     */
    void process(const cv::Mat & img, GpuIntegralImage & outImage, cudaStream_t stream = 0);
    
    /** 
     * Creates the integral image. The image must have been previously uploaded to the device by calling
     * \code
     * upload(img)
     * \endcode
     * 
     * @param outImage 
     * @param stream 
     */
    void process(GpuIntegralImage & outImage, cudaStream_t stream = 0);

    /** 
     * 
     * @return The expected width of the input image (pixels)
     */
    int width(){ return m_width; }
    /** 
     * 
     * @return The expected height of the input image (pixels)
     */
    int height(){ return m_height; }
  private:
    /// Initial storage on the gpu of the unsigned char data
    std::shared_ptr<unsigned char> char_data;	
    
    /// Storage of floats on the GPU, for the standard orientation of the image
    std::shared_ptr<float> norm_data;
    /// A third buffer on the GPU used for the transpose operation
    std::shared_ptr<float> trans_data;
    
    /// The pitch (spacing between rows of data) of rows of the char_data array, in elements
    size_t char_pitch;
    /// The pitch (spacing between rows of data) of rows of the norm_data array, in elements
    size_t norm_pitch;
    /// The pitch (spacing between rows of data) of rows of the trans_data array, in elements
    size_t trans_pitch;
    
    /// The CUDPP configuration handle for a parallel prefix sum operation on the norm_data
    CUDPPHandle rowPlan;
    /// The CUDPP configuration handle for a parallel prefix sum operation on the trans_data
    CUDPPHandle colPlan;
    
    /// The width of the input image (pixels)
    int m_width;
    /// The height of the input image (pixels)
    int m_height;
  };

} // namespace asrl

#endif // ASRL_GPU_INTEGRAL_IMAGE_PROCESSOR_HPP

