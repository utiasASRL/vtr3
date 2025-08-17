#include "utils.hpp"
#include <stdexcept>
#include <fstream>

namespace vtr {
namespace radar {

namespace utils {

using torch::indexing::Slice;

torch::Tensor dopplerUpDown(const RadarFrame& rf) {
    // Compute base path by removing last directory
    auto pos = rf.sensor_root.find_last_of('/');
    std::string base = (pos != std::string::npos)
        ? rf.sensor_root.substr(0, pos)
        : rf.sensor_root;
    std::string doppler_path = base + "/radar/" + rf.frame;

    cv::Mat img = cv::imread(doppler_path, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        throw std::runtime_error("Failed to load image: " + doppler_path);
    }

    // Extract column 10 as 1D tensor
    cv::Mat col = img.col(10).clone();
    auto options = torch::TensorOptions().dtype(torch::kUInt8);
    torch::Tensor t = torch::from_blob(
        col.data, {col.rows}, options
    ).to(torch::kInt32).clone();
    return t;
}

bool checkChirp(const RadarFrame& rf) {
    torch::Tensor up_chirps = dopplerUpDown(rf);
    // Check even indices == 255 and odd indices == 0
    auto evens = up_chirps.index({torch::arange(0, up_chirps.size(0), 2)});
    auto odds  = up_chirps.index({torch::arange(1, up_chirps.size(0), 2)});
    bool even_ok = evens.eq(255).all().item<bool>();
    bool odd_ok  = odds.eq(0).all().item<bool>();
    // If pattern invalid, return true to indicate chirp_up (needs flip)
    return !(even_ok && odd_ok);
}

torch::Tensor getGaussianKernel2D(int ksize_x, int ksize_y, double sigma_x, double sigma_y, torch::Device device) 
{
    int half_x = ksize_x / 2;
    int half_y = ksize_y / 2;

    auto x = torch::arange(-half_x, half_x + 1, torch::TensorOptions().dtype(torch::kFloat32).device(device));
    auto y = torch::arange(-half_y, half_y + 1, torch::TensorOptions().dtype(torch::kFloat32).device(device));

    auto xx = x.pow(2).div(2 * sigma_x * sigma_x).unsqueeze(1);  // shape (kx, 1)
    auto yy = y.pow(2).div(2 * sigma_y * sigma_y).unsqueeze(0);  // shape (1, ky)

    auto kernel = torch::exp(-(xx + yy));  // shape (kx, ky)
    kernel /= kernel.sum();  // normalize

    return kernel;
}

torch::Tensor applyGaussianBlur2D(const torch::Tensor& input, int kx, int ky, double sx, double sy) 
{
    torch::NoGradGuard no_grad;
    if (kx % 2 == 0 || ky % 2 == 0) {
        throw std::invalid_argument("Kernel size must be odd");
    }
    auto kernel = getGaussianKernel2D(kx, ky, sx, sy, input.device());
    kernel = kernel.view({1, 1, ky, kx});  // Conv2d expects [out_channels, in_channels, H, W]

    auto conv = torch::nn::Conv2d(torch::nn::Conv2dOptions(1, 1, {ky, kx}).padding({ky / 2, kx / 2}).bias(false));
    conv->weight.set_data(kernel.clone());  // clone to detach from computation graph
    conv->to(input.device());
    return conv->forward(input);
}

Eigen::MatrixXd loadCsv(const std::string& filename,
                               char delimiter,
                               int skiprows) {
    std::ifstream in(filename);
    if (!in.is_open())
        throw std::runtime_error("Could not open CSV file: " + filename);

    std::vector<std::vector<double>> data;
    std::string line;
    // Skip header
    for (int i = 0; i < skiprows && std::getline(in, line); ++i) {}

    // Read data lines
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string cell;
        std::vector<double> row;
        while (std::getline(ss, cell, delimiter)) {
            row.push_back(std::stod(cell));
        }
        if (!data.empty() && row.size() != data[0].size())
            throw std::runtime_error("Inconsistent column count in " + filename);
        data.push_back(std::move(row));
    }
    in.close();

    if (data.empty())
        return Eigen::MatrixXd();

    // Build Eigen matrix
    size_t rows = data.size();
    size_t cols = data[0].size();
    Eigen::MatrixXd mat(rows, cols);
    for (size_t i = 0; i < rows; ++i)
        for (size_t j = 0; j < cols; ++j)
            mat(i, j) = data[i][j];
    return mat;
}

Eigen::Isometry3d loadIsometry3dFromFile(const std::string& filename) {
    std::ifstream in(filename);
    if (!in.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    Eigen::Matrix4d mat;
    std::string line;
    for (int row = 0; row < 4; ++row) {
        if (!std::getline(in, line)) {
            throw std::runtime_error("Unexpected end of file in " + filename);
        }
        std::istringstream ss(line);
        for (int col = 0; col < 4; ++col) {
            float v;
            if (!(ss >> v)) {
                throw std::runtime_error("Failed to parse float at row " +
                                          std::to_string(row) + ", col " +
                                          std::to_string(col) + " in " + filename);
            }
            mat(row, col) = v;
        }
    }

    Eigen::Isometry3d iso(mat);
    return iso;
}

RadarData loadRadarData(
    const fs::path &filename,
    int encoder_size,
    int min_id) 
{
    // get filename from path
    double timestamp = std::stod(filename.stem().string()) * 1.0e-6;

    // 1) Read as 8‑bit grayscale
    cv::Mat img = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        throw std::runtime_error("Failed to load image: " + filename.string());
    }

    int H = img.rows;
    int W = img.cols;
    int polarW = W - 11;
    if (polarW <= 0) {
        throw std::runtime_error("Image too narrow for polar data after skipping");
    }

    // 2) Allocate output tensors on CPU
    torch::Tensor timestamps = torch::empty({H}, torch::kFloat64);
    torch::Tensor azimuths  = torch::empty({H}, torch::kFloat64);
    torch::Tensor polar      = torch::empty({H, polarW}, torch::kFloat32);

    // 3) Fill in each row
    for (int i = 0; i < H; ++i) {
        const uint8_t* row = img.ptr<uint8_t>(i);
        // // --- timestamps: bytes 0..7 as uint64, then *1e‑3
        // uint64_t ts = 0;
        // for (int b = 0; b < 8; ++b) {
        //     ts |= uint64_t(row[b]) << (8 * b);
        // }
        // timestamps[i] = static_cast<double>(ts);
        // // --- azimuth: bytes 8..9 as uint16 a, then /encoder_size
        // uint16_t enc = uint16_t(row[8]) | (uint16_t(row[9]) << 8);
        // float az = static_cast<float>(enc) * 2.0f * float(M_PI) / float(encoder_size);
        // azimuths[i] = az;
        // --- polar: bytes 11+min_id ... W-1, normalized [0,1]
        // timestamps[i] = *((int64_t *)(row));
        // azimuths[i] = *((uint16_t *)(row + 8)) * 2.0f * float(M_PI) / double(encoder_size);
        // for (int j = 0; j < polarW; ++j) {
        //     polar[i][j] = float(row[11 + min_id + j]) / 255.0f;
        // }

        int64_t ts = 0;
        for (int j = 0; j < 8; ++j) {
            ts |= static_cast<int64_t>(row[j]) << (j * 8);
        }
        timestamps[i] = static_cast<int64_t>(ts);

        uint16_t az_raw = static_cast<uint16_t>(row[8]) |
                            (static_cast<uint16_t>(row[9]) << 8);
        double angle = static_cast<float>(az_raw) / encoder_size * 2.0f * static_cast<float>(M_PI);
        azimuths[i] = angle;

        for (int j = 0; j < polarW; ++j) {
            float value = static_cast<float>(row[j + 11]) / 255.0f;
            polar[i][j] = value;
        }

    }

    polar.index_put_({Slice(), Slice(0, min_id)}, torch::zeros({H, min_id}, torch::TensorOptions().dtype(torch::kFloat64)));

    RadarData radar_data;
    radar_data.timestamps = timestamps;
    radar_data.azimuths  = azimuths;
    radar_data.polar     = polar;
    radar_data.timestamp = timestamp;

    return radar_data;
}

} // namespace utils

}  // namespace radar
}  // namespace vtr