#include <optional>
#include <vtr_radar/dr_ba/scans/local_map_scan.hpp>
#include <lgmath/se2/Operations.hpp>

namespace ba {

std::optional<Scan::Measurement> LocalMapScan::interpolate(double x, double y) const {
    if (local_map_.size() == 0) {
        throw std::runtime_error("Local map data not loaded for scan ID: " + std::to_string(id()));
    }
    // Check coverage first
    if (!check_coverage_at_point(x, y)) {
        return std::nullopt;
    }
    // Get pixel coordinates and Jacobian
    Eigen::Matrix<double, 2, 3> d_g_d_T;
    PixelCoords p = coord_to_pixel(x, y, &d_g_d_T);
    double u = p.first;
    double v = p.second;

    // Get root pixel coords
    Index root_px = get_root_pixel_coords(x, y);
    int a = root_px.first;
    int b = root_px.second;

    // Check value of cumulative image at this point
    if (cumul_img_.rows() != 0 && cumul_img_.cols() != 0) {
        // Threshold
        double cumul_value = cumul_img_(b, a);
        double root_intensity = local_map_(b, a);
        if (cumul_value > cumul_thresh_) {
            return std::nullopt;
        }
        if (root_intensity < 0.1 && cumul_value > zero_thresh_) {
            return std::nullopt;
        }
    }

    // Get intensities at four corners
    double int_ab = local_map_(b, a);
    double int_ab1 = local_map_(b + 1, a);
    double int_a1b = local_map_(b, a + 1);
    double int_a1b1 = local_map_(b + 1, a + 1);

    // Get weights
    double u_tilde = u - static_cast<double>(a);
    double v_tilde = v - static_cast<double>(b);
    double w0 = (1.0 - u_tilde) * (1.0 - v_tilde);
    double w1 = (1.0 - u_tilde) * v_tilde;
    double w2 = u_tilde * (1.0 - v_tilde);
    double w3 = u_tilde * v_tilde;

    // Bilinear interpolation
    double int_xy = w0 * int_ab + w1 * int_ab1 + w2 * int_a1b + w3 * int_a1b1;

    // Compute Jacobian
    double d_B_d_u = (1.0 - v_tilde) * (int_a1b - int_ab) + v_tilde * (int_a1b1 - int_ab1);
    double d_B_d_v = (1.0 - u_tilde) * (int_ab1 - int_ab) + u_tilde * (int_a1b1 - int_a1b);
    Eigen::Matrix<double, 1, 2> d_B_d_g;
    d_B_d_g << d_B_d_u, d_B_d_v;
    Eigen::Matrix<double, 1, 3> jacobian = d_B_d_g * d_g_d_T;

    // Compute covariance
    double raw_meas_cov = meas_std_ * meas_std_;
    double covariance = (w0 * w0 + w1 * w1 + w2 * w2 + w3 * w3) * raw_meas_cov;
    // Add range-dependent uncertainty
    PixelCoords img_coords = coord_to_image_coord(x, y);
    double range2 = std::pow(img_coords.first, 2) + std::pow(img_coords.second, 2);
    covariance += (range_factor_ * range_factor_) * range2;

    // Form measurement
    Measurement meas;
    meas.x = x;
    meas.y = y;
    meas.intensity = int_xy;
    meas.covariance = covariance;
    meas.jacobian = jacobian;

    return meas;
}

bool LocalMapScan::check_coverage_at_point(double x, double y) const {
    // Check if the four pixels surrounding (x, y) are within image bounds
    Index root_px = get_root_pixel_coords(x, y);
    int a = root_px.first;
    int b = root_px.second;
    return (a >= 0 && b >= 0 && a < (img_width_ - 1) && b < (img_height_ - 1));
}

LocalMapScan::PixelCoords LocalMapScan::coord_to_image_coord(double x, double y) const {
    Eigen::Matrix<double, 3, 3> pose2d_inv_mat = pose_.toSE2().inverse().matrix();
    Eigen::Matrix<double,  3, 1> p_hom = pose2d_inv_mat * Eigen::Vector3d(x, y, 1.0);
    return {p_hom(0), p_hom(1)};
}

LocalMapScan::PixelCoords LocalMapScan::coord_to_pixel(double x, double y, Eigen::Matrix<double, 2, 3> *jacobian) const {
    Eigen::Matrix<double, 2, 3> D;
    D << 0, 1/res_, 0,
         -1/res_, 0, 0;
    PixelCoords img_coords = coord_to_image_coord(x, y);
    Eigen::Matrix<double,  3, 1> p_hom = Eigen::Vector3d(img_coords.first, img_coords.second, 1.0);
    Eigen::Vector2d p = D * p_hom + 0.5 * Eigen::Vector2d(img_width_ - 1, img_height_ - 1);

    if (jacobian) {
        *jacobian = D * lgmath::se2::point2fs(p_hom.head<2>(), 1.0);
    }

    return {p(0), p(1)};
}

LocalMapScan::Index LocalMapScan::get_root_pixel_coords(double x, double y) const {
    PixelCoords p = coord_to_pixel(x, y);
    int px = static_cast<int>(std::floor(p.first));
    int py = static_cast<int>(std::floor(p.second));
    return {px, py};
}

} // namespace ba