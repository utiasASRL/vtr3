/**
 * BSD 3-Clause License
 * 
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 *
 * Verbatim port of ouster_client AutoExposure — standalone (no SDK dependency).
 * 
 * Original source:
 * [1] https://github.com/ouster-lidar/ouster-sdk/blob/master/ouster_client/include/ouster/image_processing.h
 * [2] https://github.com/ouster-lidar/ouster-sdk/blob/master/ouster_client/src/image_processing.cpp
 * 
 */

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace vtr {

template <typename T>
using img_t = Eigen::Array<T, -1, -1, Eigen::RowMajor>;

namespace lidar {

class AutoExposure {
    static constexpr double ae_damping            = 0.90;
    static constexpr int    ae_default_update_every = 3;
    static constexpr size_t ae_stride             = 4;
    static constexpr size_t ae_min_nonzero_points = 100;
    static constexpr double ae_default_percentile = 0.1;

    const double lo_percentile;
    const double hi_percentile;
    const int    ae_update_every;

    double lo_state   = -1.0;
    double hi_state   = -1.0;
    double lo         = -1.0;
    double hi         = -1.0;
    bool   initialized = false;
    int    counter     = 0;

    template <typename T>
    void update(Eigen::Ref<img_t<T>> image, bool update_state) {
        Eigen::Map<Eigen::Array<T, -1, 1>> key_eigen(image.data(), image.size());

        if (counter == 0 && update_state) {
            const size_t n = static_cast<size_t>(key_eigen.rows());
            std::vector<size_t> indices;
            indices.reserve(n);
            for (size_t i = 0; i < n; i += ae_stride) {
                if (key_eigen[i] > 0) indices.push_back(i);
            }
            if (indices.size() < ae_min_nonzero_points) return;

            auto cmp = [&](size_t a, size_t b) {
                return key_eigen(a) < key_eigen(b);
            };

            const size_t lo_kth = static_cast<size_t>(indices.size() * lo_percentile);
            std::nth_element(indices.begin(), indices.begin() + lo_kth, indices.end(), cmp);
            lo = key_eigen[*(indices.begin() + lo_kth)];

            const size_t hi_kth = static_cast<size_t>(indices.size() * hi_percentile);
            std::nth_element(indices.begin() + lo_kth,
                             indices.end() - hi_kth - 1, indices.end(), cmp);
            hi = key_eigen[*(indices.end() - hi_kth - 1)];

            if (!initialized) {
                initialized = true;
                lo_state    = lo;
                hi_state    = hi;
            }
        }
        if (!initialized) return;

        if (update_state) {
            lo_state = ae_damping * lo_state + (1.0 - ae_damping) * lo;
            hi_state = ae_damping * hi_state + (1.0 - ae_damping) * hi;
        }

        double lo_hi_scale = (1.0 - (lo_percentile + hi_percentile)) / (hi_state - lo_state);

        if (std::isinf(lo_hi_scale) || std::isnan(lo_hi_scale)) {
            key_eigen *= static_cast<T>(0.5 / hi_state);
        } else if (lo_hi_scale * (0.0 - lo_state) + lo_percentile <= 0.0) {
            key_eigen -= static_cast<T>(lo_state);
            key_eigen *= static_cast<T>(lo_hi_scale);
            key_eigen += static_cast<T>(lo_percentile);
        } else {
            key_eigen *= static_cast<T>((1.0 - hi_percentile) / hi_state);
        }

        key_eigen = key_eigen.max(static_cast<T>(0)).min(static_cast<T>(1));

        if (update_state) counter = (counter + 1) % ae_update_every;
    }

   public:
    AutoExposure()
        : lo_percentile(ae_default_percentile),
          hi_percentile(ae_default_percentile),
          ae_update_every(ae_default_update_every) {}

    explicit AutoExposure(int update_every)
        : lo_percentile(ae_default_percentile),
          hi_percentile(ae_default_percentile),
          ae_update_every(update_every) {}

    AutoExposure(double lo_pct, double hi_pct, int update_every)
        : lo_percentile(lo_pct),
          hi_percentile(hi_pct),
          ae_update_every(update_every) {}

    void operator()(Eigen::Ref<img_t<float>>  image, bool update_state = true) {
        update(image, update_state);
    }
    void operator()(Eigen::Ref<img_t<double>> image, bool update_state = true) {
        update(image, update_state);
    }
};

}  // namespace lidar
}  // namespace vtr
