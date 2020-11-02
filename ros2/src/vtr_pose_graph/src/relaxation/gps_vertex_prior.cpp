#if 0
#include <Eigen/Core>
#define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H 1
#include <proj_api.h>
#endif

#include <vtr_pose_graph/relaxation/gps_vertex_prior.hpp>

namespace vtr {
namespace pose_graph {
#if 0
constexpr char GpsVertexPrior::gpsStream[];

GpsVertexPrior::GpsVertexPrior(const Eigen::Matrix3d& cov,
                               const TransformType& T_gps_base, bool useAdj,
                               const LossFuncPtr& lossFunc)
    : noiseModel_(new NoiseModel(cov)),
      T_gps_base_(T_gps_base),
      lossFunc_(lossFunc),
      useAdj_(useAdj) {}

GpsVertexPrior::GpsVertexPrior(const NoiseModelPtr& noiseModel,
                               const TransformType& T_gps_base, bool useAdj,
                               const LossFuncPtr& lossFunc)
    : noiseModel_(noiseModel),
      T_gps_base_(T_gps_base),
      lossFunc_(lossFunc),
      useAdj_(useAdj) {}

void GpsVertexPrior::addCostTerms(const GraphPtr& graph,
                                  const VertexIdType& root,
                                  StateMapType& stateMap,
                                  CostTermPtr& costTerms,
                                  const Eval::Mask::Ptr& mask) {
  if (!graph->hasVertexStream(gpsStream)) {
    LOG(WARNING) << "Attempted to use GPS prior on a graph with no GPS data!";
    return;
  } else if (!graph->hasMap()) {
    LOG(WARNING) << "Attempted to use GPS prior on an uncalibrated map!";
    return;
  } else if (!graph->mapInfo().has_utm_zone()) {
    LOG(WARNING) << "Attempted to use GPS prior without a utm zone configured!";
    return;
  }

  graph->loadVertexStream(gpsStream);

  bool offset = (T_gps_base_.matrix() == TransformType().matrix());
  auto T_gps_base_eval =
      steam::se3::FixedTransformEvaluator::MakeShared(T_gps_base_);

  std::string pstr(
      "+proj=utm +ellps=WGS84 +datum=WGS84 +units=m +no_defs +zone=");
  pstr += std::to_string(graph->mapInfo().utm_zone());
  projPJ pj_utm;

  TransformType T_root_world = Eigen::Matrix<double, 6, 1>(
      graph->mapInfo().t_map_vtr().entries().data());
  Eigen::Vector3d r = T_root_world.r_ab_inb();
  size_t count = 0;

  if (!(pj_utm = pj_init_plus(pstr.c_str()))) {
    LOG(ERROR)
        << "[GpsVertexPrior::addCostTerms] Could not build UTM projection";
    return;
  }

  //  auto workingGraph = graph->getManualSubgraph();

  //  for(auto it = workingGraph->begin(); it != workingGraph->end(); ++it) {
  for (auto it = graph->begin(root, 0, mask); it != graph->end(); ++it) {
    auto data = it->v()->template retrieveData<MsgType>(gpsStream);

    // If this is a privileged run and we elected to use adjacent vertex GPS
    // data...
    if (useAdj_ && graph->run(it->v()->id().majorId())->isManual()) {
      for (auto&& jt : it->v()->spatialNeighbours()) {
        // Consider all spatial neighbours that aren't in the relaxation problem
        // themselves
        if (!mask->operator[](jt)) {
          // Lump their vertex data in with the original data.  The offset of
          // one keyframe is less than the GPS noise, and should end up being
          // random in the long term
          auto other_data =
              graph->at(jt)->template retrieveData<MsgType>(gpsStream);
          if (other_data.size() > 0) {
            data.reserve(data.size() + other_data.size());
            data.insert(data.end(), other_data.begin(), other_data.end());
          }
        }
      }
    }

    if (data.size() > 0) {
      if (++count % 3) {
        continue;
      }

      Eigen::Vector3d mean(0, 0, 0);
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      size_t N = 0;

      for (auto&& jt : data) {
        mean[0] += jt->latitude();
        mean[1] += jt->longitude();
        mean[2] += jt->altitude();

        if (jt->position_covariance_size() == 9) {
          N += 1;
          cov += Eigen::Map<const Eigen::Matrix3d>(
              jt->position_covariance().data());
        }
      }

      mean /= data.size();

      projUV src, res;
      src.u = mean(1) * DEG_TO_RAD;
      src.v = mean(0) * DEG_TO_RAD;
      res = pj_fwd(src, pj_utm);

      mean(0) = res.u;
      mean(1) = res.v;

      NoiseModelPtr model;

      //      if (N > 0) {
      //        model.reset(new NoiseModel(cov / N / N * 50));
      //      } else {
      model = noiseModel_;
      //      }

      steam::PositionErrorEval::Ptr errorFunc;
      auto eval = steam::se3::TransformStateEvaluator::MakeShared(
          stateMap[it->v()->id()]);

      if (offset) {
        errorFunc.reset(new steam::PositionErrorEval(
            mean - r, steam::se3::compose(T_gps_base_eval, eval)));
      } else {
        errorFunc.reset(new steam::PositionErrorEval(mean - r, eval));
      }

      steam::WeightedLeastSqCostTerm<3, 6>::Ptr cost;
      cost.reset(new steam::WeightedLeastSqCostTerm<3, 6>(errorFunc, model,
                                                          lossFunc_));
      costTerms->add(cost);
    }
  }

  graph->unloadVertexStream(gpsStream);
}
#endif
}  // namespace pose_graph
}  // namespace vtr