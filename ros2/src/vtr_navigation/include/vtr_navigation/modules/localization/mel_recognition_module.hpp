#pragma once

#include <vtr_navigation/modules/base_module.hpp>
#include <vtr_messages/msg/features.hpp>
#include <vtr_messages/msg/exp_recog_status.hpp>
#include <vtr_vision/features/bow/sliding_window_bow_descriptor.hpp>

namespace vtr {
namespace navigation {

////////////////////////////////////////////////////////////////////////////////
/// @brief Match the current live view to a multi-experience map.
///
/// Builds a local Bag-of-Words vocabulary for each 'place' along the teach.
/// Describes each image using its local vocabulary, incrementally adding new words.
/// Compares the live view to past experiences using the Bag-of-Words descriptors.
////////////////////////////////////////////////////////////////////////////////
class MelRecognitionModule : public BaseModule {
 public:
  static constexpr auto type_str_ = "mel_recognition";

  // A binary suite-rig-channel-index mask (used to record tracked/worded landmarks)
  struct LandmarkMask { bool tracked; bool worded; };
  typedef std::vector<LandmarkMask> ChannelMask;
  typedef std::vector<ChannelMask> RigMask;
  typedef std::vector<RigMask> SuiteMask;
  // A lookup from landmark index to vocabulary word for VO tracks
  typedef std::map<vision::LandmarkId, vision::LandmarkId> BowLibrary;
  typedef std::shared_ptr<BowLibrary> BowLibraryPtr;
  // A collection of vertices to match against
  typedef std::map<uint32_t, std::list<Vertex::Ptr>> MapVertices;
  // A workaround two mismatched types (RigLandmarks and RigFrame), which should be the same
  typedef std::vector<const vision::RigLandmarks *> RigLandmarkConstPtrs;
  // Vision BOW typedefs
  typedef vision::SparseBOWDescriptor<vision::LandmarkId> SparseBow;
  typedef vision::SlidingWindowBOWDescriptor<vision::LandmarkId> SlidingWindowBow;

  /// The struct that holds bow-in-progress construction for a vertex.
  /// We store everything in here, so that we can process non-current vertices if necessary,
  /// and don't have to worry about conflicting member variables.
  struct BowConstruct {
    pose_graph::VertexId vid;             ///< The live vertex being processed
    SuiteMask matched_landmarks;          ///< Which landmarks have been dealt with (worded)?
    vision::SuiteFeatures vocabulary;     ///< Features that are new words (unmatched to vocabulary)
    BowLibrary obs2word;                  ///< Which landmarks mapped to which word (for future lookups through tracks)?
    vision::BowDescriptor bow;            ///< The final bow descriptor
  };

  /// Configuration for the module, generally set by the module factory.
  struct Config {

    /// How many vertices forward/back in time should we include in the vocabulary.
    /// Larger values will match against more vertices, which makes the vocabulary
    /// more robust to localization errors, but larger and slower.
    /// 5 gives us roughly 2 m tolerance on either side.
    int temporal_depth = 5;

    /// Enables debugging logs.
    bool verbose = true;

    /// Right now this is the only option, keep it out of the params.
    /// When we use vo-untracked features it's too noisy and slow.
    bool only_use_tracked = true;

    /// Use a sliding window Bag-of-Words (concatenated over temporal depth.)
    /// This basically unions the last *temporal_depth* single-image
    /// BoW descriptors, and provides filtering and robustness to misalignment.
    /// The descriptors are still relatively small, so comparison and unions are fast.
    bool sliding_window = true;

    // VOCABULARY //

    /// The vocabulary cluster size, beyond this a new word is created.
    double cluster_size = 0.12;
    /// Thether to compare octaves for dictionary lookups.
    bool compare_octave = true;
    /// Whether to compare the Laplacian bit for dictionary lookups.
    bool compare_laplacian = true;

    // INTERFACING //

    /// The number of experiences (including the privileged)
    /// that we should recommend for localization.
    int num_desired_experiences = 4;
    /// Whether we should broadcast our recommendations (are we enabled).
    bool in_the_loop = false;
  };

  /// The main entry point to running this module.
  /// See the module-level description for more info.
  virtual void run(QueryCache &qdata, MapCache &mdata,
                   const std::shared_ptr<const Graph> &graph);

  /// Currently we update the graph in run().
  /// TODO
  virtual void updateGraph(QueryCache &qdata, MapCache &mdata,
                           const std::shared_ptr<Graph> &graph, VertexId vid);

  /// Set the config for the module.
  void setConfig(
      std::shared_ptr<Config> &config ///< The config we'd like to use
  ) {
    config_ = config;
  }

 protected:

  /// Empty right now, no visualization :(.
  void visualizeImpl(QueryCache &, MapCache &,
                     const std::shared_ptr<const Graph> &) {}

 private:

  /// Initialize persistent member variables.
  /// @returns whether we should continue processing this frame
  bool initialize(
      QueryCache &qdata, ///< The query cache for the live frame
      MapCache &mdata    ///< The vo map cache for the live frame
  );

  /// Initialize the BoW construction container.
  /// Sets up rig/channel sizes, etc.
  static void initializeConstruct(
      const RigLandmarkConstPtrs &query_suite_lm,  ///< The query landmarks (to get rig/channel sizes)
      BowConstruct *construct_ptr                  ///< [in,out] The container to be initialized
  );

  /// Get the localization map that surrounds us by our temporal window
  MapVertices ///< The vertices that we'll localize against, sorted by run
  getMapVertices(
      const pose_graph::RCGraph::ConstPtr &graph,  ///< The graph we're searching
      const VertexId &root_id,                     ///< Where we're starting the search
      const VertexId &live_node                    ///< Our live node (so we don't add it to the map)
  );

  /// Follow VO tracks, and re-use word assignments for tracked features.
  /// This means we don't have to re-match features that have been tracked.
  bool followVoTracks(
      BowConstruct &live_construct,  ///< The BoW container that we're building
      MapCache &mdata,               ///< The vo map cache with vo tracks
      const std::shared_ptr<const Graph> &graph  ///< The graph is used to convert persistent ids
  );

  /// Match un-worded, newly-tracked features against the vocabulary in our local map.
  int ///< The total number of matches found in the map
  matchAgainstMap(
      const RigLandmarkConstPtrs &query_suite_lm,  ///< The query landmarks (all of them)
      BowConstruct *construct_ptr,                 ///< [in,out] The BoW construction container (knows which landmarks to match)
      const MapVertices &map_vertices,             ///< The local map to take the vocabulary from
      const std::shared_ptr<const Graph> &graph    ///< The graph used to convert to persistent ids
  );

  /// Match un-worded, newly-tracked features against the vocabulary of a particular vertex.
  int ///< The total number of matches found at this vertex
  matchVertex(
      const RigLandmarkConstPtrs &query_suite_lm,  ///< The query landmarks
      BowConstruct *construct_ptr,                 ///< [in,out] The BoW construction container (knows which landmarks to match)
      const Vertex::Ptr &map_vertex,               ///< The map vertex to match against
      const std::shared_ptr<const Graph> &graph    ///< The graph used to convert to persistent ids
  );

  /// Match un-worded, newly-tracked features against the vocabulary of a particular vertex's channel.
  int ///< The total number of matches we found on this channel
  matchChannel(
      const vision::Features &query_channel_lm,  ///< The query landmarks
      vision::LandmarkId query_ids,               ///< The rig and channel id we're processing
      BowConstruct *construct_ptr,               ///< [in,out] The BoW construction container (knows which landmarks to match)
      const vtr_messages::msg::Features &map_channel_vocab_msg,  ///< The vocabulary features for the channel (points to landmarks)
      vision::LandmarkId map_vocab_lm_id
  );

  /// Cluster un-worded, newly-tracked features into new vocabulary words for the vertex.
  void clusterUnmatchedObs(
      const RigLandmarkConstPtrs &query_suite_lm,  ///< The query landmarks
      BowConstruct *construct_ptr,    ///< [in,out] The BoW construction container (knows which landmarks to cluster)
      const std::shared_ptr<const Graph> &graph    ///< The graph used to convert to persistent ids
  );

  /// Convert the BoW descriptor to a sliding window descriptor.
  /// The sliding window builder is a member variable that accumulates new single-image descriptors,
  /// appends them to the windowed descriptor, and pops old single-image descriptors off when done.
  /// This function appends the new bow descriptor to the window.
  /// @returns the new windowed descriptor.
  const vision::BowDescriptor &slidingWindowify(
      const vision::BowDescriptor &live_bow ///< [in] convert the BoW descriptor to a sliding window descriptor
  );

  /// Compute the Bag-of-Words (BoW) cosine distance between the live query descriptor,
  /// and the BoW descriptors for each experience in the map
  ExperienceDifferences ///< The cosine distance (lower is better) for each experience
  recognizeExperience(
      const vision::BowDescriptor &query_bow,  ///< The query BOW descriptor
      uint32_t live_run_id,                     ///< The live run id (so we don't compare to ourselves)
      const MapVertices &map_vertices          ///< The vertices in the map to compare to, sorted by run
  ) const;

  /// Take the differences for each experience in the map, pick the top experiences we'll use,
  /// and add those to a set that will get passed to the localization pipeline
  RunIdSet  ///< The top scoring experiences that we'll use to localize against
  filterExperience(
      const ExperienceDifferences &differences ///< The cosine distance from comparing BoW descriptors
  ) const;

  /// Save results to a csv file for analysis
  void recordResults(
      const Vertex::Ptr &query_vertex,  ///< The live query vertex we're processing
      double run_time,                   ///< The module run time
      const ExperienceDifferences &differences, ///< The cosine distance for each experience
      const RunIdSet &recommended  ///< The recommended runs to use for localization
  );

  /// Algorithm configuration
  std::shared_ptr<Config> config_;

  /// Keep a history of the previous vertex.
  pose_graph::VertexId persistent_prev_id_;
  /// The previous observation-to-word map, for quick VO lookups.
  std::shared_ptr<BowLibrary> persistent_prev_obs2word_;
  /// The sliding window bag-of-words descriptor
  std::shared_ptr<SlidingWindowBow> window_bow_;
  /// The bow and vocabulary that we will save to the graph
  std::shared_ptr<BowConstruct> constructed_bow_to_save_;
  /// The status message to save to the graph
  vtr_messages::msg::ExpRecogStatus status_msg_to_save_;

  /// Results output file
  std::ofstream out_stream_;

  /// Keep track of whether the necessary ROSBag2 streams have been initialized
  bool streams_init_ = false;
};

}
}
