#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <vtr_common/timing/stopwatch.hpp>
#include <vtr_logging/logging_init.hpp>
#include <vtr_messages/msg/sensor_gps.hpp>
#include <vtr_messages/msg/time_stamp.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#include <vtr_storage/data_bubble.hpp>
#include <vtr_storage/data_stream_reader.hpp>
#include <vtr_storage/data_stream_writer.hpp>

#include <test_msgs/msg/basic_types.hpp>

namespace fs = std::filesystem;

/* Create the following graph:
 *   R0: 0 --- 1 --- 2 --- ... --- X
 * Store some dummy data in each vertex and then load the data back from
 * vertices.
 */
int main() {
  using namespace vtr::pose_graph;

  int min = 1e4, max = 6e6, mul = 4, num_trial = 3;  // change this!!
  fs::path result_dir{fs::temp_directory_path() / "vtr_pose_graph_test_result"};
  fs::remove_all(result_dir);  // make sure the directoy is empty.
  fs::create_directory(result_dir);
  std::ofstream read_time_file, write_time_file, data_size_file;
  read_time_file.open(fs::path{result_dir / "read_time.csv"});
  write_time_file.open(fs::path{result_dir / "write_time.csv"});
  data_size_file.open(fs::path{result_dir / "data_size.csv"});

  read_time_file << "numberofdata,";
  write_time_file << "numberofdata,";
  data_size_file << "numberofdata,";
  for (int num_read_write = min; num_read_write <= max; num_read_write *= mul) {
    read_time_file << num_read_write << ",";
    write_time_file << num_read_write << ",";
    data_size_file << num_read_write << ",";
  }
  read_time_file << "\n";
  write_time_file << "\n";
  data_size_file << "\n";

  for (int trial = 0; trial < num_trial; trial++) {
    read_time_file << "trial" << trial << ",";
    write_time_file << "trial" << trial << ",";
    data_size_file << "trial" << trial << ",";
    for (int num_read_write = min; num_read_write <= max;
         num_read_write *= mul) {
      vtr::common::timing::Stopwatch read_stopwatch;
      vtr::common::timing::Stopwatch write_stopwatch;

      fs::path working_dir{fs::temp_directory_path() / "vtr_pose_graph_test"};
      fs::remove_all(working_dir);  // make sure the directoy is empty.
      fs::path graph_index_file{"graph_index"};
      int robot_id{666};

      // Initialize pose graph
      std::unique_ptr<RCGraph> graph{
          new RCGraph((working_dir / graph_index_file).string(), 0)};

      // Add the first run
      auto run_id = graph->addRun(robot_id);

      // Register a data read&write stream named test_data.
      std::string stream_name = "test_data";
      graph->registerVertexStream<test_msgs::msg::BasicTypes>(run_id,
                                                              stream_name);

      // Add the first vertex
      auto stamp = vtr_messages::msg::TimeStamp();  // a custom ros2 message.
      stamp.nanoseconds_since_epoch = 0;
      graph->addVertex(stamp);
      for (int idx = 1; idx < num_read_write; ++idx) {
        // Add following edges and vertexes
        stamp.nanoseconds_since_epoch = idx;
        graph->addVertex(stamp);
        graph->addEdge(RCVertex::IdType(0, idx - 1), RCVertex::IdType(0, idx));
      }

      // Make some test data
      // The fake sensor data we use in this test,
      // `test_msgs::msg::BasicTypes{}`, has only 1 entry called
      // `float64_value`, which stores a `float64`.
      std::default_random_engine generator;
      std::uniform_real_distribution<double> distribution(0.0, 1.0);

      // Insert some dummy messages to each vertex
      for (int vertex_idx = 0; vertex_idx < num_read_write; ++vertex_idx) {
        RCVertex::IdType vertex_id(0, vertex_idx);
        auto vertex = graph->at(vertex_id);
        auto test_msg = test_msgs::msg::BasicTypes{};
        test_msg.float64_value = distribution(generator);
        // std::cout << "Store " << test_msg.float64_value << " into vertex "
        //           << vertex_id << std::endl;
        /// robochunk::std_msgs::TimeStamp stamp;
        auto stamp = vtr_messages::msg::TimeStamp();
        stamp.nanoseconds_since_epoch = vertex_idx;
        vertex->insert(stream_name, test_msg, stamp);
      }

      // Now save out the data for all but the last few vertices
      std::cout << "Saving data to disk" << std::endl;
      for (int vertex_idx = 0; vertex_idx < num_read_write; ++vertex_idx) {
        // access the vertex
        RCVertex::IdType vertex_id(0, vertex_idx);
        auto vertex = graph->at(vertex_id);
        write_stopwatch.start();
        vertex->write();
        write_stopwatch.stop();
        vertex->unload();
      }

      // Now load all the data back from disk.
      std::cout << "Loading data from disk" << std::endl;
      for (int vertex_idx = num_read_write - 1; vertex_idx >= 0; --vertex_idx) {
        // access the vertex
        RCVertex::IdType vertex_id(0, vertex_idx);
        auto vertex = graph->at(vertex_id);
        read_stopwatch.start();
        vertex->load(stream_name);
        read_stopwatch.stop();
        auto message =
            vertex->retrieveData<test_msgs::msg::BasicTypes>(stream_name, 0);
        // std::cout << "Vertex " << vertex_id << " has value "
        //           << (message->float64_value) << std::endl;
      }
      std::cout << "Finished: robot id: " << robot_id << " run id: " << run_id
                << std::endl;

      auto read_time = read_stopwatch.count<std::chrono::nanoseconds>();
      std::cout << "Total reading time: " << read_time << std::endl;
      std::cout << "Per reading time: " << read_time / num_read_write
                << std::endl;
      read_time_file << read_time << ",";

      auto write_time = write_stopwatch.count<std::chrono::nanoseconds>();
      std::cout << "Total writing time: " << write_time << std::endl;
      std::cout << "Per writing time: " << write_time / num_read_write
                << std::endl;
      write_time_file << write_time << ",";

      auto filesize = fs::file_size(fs::path{
          working_dir / "graph_index/run_000000/test_data/test_data_0.db3"});
      std::cout << "Total database size: " << filesize << std::endl;
      data_size_file << filesize << ",";

      // Cleanup
      fs::remove_all(working_dir);
    }
    read_time_file << "\n";
    write_time_file << "\n";
    data_size_file << "\n";
  }

  read_time_file.close();
  write_time_file.close();
  data_size_file.close();
  std::cout << "Done! Result stored into: " << result_dir << std::endl;
}
