#include <electric_sheep/data_player.hpp>

int main(int argc, char *argv[]) {
  // Default path
  fs::path data_dir{fs::current_path()};
  std::string stream_name = "gps";

  // temp
  int replay_mode = 0;
  int start_index = 1;
  int stop_index = 9999999;
  double delay_scale = 1.0;  // make playblack slower
  double time_shift = 0;     // shift time stamp for repeat
  int frame_skip = 1;

  // User specified path
  if (argc >= 3) {
    data_dir = argv[1];
    stream_name = argv[2];
    if (argc >= 4) replay_mode = atoi(argv[3]);
    if (argc >= 5) start_index = atof(argv[4]);
    if (argc >= 6) stop_index = atof(argv[5]);
    if (argc >= 7) delay_scale = atof(argv[6]);
    if (argc >= 8) time_shift = atof(argv[7]);
    if (argc >= 9) frame_skip = atof(argv[8]);
  } else if (argc != 1) {
    throw std::invalid_argument("Wrong number of arguments provided!");
  }

  rclcpp::init(argc, argv);
  auto replay = DataPlayer<NavSatFixMsg>(data_dir.string(), stream_name, "gps",
                                         replay_mode, start_index, stop_index,
                                         delay_scale, time_shift, frame_skip);
  rclcpp::shutdown();
}
