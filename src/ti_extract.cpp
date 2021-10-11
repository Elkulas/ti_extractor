#include "callbacker.cpp"

using namespace ti;

int main(int argc,char **argv) {
  ros::init(argc, argv ,"ti_live");

  for (int i = 1; i < argc; i++) parseArgument(argv[i]);

  if (out_root.at(out_root.length() - 1) != '/') out_root = out_root + "/";

  std::cout <<  "1 main: outpath: " << out_root << std::endl;

  constructFolder();
  const std::string imu_rm = "rm " + out_root + "imu.txt";
  const int rm_err = system(imu_rm.c_str());
  const std::string time_rm = "rm " + out_root + "time.txt";
  const int time_err = system(time_rm.c_str());

  ros::NodeHandle nh("~");
  // std::string imagetopic = "/camera/image_raw";
  std::string thermalTopic = "/tau_nodelet/thermal_image";
  std::string imuTopic = "/vn100/imu";

  // subscribe
  ros::Subscriber imuSub = nh.subscribe(imuTopic, 1, &imuCb);
  ros::Subscriber ThermalSub = nh.subscribe(thermalTopic, 10, &thermalCb);

  // ros::Rate loop_rate(10);

  ROS_INFO("Hello World!");

  ros::spin();
  return 0;
}