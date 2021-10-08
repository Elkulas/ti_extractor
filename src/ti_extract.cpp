#include "callbacker.cpp"

using namespace ti;

int main(int argc,char **argv) {
  ros::init(argc, argv ,"ti_live");

  for (int i = 1; i < argc; i++) parseArgument(argv[i]);

  if (out_root.at(out_root.length() - 1) != '/') out_root = out_root + "/";

  std::cout <<  "1 main: outpath: " << out_root << std::endl;

  constructFolder();


  ros::NodeHandle nh("~");
  // std::string imagetopic = "/camera/image_raw";
  std::string thermalTopic = "/optris/thermal_image";
  std::string imuTopic = "/imu/data";

  // subscribe
  ros::Subscriber imuSub = nh.subscribe(imuTopic, 1, &imuCb);
  ros::Subscriber ThermalSub = nh.subscribe(thermalTopic, 10, &thermalCb);




  // ros::Rate loop_rate(10);

  ROS_INFO("Hello World!");

  ros::spin();
  return 0;
}