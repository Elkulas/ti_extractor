#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <locale.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

#include <opencv2/highgui/highgui.hpp>
#include <tuple>

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/Pose.h"
#include "math.h"

namespace ti {

// argu
std::string out_root = "/home/jjj/NGCLAB/catkin_ws/src/ti_extractor/data";
std::tuple<std::string, std::string> outPaths;
// func 1
// argument parsing
void parseArgument(char *arg) {
  int option;
  char buf[1000];

  if (1 == sscanf(arg, "out_root=%s", buf)) {
    out_root = buf;
    printf("output path %s!\n", out_root.c_str());
    return;
  } else {
    printf("no out_root path !!\n");
    exit(1);
  }

  printf("could not parse argument \"%s\"!!\n", arg);
}

void CreateFolder(const std::string &path) {
  const std::string path_make = "mkdir -p " + path;
  const std::string path_rm = "rm -r " + path;
  const int rm_err = system(path_rm.c_str());
  const int err = system(path_make.c_str());
  if (err == -1) {
    std::cout << "0 CreateFolder: can't create " << path;
  }
  std::cout << "1 CreateFolder: Created! " << path << std::endl;

}

void constructFolder() {
  outPaths = std::make_tuple(
      out_root + "thermal/", out_root + "thermalRaw/");
  std::cout << "1 constructFolder: save in " << out_root << std::endl;
  std::cout << "1 constructFolder: save rgb in " << std::get<0>(outPaths) << std::endl;
  CreateFolder(std::get<0>(outPaths));
  CreateFolder(std::get<1>(outPaths));
}

void imuCb(const sensor_msgs::Imu &msg) {
  double time = msg.header.stamp.toSec();

  FILE *fab;
  char buf[1000];
  snprintf(buf, 1000, "%s/imu.txt", out_root.c_str());
  fab = fopen(buf, "a");

  fprintf(fab, "%lf, %lf, %lf, %lf, %lf, %lf, %lf\n", time,
          msg.angular_velocity.x, msg.angular_velocity.y,
          msg.angular_velocity.z, msg.linear_acceleration.x,
          msg.linear_acceleration.y, msg.linear_acceleration.z);
  fclose(fab);
}

void thermalCb(const sensor_msgs::ImageConstPtr img) {
  // thermal 
  cv_bridge::CvImagePtr right_cv_ptr =
      cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
  cv::Mat lwir_image = right_cv_ptr->image.clone();

  assert(lwir_image.type() == CV_16UC1);
  assert(lwir_image.channels() == 1);
  // process thermal
  cv::Mat m;
  cv::Mat v;
  cv::meanStdDev(lwir_image, m, v);
  double maxinm = 0.0;
  double mininm = 0.0;
  cv::minMaxIdx(lwir_image, &mininm, &maxinm);
  std::cout << "1 thermalCb: Optris mean, std, max, min:" << m << "\t" << v << "\t" << maxinm
            << "\t" << mininm << std::endl;

  double min = mininm;
  double max = maxinm;

  double alpha = (255.0f) / (max - min);

  for (int i = 0; i < (lwir_image).rows; ++i) {
    for (int j = 0; j < (lwir_image).cols; ++j) {
      double x = (double)(lwir_image.at<ushort>(i, j)) - min;
      if (x < 0.0f) {
        lwir_image.at<ushort>(i, j) = 0;
      } else {
        if (x > max) {
          lwir_image.at<ushort>(i, j) = 255;
        } else {
          lwir_image.at<ushort>(i, j) = (ushort)alpha * x;
        }
      }
    }
  }

  double right_time = right_cv_ptr->header.stamp.toSec();

  lwir_image.convertTo(lwir_image, CV_8UC1);

  char bufoptris[1000];
  snprintf(bufoptris, 1000, "%s/%lf.png", std::get<0>(outPaths).c_str(),
           right_cv_ptr->header.stamp.toSec());
  imwrite(bufoptris, lwir_image);

  char bufoptris_raw[1000];
  snprintf(bufoptris_raw, 1000, "%s/%lf.png", std::get<1>(outPaths).c_str(),
           right_cv_ptr->header.stamp.toSec());
  imwrite(bufoptris_raw, right_cv_ptr->image);



}




}