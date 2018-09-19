#include <kuuvef1_hw_interface/kuuvef1_hw_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tr1_hardware_interface");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(1);
  spinner.start();

  kuuvef1_hw_interface::Kuuvef1HardwareInterface kuuvef1(nh);

  ros::spin();

  return 0;
}
