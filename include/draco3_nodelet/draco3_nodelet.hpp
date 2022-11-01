#include <ros/ros.h>
#include <std_msgs/String.h>

#include <nodelet/nodelet.h>

#include <apptronik_msgs/Float32Stamped.h>
#include <apptronik_srvs/Float32.h>
#include <cortex_utils/debug_interfacer.hpp>
#include <rt_utils/synchronizer.hpp>

#include "configuration.hpp"
#include "controller/draco_controller/draco_interface.hpp"

namespace draco3_nodelet {

class Draco3Nodelet : public nodelet::Nodelet {
public:
  Draco3Nodelet();
  ~Draco3Nodelet();
  void onInit();
  void spinThread();

private:
  // RT kernel
  ros::NodeHandle nh_;
  boost::shared_ptr<aptk::comm::Synchronizer> sync_;
  boost::shared_ptr<boost::thread> spin_thread_;
  boost::shared_ptr<aptk::util::DebugInterfacer> debug_interface_;

  // publisher, subscriber, and callback function for testing nodelet
  ros::Publisher pub_;
  ros::Subscriber sub_;
  void Callback(const std_msgs::String::ConstPtr &input);

  DracoInterface *draco_interface_;
};

} // namespace draco3_nodelet
