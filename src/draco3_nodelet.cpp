#include <draco3_nodelet/draco3_nodelet.hpp>

namespace draco3_nodelet {
Draco3Nodelet::Draco3Nodelet() { draco_interface_ = new DracoInterface(); }

Draco3Nodelet::~Draco3Nodelet() {
  spin_thread_->join();
  delete draco_interface_;
}

void Draco3Nodelet::onInit() {
  ROS_INFO("Draco3Nodelet::onInit()");
  spin_thread_.reset(
      new boost::thread(boost::bind(&Draco3Nodelet::spinThread, this)));
}

void Draco3Nodelet::spinThread() {
  sync_.reset(new aptk::comm::Synchronizer(true, "draco3_nodelet"));
  debug_interface_.reset(new aptk::util::DebugInterfacer(
      "draco3", sync_->getNodeHandle(), sync_->getLogger()));
  // TODO:
  // add debug variables for the nodelet memeber variables

  nh_ = getNodeHandle();
  // Create a publisher topic
  pub_ = nh_.advertise<std_msgs::String>("ros_out", 10);
  // Create a subscriber topic
  sub_ = nh_.subscribe("ros_in", 10, &Draco3Nodelet::Callback, this);

  aptk::comm::enableRT(5, 2);

  // TODO
  // RegisterData
  // SetGainAndLimits
  // TurnOffMotors
  // ClearFaults

  sync_->connect();
  while (sync_->ok()) {
    sync_->awaitNextControl(); // wait for bus transaction
    // TODO
    // ProcessServiceCalls
    // CopyData
    // GetCommand
    // CopyCommand
    ROS_INFO("I'm in control loop");
    sync_->finishControl(); // indicate that we're done
  }
  sync_->awaitShutdownComplete();
}

void Draco3Nodelet::Callback(const std_msgs::String::ConstPtr &input) {
  std_msgs::String output;
  output.data = input->data;
  ROS_INFO("msg data = %s", output.data.c_str());
  pub_.publish(output);
}

} // namespace draco3_nodelet

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(draco3_nodelet::Draco3Nodelet, nodelet::Nodelet);
