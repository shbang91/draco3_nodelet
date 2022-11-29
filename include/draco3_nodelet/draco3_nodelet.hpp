#include <ros/ros.h>
#include <std_msgs/String.h>

#include <nodelet/nodelet.h>

#include <apptronik_msgs/Float32Stamped.h>
#include <apptronik_srvs/Bool.h>
#include <apptronik_srvs/Float32.h>
#include <apptronik_srvs/Int8.h>
#include <cortex_utils/debug_interfacer.hpp>
#include <rt_utils/synchronizer.hpp>

#include <cassert>
#include <iostream>
#include <stdexcept>

#include "configuration.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_interface.hpp"
#include "util/clock.hpp"
#include "util/util.hpp"

namespace draco3_nodelet {

namespace control_mode {
constexpr int kOff = 0;
constexpr int kMotorCurrent = 1;
constexpr int kJointImpedance = 2;
} // namespace control_mode

class Draco3Nodelet : public nodelet::Nodelet {
public:
  Draco3Nodelet();
  ~Draco3Nodelet();
  void onInit();
  void spinThread();

private:
  // methods
  void _LoadConfigFile();
  void _InitializePlaceHolders();

  void _RegisterData();
  void _SetGainsAndLimits();
  void _ClearFaults();
  void _TurnOffMotors();
  void _TurnOnMotorCurrent();
  void _TurnOnJointImpedance();
  void _TurnOnTargetJointImpedance();
  void _ConstructRPC();
  void _DestructRPC();

  void _ProcessServiceCalls();
  void _ExecuteSafeCommand();

  void _CopyData();
  void _CopyCommand();

  // service call callback functions
  void _Callback(const std_msgs::String::ConstPtr &input);
  bool _FaultHandlerCallback(apptronik_srvs::Bool::Request &req,
                             apptronik_srvs::Bool::Response &res);
  bool _FakeEstopHandlerCallback(apptronik_srvs::Bool::Request &req,
                                 apptronik_srvs::Bool::Response &res);
  bool _MotorModeHandlerCallback(apptronik_srvs::Int8::Request &req,
                                 apptronik_srvs::Int8::Response &res);
  bool _RPCHandlerCallback(apptronik_srvs::Int8::Request &req,
                           apptronik_srvs::Int8::Response &res);
  bool _GainsAndLimitsHandlerCallback(apptronik_srvs::Bool::Request &req,
                                      apptronik_srvs::Bool::Response &res);

  template <class SrvType>
  void _CallGetService(const std::string &slave_name,
                       const std::string &srv_name, SrvType &srv_obj);
  template <class SrvType>
  void _CallSetService(const std::string &slave_name,
                       const std::string &srv_name, SrvType &srv_obj);

  // RT kernel
  ros::NodeHandle nh_;
  boost::shared_ptr<aptk::comm::Synchronizer> sync_;
  boost::shared_ptr<boost::thread> spin_thread_;
  boost::shared_ptr<aptk::util::DebugInterfacer> debug_interface_;

  // publisher, subscriber, and callback function for testing nodelet
  ros::Publisher pub_;
  ros::Subscriber sub_;

  // service
  ros::ServiceServer fault_handler_;
  ros::ServiceServer fake_estop_handler_;
  ros::ServiceServer motor_mode_handler_;
  ros::ServiceServer rpc_handler_;
  ros::ServiceServer gains_limits_handler_;

  // RegisterData
  std::vector<std::string> axons_;
  std::vector<std::string> medullas_;
  std::vector<std::string> sensillums_;
  std::vector<int> pinocchio_robot_jidx_;

  int num_joints_;
  int num_medullas_;
  int num_sensillums_;

  // placeholders for MISO data
  std::vector<float *> ph_jpos_data_; // joint positions encoder data
  std::vector<float *> ph_jvel_data_; // joint velocities encoder data
  std::vector<float *> ph_mpos_data_; // motor positions encoder data
  std::vector<float *>
      ph_linkage_speed_ratio_data_; // linkage speed ratio data(

  float *ph_imu_quat_w_ned_, *ph_imu_quat_x_ned_, *ph_imu_quat_y_ned_,
      *ph_imu_quat_z_ned_;
  float *ph_imu_ang_vel_x_, *ph_imu_ang_vel_y_, *ph_imu_ang_vel_z_;
  float *ph_imu_dvel_x_, *ph_imu_dvel_y_, *ph_imu_dvel_z_;

  float *ph_rfoot_sg_, *ph_lfoot_sg_;

  // placeholders for MOSI data
  std::vector<float *> ph_jpos_cmd_;
  std::vector<float *> ph_jvel_cmd_;
  std::vector<float *> ph_jtrq_cmd_;
  std::vector<float *> ph_current_cmd_;
  std::vector<float *> ph_kp_cmd_;
  std::vector<float *> ph_kd_cmd_;

  // misc
  std::vector<float> actuator_speed_ratio_;
  std::vector<float> motor_pos_polarity_;

  // debug interface variables (for plotting)
  double ctrl_computation_time_;
  Eigen::Quaternion<double> Q_0_imu_; // imu variables
  Eigen::Vector3d ang_vel_0_imu_;     // imu variables
  Eigen::VectorXd diff_jpos_mjpos_;

  // nodelet related params
  std::string target_joint_name_;
  float contact_threshold_;
  double sleep_time_;
  bool b_measure_computation_time_;

  int motor_control_mode_;

  bool b_clear_faults_;
  bool b_fake_estop_released_;
  bool b_motor_off_mode_;
  bool b_motor_current_mode_;
  bool b_joint_impedance_mode_;
  bool b_construct_rpc_;
  bool b_destruct_rpc_;
  bool b_set_motor_gains_limits_;
  bool b_target_joint_impedance_mode_;

  bool b_rpc_alive_;

  // clock instance
  Clock clock_;

  // YAML node
  YAML::Node nodelet_cfg_;
  YAML::Node rpc_cfg_;

  // rpc stuff
  DracoInterface *draco_interface_;
  DracoSensorData *draco_sensor_data_;
  DracoCommand *draco_command_;
  bool b_sim_;
};

} // namespace draco3_nodelet
