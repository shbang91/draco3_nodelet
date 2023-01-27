#include <draco3_nodelet/draco3_nodelet.hpp>

using namespace draco_joint;

namespace draco3_nodelet {
Draco3Nodelet::Draco3Nodelet() {
  ROS_INFO("Draco3Nodelet::Draco3Nodelet()");

  _LoadConfigFile();

  axons_ = {"Neck_Pitch",    "R_Hip_IE",      "R_Hip_AA",      "R_Hip_FE",
            "R_Knee_FE",     "R_Ankle_FE",    "R_Ankle_IE",    "L_Hip_IE",
            "L_Hip_AA",      "L_Hip_FE",      "L_Knee_FE",     "L_Ankle_FE",
            "L_Ankle_IE",    "L_Shoulder_FE", "L_Shoulder_AA", "L_Shoulder_IE",
            "L_Elbow",       "L_Wrist_Roll",  "L_Wrist_Pitch", "R_Shoulder_FE",
            "R_Shoulder_AA", "R_Shoulder_IE", "R_Elbow",       "R_Wrist_Roll",
            "R_Wrist_Pitch"};
  joint_idx_map_ = {
      {"Neck_Pitch", 0},     {"R_Hip_IE", 1},       {"R_Hip_AA", 2},
      {"R_Hip_FE", 3},       {"R_Knee_FE", 4},      {"R_Ankle_FE", 5},
      {"R_Ankle_IE", 6},     {"L_Hip_IE", 7},       {"L_Hip_AA", 8},
      {"L_Hip_FE", 9},       {"L_Knee_FE", 10},     {"L_Ankle_FE", 11},
      {"L_Ankle_IE", 12},    {"L_Shoulder_FE", 13}, {"L_Shoulder_AA", 14},
      {"L_Shoulder_IE", 15}, {"L_Elbow", 16},       {"L_Wrist_Roll", 17},
      {"L_Wrist_Pitch", 18}, {"R_Shoulder_FE", 19}, {"R_Shoulder_AA", 20},
      {"R_Shoulder_IE", 21}, {"R_Elbow", 22},       {"R_Wrist_Roll", 23},
      {"R_Wrist_Pitch", 24}};

  medullas_ = {"Medulla", "Medulla_V4"};
  sensillums_ = {"Sensillum_v2"};

  pinocchio_robot_jidx_ = {
      neck_pitch,    r_hip_ie,      r_hip_aa,   r_hip_fe,      r_knee_fe_jd,
      r_ankle_fe,    r_ankle_ie,    l_hip_ie,   l_hip_aa,      l_hip_fe,
      l_knee_fe_jd,  l_ankle_fe,    l_ankle_ie, l_shoulder_fe, l_shoulder_aa,
      l_shoulder_ie, l_elbow_fe,    l_wrist_ps, l_wrist_pitch, r_shoulder_fe,
      r_shoulder_aa, r_shoulder_ie, r_elbow_fe, r_wrist_ps,    r_wrist_pitch};

  num_joints_ = axons_.size();
  num_medullas_ = medullas_.size();
  num_sensillums_ = sensillums_.size();

  ph_jpos_data_.resize(num_joints_);
  ph_jvel_data_.resize(num_joints_);
  ph_mpos_data_.resize(num_joints_);
  ph_linkage_speed_ratio_data_.resize(num_joints_);

  ph_jpos_cmd_.resize(num_joints_);
  ph_jvel_cmd_.resize(num_joints_);
  ph_jtrq_cmd_.resize(num_joints_);
  ph_current_cmd_.resize(num_joints_);
  ph_kp_cmd_.resize(num_joints_);
  ph_kd_cmd_.resize(num_joints_);
  joint_kp_cmd_vector_.resize(num_joints_);
  joint_kd_cmd_vector_.resize(num_joints_);

  actuator_speed_ratio_.resize(num_joints_);
  motor_pos_polarity_.resize(num_joints_);

  _InitializePlaceHolders();

  ctrl_computation_time_ = 0.;
  Q_0_imu_.setIdentity();
  ang_vel_0_imu_.setZero();
  diff_jpos_mjpos_ = Eigen::VectorXd::Zero(num_joints_);

  motor_control_mode_ = control_mode::kOff;

  b_clear_faults_ = false;
  b_fake_estop_released_ = false;
  b_motor_off_mode_ = false;
  b_motor_current_mode_ = false;
  b_joint_impedance_mode_ = false;
  b_construct_rpc_ = false;
  b_destruct_rpc_ = false;
  b_set_motor_gains_limits_ = false;
  b_target_joint_impedance_mode_ = false;

  b_rpc_alive_ = true;
  clock_ = Clock();

  draco_interface_ = new DracoInterface();
  draco_sensor_data_ = new DracoSensorData();
  draco_command_ = new DracoCommand();
}

Draco3Nodelet::~Draco3Nodelet() {
  ROS_INFO("Draco3Nodelet::~Draco3Nodelet()");

  spin_thread_->join();

  for (int i = 0; i < num_joints_; i++) {
    delete ph_jpos_data_[i];
    delete ph_jvel_data_[i];
    delete ph_mpos_data_[i];
    delete ph_linkage_speed_ratio_data_[i];
    delete ph_jpos_cmd_[i];
    delete ph_jvel_cmd_[i];
    delete ph_jtrq_cmd_[i];
    delete ph_current_cmd_[i];
    delete ph_kp_cmd_[i];
    delete ph_kd_cmd_[i];
  }

  delete ph_imu_quat_w_ned_;
  delete ph_imu_quat_x_ned_;
  delete ph_imu_quat_y_ned_;
  delete ph_imu_quat_z_ned_;

  delete ph_imu_ang_vel_x_;
  delete ph_imu_ang_vel_y_;
  delete ph_imu_ang_vel_z_;

  delete ph_imu_dvel_x_;
  delete ph_imu_dvel_y_;
  delete ph_imu_dvel_z_;

  delete ph_rfoot_sg_;
  delete ph_lfoot_sg_;

  delete draco_interface_;
  delete draco_sensor_data_;
  delete draco_command_;
}

void Draco3Nodelet::onInit() {
  ROS_INFO("Draco3Nodelet::onInit()");

  spin_thread_.reset(
      new boost::thread(boost::bind(&Draco3Nodelet::spinThread, this)));
}

void Draco3Nodelet::spinThread() {
  ROS_INFO("Draco3Nodelet::spinThread()");

  sync_.reset(new aptk::comm::Synchronizer(true, "draco3_nodelet"));
  sync_->connect();

  debug_interface_.reset(new aptk::util::DebugInterfacer(
      "draco3", sync_->getNodeHandle(), sync_->getLogger()));

  //=============================================================
  // add debug variables for the nodelet memeber variables
  //=============================================================
  debug_interface_->addPrimitive(&ctrl_computation_time_,
                                 "rpc_computation_time");
  debug_interface_->addEigen(&Q_0_imu_.coeffs(), "Q_0_imu",
                             {"x", "y", "z", "w"});
  debug_interface_->addEigen(&ang_vel_0_imu_, "ang_vel_0_imu", {"x", "y", "z"});
  debug_interface_->addEigen(&diff_jpos_mjpos_, "diff_jpos_mpos", axons_);

  //=============================================================
  // TODO: comment out after testing
  //=============================================================
  nh_ = getNodeHandle();

  fault_handler_ = nh_.advertiseService(
      "/fault_handler", &Draco3Nodelet::_FaultHandlerCallback, this);
  fake_estop_handler_ = nh_.advertiseService(
      "/fake_estop_handler", &Draco3Nodelet::_FakeEstopHandlerCallback, this);
  motor_mode_handler_ = nh_.advertiseService(
      "/motor_mode_handler", &Draco3Nodelet::_MotorModeHandlerCallback, this);
  rpc_handler_ = nh_.advertiseService(
      "/rpc_handler", &Draco3Nodelet::_RPCHandlerCallback, this);
  gains_limits_handler_ = nh_.advertiseService(
      "/gains_limits_handler", &Draco3Nodelet::_GainsAndLimitsHandlerCallback,
      this);
  joint_gains_handler_ = nh_.advertiseService(
      "/joint_gains_handler", &Draco3Nodelet::_JointGainsHandlerCallback, this);

  // Create a publisher topic
  pub_ = nh_.advertise<std_msgs::String>("ros_out", 10);
  // Create a subscriber topic
  sub_ = nh_.subscribe("ros_in", 10, &Draco3Nodelet::_Callback, this);

  //=============================================================
  // main control loop
  //=============================================================
  aptk::comm::enableRT(5, 2);

  _RegisterData();

  _SetGainsAndLimits();

  _TurnOffMotors();

  _ClearFaults();

  while (sync_->ok()) {
    sync_->awaitNextControl(); // wait for bus transaction

    _ProcessServiceCalls(); // process interrupt service call

    _CopyData(); // copy data

    if (b_sim_)
      ROS_INFO(
          "Please do 1) DestructRPC -> 2) change b_sim in pnc.yaml to FALSE -> "
          "3) ConstructRPC -> 4) ResetGainsAndLimits"); // for safety

    if (sync_->printIndicatedFaults() && !b_fake_estop_released_) {
      _ExecuteSafeCommand();
      //_CopyJointGainsCommand();
    } else {
      if (motor_control_mode_ == control_mode::kMotorCurrent) {
        _ExecuteSafeCommand();
      } else {
        if (b_measure_computation_time_)
          clock_.Start();
        draco_interface_->GetCommand(draco_sensor_data_, draco_command_);
        if (b_measure_computation_time_) {
          clock_.Stop();
          ctrl_computation_time_ = clock_.duration();
        }
        _CopyCommand();
      }
    }
    sync_->finishControl(); // indicate that we're done
    debug_interface_->updateDebug();
  }
  sync_->awaitShutdownComplete();
}

void Draco3Nodelet::_RegisterData() {
  ROS_INFO("Draco3Nodelet::_RegisterData()");

  for (int i = 0; i < num_joints_; i++) {

    // for jpos debugging from absolute encoder and motor rel encoder
    sync_->getParam<float>("Actuator__Speed_ratio", actuator_speed_ratio_[i],
                           axons_[i]);
    motor_pos_polarity_[i] = (axons_[i] == "L_Ankle_IE") ? -1. : 1.;

    // register encoder data
    sync_->registerMISOPtr(ph_jpos_data_[i], "js__joint__position__rad",
                           axons_[i], false);
    sync_->registerMISOPtr(ph_jvel_data_[i], "js__joint__velocity__radps",
                           axons_[i], false);

    // TODO: to check motor drift issue (might not be needed later)
    sync_->registerMISOPtr(ph_mpos_data_[i], "motor__position__rad", axons_[i],
                           false);
    sync_->registerMISOPtr(ph_linkage_speed_ratio_data_[i],
                           "linkage__speedRatio", axons_[i], false);

    // register commands for joint impedance control mode
    sync_->registerMOSIPtr(ph_jpos_cmd_[i], "cmd__joint__position__rad",
                           axons_[i], false);
    sync_->registerMOSIPtr(ph_jvel_cmd_[i], "cmd__joint__velocity__radps",
                           axons_[i], false);
    sync_->registerMOSIPtr(ph_jtrq_cmd_[i], "cmd__joint__effort__nm", axons_[i],
                           false);
    sync_->registerMOSIPtr(ph_current_cmd_[i], "cmd__motor__effort__a",
                           axons_[i], false);
    sync_->registerMOSIPtr(ph_kp_cmd_[i], "gain__joint_impedance_kp__nmprad",
                           axons_[i], false);
    sync_->registerMOSIPtr(ph_kd_cmd_[i], "gain__joint_impedance_kd__nmsprad",
                           axons_[i], false);

    // for linkage table
    if (axons_[i] == "R_Ankle_FE")
      sync_->registerMOSIPtr(ph_jpos_data_[i], "ext__joint_position__rad",
                             "R_Ankle_IE", false);
    if (axons_[i] == "L_Ankle_FE")
      sync_->registerMOSIPtr(ph_jpos_data_[i], "ext__joint_position__rad",
                             "L_Ankle_IE", false);
  }

  // Register imu data
  sync_->registerMISOPtr(ph_imu_quat_w_ned_, "IMU__quaternion_w__mps2",
                         sensillums_[0], false);
  sync_->registerMISOPtr(ph_imu_quat_x_ned_, "IMU__quaternion_x__mps2",
                         sensillums_[0], false);
  sync_->registerMISOPtr(ph_imu_quat_y_ned_, "IMU__quaternion_y__mps2",
                         sensillums_[0], false);
  sync_->registerMISOPtr(ph_imu_quat_z_ned_, "IMU__quaternion_z__mps2",
                         sensillums_[0], false);
  sync_->registerMISOPtr(ph_imu_ang_vel_x_, "IMU__comp_angularRate_x__radps",
                         sensillums_[0], false);
  sync_->registerMISOPtr(ph_imu_ang_vel_y_, "IMU__comp_angularRate_y__radps",
                         sensillums_[0], false);
  sync_->registerMISOPtr(ph_imu_ang_vel_z_, "IMU__comp_angularRate_z__radps",
                         sensillums_[0], false);
  sync_->registerMISOPtr(ph_imu_dvel_x_, "IMU__dVel_x__rad", sensillums_[0],
                         false);
  sync_->registerMISOPtr(ph_imu_dvel_y_, "IMU__dVel_y__rad", sensillums_[0],
                         false);
  sync_->registerMISOPtr(ph_imu_dvel_z_, "IMU__dVel_z__rad", sensillums_[0],
                         false);
  // Register foot strain gauge sensor
  sync_->registerMISOPtr(ph_rfoot_sg_, "foot__sg__x", "R_Ankle_IE", false);
  sync_->registerMISOPtr(ph_lfoot_sg_, "foot__sg__x", "L_Ankle_IE", false);
}

void Draco3Nodelet::_SetGainsAndLimits() {
  ROS_INFO("Draco3Nodelet::_SetGainsAndLimits()");

  this->_LoadConfigFile(); // reload changed yaml node

  try {
    bool b_conservative =
        util::ReadParameter<bool>(nodelet_cfg_["service_call"], "conservative");

    apptronik_srvs::Float32 srv_float_current_limit;

    for (int i = 0; i < num_joints_; ++i) {
      if (b_conservative) {
        *(ph_kp_cmd_[i]) = util::ReadParameter<float>(
            nodelet_cfg_["service_call"][axons_[i]], "weak_kp");
        *(ph_kd_cmd_[i]) = util::ReadParameter<float>(
            nodelet_cfg_["service_call"][axons_[i]], "weak_kd");
        srv_float_current_limit.request.set_data = util::ReadParameter<float>(
            nodelet_cfg_["service_call"][axons_[i]],
            "weak_current_limit"); // for current limit srv call
      } else {
        *(ph_kp_cmd_[i]) = util::ReadParameter<float>(
            nodelet_cfg_["service_call"][axons_[i]], "kp");
        *(ph_kd_cmd_[i]) = util::ReadParameter<float>(
            nodelet_cfg_["service_call"][axons_[i]], "kd");
        srv_float_current_limit.request.set_data = util::ReadParameter<float>(
            nodelet_cfg_["service_call"][axons_[i]],
            "current_limit"); // for current limit srv call
      }
      joint_kp_cmd_vector_[i] = *(ph_kp_cmd_[i]);
      joint_kd_cmd_vector_[i] = *(ph_kd_cmd_[i]);
      _CallSetService(axons_[i],
                      "Limits__Motor__Effort__Saturate__Relative_val",
                      srv_float_current_limit);
      sleep(sleep_time_);
    }

    // for (int i = 0; i < lower_leg_axons_.size(); ++i) {
    // lb_low_level_kp_gains_[i] = util::ReadParameter<float>(
    // nodelet_cfg_["service_call"][lower_body_joint_names_[i]], "kp");
    // lb_low_level_kd_gains_[i] = util::ReadParameter<float>(
    // nodelet_cfg_["service_call"][lower_body_joint_names_[i]], "kd");
    //}
  } catch (std::runtime_error &ex) {
    std::cerr << "Error Readinig Parameter [" << ex.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }
} // namespace draco3_nodelet

void Draco3Nodelet::_ClearFaults() {
  ROS_INFO("Draco3Nodelet::_ClearFaults()");
  for (int i = 0; i < num_joints_; i++) {
    sync_->clearFaults(axons_[i]);
    sleep(sleep_time_);
  }
}

void Draco3Nodelet::_TurnOffMotors() {
  b_fake_estop_released_ = false;
  for (int i = 0; i < num_joints_; i++) {
    sync_->changeMode("OFF", axons_[i]);
    sleep(sleep_time_);
  }
}

void Draco3Nodelet::_TurnOnMotorCurrent() {
  for (int i = 0; i < num_joints_; i++) {
    sync_->changeMode("MOTOR_CURRENT", axons_[i]);
    sleep(1.0);
  }
}

void Draco3Nodelet::_TurnOnJointImpedance() {
  if (b_rpc_alive_) {
    for (int i = 0; i < num_joints_; i++) {
      sync_->changeMode("JOINT_IMPEDANCE", axons_[i]);
      sleep(sleep_time_);
    }
  } else
    ROS_INFO("[[WARNING]] rpc is not constructed. Please contstruct rpc");
}

void Draco3Nodelet::_TurnOnTargetJointImpedance() {
  b_fake_estop_released_ = true;
  if (b_rpc_alive_) {
    sync_->changeMode("JOINT_IMPEDANCE", target_joint_name_);
    sleep(sleep_time_);
  } else
    ROS_INFO("[[WARNING]] rpc is not constructed. Please contstruct rpc");
}

void Draco3Nodelet::_ConstructRPC() {
  if (!b_rpc_alive_) {
    draco_interface_ = new DracoInterface();
    b_rpc_alive_ = true;
  } else
    ROS_INFO("[[WARNING]] rpc is already constructed");
}

void Draco3Nodelet::_DestructRPC() {
  if (b_rpc_alive_) {
    delete draco_interface_;
    b_rpc_alive_ = false;
  } else
    ROS_INFO("[[WARNING]] rpc is already destructed");
}

void Draco3Nodelet::_LoadConfigFile() {
  try {
    nodelet_cfg_ = YAML::LoadFile(THIS_COM "config/draco/nodelet.yaml");
    rpc_cfg_ = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");

    target_joint_name_ =
        util::ReadParameter<std::string>(nodelet_cfg_, "target_joint_name");
    contact_threshold_ =
        util::ReadParameter<float>(nodelet_cfg_, "contact_threshold");
    sleep_time_ = util::ReadParameter<double>(nodelet_cfg_, "sleep_time");
    b_measure_computation_time_ =
        util::ReadParameter<bool>(nodelet_cfg_, "b_measure_computation_time");

    b_sim_ =
        util::ReadParameter<bool>(rpc_cfg_, "b_sim"); // this should be false

  } catch (const YAML::ParserException &ex) {
    std::cerr << "Error Readinig Parameter [" << ex.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;

  } catch (const std::runtime_error &ex) {
    std::cerr << "Error Readinig Parameter [" << ex.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }
}

void Draco3Nodelet::_ProcessServiceCalls() {
  if (b_clear_faults_) {
    _ClearFaults();
    b_clear_faults_ = false;
  }
  if (b_motor_off_mode_) {
    _TurnOffMotors();
    b_motor_off_mode_ = false;
  }
  if (b_motor_current_mode_) {
    _TurnOnMotorCurrent();
    b_motor_current_mode_ = false;
  }
  if (b_joint_impedance_mode_) {
    _TurnOnJointImpedance();
    b_joint_impedance_mode_ = false;
  }
  if (b_construct_rpc_) {
    _ConstructRPC();
    b_construct_rpc_ = false;
  }
  if (b_destruct_rpc_) {
    _DestructRPC();
    b_destruct_rpc_ = false;
  }
  if (b_set_motor_gains_limits_) {
    _SetGainsAndLimits();
    b_set_motor_gains_limits_ = false;
  }

  if (b_target_joint_impedance_mode_) {
    _TurnOnTargetJointImpedance();
    b_target_joint_impedance_mode_ = false;
  }
}

void Draco3Nodelet::_ExecuteSafeCommand() {
  for (int i = 0; i < num_joints_; i++) {
    *(ph_jpos_cmd_[i]) = *(ph_jpos_data_[i]);
    *(ph_jvel_cmd_[i]) = 0.;
    *(ph_jtrq_cmd_[i]) = 0.;
    *(ph_current_cmd_[i]) = 0.;
  }
}

void Draco3Nodelet::_CopyData() {

  //=============================================================
  // Process IMU data
  //=============================================================
  Eigen::Quaternion<double> Q_ned_imu(*ph_imu_quat_w_ned_, *ph_imu_quat_x_ned_,
                                      *ph_imu_quat_y_ned_, *ph_imu_quat_z_ned_);
  Eigen::Vector3d ang_vel_imu_imu(*ph_imu_ang_vel_x_, *ph_imu_ang_vel_y_,
                                  *ph_imu_ang_vel_z_);

  // for changing orientation standard local NED -> ROS standard by rotating
  // 180deg on x axis
  Eigen::Quaternion<double> Q_0_ned(0, 1, 0, 0);

  Q_0_imu_ = Q_0_ned * Q_ned_imu;
  ang_vel_0_imu_ = Q_0_imu_ * ang_vel_imu_imu;

  // pass imu data to rpc
  draco_sensor_data_->imu_frame_quat_ << Q_0_imu_.coeffs(); // (x,y,z,w) order
  draco_sensor_data_->imu_ang_vel_ = ang_vel_0_imu_;
  draco_sensor_data_->imu_dvel_ << *ph_imu_dvel_x_, *ph_imu_dvel_y_,
      *ph_imu_dvel_z_;

  //=============================================================
  // Compute jpos from motor (for debugging purpose)
  //=============================================================
  for (int i = 0; i < num_joints_; i++) {
    float mom_arm =
        actuator_speed_ratio_[i] * (*(ph_linkage_speed_ratio_data_[i]));
    diff_jpos_mjpos_[i] = *(ph_jpos_data_[i]) - motor_pos_polarity_[i] *
                                                    (*(ph_mpos_data_[i])) /
                                                    mom_arm;
  }

  //=============================================================
  // Process contact switch data
  //=============================================================
  draco_sensor_data_->b_lf_contact_ =
      (*ph_lfoot_sg_ > contact_threshold_) ? true : false;
  draco_sensor_data_->b_rf_contact_ =
      (*ph_rfoot_sg_ > contact_threshold_) ? true : false;

  //=============================================================
  // Process joint pos, vel MISO data
  //=============================================================
  for (int i = 0; i < num_joints_; i++) {
    if (pinocchio_robot_jidx_[i] == l_knee_fe_jd ||
        pinocchio_robot_jidx_[i] == r_knee_fe_jd) {
      // process rolling contact joints (jp & jd)
      draco_sensor_data_->joint_pos_[pinocchio_robot_jidx_[i]] =
          static_cast<double>(*(ph_jpos_data_[i])) / 2.;
      draco_sensor_data_->joint_pos_[pinocchio_robot_jidx_[i] - 1] =
          static_cast<double>(*(ph_jpos_data_[i])) / 2.;
      draco_sensor_data_->joint_vel_[pinocchio_robot_jidx_[i]] =
          static_cast<double>(*(ph_jvel_data_[i])) / 2.;
      draco_sensor_data_->joint_vel_[pinocchio_robot_jidx_[i] - 1] =
          static_cast<double>(*(ph_jvel_data_[i])) / 2.;
    } else {
      // process other joints
      draco_sensor_data_->joint_pos_[pinocchio_robot_jidx_[i]] =
          static_cast<double>(*(ph_jpos_data_[i]));
      draco_sensor_data_->joint_vel_[pinocchio_robot_jidx_[i]] =
          static_cast<double>(*(ph_jvel_data_[i]));
    }
  }
}

void Draco3Nodelet::_CopyCommand() {
  //=============================================================
  // Process joint pos, vel, trq MOSI data
  //=============================================================
  for (int i = 0; i < num_joints_; i++) {
    if (pinocchio_robot_jidx_[i] == l_knee_fe_jd ||
        pinocchio_robot_jidx_[i] == r_knee_fe_jd) {
      *(ph_jpos_cmd_[i]) = static_cast<float>(
          draco_command_->joint_pos_cmd_[pinocchio_robot_jidx_[i]] * 2.);
      *(ph_jvel_cmd_[i]) = static_cast<float>(
          draco_command_->joint_vel_cmd_[pinocchio_robot_jidx_[i]] * 2.);
      *(ph_jtrq_cmd_[i]) = static_cast<float>(
          draco_command_->joint_trq_cmd_[pinocchio_robot_jidx_[i]] / 2.);
    } else {
      *(ph_jpos_cmd_[i]) = static_cast<float>(
          draco_command_->joint_pos_cmd_[pinocchio_robot_jidx_[i]]);
      *(ph_jvel_cmd_[i]) = static_cast<float>(
          draco_command_->joint_vel_cmd_[pinocchio_robot_jidx_[i]]);
      *(ph_jtrq_cmd_[i]) = static_cast<float>(
          draco_command_->joint_trq_cmd_[pinocchio_robot_jidx_[i]]);
    }
    *(ph_kp_cmd_[i]) = joint_kp_cmd_vector_[i];
    *(ph_kd_cmd_[i]) = joint_kd_cmd_vector_[i];
  }
}

void Draco3Nodelet::_CopyJointGainsCommand() {
  for (int i = 0; i < num_joints_; i++) {
    *(ph_kp_cmd_[i]) = joint_kp_cmd_vector_[i];
    *(ph_kd_cmd_[i]) = joint_kd_cmd_vector_[i];
  }
}

void Draco3Nodelet::_InitializePlaceHolders() {
  for (int i = 0; i < num_joints_; i++) {
    ph_jpos_data_[i] = new float(0.);
    ph_jvel_data_[i] = new float(0.);
    ph_mpos_data_[i] = new float(0.);
    ph_linkage_speed_ratio_data_[i] = new float(0.);
    ph_jpos_cmd_[i] = new float(0.);
    ph_jvel_cmd_[i] = new float(0.);
    ph_jtrq_cmd_[i] = new float(0.);
    ph_current_cmd_[i] = new float(0.);
    ph_kp_cmd_[i] = new float(0.);
    ph_kd_cmd_[i] = new float(0.);
  }

  ph_imu_quat_w_ned_ = new float(0.);
  ph_imu_quat_x_ned_ = new float(0.);
  ph_imu_quat_y_ned_ = new float(0.);
  ph_imu_quat_z_ned_ = new float(0.);
  ph_imu_ang_vel_x_ = new float(0.);
  ph_imu_ang_vel_y_ = new float(0.);
  ph_imu_ang_vel_z_ = new float(0.);
  ph_imu_dvel_x_ = new float(0.);
  ph_imu_dvel_y_ = new float(0.);
  ph_imu_dvel_z_ = new float(0.);
  ph_rfoot_sg_ = new float(0.);
  ph_lfoot_sg_ = new float(0.);
}

void Draco3Nodelet::_Callback(const std_msgs::String::ConstPtr &input) {
  std_msgs::String output;
  output.data = input->data;
  ROS_INFO("msg data = %s", output.data.c_str());
  pub_.publish(output);
}

bool Draco3Nodelet::_FaultHandlerCallback(apptronik_srvs::Bool::Request &req,
                                          apptronik_srvs::Bool::Response &res) {
  b_clear_faults_ = req.set_data;
  if (b_clear_faults_)
    ROS_INFO("[[Clear Faults Invoked]]");
  return true;
}

bool Draco3Nodelet::_FakeEstopHandlerCallback(
    apptronik_srvs::Bool::Request &req, apptronik_srvs::Bool::Response &res) {
  b_fake_estop_released_ = req.set_data;
  if (b_fake_estop_released_)
    ROS_INFO("[[Fake Estop Released]]");

  return true;
}

bool Draco3Nodelet::_MotorModeHandlerCallback(
    apptronik_srvs::Int8::Request &req, apptronik_srvs::Int8::Response &res) {
  int data = req.set_data;
  if (data == 0) {
    ROS_INFO("Change to [[MOTOR_OFF]] Mode");
    b_motor_off_mode_ = true;
    motor_control_mode_ = control_mode::kOff;
  } else if (data == 1) {
    ROS_INFO("Change to [[MOTOR_CURRENT]] Mode");
    b_motor_current_mode_ = true;
    motor_control_mode_ = control_mode::kMotorCurrent;
  } else if (data == 2) {
    ROS_INFO("Change to [[JOINT_IMPEDNACE]] Mode");
    b_joint_impedance_mode_ = true;
    motor_control_mode_ = control_mode::kJointImpedance;
  } else if (data == 3) {
    ROS_INFO("Change to target joint [[JOINT_IMPEDNACE]] Mode");
    b_target_joint_impedance_mode_ = true;
    motor_control_mode_ = control_mode::kJointImpedance;
  } else {
    ROS_INFO("[[WARNING]] Invalid Data Received in MotorModeHandler");
    return false;
  }

  return true;
}

bool Draco3Nodelet::_RPCHandlerCallback(apptronik_srvs::Int8::Request &req,
                                        apptronik_srvs::Int8::Response &res) {
  int data = req.set_data;
  if (data == 0) {
    b_construct_rpc_ = true;
    ROS_INFO("[[RPC Constructed]]");
  } else if (data == 1) {
    b_destruct_rpc_ = true;
    ROS_INFO("[[RPC Destructed]]");
  } else if (data == 2) {
    draco_interface_->interrupt_->PressOne(); // com swaying
  } else if (data == 3) {
    draco_interface_->interrupt_->PressFive(); // inplace walk
  } else {
    ROS_INFO("[[WARNING]] Invalid Data Received in MotorModeHandler");
    return false;
  }

  return true;
}

bool Draco3Nodelet::_GainsAndLimitsHandlerCallback(
    apptronik_srvs::Bool::Request &req, apptronik_srvs::Bool::Response &res) {
  b_set_motor_gains_limits_ = req.set_data;
  if (b_set_motor_gains_limits_)
    ROS_INFO("[[Reset Gains and Motor Current Limits]]");
  return true;
}

bool Draco3Nodelet::_JointGainsHandlerCallback(
    draco3_nodelet::TuneJointGainsSrv::Request &req,
    draco3_nodelet::TuneJointGainsSrv::Response &res) {

  std::string joint_name = req.joint.joint_name;
  int jidx = joint_idx_map_[req.joint.joint_name];

  joint_kp_cmd_vector_[jidx] = req.joint.kp.data;
  joint_kd_cmd_vector_[jidx] = req.joint.kd.data;

  std::cout << "joint name: " << req.joint.joint_name << " joint idx: " << jidx
            << " kp: " << joint_kp_cmd_vector_[jidx]
            << " kd: " << joint_kd_cmd_vector_[jidx] << std::endl;

  res.success = true;

  ROS_INFO("[[Tune Joint Gains Done]]");

  return res.success;
}
template <class SrvType>
void Draco3Nodelet::_CallSetService(const std::string &slave_name,
                                    const std::string &srv_name,
                                    SrvType &srv_obj) {
  std::string full_set_service =
      "/" + slave_name + "/" + srv_name + "/" + "set";
  ros::NodeHandle nh = getPrivateNodeHandle(); // for Nodelets

  ros::ServiceClient client = nh.serviceClient<SrvType>(full_set_service);

  if (client.call(srv_obj)) {
    NODELET_INFO_STREAM("Called /" << slave_name.c_str() << "/"
                                   << srv_name.c_str()); // for Nodelets
  } else {
    NODELET_INFO_STREAM(
        "Failed to call service: " << full_set_service.c_str()); // for Nodelets
  }
}

template <class SrvType>
void Draco3Nodelet::_CallGetService(const std::string &slave_name,
                                    const std::string &srv_name,
                                    SrvType &srv_obj) {
  std::string full_get_service =
      "/" + slave_name + "/" + srv_name + "/" + "get";
  ros::NodeHandle nh = getPrivateNodeHandle(); // for Nodelets

  ros::ServiceClient client = nh.serviceClient<SrvType>(full_get_service);

  if (client.call(srv_obj)) {
    NODELET_INFO_STREAM("Called /" << slave_name.c_str() << "/"
                                   << srv_name.c_str()); // for Nodelets
  } else {
    NODELET_INFO_STREAM(
        "Failed to call service: " << full_get_service.c_str()); // for Nodelets
  }
}

} // namespace draco3_nodelet

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(draco3_nodelet::Draco3Nodelet, nodelet::Nodelet);
