#include "elkapod_gait_controller/elkapod_gait_controller.hpp"

#include <algorithm>
#include <chrono>
#include <eigen3/Eigen/Eigen>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"

namespace {
constexpr auto INPUT_VEL_TOL = 1e-5;
constexpr auto VEL_TOL = 1e-3;

constexpr auto EMA_FILTER_TAU = 0.15;
constexpr auto EMA_FILTER_TAU_BASE_HEIGHT = 0.15;

}  // namespace

static bool is_close(const double a, const double b, const double atol = 1e-8,
                     const double rtol = 1e-5) {
  return std::abs(a - b) <= (atol + rtol * std::abs(b));
}

static Eigen::Matrix3d rotZ(const double theta) {
  return Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

namespace elkapod_gait_controller {
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;

ElkapodGaitController::ElkapodGaitController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn ElkapodGaitController::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception &e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration ElkapodGaitController::command_interface_configuration() const {
  std::vector<std::string> conf_names;
  for (const auto &joint_name : params_.joints) {
    conf_names.push_back(joint_name);
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration ElkapodGaitController::state_interface_configuration() const {
  return {interface_configuration_type::NONE, {}};
}

void ElkapodGaitController::fillLegsTransformationsVect() {
  const std::vector<double> &mounting_rot = params_.kinematics.leg_tf.mounting_rot;
  const std::vector<double> &mounting_trans_x = params_.kinematics.leg_tf.mounting_trans_x;
  const std::vector<double> &mounting_trans_y = params_.kinematics.leg_tf.mounting_trans_y;
  const std::vector<double> &mounting_trans_z = params_.kinematics.leg_tf.mounting_trans_z;

  for (size_t i = 0; i < kLegsNb; ++i) {
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() << mounting_trans_x[i], mounting_trans_y[i], mounting_trans_z[i];

    tf.linear() = rotZ(mounting_rot[i]);
    leg_transformations_[i] = tf;
  }
}

controller_interface::CallbackReturn ElkapodGaitController::on_configure(
    const rclcpp_lifecycle::State &) {
  auto logger = get_node()->get_logger();

  leg_clock_ = std::vector<double>(kLegsNb, 0.);
  leg_phase_ = std::vector<int>(kLegsNb, 0);
  leg_phase_shift_ = std::vector<double>(kLegsNb, 0.);
  phase_offset_ = std::vector<double>(kLegsNb, 0.);

  current_velocity_ = std::vector<Eigen::Vector2d>(kLegsNb, {0.0, 0.0});
  last_leg_position_ = std::vector<Eigen::Vector3d>(kLegsNb, {0.0, 0.0, 0.0});
  last_leg_position_relative_ = std::vector<Eigen::Vector3d>(kLegsNb, {0.0, 0.0, 0.0});

  if (param_listener_->try_update_params(params_)) {
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  fillLegsTransformationsVect();

  min_swing_time_sec_ = params_.gait.min_swing_time_sec;
  phase_lag_ = params_.gait.default_phase_lag;

  leg_spacing_ = params_.leg_spacing.default_leg_spacing;
  step_height_ = params_.gait.step.height.default_step_height;

  default_base_height_ = params_.base_height.default_base_height;
  base_height_offset_ = params_.base_height.base_link_offset;
  kinematics_m1_ = params_.kinematics.m1;
  base_height_min_ = params_.base_height.min_base_height;
  base_height_max_ = params_.base_height.max_base_height;

  default_base_height_ += kinematics_m1_[2] - base_height_offset_;
  base_height_ = default_base_height_;
  set_base_height_ = default_base_height_;

  const double k_roll = params_.roll.pid.k;
  const double ti_roll = params_.roll.pid.ti;
  const double td_roll = params_.roll.pid.td;
  const double cmd_lo_roll = params_.roll.pid.cmd_lo;
  const double cmd_hi_roll = params_.roll.pid.cmd_hi;
  roll_limit_ = params_.roll.max_rad;

  const double k_pitch = params_.pitch.pid.k;
  const double ti_pitch = params_.pitch.pid.ti;
  const double td_pitch = params_.pitch.pid.td;
  const double cmd_lo_pitch = params_.pitch.pid.cmd_lo;
  const double cmd_hi_pitch = params_.pitch.pid.cmd_hi;
  pitch_limit_ = params_.pitch.max_rad;

  control_toolbox::AntiWindupStrategy aw_strategy_pitch, aw_strategy_roll;
  aw_strategy_pitch.set_type("back_calculation");
  aw_strategy_roll.set_type("back_calculation");

  max_vel_dict_ = {
      {GaitType::TRIPOD, params_.max_vel.tripod},
      {GaitType::RIPPLE, params_.max_vel.ripple},
      {GaitType::WAVE, params_.max_vel.wave},
  };

  max_angular_vel_dict_ = {
      {GaitType::TRIPOD, params_.max_angular_vel.tripod},
      {GaitType::RIPPLE, params_.max_angular_vel.ripple},
      {GaitType::WAVE, params_.max_angular_vel.wave},
  };

  roll_ = 0.0;
  pitch_ = 0.0;
  yaw_ = 0.0;
  current_vel_command_ = Eigen::Vector2d::Zero();

  current_angular_velocity_ = 0.0;
  current_vel_scalar_ = 0.0;
  received_vel_command_ = VelCmd();

  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  auto change_gait_qos = rclcpp::SystemDefaultsQoS();
  change_gait_qos.reliable();
  change_gait_qos.keep_last(5);

  roll_pid_ = std::make_unique<control_toolbox::Pid>(k_roll, k_roll / ti_roll, k_roll * td_roll,
                                                     cmd_hi_roll, cmd_lo_roll, aw_strategy_roll);
  pitch_pid_ =
      std::make_unique<control_toolbox::Pid>(k_pitch, k_pitch / ti_pitch, k_pitch * td_pitch,
                                             cmd_hi_pitch, cmd_lo_pitch, aw_strategy_pitch);

  leg_path_gen_ = std::make_unique<elkapod_leg_paths::BasicPathBezier>(0.0, step_height_);

  // Subscriptions
  velocity_sub_ = get_node()->create_subscription<VelCmd>(
      "/cmd_vel_out", subscribers_qos,
      std::bind(&ElkapodGaitController::velocityCallback, this, std::placeholders::_1));

  param_sub_ = get_node()->create_subscription<FloatMsg>(
      "/cmd_base_height", subscribers_qos,
      std::bind(&ElkapodGaitController::baseHeightCallback, this, std::placeholders::_1));

  gait_type_sub_ = get_node()->create_subscription<IntMsg>(
      "/cmd_gait_type", change_gait_qos,
      std::bind(&ElkapodGaitController::gaitTypeCallback, this, std::placeholders::_1));

  imu_sub_ = get_node()->create_subscription<IMUMsg>(
      "/imu", subscribers_qos,
      std::bind(&ElkapodGaitController::imuCallback, this, std::placeholders::_1));

  roll_sub_ = get_node()->create_subscription<FloatMsg>(
      "/roll_setpoint", subscribers_qos,
      std::bind(&ElkapodGaitController::rollCallback, this, std::placeholders::_1));

  pitch_sub_ = get_node()->create_subscription<FloatMsg>(
      "/pitch_setpoint", subscribers_qos,
      std::bind(&ElkapodGaitController::pitchCallback, this, std::placeholders::_1));

  if (params_.publish_debug_info) {
    // TODO print essential parameters
  }

  publish_loop_execution_time_ = params_.publish_debug_info;
  loop_exec_duration_publisher_ = get_node()->create_publisher<DurationMsg>(
      "/elkapod_gait_controller/loop_exec_time", rclcpp::SystemDefaultsQoS());
  loop_exec_duration_publisher_rt_ =
      std::make_unique<realtime_tools::RealtimePublisher<DurationMsg>>(
          loop_exec_duration_publisher_);

  configured_ = true;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElkapodGaitController::on_activate(
    const rclcpp_lifecycle::State &) {
  reset();
  RCLCPP_INFO(get_node()->get_logger(), "Elkapod gait controller activated!");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElkapodGaitController::on_deactivate(
    const rclcpp_lifecycle::State &) {
  if (state_ == State::WALKING) {
    return controller_interface::CallbackReturn::FAILURE;
  } else {
    halt();
    state_ = State::DISABLED;
    RCLCPP_INFO(get_node()->get_logger(), "Elkapod gait controller deactivated!");
    return controller_interface::CallbackReturn::SUCCESS;
  }
}

controller_interface::CallbackReturn ElkapodGaitController::on_cleanup(
    const rclcpp_lifecycle::State &) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ElkapodGaitController::on_error(
    const rclcpp_lifecycle::State &) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ElkapodGaitController::update(const rclcpp::Time &time,
                                                                const rclcpp::Duration &period) {
  if (period.seconds() <= 0.0) {
    return controller_interface::return_type::OK;
  }

  const auto start_time = std::chrono::steady_clock::now();
  if (is_first_update_) {
    last_duration_msg_publish_time_ = time;
    is_first_update_ = false;
    return controller_interface::return_type::OK;
  }

  if (!configured_ && state_ != State::DISABLED) {
    return controller_interface::return_type::OK;
  }
  auto logger = get_node()->get_logger();

  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();

    const double k_roll = params_.roll.pid.k;
    const double ti_roll = params_.roll.pid.ti;
    const double td_roll = params_.roll.pid.td;
    const double cmd_lo_roll = params_.roll.pid.cmd_lo;
    const double cmd_hi_roll = params_.roll.pid.cmd_hi;
    roll_limit_ = params_.roll.max_rad;

    const double k_pitch = params_.pitch.pid.k;
    const double ti_pitch = params_.pitch.pid.ti;
    const double td_pitch = params_.pitch.pid.td;
    const double cmd_lo_pitch = params_.pitch.pid.cmd_lo;
    const double cmd_hi_pitch = params_.pitch.pid.cmd_hi;
    pitch_limit_ = params_.pitch.max_rad;

    control_toolbox::AntiWindupStrategy aw_strategy_pitch, aw_strategy_roll;
    aw_strategy_pitch.set_type("back_calculation");
    aw_strategy_roll.set_type("back_calculation");

    roll_pid_->set_gains(k_roll, k_roll / ti_roll, k_roll * td_roll, cmd_hi_roll, cmd_lo_roll,
                         aw_strategy_roll);
    pitch_pid_->set_gains(k_pitch, k_pitch / ti_pitch, k_pitch * td_pitch, cmd_hi_pitch,
                          cmd_lo_pitch, aw_strategy_pitch);
    RCLCPP_INFO(logger, "Gait generator parameters updated!");
  }

  ema_filter_alfa_ = 1. - std::exp(-period.seconds() / EMA_FILTER_TAU);
  base_height_ema_filter_alfa_ = 1. - std::exp(-period.seconds() / EMA_FILTER_TAU_BASE_HEIGHT);

  auto input_vel_cmd_op = input_vel_command_.try_get();
  if (input_vel_cmd_op.has_value()) {
    received_vel_command_ = input_vel_cmd_op.value();
    updateVelocityCommand();
  }

  auto status = leg_path_gen_->updateBasicParameters(step_length_, step_height_);
  if (status.has_value()) {
    leg_path_gen_->updateBasicParameters(0.0, 0.0);
    RCLCPP_ERROR_THROTTLE(
        logger, *(get_node()->get_clock()), 500,
        "Error setting new step_length and step_height! Fallback to zero values...");
  }

  if (state_ == State::IDLE) {
    for (size_t leg_nb = 0; leg_nb < kLegsNb; ++leg_nb) {
      leg_phase_[leg_nb] = 0.0;
      leg_clock_[leg_nb] = 0.0;
    }
  } else {
    double elapsed_time_sec = (time - init_time_).nanoseconds() / 1e9;
    for (int leg_nb = 0; leg_nb < 6; ++leg_nb) {
      clockFunction(elapsed_time_sec, cycle_time_, phase_offset_[leg_nb], leg_nb);
    }
  }

  // Set height
  base_height_ = base_height_ + base_height_ema_filter_alfa_ * (set_base_height_ - base_height_);

  if (state_ == State::WALKING || state_ == State::IDLE) {
    for (size_t leg_nb = 0; leg_nb < kLegsNb; ++leg_nb) {
      Eigen::Vector3d p = Eigen::Vector3d::Zero();

      if (state_ == State::WALKING) {
        p = leg_path_gen_->eval(leg_clock_[leg_nb], leg_phase_[leg_nb])
                .value_or(Eigen::Vector3d::Zero());

        const double omega = current_angular_velocity_;
        Eigen::Vector2d last_leg_xy(last_leg_position_[leg_nb][0], last_leg_position_[leg_nb][1]);
        Eigen::Vector2d angular_part = omega * Eigen::Vector2d(-last_leg_xy[1], last_leg_xy[0]);

        Eigen::Vector2d vel2D = current_velocity_[leg_nb] + angular_part;

        double angle = std::atan2(vel2D[1], vel2D[0]);
        p = rotZ(angle) * p;
      }

      p = leg_transformations_[leg_nb].linear().transpose() * p;
      p += Eigen::Vector3d(leg_spacing_, 0.0, -base_height_);

      last_leg_position_relative_[leg_nb] = p;
    }
  }

  // Pitch && Roll PIDs
  const double e_roll = set_roll_ - roll_;
  double u_roll = 0;  // roll_pid_->compute_command(e_roll, period);

  const double e_pitch = set_pitch_ - pitch_;
  double u_pitch = 0;  // pitch_pid_->compute_command(e_pitch, period);

  if (state_ != State::IDLE) {
    u_roll = 0.0;
    u_pitch = 0.0;
  }

  for (size_t i = 0; i < kLegsNb; ++i) {
    auto p = last_leg_position_relative_[i];
    const Eigen::Vector4d p_homogeneous(p[0], p[1], p[2], 1.0);
    Eigen::Vector4d p_base_homogeneous = leg_transformations_[i] * p_homogeneous;

    double dz = 0.0;
    dz += -u_roll * p_base_homogeneous[1];
    dz += u_pitch * p_base_homogeneous[0];
    const double pz = std::clamp(p[2] + dz, -0.2, -0.11);

    p_base_homogeneous[2] = p[2];
    last_leg_position_[i] = p_base_homogeneous.head<3>();
    last_leg_position_relative_[i] = p;

    if (std::isnan(p[0]) || std::isnan(p[1]) || std::isnan(p[2])) {
      RCLCPP_FATAL(logger, "Nan values in output!");
      return controller_interface::return_type::OK;
    }

    (void)command_interfaces_[i * 3 + 0].set_value(p[0]);
    (void)command_interfaces_[i * 3 + 1].set_value(p[1]);
    (void)command_interfaces_[i * 3 + 2].set_value(pz);
  }

  if (publish_loop_execution_time_ &&
      (time - last_duration_msg_publish_time_).seconds() > duration_msg_publish_period_) {
    const auto end_time = std::chrono::steady_clock::now();
    const auto duration =
        std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    RCLCPP_DEBUG_THROTTLE(
        logger, *(get_node()->get_clock()), 100,
        std::format("Gait generator loop time {:.3f} ms", static_cast<double>(duration) / 1e6)
            .c_str());

    rclcpp::Duration duration_obj = rclcpp::Duration::from_nanoseconds(duration);
    builtin_interfaces::msg::Duration duration_msg = duration_obj;
    loop_exec_duration_publisher_rt_->try_publish(duration_msg);
    last_duration_msg_publish_time_ = time;
  }
  return controller_interface::return_type::OK;
}

bool ElkapodGaitController::reset() {
  state_ = State::IDLE;
  leg_path_gen_->init();
  changeGait();
  init_time_ = get_node()->now();

  received_vel_command_ = VelCmd();
  set_base_height_ = default_base_height_;
  current_vel_command_.setZero();

  set_roll_ = 0.0;
  set_pitch_ = 0.0;
  imu_received_ = false;

  roll_pid_->reset();
  pitch_pid_->reset();

  return true;
}

void ElkapodGaitController::reset_buffers() {}

void ElkapodGaitController::halt() {
  for (size_t i = 0; i < kLegsNb; ++i) {
    Eigen::Vector3d p;

    p = Eigen::Vector3d::Zero();
    p = leg_transformations_[i].linear().transpose() * p;
    p += Eigen::Vector3d(leg_spacing_, 0.0, -base_height_);

    (void)command_interfaces_[i * 3 + 0].set_value(p[0]);
    (void)command_interfaces_[i * 3 + 1].set_value(p[1]);
    (void)command_interfaces_[i * 3 + 2].set_value(p[2]);
  }
}

void ElkapodGaitController::baseHeightCallback(const FloatMsg::SharedPtr msg) {
  if (msg->data >= base_height_min_ && msg->data <= base_height_max_) {
    set_base_height_ = msg->data + kinematics_m1_[2] - base_height_offset_;
  } else {
    RCLCPP_WARN(this->get_node()->get_logger(),
                "Couldn't set new base height goal - value out of allowed range");
  }
}

void ElkapodGaitController::gaitTypeCallback(const IntMsg::SharedPtr msg) {
  auto logger = get_node()->get_logger();
  if (state_ == State::IDLE) {
    if (msg->data == 0) {
      gait_type_ = GaitType::WAVE;
      changeGait();
      RCLCPP_INFO(logger, "Gait set to WAVE");
    } else if (msg->data == 1) {
      gait_type_ = GaitType::RIPPLE;
      changeGait();
      RCLCPP_INFO(logger, "Gait set to RIPPLE");
    } else if (msg->data == 2) {
      gait_type_ = GaitType::TRIPOD;
      changeGait();
      RCLCPP_INFO(logger, "Gait set to TRIPOD");
    }
  }
}

void ElkapodGaitController::velocityCallback(const VelCmd::SharedPtr msg) {
  if ((!is_close(msg->linear.x, 0.0, INPUT_VEL_TOL) ||
       !is_close(msg->linear.y, 0.0, INPUT_VEL_TOL)) &&
      !is_close(msg->angular.z, 0.0, INPUT_VEL_TOL)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 500,
                         "Invalid Twist message! Cannot apply both linear and angular velocities!");
  } else {
    input_vel_command_.set(*msg);
  }
}

void ElkapodGaitController::imuCallback(const IMUMsg::SharedPtr msg) {
  tf2::fromMsg(msg->orientation, q_);
  q_.normalize();
  tf2::Matrix3x3(q_).getRPY(roll_, pitch_, yaw_);
  imu_received_ = true;
}

void ElkapodGaitController::rollCallback(const FloatMsg::SharedPtr msg) {
  auto x = std::pow(msg->data, 2) / std::pow(roll_limit_, 2) +
           std::pow(set_pitch_, 2) / std::pow(pitch_limit_, 2);

  if (x <= 1.0) {
    set_roll_ = msg->data;
  } else {
    RCLCPP_WARN(get_node()->get_logger(), "Couldn't set new roll value - out of allowed range");
  }
}

void ElkapodGaitController::pitchCallback(const FloatMsg::SharedPtr msg) {
  auto x = std::pow(set_roll_, 2) / std::pow(roll_limit_, 2) +
           std::pow(msg->data, 2) / std::pow(pitch_limit_, 2);

  if (x <= 1.0) {
    set_pitch_ = msg->data;
  } else {
    RCLCPP_WARN(get_node()->get_logger(), "Couldn't set new pitch value - out of allowed range");
  }
}

void ElkapodGaitController::changeGait() {
  max_vel_ = max_vel_dict_[gait_type_];
  max_angular_vel_ = max_angular_vel_dict_[gait_type_];
  cycle_time_ = cycle_time_dict_[gait_type_];

  if (gait_type_ == GaitType::TRIPOD) {
    duty_factor_ = 1 / 2.;
    phase_offset_ = {0.0, cycle_time_ / 2., cycle_time_ / 2., 0.0, 0.0, cycle_time_ / 2.};

  } else if (gait_type_ == GaitType::WAVE) {
    phase_offset_.assign(6, 0.0);
    std::array<int, 6> order = {0, 3, 4, 1, 2, 5};

    for (size_t k = 0; k < 6; ++k) phase_offset_[order[k]] = k * cycle_time_ / 6.0;
    duty_factor_ = 5 / 6.;
  } else if (gait_type_ == GaitType::RIPPLE) {
    phase_offset_.assign(6, 0.0);
    std::array<int, 6> order = {2, 1, 4, 3, 0, 5};
    for (size_t k = 0; k < 6; ++k) {
      phase_offset_[order[k]] = 0.15 * k * cycle_time_;
    }
    duty_factor_ = 0.75;
  }
}

void ElkapodGaitController::clockFunction(double t, double T, double phase_shift, int leg_nb) {
  leg_phase_shift_[leg_nb] = phase_shift;

  const double t_mod = std::fmod((t + leg_phase_shift_[leg_nb]), T);

  if (t_mod < T * phase_lag_ * 0.5) {  // swing lag phase
    leg_phase_[leg_nb] = 1;
    leg_clock_[leg_nb] = 0.;
  } else if (T * phase_lag_ * 0.5 <= t_mod &&
             t_mod < T * (1. - duty_factor_) - T * phase_lag_ * 0.5) {  // swing
    leg_phase_[leg_nb] = 1;
    leg_clock_[leg_nb] =
        (t_mod - T * phase_lag_ * 0.5) / (T * (1. - duty_factor_) - T * phase_lag_ * 0.5);
  } else if (T * (1. - duty_factor_) - T * phase_lag_ * 0.5 <= t_mod &&
             t_mod < T * (1. - duty_factor_) + 0.5 * T * phase_lag_) {  // swing - stance lag phase
    leg_phase_[leg_nb] = 0;
    leg_clock_[leg_nb] = 0.;
  } else if (T * (1. - duty_factor_) + 0.5 * T * phase_lag_ < t_mod &&
             t_mod < T - 0.5 * T * phase_lag_) {  // stance
    leg_phase_[leg_nb] = 0;
    leg_clock_[leg_nb] = (t_mod - T * (1. - duty_factor_)) / (T - T * (1. - duty_factor_));
  } else {  // stance lag phase
    leg_phase_[leg_nb] = 0;
    leg_clock_[leg_nb] = 1.;
  }
}

void ElkapodGaitController::updateVelocityCommand() {
  Eigen::Vector2d vel_command =
      Eigen::Vector2d({received_vel_command_.linear.x, received_vel_command_.linear.y});
  double angular_vel = received_vel_command_.angular.z;

  velocityDeadzone(vel_command, angular_vel);
  velocityClamp(vel_command, angular_vel);

  if (!vel_command.isZero() || std::abs(angular_vel) > 1e-6) {
    set_roll_ = 0.0;
    set_pitch_ = 0.0;
  }

  current_vel_command_ =
      current_vel_command_ + ema_filter_alfa_ * (vel_command - current_vel_command_);
  current_angular_velocity_ =
      current_angular_velocity_ + ema_filter_alfa_ * (angular_vel - current_angular_velocity_);

  if (!is_close(angular_vel, 0.0, VEL_TOL)) {
    for (auto &leg_vel : current_velocity_) {
      leg_vel.setZero();
    }

    constexpr double kBaseToLegMountDist = 0.20204;
    const double R = leg_spacing_ + kBaseToLegMountDist;
    step_length_ = cycle_time_ * duty_factor_ * R * std::fabs(angular_vel);
  } else {
    for (auto &leg_vel : current_velocity_) {
      leg_vel = current_vel_command_;
    }

    current_vel_scalar_ = current_vel_command_.norm();
    const double T_stride = cycle_time_ * (duty_factor_ - phase_lag_);
    step_length_ = current_vel_scalar_ * T_stride;

    if (gait_type_ == GaitType::TRIPOD) {
      step_length_ /= 0.7;
    }
  }

  RCLCPP_DEBUG_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 100,
                        std::format("Current velocity: {:.4f} m/s\tAngular: {:.4f} rad/s",
                                    current_vel_scalar_, current_angular_velocity_)
                            .c_str());

  if (is_close(current_vel_scalar_, 0.0, VEL_TOL) &&
      is_close(current_angular_velocity_, 0.0, VEL_TOL) && state_ == State::WALKING) {
    RCLCPP_INFO(get_node()->get_logger(), "Going to IDLE state");
    if (roll_pid_ != nullptr) {
      roll_pid_->reset(false);
    }
    if (pitch_pid_ != nullptr) {
      pitch_pid_->reset(false);
    }
    state_ = State::IDLE;
  } else if ((!is_close(current_vel_scalar_, 0.0, VEL_TOL) ||
              !is_close(current_angular_velocity_, 0.0, VEL_TOL)) &&
             state_ == State::IDLE) {
    RCLCPP_INFO(get_node()->get_logger(), "Going to WALKING state");
    init_time_ = get_node()->now();
    state_ = State::WALKING;
    std::fill(leg_phase_shift_.begin(), leg_phase_shift_.end(), 0.);
  }
}

void ElkapodGaitController::velocityDeadzone(Eigen::Vector2d &vel, double &angular_vel) {
  if (vel.squaredNorm() < deadzone_d_ * deadzone_d_) {
    vel.setZero();
  }

  if (std::abs(angular_vel) < deadzone_d_) {
    angular_vel = 0.0;
  }
}

void ElkapodGaitController::velocityClamp(Eigen::Vector2d &vel, double &angular_vel) {
  const double linear_vel = vel.norm();
  if (linear_vel > max_vel_ && linear_vel > 0) {
    vel *= max_vel_ / linear_vel;
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *(get_node()->get_clock()), 500,
        std::format(
            "Provided velocity exceeds max velocity of {:.3f} m / s! Clamping to the limit...",
            max_vel_)
            .c_str());
  }

  if (std::fabs(angular_vel) > max_angular_vel_ && std::fabs(angular_vel) > 0) {
    angular_vel *= max_angular_vel_ / std::fabs(angular_vel);
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 500,
                         std::format("Provided angular velocity exceeds max velocity of {:.3f} rad "
                                     "/ s! Clamping to the limit...",
                                     max_angular_vel_)
                             .c_str());
  }
}

}  // namespace elkapod_gait_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(elkapod_gait_controller::ElkapodGaitController,
                       controller_interface::ControllerInterface)