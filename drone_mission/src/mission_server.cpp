// Mission server implementation. See include/drone_mission/mission_server.hpp for the
// overview, the state list and the class outline.

#include "drone_mission/mission_server.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <functional>

namespace drone_mission
{


// ── Vocabulary helpers ────────────────────────────────────────────────────────────────────────────────────

const char * toString(State s)
{
  switch (s) {
    case State::IDLE: return "IDLE";
    case State::WAIT_POSITION: return "WAIT_POSITION";
    case State::PRE_OFFBOARD: return "PRE_OFFBOARD";
    case State::WAIT_OFFBOARD: return "WAIT_OFFBOARD";
    case State::ARMING: return "ARMING";
    case State::TAKEOFF: return "TAKEOFF";
    case State::HOVER: return "HOVER";
    case State::FLYING: return "FLYING";
    case State::LANDING: return "LANDING";
    case State::EXTERNAL: return "EXTERNAL";
  }
  return "?";
}

const char * toString(GoalKind k)
{
  switch (k) {
    case GoalKind::CIRCLE: return "circle";
    case GoalKind::SQUARE: return "square";
    case GoalKind::TRIANGLE: return "triangle";
    case GoalKind::LAND: return "land";
    case GoalKind::NONE: break;
  }
  return "none";
}

const char * ackResultName(uint8_t result)
{
  switch (result) {
    case VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED: return "ACCEPTED";
    case VehicleCommandAck::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: return "TEMPORARILY_REJECTED";
    case VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED: return "DENIED";
    case VehicleCommandAck::VEHICLE_CMD_RESULT_UNSUPPORTED: return "UNSUPPORTED";
    case VehicleCommandAck::VEHICLE_CMD_RESULT_FAILED: return "FAILED";
    case VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS: return "IN_PROGRESS";
    case VehicleCommandAck::VEHICLE_CMD_RESULT_CANCELLED: return "CANCELLED";
    default: return "UNKNOWN";
  }
}

std::optional<GoalKind> parseGoalKind(const std::string & cmd)
{
  if (cmd == "circle") { return GoalKind::CIRCLE; }
  if (cmd == "square") { return GoalKind::SQUARE; }
  if (cmd == "triangle") { return GoalKind::TRIANGLE; }
  if (cmd == "land") { return GoalKind::LAND; }
  return std::nullopt;
}



// ── Setup and teardown ────────────────────────────────────────────────────────────────────────────────────

MissionServer::MissionServer()
: Node("mission_server")
{
  params_ = declareParams();

  // PX4 topic names - these are the routing keys for messages 
  // Subscriber topics
  const auto status_topic = declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status_v4");
  const auto lpos_topic = declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1");
  const auto land_topic = declare_parameter("vehicle_land_detected_topic", "/fmu/out/vehicle_land_detected");
  // Publisher topics
  const auto ocm_topic = declare_parameter("offboard_control_mode_topic", "/fmu/in/offboard_control_mode");
  const auto sp_topic = declare_parameter("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint");
  const auto telem_topic = declare_parameter("telemetry_status_topic", "/fmu/in/telemetry_status");

  // Create service (request = VehicleCommand, reply = VehicleCommandAck)
  const auto cmd_service = declare_parameter("vehicle_command_service", "/fmu/vehicle_command");

  
  // Sets delivery policy: freshness over ensuring every message arrives
  const auto qos = rclcpp::SensorDataQoS(); 
  // vehicle_status: arming state, nav (flight) mode, failsafe flag
  status_sub_ = create_subscription<VehicleStatus>(
    status_topic, qos, [this](VehicleStatus::SharedPtr m) { status_ = *m; status_rx_ = now(); });
  
  // vehicle_local_position: EKF position/velocity in NED metres + heading
  lpos_sub_ = create_subscription<VehicleLocalPosition>(
    lpos_topic, qos, [this](VehicleLocalPosition::SharedPtr m) { lpos_ = *m; lpos_rx_ = now(); });
  
  // vehicle_land_detected: PX4's own "we are on the ground" detector
  land_sub_ = create_subscription<VehicleLandDetected>(
    land_topic, qos, [this](VehicleLandDetected::SharedPtr m) { land_ = *m; });


  // offboard_control_mode: "which setpoint fields we use" - must be streamed in OFFBOARD
  ocm_pub_ = create_publisher<OffboardControlMode>(ocm_topic, 10);

  // trajectory_setpoint: the position / velocity target itself
  sp_pub_ = create_publisher<TrajectorySetpoint>(sp_topic, 10);

  // vehicle_command: MAVLink-style commands: set mode, arm, land
  cmd_client_ = create_client<VehicleCommandSrv>(cmd_service);

  // telemetry_status: fake GCS heartbeat (see heartbeatTick)
  telem_pub_ = create_publisher<TelemetryStatus>(telem_topic, 10);


  using namespace std::placeholders;
  // Create action server and register callbacks
  action_server_ = rclcpp_action::create_server<MissionCommand>(
    this, "mission_command",
    std::bind(&MissionServer::handleGoal, this, _1, _2),
    std::bind(&MissionServer::handleCancel, this, _1),
    std::bind(&MissionServer::handleAccepted, this, _1));

  // Set a value for last_tick_
  last_tick_ = now();
  // The retry clock must hold a valid ROS time before the first successful send: sendCommand()
  // can skip while the service is undiscovered, and commandRetryDue() subtracts this from now()
  last_cmd_time_ = now();

  using namespace std::chrono_literals;
  // Set control timer that drives tick() loop
  const auto tick_period = std::chrono::duration<double>(1.0 / params_.control_rate_hz);
  control_timer_ = create_wall_timer(tick_period, [this] { tick(); });

  // Set gcs heartbeat
  if (params_.publish_gcs_heartbeat) { 
    heartbeat_timer_ = create_wall_timer(1s, [this] { heartbeatTick(); });
  }

  // Set up the logger
  RCLCPP_INFO(get_logger(),
    "mission_server ready: altitude %.1f m, cruise %.1f m/s, patterns %.1f..%.1f m; action 'mission_command'",
    params_.flight_altitude, params_.cruise_speed, params_.min_pattern_size, params_.max_pattern_size);
}

// Instantiate all parameters
Params MissionServer::declareParams()
{
  Params p;
  // Flight
  p.flight_altitude = declare_parameter("flight_altitude", p.flight_altitude);
  p.cruise_speed = declare_parameter("cruise_speed", p.cruise_speed);
  p.max_accel = declare_parameter("max_accel", p.max_accel);
  p.min_pattern_size = declare_parameter("min_pattern_size", p.min_pattern_size);
  p.max_pattern_size = declare_parameter("max_pattern_size", p.max_pattern_size);
  p.position_tolerance = declare_parameter("position_tolerance", p.position_tolerance);
  p.velocity_tolerance = declare_parameter("velocity_tolerance", p.velocity_tolerance);
  p.max_tracking_error = declare_parameter("max_tracking_error", p.max_tracking_error);
  p.control_rate_hz = declare_parameter("control_rate_hz", p.control_rate_hz);
  // PX4 addressing / behaviour
  p.target_system = declare_parameter("target_system", p.target_system);
  p.publish_gcs_heartbeat = declare_parameter("publish_gcs_heartbeat", p.publish_gcs_heartbeat);
  return p;
}

void MissionServer::shutdown()
{
  if (goal_ && goal_->is_active()) {
    finishGoal(false, "mission_server shut down");
  }
  stopStreaming();
  control_timer_->cancel();
  if (heartbeat_timer_) {
    heartbeat_timer_->cancel();
  }
}



// ── Action server callbacks (CLI is the client) ───────────────────────────────────────────────────────────

rclcpp_action::GoalResponse MissionServer::handleGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const MissionCommand::Goal> goal)
{
  // Check that goal kind is valid
  const auto kind = parseGoalKind(goal->command);
  if (!kind) {
    RCLCPP_WARN(get_logger(), "Rejected goal: unknown command '%s'", goal->command.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  // One goal at a time: the client has to cancel ("stop") before sending another.
  if (goal_ && goal_->is_active()) {
    RCLCPP_WARN(get_logger(), "Rejected '%s': busy with '%s' - send stop first",
      goal->command.c_str(), toString(goal_kind_));
    return rclcpp_action::GoalResponse::REJECT;
  }
  
  // Decide whether to accept LAND goal
  if (*kind == GoalKind::LAND) {
    if (state_ != State::HOVER && !airborneWithoutUs()) {
      RCLCPP_WARN(get_logger(), "Rejected 'land': vehicle is not airborne (state %s)", toString(state_));
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Check shape size is in range
  const double size = goal->value;
  const bool size_out_of_range = 
    !std::isfinite(size) ||
    size < params_.min_pattern_size ||
    size > params_.max_pattern_size;

  if (size_out_of_range) {
    RCLCPP_WARN(get_logger(), "Rejected '%s %.2f': size must be within [%.1f, %.1f] m",
      goal->command.c_str(), size, params_.min_pattern_size, params_.max_pattern_size);
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void MissionServer::handleAccepted(std::shared_ptr<GoalHandle> handle)
{
  // Remember what was asked for; processStart() on the next tick kicks off the state machine.
  goal_ = handle;
  goal_kind_ = *parseGoalKind(handle->get_goal()->command);
  goal_value_ = handle->get_goal()->value;
  pending_start_ = true;
  RCLCPP_INFO(get_logger(), "Accepted '%s %.2f' in state %s", toString(goal_kind_), goal_value_, toString(state_));
}

rclcpp_action::CancelResponse MissionServer::handleCancel(std::shared_ptr<GoalHandle>)
{
  // Don't cancel a goal while landing
  if (state_ == State::LANDING) {
    RCLCPP_WARN(get_logger(), "Cancel rejected: landing cannot be interrupted");
    return rclcpp_action::CancelResponse::REJECT;
  }

  // Accept cancellation
  RCLCPP_INFO(get_logger(), "Cancel requested for '%s'", toString(goal_kind_));
  cancel_requested_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}



// ── Topics in: read last cached messages ──────────────────────────────────────────────────────────────────

bool MissionServer::px4LinkAlive() const
{
  return status_.has_value() && (now() - status_rx_).seconds() < kStatusStaleTimeout;
}

bool MissionServer::isArmed() const
{
  return status_ && status_->arming_state == VehicleStatus::ARMING_STATE_ARMED;
}

bool MissionServer::inOffboard() const
{
  return status_ && status_->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
}

// PX4 tells us whether its own EKF estimate is valid 
bool MissionServer::positionValid() const
{
  if (!lpos_) {
    return false;
  }
  const bool valid = lpos_->xy_valid && lpos_->z_valid;
  const bool fresh = (now() - lpos_rx_).seconds() < kPositionStaleSec;
  return valid && fresh;
}

bool MissionServer::isOnGround() const
{
  return !land_ || land_->landed;
}

bool MissionServer::airborneWithoutUs() const
{
  return (state_ == State::IDLE || state_ == State::EXTERNAL) && px4LinkAlive() && isArmed() && !isOnGround();
}



// ── Topics out: Streamed to PX4 ───────────────────────────────────────────────────────────────────────────
void MissionServer::publishSetpoints()
{
  const uint64_t t = nowUs();

  // Tell PX4 which controller level we want. This is basically saying: "the setpoint I am about to send is a position cmd"
  OffboardControlMode ocm;
  ocm.timestamp = t;
  ocm.position = true;
  ocm.velocity = false;
  ocm.acceleration = false;
  ocm.attitude = false;
  ocm.body_rate = false;
  ocm.thrust_and_torque = false;
  ocm.direct_actuator = false;
  ocm_pub_->publish(ocm);

  // The target itself, in NED metres. NaN means "not controlled" for that axis/field.
  TrajectorySetpoint sp;
  sp.timestamp = t;
  sp.position = {
    static_cast<float>(cmd_xy_.x), 
    static_cast<float>(cmd_xy_.y),
    self_landing_ ? NAN : static_cast<float>(cmd_z_)};
  sp.velocity = {
    static_cast<float>(cmd_vel_.x), 
    static_cast<float>(cmd_vel_.y),
    self_landing_ ? kFallbackDescentRate : NAN};
  sp.acceleration = {NAN, NAN, NAN};
  sp.jerk = {NAN, NAN, NAN};
  sp.yaw = static_cast<float>(hold_yaw_);
  sp.yawspeed = NAN;
  sp_pub_->publish(sp);
  ++setpoint_count_;
}

// 1 Hz  and only while publish_gcs_heartbeat. It makes PX4 consider a GCS connected 
void MissionServer::heartbeatTick()
{
  TelemetryStatus ts;
  ts.timestamp = nowUs();
  ts.type = TelemetryStatus::LINK_TYPE_GENERIC;
  ts.heartbeat_type_gcs = true;
  telem_pub_->publish(ts);
}



// ── Service out: Send a request ───────────────────────────────────────────────────────────────────────────
// Send a MAVLink-style command to the autopilot over the vehicle_command service. The params
// mean different things per command. Returns false when the request was not dispatched.
bool MissionServer::sendCommand(
  uint32_t command, float p1, float p2, float p3, float p4, double p5, double p6, float p7)
{
  // Skip while the service is undiscovered; commandRetryDue() stays due, so callers re-attempt
  if (!cmd_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "vehicle_command service not available - command %u not sent", command);
    return false;
  }

  // PX4's replier answers one request at a time: drop the stale pending request (future and
  // callback) before re-sending, so a late reply to it is discarded inside rclcpp
  if (pending_request_id_) {
    cmd_client_->remove_pending_request(*pending_request_id_);
    pending_request_id_.reset();
  }

  auto req = std::make_shared<VehicleCommandSrv::Request>();
  VehicleCommand & msg = req->request;
  msg.timestamp = nowUs();
  msg.command = command;
  msg.param1 = p1;
  msg.param2 = p2;
  msg.param3 = p3;
  msg.param4 = p4;
  msg.param5 = p5;
  msg.param6 = p6;
  msg.param7 = p7;
  msg.target_system = static_cast<uint8_t>(params_.target_system);  // MAVLink system id of vehicle we command
  msg.target_component = 1;                                         // MAV_COMP_ID_AUTOPILOT1: the flight controller itself
  msg.source_system = static_cast<uint8_t>(params_.target_system);  // Same vehicle as we are a companion on board, not separate craft
  msg.source_component = 191;                                       // MAV_COMP_ID_ONBOARD_COMPUTER: us, not the flight controller
  msg.confirmation = 0;
  msg.from_external = true;   // "came from a companion / GCS", so PX4 acks it

  // Never block on the reply: a stalled tick would starve the setpoint stream and PX4 would
  // drop OFFBOARD. The reply lands in onCommandReply() on this same thread.
  auto future = cmd_client_->async_send_request(req,
    [this](rclcpp::Client<VehicleCommandSrv>::SharedFuture f) { onCommandReply(f.get()->reply); });
  pending_request_id_ = future.request_id;

  last_cmd_time_ = now();     // Start of retry window
  last_ack_.reset();          // Drop previous command's result
  return true;
}

// Ask PX4 to switch to OFFBOARD mode (it follows our setpoints instead of the RC / mission).
void MissionServer::requestOffboard()
{
  // param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, param2 = PX4_CUSTOM_MAIN_MODE_OFFBOARD
  sendCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
}

// Arm (motors may spin) or disarm the vehicle.
void MissionServer::requestArm(bool arm)
{
  const auto action = arm ? VehicleCommand::ARMING_ACTION_ARM : VehicleCommand::ARMING_ACTION_DISARM;
  sendCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, static_cast<float>(action));
}



// ── Service in ────────────────────────────────────────────────────────────────────────────────────────────
void MissionServer::onCommandReply(const VehicleCommandAck & ack)
{
  pending_request_id_.reset();

  last_ack_ = ack.result;
  const bool accepted =
    ack.result == VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED ||
    ack.result == VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS;

  if (accepted) {
    RCLCPP_INFO(get_logger(), "PX4 ack: command %u %s", ack.command, ackResultName(ack.result));
    return;
  }

  RCLCPP_WARN(get_logger(), "PX4 ack: command %u %s (reason %d)", ack.command, ackResultName(ack.result),
    static_cast<int>(ack.result_param1));
}

std::string MissionServer::lastAckName() const
{
  return last_ack_ ? ackResultName(*last_ack_) : "no ack";
}

bool MissionServer::commandRetryDue() const
{
  return (now() - last_cmd_time_).seconds() > kCommandRetrySec;
}

// ── Shared by topics and service ──
// PX4 message timestamps are microseconds.
uint64_t MissionServer::nowUs() const
{
  return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
}



// ── State machine predicates ────────────────────────────────────────────────────────────────────
// Deliver the goal result and cleanup. 
void MissionServer::finishGoal(bool success, const std::string & message)
{
  if (!goal_) {
    return;
  }

  auto result = std::make_shared<MissionCommand::Result>(); // Create a result object
  result->success = success;
  result->message = message;

  // Pick a terminal state for the goal and log
  if (goal_->is_active()) {
    if (goal_->is_canceling()) {
      goal_->canceled(result);  // Update ROS internally
      RCLCPP_INFO(get_logger(), "Goal '%s' canceled: %s", toString(goal_kind_), message.c_str());
    } else if (success) {
      goal_->succeed(result);
      RCLCPP_INFO(get_logger(), "Goal '%s' succeeded: %s", toString(goal_kind_), message.c_str());
    } else {
      goal_->abort(result);
      RCLCPP_ERROR(get_logger(), "Goal '%s' aborted: %s", toString(goal_kind_), message.c_str());
    }
  }

  // Clear all per-goal state.
  goal_.reset();
  goal_kind_ = GoalKind::NONE;
  pending_start_ = false;
  cancel_requested_ = false;
  sampler_.reset();
}

// Progress report for the client (the CLI prints it as throttled progress lines).
void MissionServer::publishFeedback()
{
  if (!goal_ || !goal_->is_active()) {
    return;
  }

  auto fb = std::make_shared<MissionCommand::Feedback>();
  fb->state = toString(state_);
  fb->progress = static_cast<float>(progress_);

  // Copy vehicle curent position into feedback
  if (lpos_) {
    fb->x = lpos_->x;
    fb->y = lpos_->y;
    fb->z = lpos_->z;
  }
  goal_->publish_feedback(fb);
}

// Switch state, logging the transition and resetting the per-state clock and progress.
void MissionServer::enterState(State s)
{
  if (s != state_) {
    RCLCPP_INFO(get_logger(), "State %s -> %s", toString(state_), toString(s));
  }
  state_ = s;
  state_entry_time_ = now();
  progress_ = 0.0;
}

double MissionServer::timeInState() const
{
  return (now() - state_entry_time_).seconds();
}

// Begin streaming setpoints from a clean slate 
void MissionServer::startStreaming()
{
  streaming_ = true;
  setpoint_count_ = 0;
  not_offboard_since_.reset();
}

// Stop publishing setpoints. Only sensible when the vehicle is on the ground or PX4 has taken over
void MissionServer::stopStreaming()
{
  streaming_ = false;
  self_landing_ = false;
  cmd_vel_ = {NAN, NAN};
  offboard_confirmed_ = false;
  armed_confirmed_ = false;
}

// Freeze the setpoint where it is: keep streaming the last commanded point 
void MissionServer::holdHere()
{
  cmd_vel_ = {NAN, NAN};
  self_landing_ = false;
}



// ── Goal exit paths ────────────────────────────────────────────────────────────────────
// End the goal while airborne under our control: hold position and wait for the next one.
void MissionServer::finishInHover(bool success, const std::string & message)
{
  holdHere();
  finishGoal(success, message);
  enterState(State::HOVER);
}

// End the goal with the vehicle on the ground and PX4 back in charge: nothing left to
// stream or undo, return to a clean IDLE.
void MissionServer::finishOnGround(bool success, const std::string & message)
{
  finishGoal(success, message);
  stopStreaming();
  enterState(State::IDLE);
}

// Abort the goal while still on the ground: undo whatever the start-up sequence did so far
// and give the vehicle back to PX4.
void MissionServer::abortOnGround(const std::string & reason)
{
  finishGoal(false, reason);
  stopStreaming();
  if (isArmed()) {
    requestArm(false);
  }
  enterState(State::IDLE);
}

// PX4 is no longer doing what we expect: drop the goal and let PX4 handle itself.
void MissionServer::bailOut(const std::string & reason)
{
  RCLCPP_ERROR(get_logger(), "Bail-out in state %s: %s", toString(state_), reason.c_str());
  finishGoal(false, reason);
  stopStreaming();
  enterState(State::EXTERNAL);
}



// ── The control loop ──────────────────────────────────────────────────────────────────────────────────────

// Called by control_timer_ (default 20 Hz). Order matters:
//   1. Apply what the action callbacks recorded (cancel / new goal),
//   2. Safety watchdogs (may bail out and change the state),
//   3. The logic of the current state,
//   4. Stream the setpoints and, every few ticks, publish feedback.
void MissionServer::tick()
{
  const auto t = now();
  // Clamp dt so a program stall cannot make the carrot jump far ahead of the drone in one step.
  const double dt = std::clamp((t - last_tick_).seconds(), 0.0, kMaxTickDt);
  last_tick_ = t;

  if (cancel_requested_) {
    processCancel();
  }
  if (pending_start_) {
    processStart();
  }
  checkWatchdogs();

  switch (state_) {
    case State::IDLE: break;
    case State::EXTERNAL: tickExternal(); break;
    case State::WAIT_POSITION: tickWaitPosition(); break;
    case State::PRE_OFFBOARD: tickPreOffboard(); break;
    case State::WAIT_OFFBOARD: tickWaitOffboard(); break;
    case State::ARMING: tickArming(); break;
    case State::TAKEOFF: tickTakeoff(); break;
    case State::HOVER: break;
    case State::FLYING: tickFlying(dt); break;
    case State::LANDING: tickLanding(); break;
  }

  if (streaming_) {
    publishSetpoints();
  }
  if (++tick_count_ % kFeedbackEveryNTicks == 0) {
    publishFeedback();
  }
}

// "stop": what it means depends on where we are.
void MissionServer::processCancel()
{
  cancel_requested_ = false;
  switch (state_) {
    case State::TAKEOFF:
    case State::FLYING:
      // Airborne under our control: just stop moving and hover where the setpoint is.
      finishInHover(false, "stopped - holding position");
      break;
    case State::WAIT_POSITION:
    case State::PRE_OFFBOARD:
    case State::WAIT_OFFBOARD:
    case State::ARMING:
      abortOnGround("stopped before takeoff");
      break;
    default:
      // HOVER with a not-yet-started goal, or nothing to do.
      finishGoal(false, "stopped");
      break;
  }
}

// A freshly accepted goal: pick the entry point of the state machine.
void MissionServer::processStart()
{
  pending_start_ = false;
  if (goal_kind_ == GoalKind::LAND) {
    landing_cmd_sent_ = false;
    enterState(State::LANDING);
  } else if (state_ == State::HOVER) {
    // Already airborne in OFFBOARD: fly the shape right away.
    startPattern();
  } else {
    // IDLE or EXTERNAL: full sequence, WAIT_POSITION figures out whether we are airborne.
    enterState(State::WAIT_POSITION);
  }
}

// States in which we are (or are about to be) in charge of the vehicle.
bool MissionServer::controllingVehicle() const
{
  switch (state_) {
    case State::PRE_OFFBOARD:
    case State::WAIT_OFFBOARD:
    case State::ARMING:
    case State::TAKEOFF:
    case State::HOVER:
    case State::FLYING:
      return true;
    default:
      return false;
  }
}

// Safety checks that run every tick while we control the vehicle. Each one hands the
// vehicle back to PX4 (bailOut) rather than fighting it.
void MissionServer::checkWatchdogs()
{
  if (!controllingVehicle()) {
    return;
  }
  // No news from PX4: the uXRCE-DDS link or PX4 itself is gone.
  if (!px4LinkAlive()) {
    bailOut("PX4 vehicle_status stale - link lost?");
    return;
  }
  // PX4 declared a failsafe (RC loss, battery, geofence, ...): it is flying its own plan now.
  if (status_->failsafe) {
    bailOut("PX4 failsafe active (nav_state " + std::to_string(status_->nav_state) + ")");
    return;
  }
  // Checks if someone else (RC switch, GCS) changed the flight mode after we had OFFBOARD.
  if (offboard_confirmed_) {
    if (inOffboard()) {
      not_offboard_since_.reset();
    } else if (!not_offboard_since_) { // First tick we are offboard
      not_offboard_since_ = now();
    } else if ((now() - *not_offboard_since_).seconds() > kOffboardDropoutSec) {
      bailOut("PX4 left OFFBOARD (nav_state " + std::to_string(status_->nav_state) + ")");
      return;
    }
  }
  // Disarmed while we thought we were flying (kill switch, land detector, ...).
  if (armed_confirmed_ && !isArmed()) {
    bailOut("PX4 disarmed");
    return;
  }
  // From ARMING onwards we rely on the position estimate every tick. 
  // If we don't know where the drone is, we should not be flying it
  const bool need_position = state_ != State::PRE_OFFBOARD && state_ != State::WAIT_OFFBOARD;
  if (need_position && !positionValid()) {
    bailOut("local position estimate invalid or stale");
    return;
  }
}



// ── Per-state handlers, in the order the states are normally visited ──────────────────────────────────────

// PX4 is flying the vehicle by itself; once it has landed and disarmed we are back to a
// clean IDLE.
void MissionServer::tickExternal()
{
  if (px4LinkAlive() && !isArmed()) {
    enterState(State::IDLE);
  }
}

// First step of a flight from IDLE/EXTERNAL: wait for a usable position estimate.
void MissionServer::tickWaitPosition()
{
  if (positionValid() && px4LinkAlive()) {
    startOffboardSequence();
  } else if (timeInState() > kWaitPositionTimeout) {
    finishOnGround(false, px4LinkAlive() ? "no valid local position estimate (EKF not ready?)"
                                         : "no vehicle_status from PX4 - is the uXRCE-DDS agent running?");
  }
}

// Fix reference frame for the flight from the current estimate and start streaming setpoints 
void MissionServer::startOffboardSequence()
{
  // Normally we are on the ground. If vehicle already airborn, take over at the current altitude 
  const bool airborne = isArmed() && !isOnGround();
  already_airborne_ = airborne;

  hold_yaw_ = lpos_->heading;       // Yaw kept for the whole flight
  cmd_xy_ = {lpos_->x, lpos_->y};   // Hover / pattern anchor point

  // Set NED z variables
  takeoff_start_z_ = lpos_->z;                                          // NED z where we start
  ground_z_ = airborne ? lpos_->z + params_.flight_altitude : lpos_->z; // NED of ground level 
  cmd_z_ = airborne ? lpos_->z : lpos_->z - params_.flight_altitude;    // Commanded height

  startStreaming();
  RCLCPP_INFO(
    get_logger(), "Local position valid: N=%.2f E=%.2f D=%.2f heading %.0f deg (%s)",
    lpos_->x, lpos_->y, lpos_->z, hold_yaw_ * 180.0 / M_PI, 
    airborne ? "airborne" : "on ground");
  enterState(State::PRE_OFFBOARD);
}

// PX4 rejects DO_SET_MODE OFFBOARD unless it has recently received setpoints; send 10 before asking
void MissionServer::tickPreOffboard()
{
  if (setpoint_count_ >= kSetpointsBeforeOffboard) {
    requestOffboard();
    enterState(State::WAIT_OFFBOARD);
  }
}

// DO_SET_MODE was sent; wait until vehicle_status confirms OFFBOARD, then arm / take off.
void MissionServer::tickWaitOffboard()
{
  if (inOffboard()) {
    offboard_confirmed_ = true;  // From now on leaving OFFBOARD is a bail-out condition
    RCLCPP_INFO(get_logger(), "OFFBOARD confirmed");

    if (already_airborne_) {      // Took over an airborne vehicle: no takeoff needed
      armed_confirmed_ = true;
      startPattern();              
    } else {                      // Harmless if already armed: tickArming moves straight on to TAKEOFF
      requestArm(true);            
      enterState(State::ARMING);
    }
    return;
  }
  // Retry command after timeout
  if (commandRetryDue()) {
    requestOffboard();
  }
  // Give up after time in state timer runs out
  if (timeInState() > kWaitOffboardTimeout) {
    abortOnGround("timeout waiting for OFFBOARD mode (last ack: " + lastAckName() + ")");
  }
}

// ARM was sent; wait for arming_state == ARMED (PX4 runs its preflight checks meanwhile).
void MissionServer::tickArming()
{
  if (isArmed()) {
    armed_confirmed_ = true;  // From now on a disarm is a bail-out condition
    RCLCPP_INFO(get_logger(), "Armed - taking off to %.1f m", params_.flight_altitude);
    enterState(State::TAKEOFF);
    return;
  }
  // Retry command after timeout
  if (commandRetryDue()) {
    requestArm(true);
  }
  // Give up after time in state timer runs out
  if (timeInState() > kArmingTimeout) {
    abortOnGround("timeout waiting for arming (last ack: " + lastAckName() +
      ") - check the PX4 console for the failing preflight check");
  }
}

// Setpoint indicate flight altitude (cmd_z_); PX4's position does the climbing, we just detect arrival 
void MissionServer::tickTakeoff()
{
  // For CLI feedback
  const double climbed = takeoff_start_z_ - lpos_->z;  // Metres climbed so far (z is down)
  const double total = takeoff_start_z_ - cmd_z_;
  progress_ = total > 0.0 ? std::clamp(climbed / total, 0.0, 1.0) : 1.0;

  // Arrived when close to the target altitude and no longer moving vertically.
  const bool at_altitude = std::fabs(lpos_->z - cmd_z_) < params_.position_tolerance;
  const bool settled = std::fabs(lpos_->vz) < params_.velocity_tolerance;

  if (at_altitude && settled) {
    RCLCPP_INFO(get_logger(), "Takeoff complete at %.1f m", -(lpos_->z - ground_z_));
    startPattern();
    return;
  }

  // Give up after time in state timer runs out
  if (timeInState() > kTakeoffTimeout) {
    finishInHover(false, "takeoff timeout - holding position");
  }
}

// Build the requested shape and start following it (state FLYING).
void MissionServer::startPattern()
{
  // Create the requested path object
  const double heading = hold_yaw_;
  std::shared_ptr<const Path> path;
  switch (goal_kind_) {
    case GoalKind::CIRCLE: path = makeCircle(cmd_xy_, goal_value_, heading); break;
    case GoalKind::SQUARE: path = makeSquare(cmd_xy_, goal_value_, heading); break;
    case GoalKind::TRIANGLE: path = makeTriangle(cmd_xy_, goal_value_, heading); break;
    default:
      finishInHover(false, "internal error: no pattern to fly");
      return;
  }

  // The sampler turns path into a moving carrot with a speed profile.
  sampler_ = std::make_unique<PathSampler>(path, params_.cruise_speed, params_.max_accel);
  carrot_ = sampler_->current();
  max_pattern_time_ = 2.0 * path->perimeter() / params_.cruise_speed + 15.0;

  RCLCPP_INFO(get_logger(), "Flying %s %.1f m (path %.1f m, heading %.0f deg) from N=%.1f E=%.1f at %.1f m AGL",
    toString(goal_kind_), goal_value_, path->perimeter(), heading * 180.0 / M_PI, cmd_xy_.x, cmd_xy_.y,
    -cmd_z_ + ground_z_);

  enterState(State::FLYING);
}

// Follow the pattern: move the carrot along the path and send it as the setpoint.
void MissionServer::tickFlying(double dt)
{
  const Vec2 measured{lpos_->x, lpos_->y};
  const double tracking_error = (measured - carrot_.position).norm();
  
  // Only advance the carrot while  drone keeps up. If it lags behind,the carrot waits
  if (tracking_error <= params_.max_tracking_error) {
    carrot_ = sampler_->advance(dt);  // Move the carrot
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Tracking error %.2f m > %.2f m - pausing the reference", tracking_error, params_.max_tracking_error);
    carrot_ = sampler_->current();    // Keep the carrot at the same spot
  }

  // Update command and progress
  cmd_xy_ = carrot_.position;
  cmd_vel_ = carrot_.done ? Vec2{NAN, NAN} : carrot_.velocity;  // Feed-forward
  progress_ = carrot_.progress;

  // When the shape is done, check drone is within the accepted distance and velocity range
  if (carrot_.done) {
    const double speed = std::hypot(lpos_->vx, lpos_->vy);
    if (tracking_error < params_.position_tolerance && speed < params_.velocity_tolerance) {
      finishInHover(true, std::string(toString(goal_kind_)) + " complete");
      return;
    }
  }

  // Give up after time in state timer runs out
  if (timeInState() > max_pattern_time_) {
    finishInHover(false, "pattern timeout - holding position");
  }
}

// Land and disarm. Try to make PX4 land the drone. Descend ourselves if it refuses
void MissionServer::tickLanding()
{
  if (!landing_cmd_sent_) {
    // A skipped send (service not ready) leaves the flag false and is retried next tick
    landing_cmd_sent_ = sendCommand(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    return;
  }
  if (!px4LinkAlive()) {
    bailOut("PX4 vehicle_status stale during landing");
    return;
  }
  if (streaming_ && !self_landing_) {
    chooseLandingMethod();
  }
  // In the fallback PX4 will not disarm by itself: do it once the land detector fires
  if (self_landing_ && isOnGround() && isArmed() && commandRetryDue()) {
    requestArm(false);
  }
  // Whether we landed drone or PX4 did, once on the ground and disarmed = done.
  if (isOnGround() && !isArmed()) {
    finishOnGround(true, "landed and disarmed");
    return;
  }
  // Give up after time in state timer runs out
  if (timeInState() > kLandingTimeout) {
    if (streaming_ && inOffboard() && !isOnGround()) {
      // Still airborne under our control: stop descending and hover, the user can retry.
      finishInHover(false, "landing timeout");
    } else {
      bailOut("landing timeout");
    }
  }
}

// NAV_LAND has been sent and we are still streaming: hand over to PX4 once it switches to
// LAND mode, or start the offboard descent if it does not switch in time.
void MissionServer::chooseLandingMethod()
{
  if (status_->nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_LAND) {
    RCLCPP_INFO(get_logger(), "PX4 is in LAND mode - handing over");
    stopStreaming();

  } else if (timeInState() > kLandModeWaitSec && inOffboard()) {
    RCLCPP_WARN(get_logger(), "PX4 did not switch to LAND mode (last ack: %s) - descending in offboard instead",
      lastAckName().c_str());
    self_landing_ = true;
  }
}

}  // namespace drone_mission



// ── main ──────────────────────────────────────────────────────────────────────────────────────────────────

namespace
{
std::atomic<bool> g_shutdown_requested{false};
void requestShutdown(int) { g_shutdown_requested = true; }
}  // namespace

int main(int argc, char ** argv)
{
  // Handle SIGINT/SIGTERM ourselves so that an active goal can be aborted (and the result
  // delivered to the client) while the ROS context is still alive.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, requestShutdown);
  std::signal(SIGTERM, requestShutdown);

  auto node = std::make_shared<drone_mission::MissionServer>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Spin in short slices so the loop notices the signal flag promptly.
  while (rclcpp::ok() && !g_shutdown_requested) {
    executor.spin_once(std::chrono::milliseconds(100));
  }

  node->shutdown();

  // Give the action server a moment to publish the result / status.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(std::chrono::milliseconds(50));
  }
  
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
