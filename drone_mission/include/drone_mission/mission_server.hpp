// Mission server: arms a PX4 vehicle, switches it to OFFBOARD, takes off, flies geometric
// patterns and lands, driven by the MissionCommand action.
//
// Life of a pattern goal ("circle 5") starting on the ground:
//   IDLE -> WAIT_POSITION (EKF position valid?) -> PRE_OFFBOARD (stream a few setpoints)
//        -> WAIT_OFFBOARD (DO_SET_MODE) -> ARMING (ARM) -> TAKEOFF -> FLYING -> HOVER
// A pattern from HOVER goes straight to FLYING; "land" goes HOVER -> LANDING -> IDLE.
// "stop" (action cancel) returns to HOVER when airborne, or IDLE when still on the ground.
// Anything unexpected from PX4 (failsafe, mode change, lost link) ends in EXTERNAL, where we
// stop touching the vehicle until PX4 reports it disarmed.
#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "drone_mission/path.hpp"
#include "drone_mission_interfaces/action/mission_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/telemetry_status.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/srv/vehicle_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace drone_mission
{

using MissionCommand = drone_mission_interfaces::action::MissionCommand;
using GoalHandle = rclcpp_action::ServerGoalHandle<MissionCommand>;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TelemetryStatus;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleCommandAck;
using px4_msgs::msg::VehicleLandDetected;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;
// Aliased: a bare using would collide with the VehicleCommand message above
using VehicleCommandSrv = px4_msgs::srv::VehicleCommand;


// ──────────────────────────────────────────────────────────────────────────────────────────────────────────
// General
// ──────────────────────────────────────────────────────────────────────────────────────────────────────────
enum class State
{
  IDLE,           // On the ground, nothing streamed
  WAIT_POSITION,  // Goal accepted, waiting for a valid EKF local position estimate
  PRE_OFFBOARD,   // Streaming setpoints before requesting OFFBOARD (after kSetpointsBeforeOffboard)
  WAIT_OFFBOARD,  // DO_SET_MODE sent, waiting for nav_state == OFFBOARD
  ARMING,         // ARM sent, waiting for arming_state == ARMED
  TAKEOFF,        // Climbing to the flight altitude
  HOVER,          // Holding position, ready for the next command
  FLYING,         // Following a pattern
  LANDING,        // NAV_LAND sent, waiting for touchdown + disarm
  EXTERNAL,       // PX4 took over (failsafe / mode change); 'land' still accepted
};

enum class GoalKind { NONE, CIRCLE, SQUARE, TRIANGLE, LAND };
// Parse command
std::optional<GoalKind> parseGoalKind(const std::string & cmd);

// Convert to human readable string
const char * toString(State s);
const char * toString(GoalKind k);
const char * ackResultName(uint8_t result);

// Tunables, each exposed as a ROS parameter of the same name. 
struct Params
{
  // Flight
  double flight_altitude{5.0};            // m above the takeoff point
  double cruise_speed{1.5};               // m/s along the pattern
  double max_accel{1.0};                  // m/s^2 for the speed ramps into / out of corners
  double min_pattern_size{0.5};           // m, smallest accepted radius / side
  double max_pattern_size{50.0};          // m, largest accepted radius / side
  double position_tolerance{0.5};         // m, "reached" distance
  double velocity_tolerance{0.3};         // m/s, "settled" speed
  double max_tracking_error{1.0};         // m, the carrot pauses if the drone lags further
  double control_rate_hz{20.0};           // Hz, how often tick() runs 
  // PX4 addressing / behaviour
  int target_system{1};                   // MAVLink system id of the vehicle (1 in SITL)
  bool publish_gcs_heartbeat{true};       // Pretend to be a GCS so PX4 arms with NAV_DLL_ACT > 0
};


// ──────────────────────────────────────────────────────────────────────────────────────────────────────────
// The Node
// ──────────────────────────────────────────────────────────────────────────────────────────────────────────
class MissionServer : public rclcpp::Node
{
public:
  MissionServer();

  // Aborts in-progress drone shape and hands control back to PX4
  void shutdown();

private:
  // Constants
  static constexpr float kFallbackDescentRate = 0.7;        // m/s downwards (NED +z)
  static constexpr double kMaxTickDt = 0.2;                 // s, clamp for dt after a hiccup
  static constexpr int kFeedbackEveryNTicks = 5;            // ~4 Hz, send feedback every 5th tick (20 Hz / 5 = 4 Hz)
  static constexpr double kPositionStaleSec = 0.5;          // s, Local position older than this is unusable
  static constexpr uint32_t kSetpointsBeforeOffboard = 10;  // PX4 wants setpoints flowing first
  static constexpr double kCommandRetrySec = 1.0;           // s, Re-send an unanswered command after this
  static constexpr double kOffboardDropoutSec = 0.5;        // s, Tolerated nav_state != OFFBOARD glitch
  static constexpr double kLandModeWaitSec = 5.0;           // s, Wait for PX4 to enter LAND before fallback
  // ── Per-state timeouts ──
  static constexpr double kWaitPositionTimeout = 60.0;      // s, Waiting for a valid EKF position
  static constexpr double kWaitOffboardTimeout = 5.0;       // s, Waiting for nav_state == OFFBOARD
  static constexpr double kArmingTimeout = 5.0;             // s, Waiting for arming_state == ARMED
  static constexpr double kTakeoffTimeout = 30.0;           // s, Climbing to the flight altitude
  static constexpr double kLandingTimeout = 90.0;           // s, Waiting for touchdown + disarm
  static constexpr double kStatusStaleTimeout = 2.0;        // s, No vehicle_status for this long = link lost

  // Register node params with ROS
  Params declareParams();


  // ── Action server callbacks (the CLI is the client) ─────────────────────────────────────────────────────
  // Accepts or rejects commands
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MissionCommand::Goal> goal);

  // Starts executing an accepted goal
  void handleAccepted(std::shared_ptr<GoalHandle> handle);

  // Handles goal cancellation
  rclcpp_action::CancelResponse handleCancel(std::shared_ptr<GoalHandle> handle);


  // ── Topics in: read latest cached messages ──────────────────────────────────────────────────────────────
  bool px4LinkAlive() const;                      // Checks if PX4 and DDS link are stil alive
  bool isArmed() const;                           // Checks if vehicle armed
  bool positionValid() const;                     // Px4 tells us if self-reported EKF is reliable
  bool inOffboard() const;                        // Checks if PX4 currently in OFFBOARD mode
  bool isOnGround() const;                        // Checks if drone on ground
  bool airborneWithoutUs() const;                 // Checks if PX4 has the vehicle in the air while we are not in control



  // ── Topics out: streamed to PX4 ─────────────────────────────────────────────────────────────────────────
  void publishSetpoints();                        // Every tick (20Hz), sends PX4 a flight target
  void heartbeatTick();                           // 1Hz, sends GCS heartbeat to pretend mission_server is a GCS

  

  // ── Service out: send a request ────────────────────────────────────────────────────────────────────────
  bool sendCommand(                               // Sends MAVLink-style VehicleCommand to PX4
    uint32_t command, float p1 = 0.0, float p2 = 0.0, float p3 = 0.0, 
    float p4 = 0.0, double p5 = 0.0, double p6 = 0.0, float p7 = 0.0);
  void requestOffboard();                         // Switches PX4 into OFFBOARD mode
  void requestArm(bool arm);                      // Arms/disarms



  // ── Service in: the reply to the last request ───────────────────────────────────────────────────────────
  void onCommandReply(const VehicleCommandAck & ack); // Called when we receive PX4 ACKs for cmd
  std::string lastAckName() const;                // Gets human readable ACK for last cmd 
  bool commandRetryDue() const;                   // Checks if need resend last cmd

  // ── Shared by topics and services ─
  uint64_t nowUs() const;                         // Current time in microseconds



  // ── State machine predicates ──────────────────────────────────────────────────────────
  void finishGoal(bool success, const std::string & message); // Resolve the goal as cancelled/succeeded/aborted
  void publishFeedback();                         // Send state/progress/position back to CLI                  
  void enterState(State s);                       // Switches to a state and logs the transition
  double timeInState() const;                     // Counts seconds spent in a state
  void startStreaming();                          // Begin publishing setpoints
  void stopStreaming();                           // Stop publishing setpoints
  void holdHere();                                // Freeze the setpoint stream and drop any active patterns

  

  // ── Goal exit paths ──────────────────────────────────────────────────────────────────
  void finishInHover(bool success, const std::string & message);  // End goal while airborn
  void finishOnGround(bool success, const std::string & message); // End goal landed and disarmed, back to IDLE
  void abortOnGround(const std::string & reason);                 // End goal on the ground
  void bailOut(const std::string & reason);                       // Drop goal and return control to PX4, in case of bug



  // ── The control loop and its per-state handlers ─────────────────────────────────────────────────────────
  void tick();                                    // The main 20Hz loop
  void processCancel();                           // Handles a "stop" command
  void processStart();                            // Handles a new accepted goal
  bool controllingVehicle() const;                // True while we are in charge of drone
  void checkWatchdogs();                          // Per-tick safety checks
  // ── Per-state handlers ──
  void tickExternal();                            // EXTERNAL state: waits for PX4 to disarm and returns to IDLE
  void tickWaitPosition();                        // WAIT_POSITION state: Checks for valid EFK position each tick
  void startOffboardSequence();                   // Starts streaming setpoints, entering PRE_OFFBOARD
  void tickPreOffboard();                         // PRE_OFFBOARD state: wait for enough setpts to be sent then request OFFBOARD
  void tickWaitOffboard();                        // WAIT_OFFBOARD state: wait for OPFFBOARD confirm, then start pattern
  void tickArming();                              // ARMING: wait for PX4 confirm armed, then proceed to TAKEOFF
  void tickTakeoff();                             // TAKEOFF state: track climb and detect arrival at takeoff height
  void startPattern();                            // Build shape path and enter FLYING
  void tickFlying(double dt);                     // FLYING state: advance path ref and detect completion
  void tickLanding();                             // LANDING state: send NAV_LAND, hand over ctrl, detect touchdown
  void chooseLandingMethod();                     // Chooses whether we land the drone or PX4 does



// ──────────────────────────────────────────────────────────────────────────────────────────────────────────
// Member Variables
// ──────────────────────────────────────────────────────────────────────────────────────────────────────────
  Params params_;


  // ── Action server: CLI is the client ────────────────────────────────────────────────────────────────────
  rclcpp_action::Server<MissionCommand>::SharedPtr action_server_;  // Receives goals from the CLI

  // ── The goal being executed ──
  std::shared_ptr<GoalHandle> goal_;
  GoalKind goal_kind_{GoalKind::NONE};
  double goal_value_{0.0};                        // m, radius / side length
  bool pending_start_{false};                     // handleAccepted ran; processStart() runs on next tick
  bool cancel_requested_{false};                  // handleCancel ran; processCancel() runs on next tick


  
  // ── Topics in: Sub + latest cached msg + arrival time ───────────────────────────────────────────────────
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  std::optional<VehicleStatus> status_;           // Arming_state, nav_state, failsafe
  rclcpp::Time status_rx_;                        // Timestamp

  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr lpos_sub_;
  std::optional<VehicleLocalPosition> lpos_;      // EKF position / velocity in NED + heading
  rclcpp::Time lpos_rx_;                          // Timestamp

  rclcpp::Subscription<VehicleLandDetected>::SharedPtr land_sub_;
  std::optional<VehicleLandDetected> land_;       // Touchdown detector



  // ── Topics out: publisher + setpoint being streamed ─────────────────────────────────────────────────────
  rclcpp::Publisher<OffboardControlMode>::SharedPtr ocm_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr sp_pub_;
  bool streaming_{false};                         // publishSetpoints() runs every tick while true
  uint32_t setpoint_count_{0};                    // Setpoints sent since streaming started 

  // ── Flight frame - NED coordinate frame ──
  Vec2 cmd_xy_;                                   // Commanded horizontal position
  Vec2 cmd_vel_{NAN, NAN};                        // Commanded orizontal velocity 
  double cmd_z_{0.0};                             // Commanded altitude as NED z (negative = above origin)
  double hold_yaw_{0.0};                          // Yaw held during the whole flight
  double ground_z_{0.0};                          // NED z of the ground, for AGL values in the log
  double takeoff_start_z_{0.0};                   // NED z when the climb started, for takeoff progress

  // ── Fake Heartbeat ──
  rclcpp::Publisher<TelemetryStatus>::SharedPtr telem_pub_; // Fake GCS heartbeat
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;            // Drives heartbeatTick() 



  // ── Service: client, the request in flight and reply ────────────────────────────────────────────────────
  rclcpp::Client<VehicleCommandSrv>::SharedPtr cmd_client_; // The service client
  std::optional<int64_t> pending_request_id_;     // Outstanding service request, pruned on re-send
  rclcpp::Time last_cmd_time_;
  std::optional<uint8_t> last_ack_;               // VEHICLE_CMD_RESULT_* id



  // ── State machine and control loop ──────────────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr control_timer_;    // Drives tick() loop
  State state_{State::IDLE};                      // Holds the state we are currently in
  rclcpp::Time state_entry_time_;                 // For timeInState() / the per-state timeouts
  rclcpp::Time last_tick_;                        // For dt in tick()
  uint64_t tick_count_{0};
  double progress_{0.0};                          // 0..1, used in TAKEOFF / FLYING states

  // ── What PX4 has confirmed so far, loosing any triggers bailout ──
  bool offboard_confirmed_{false};                // PX4 was seen in OFFBOARD
  bool armed_confirmed_{false};                   // PX4 was seen armed
  std::optional<rclcpp::Time> not_offboard_since_;  // Start of a nav_state != OFFBOARD 
  bool already_airborne_{false};                  // Took over a vehicle that was already flying

  // ── Landing ──
  bool landing_cmd_sent_{false};
  bool self_landing_{false};                      // Descending in OFFBOARD because PX4 would not land

  // ── Pattern being flown ──
  std::unique_ptr<PathSampler> sampler_;          // Null when not FLYING
  PathSample carrot_;                             // Carrot: point on the path the drone chases, sent as the setpoint in FLYING
  double max_pattern_time_{0.0};                  // Seconds allowed for the current pattern
};

}  // namespace drone_mission
