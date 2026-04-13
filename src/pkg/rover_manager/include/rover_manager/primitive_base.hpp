#ifndef ROVER_MANAGER__PRIMITIVE_BASE_HPP_
#define ROVER_MANAGER__PRIMITIVE_BASE_HPP_

#include <string>
#include <memory>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace rover_manager
{

/**
 * @brief Status of a primitive's execution.
 */
enum class PrimitiveStatus
{
  IDLE,       ///< Not running, ready to accept execute()
  RUNNING,    ///< Currently executing
  SUCCEEDED,  ///< Completed successfully
  FAILED,     ///< Completed with failure
  CANCELLED   ///< Was cancelled by the manager
};

/**
 * @brief Convert PrimitiveStatus to string for logging.
 */
inline std::string statusToString(PrimitiveStatus s)
{
  switch (s) {
    case PrimitiveStatus::IDLE:      return "IDLE";
    case PrimitiveStatus::RUNNING:   return "RUNNING";
    case PrimitiveStatus::SUCCEEDED: return "SUCCEEDED";
    case PrimitiveStatus::FAILED:    return "FAILED";
    case PrimitiveStatus::CANCELLED: return "CANCELLED";
    default:                         return "UNKNOWN";
  }
}

/**
 * @brief Abstract base interface for movement primitives.
 *
 * Every primitive (goto, cover_area, move_to_pose, return_to_base, etc.)
 * must inherit from this class and implement the pure virtual methods.
 *
 * The RoverManager loads N primitives via pluginlib, receives commands,
 * dispatches to the appropriate primitive, and calls tick() periodically.
 *
 * Lifecycle:
 *   IDLE -> execute(args) -> RUNNING -> tick() -> RUNNING (loop)
 *                                              -> SUCCEEDED
 *                                              -> FAILED
 *                         -> cancel() -> CANCELLED
 *   After SUCCEEDED/FAILED/CANCELLED -> reset() -> IDLE
 */
class PrimitiveBase
{
public:
  virtual ~PrimitiveBase() = default;
  const std::string & getInstancePredicate() const { return instance_predicate_; }

  /**
   * @brief Initialize the primitive with the parent ROS2 node.
   *
   * Called once at startup. Used to declare/read parameters,
   * create action clients, publishers, etc.
   *
   * @param node   The parent ROS2 node (RoverManager)
   * @param name   Unique name for this primitive (parameter namespace)
   * @param ns     Robot namespace (e.g., "robot1") for topic/action names
   */
  virtual void initialize(
    rclcpp::Node::SharedPtr node,
    const std::string & name,
    const std::string & ns) = 0;

  /**
   * @brief Start execution of the primitive.
   *
   * Non-blocking: sets up the goal and transitions to RUNNING.
   * The actual work happens in tick().
   *
   * @param args  Command arguments (e.g., "1.0,2.0,0.5" for goto)
   * @return true if execution started successfully
   */
  virtual bool execute(const std::vector<std::string> & args) = 0;

  /**
   * @brief Advance the primitive's state by one step.
   *
   * Called periodically by the RoverManager (e.g., every 200ms).
   * The primitive checks its internal state (e.g., has Nav2 finished?)
   * and transitions accordingly.
   */
  virtual void tick() = 0;

  /**
   * @brief Cancel the current execution.
   *
   * Transitions to CANCELLED. Should cancel any pending Nav2 goals.
   */
  virtual void cancel() = 0;

  /**
   * @brief Reset the primitive to IDLE state.
   *
   * Called after SUCCEEDED/FAILED/CANCELLED to prepare for reuse.
   */
  virtual void reset()
  {
    status_ = PrimitiveStatus::IDLE;
    feedback_msg_ = "";
  }

  /**
   * @brief Get the current status.
   */
  PrimitiveStatus getStatus() const { return status_; }

  /**
   * @brief Get human-readable feedback about current execution.
   */
  const std::string & getFeedback() const { return feedback_msg_; }

  /**
   * @brief Get the name of this primitive.
   */
  const std::string & getName() const { return name_; }

protected:
  std::string name_;
  std::string robot_ns_;
  PrimitiveStatus status_ = PrimitiveStatus::IDLE;
  std::string feedback_msg_;
  rclcpp::Logger logger_ = rclcpp::get_logger("primitive_base");
  std::string instance_predicate_;  // es. "goto(5,3,0)"
  
};

}  // namespace rover_manager

#endif  // ROVER_MANAGER__PRIMITIVE_BASE_HPP_
