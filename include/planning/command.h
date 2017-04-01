/** PerceptIn license **/

#ifndef PLANNING_INCLUDE_PLANNING_COMMAND_H_
#define PLANNING_INCLUDE_PLANNING_COMMAND_H_

namespace PI {
namespace planning {

namespace command {
/**
 * Types of robot command.
 */
enum Type {
  MOVE,   // Move forward if value is positive and backward if negative.
  TURN,   // Turn counterclockwise if positive and clockwise if negative.
};
}  // namespace command

/**
 * @brief Wraps the types and the amount of the movement.
 */
struct Command {
  /// Type of the movement.
  command::Type type;
  /// Velocity of the movement.
  double speed;

  Command();
  Command(const command::Type type, const double speed);
  ~Command();
};

}  // namespace planning
}  // namespace PI

#endif  // PLANNING_INCLUDE_PLANNING_COMMAND_H_
