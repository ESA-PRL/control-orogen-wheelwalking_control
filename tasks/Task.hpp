#pragma once

#include "exoter_kinematics/ExoterWheelwalkingControl.hpp"
#include "exoter_kinematics/ExoterWheelwalkingTypes.hpp"
#include "wheelwalking_control/TaskBase.hpp"

#include <base/commands/Joints.hpp>
#include <base/samples/Joints.hpp>

#include <vector>

using namespace exoter_kinematics;

namespace wheelwalking_control
{

class Task : public TaskBase
{
    friend class TaskBase;

  protected:
    ExoterWheelwalkingControl* wheelwalking_control;

  private:
    bool first_iteration;

    std::vector<int> last_button_values;
    std::vector<double> last_axes_values;
    std::vector<double> position_commands;
    std::vector<double> velocity_commands;
    std::vector<double> last_position_commands;
    std::vector<double> last_velocity_commands;
    std::vector<std::string> joint_commands_names;
    std::vector<std::string> joint_readings_names;
    std::vector<std::string> disabled_walking_joints;

    bool deadmans_switch;
    bool kill_switch;
    bool reset_dep_joints;
    bool discrete_speed_mode;
    int discrete_speed;
    int offset_speed;
    int step_length;

    /** Maps the button presses and joystick movements to the corresponding function calls in the
     * egress control and realizes the intended human interface behaviour.
     * \param joystick_commands Raw commands from joystick containing axes and button values.
     */
    void evaluateJoystickCommands(const controldev::RawCommand joystick_commands);

    /** Maps the input joint readings to the order that is expected by egress control.
     * \param joint_readings Joint readings from the joint dispatcher.
     */
    void evaluateJointReadings(const base::samples::Joints joint_readings);

    /** Assembles the output joint command vector from the calculated joint positions and
     * velocities.
     * \param position_commands Position commands computed by egress control.
     * \param velocity_commands Velocity commands computed by egress control.
     */
    base::commands::Joints assembleJointCommands(const std::vector<double> position_commands,
                                                 const std::vector<double> velocity_commands);

    /** Compares the last vector commands with the new vector of commands within a range.
    * \return boolean value of the comparison. True if any command in the vector differs from the
    * last value more than a given range.
    */
    bool differentCommands();

  public:
    Task(std::string const& name = "wheelwalking_control::Task");
    Task(std::string const& name, RTT::ExecutionEngine* engine);
    ~Task();

    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}
