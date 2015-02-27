/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <iostream>

using namespace wheelwalking_control;
using namespace exoter;

Task::Task(std::string const& name)
    : TaskBase(name), deadmans_switch(true), kill_switch(false), constant_speed_mode(false), constant_speed(0.0d), MAX_SPEED(0.03d)
{
    wheelwalking_control = new ExoterWheelwalkingControl(0.07);
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine), deadmans_switch(true), kill_switch(false), constant_speed_mode(false), constant_speed(0.0d), MAX_SPEED(0.03d)
{
    wheelwalking_control = new ExoterWheelwalkingControl(0.07);
}

Task::~Task()
{
    delete wheelwalking_control;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    joint_commands_names = _joint_commands_names.get();
    joint_readings_names = _joint_readings_names.get();
    last_position_commands.assign(NUMBER_OF_ACTIVE_JOINTS,0.0d);
    last_velocity_commands.assign(NUMBER_OF_ACTIVE_JOINTS,0.0d);
    first_iteration=true;
 
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
   return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    controldev::RawCommand joystick_commands;
    base::samples::Joints joint_readings;

    if (_joystick_commands.read(joystick_commands) == RTT::NewData)
        evaluateJoystickCommands(joystick_commands);

    if (_joint_readings.read(joint_readings) == RTT::NewData)
        evaluateJointReadings(joint_readings);

    std::vector<double> position_commands;
    std::vector<double> velocity_commands;

    if (kill_switch)
    {
        position_commands.assign(NUMBER_OF_ACTIVE_JOINTS, std::numeric_limits<double>::quiet_NaN());
        velocity_commands.assign(NUMBER_OF_ACTIVE_JOINTS, 0.0d);
    }
    else
    {
        wheelwalking_control->getJointCommands(position_commands, velocity_commands);
    }

    if (position_commands != last_position_commands || velocity_commands != last_velocity_commands)
    {
        //std::cout << "Wheel Walking Control: Update Hook: New command sent to joint dispatcher" << std::endl;
        base::commands::Joints joint_commands = assembleJointCommands(position_commands, velocity_commands);
        _joint_commands.write(joint_commands);
        last_position_commands=position_commands;
        last_velocity_commands=velocity_commands;
    }
}

void Task::evaluateJoystickCommands(const controldev::RawCommand joystick_commands)
{
    if (first_iteration)
    {
        last_button_values = joystick_commands.buttons.elements;
        last_axes_values = joystick_commands.axes.elements;
        first_iteration=false;
    }
    if (joystick_commands.buttons[7] == 1 && last_button_values[7] == 0)    //BTN_Z (right bottom)
    {
        kill_switch = !kill_switch;

        if (kill_switch)
            std::cout << "Kill switch engaged." << std::endl;
        else
            std::cout << "Kill switch disengaged." << std::endl;
    }
    if (joystick_commands.buttons[5] == 1 && last_button_values[5] == 0)    //BTN_TR (right topi)
    {
        wheelwalking_control->selectNextGait();
	constant_speed = 0.0d;
        std::cout << "Set speed to " << constant_speed << " m/s." << std::endl;
    }
    if (joystick_commands.axes[5] == 1 && last_axes_values[5] == 0 && constant_speed_mode)     //ABS_HAT0Y (dpad)
    {
        if (constant_speed <= MAX_SPEED - 0.01d)
        {
            constant_speed += 0.01d;
            std::cout << "Set speed to " << constant_speed << " m/s." << std::endl;
        }
    }
    if (joystick_commands.axes[5] == -1 && last_axes_values[5] == 0 && constant_speed_mode)     //ABS_HAT0Y (dpad)
    {
        if (constant_speed >= -MAX_SPEED + 0.01d)
        {
            constant_speed -= 0.01d;
            std::cout << "Set speed to " << constant_speed << " m/s." << std::endl;
        }
    }
    if (joystick_commands.buttons[10] == 1 && last_button_values[10] == 0)    //BTN_SELECT (left push button)
    {
        if (!constant_speed_mode)
        {
            constant_speed_mode = true;
	    constant_speed = 0.0d;
            std::cout << "Switched to discrete speed mode." << std::endl;
            std::cout << "Set speed to " << constant_speed << " m/s." << std::endl; 
        }
        else
        {
            constant_speed_mode = false;
            std::cout << "Switched to continuous speed mode." << std::endl;
        }
    }

    if (constant_speed_mode)
        wheelwalking_control->setSpeed(constant_speed);
    else
        wheelwalking_control->setSpeed(joystick_commands.axes[1] * MAX_SPEED);   //ABS_Y (left analog) - sign might have to be switched!!

    last_button_values = joystick_commands.buttons.elements;
    last_axes_values = joystick_commands.axes.elements;
}

void Task::evaluateJointReadings(const base::samples::Joints joint_readings)
{
    // Joint order in postion_readings & velocity_readings: [left_passive, right_passive, rear_passive, fl_walking, fr_walking, ml_walking, mr_walking, rl_walking, rr_walking, fl_steer, fr_steer, rl_steer, rr_steer, fl_drive, fr_drive, ml_drive, mr_drive, rl_drive, rr_drive]

    std::vector<double> position_readings;
    std::vector<double> velocity_readings;

    position_readings.resize(NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_ACTIVE_JOINTS);
    velocity_readings.resize(NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_ACTIVE_JOINTS);

    base::JointState current_joint;

    for (int i = 0; i < NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_ACTIVE_JOINTS; i++)
    {
        current_joint = joint_readings[joint_readings_names[i]];

        position_readings[i] = current_joint.hasPosition() ? current_joint.position : std::numeric_limits<double>::quiet_NaN();
        velocity_readings[i] = current_joint.hasSpeed() ? current_joint.speed : std::numeric_limits<double>::quiet_NaN();
    }

    wheelwalking_control->setNewJointReadings(position_readings, velocity_readings);
}

base::commands::Joints Task::assembleJointCommands(const std::vector<double> position_commands, const std::vector<double> velocity_commands)
{
    // Joint order in position_commands & velocity_commands: [WHEEL_WALK_FL, WHEEL_WALK_FR, WHEEL_WALK_CL, WHEEL_WALK_CR, WHEEL_WALK_BL, WHEEL_WALK_BR, WHEEL_STEER_FL, WHEEL_STEER_FR, WHEEL_STEER_BL, WHEEL_STEER_BR, WHEEL_DRIVE_FL, WHEEL_DRIVE_FR, WHEEL_DRIVE_CL, WHEEL_DRIVE_CR, WHEEL_DRIVE_BL, WHEEL_DRIVE_BR]

    base::commands::Joints joint_commands;
    joint_commands.resize(joint_commands_names.size());

    joint_commands.names = joint_commands_names;

    double current_position_command;
    double current_velocity_command;

    for (int i = 0; i < NUMBER_OF_WALKING_WHEELS; i++)
    {
        current_position_command = position_commands[i];
        current_velocity_command = velocity_commands[i];

        if (!(current_position_command != current_position_command))    // Only set value if position_command is not NAN.
            joint_commands.elements[NUMBER_OF_WHEELS + NUMBER_OF_STEERABLE_WHEELS + i].position = current_position_command;

        if (!(current_velocity_command != current_velocity_command))    // Only set value if velocity_command is not NAN.
            joint_commands.elements[NUMBER_OF_WHEELS + NUMBER_OF_STEERABLE_WHEELS + i].speed = current_velocity_command;
    }

    for (int i = 0; i < NUMBER_OF_STEERABLE_WHEELS; i++)
    {
        current_position_command = position_commands[NUMBER_OF_WALKING_WHEELS + i];
        current_velocity_command = velocity_commands[NUMBER_OF_WALKING_WHEELS + i];

        if (!(current_position_command != current_position_command))    // Only set value if position_command is not NAN.
            joint_commands.elements[NUMBER_OF_WHEELS + i].position = current_position_command;

        if (!(current_velocity_command != current_velocity_command))    // Only set value if velocity_command is not NAN.
            joint_commands.elements[NUMBER_OF_WHEELS + i].speed = current_velocity_command;
    }

    for (int i = 0; i < NUMBER_OF_WHEELS; i++)
    {
        current_position_command = position_commands[NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS + i];
        current_velocity_command = velocity_commands[NUMBER_OF_WALKING_WHEELS + NUMBER_OF_STEERABLE_WHEELS + i];

        if (!(current_position_command != current_position_command))    // Only set value if position_command is not NAN.
            joint_commands.elements[i].position = current_position_command;
        if (!(current_velocity_command != current_velocity_command))    // Only set value if velocity_command is not NAN.
            joint_commands.elements[i].speed = current_velocity_command;
    }

    return joint_commands;
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
