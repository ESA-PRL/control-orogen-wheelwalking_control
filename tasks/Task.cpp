/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <iostream>

using namespace wheelwalking_control;
using namespace exoter_kinematics;

static const double MAX_SPEED = 0.02d; // maximum walking body speed
static const double DISCRETE_SPEED_FACTOR = 0.005d; // walking speed increment in m/s
static const double OFFSET_SPEED_FACTOR = 0.005d; // offset speed increment in m/s
static const double MAX_OFFSET_SPEED = 0.02d;
static const double STEP_LENGTH_FACTOR = 0.02d; // step length increment in m
static const double MAX_STEP_LENGTH = 0.20d;
static const double MIN_STEP_LENGTH = 0.02d;

Task::Task(std::string const& name)
    : TaskBase(name), deadmans_switch(true), kill_switch(true), discrete_speed_mode(true), discrete_speed(4), offset_speed(0), step_length(3)
{
    wheelwalking_control = new ExoterWheelwalkingControl(0.07);
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine), deadmans_switch(true), kill_switch(true), discrete_speed_mode(true), discrete_speed(4), offset_speed(0), step_length(3)
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
    position_commands.assign(NUMBER_OF_ACTIVE_JOINTS,0.0d);
    velocity_commands.assign(NUMBER_OF_ACTIVE_JOINTS,0.0d);
    last_position_commands.assign(NUMBER_OF_ACTIVE_JOINTS,0.0d);
    last_velocity_commands.assign(NUMBER_OF_ACTIVE_JOINTS,0.0d);
    first_iteration = true;

    disabled_walking_joints = _disabled_walking_joints.get();

    std::vector<bool> walking_joints_status;
    walking_joints_status.assign(6, true);

    for (unsigned int i = 0; i < disabled_walking_joints.size(); i++)
    {
        std::string disabled_joint = disabled_walking_joints[i];

        if (disabled_joint == "WHEEL_WALK_FL")
            walking_joints_status[0] = false;
        else if (disabled_joint == "WHEEL_WALK_FR")
            walking_joints_status[1] = false;
        else if (disabled_joint == "WHEEL_WALK_CL")
            walking_joints_status[2] = false;
        else if (disabled_joint == "WHEEL_WALK_CR")
            walking_joints_status[3] = false;
        else if (disabled_joint == "WHEEL_WALK_BL")
            walking_joints_status[4] = false;
        else if (disabled_joint == "WHEEL_WALK_BR")
            walking_joints_status[5] = false;
    }

    wheelwalking_control->setWalkingJointsStatus(walking_joints_status);

    wheelwalking_control->selectMode(SIDE_BY_SIDE);

    std::cout << "Initial gait: SIDE_BY_SIDE" << std::endl;
    std::cout << "Discrete speed mode " << (discrete_speed_mode ? "enabled." : "disabled.") << std::endl;
    std::cout << "Discrete speed: " << discrete_speed * DISCRETE_SPEED_FACTOR << " m/s" << std::endl;
    std::cout << "Offset speed: " << offset_speed * OFFSET_SPEED_FACTOR << " m/s" << std::endl;
    std::cout << "Step length: " << step_length * STEP_LENGTH_FACTOR << " m" << std::endl;
    std::cout << "Kill switch " << (kill_switch ? "engaged." : "disengaged.") << std::endl;

    if (kill_switch)
    	wheelwalking_control->stopMotion();

    last_kill_switch = kill_switch;
    last_resetDepJoints = false;
    resetDepJoints = false;
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

    if(_kill_switch.read(kill_switch) == RTT::NewData)
    {
        /*if (kill_switch != last_kill_switch)
        {
            last_kill_switch = kill_switch;*/
            if (kill_switch)
            {
                wheelwalking_control->stopMotion();
                std::cout << "WWCONTROL: kill switch engaged" << std::endl;
            }
            else
            {
                wheelwalking_control->startMotion();
                std::cout << "WWCONTROL: kill switch disengaged" << std::endl;
            }
        wheelwalking_control->setSpeed(0.02);
            wheelwalking_control->setOffsetSpeed(0);
            wheelwalking_control->setStepLength(0.06);
        //}
    }

    if(_reset_dep_joints.read(resetDepJoints) == RTT::NewData)
    {
        /*if((resetDepJoints != last_resetDepJoints)&&(resetDepJoints))
        {
            last_resetDepJoints = resetDepJoints;
            wheelwalking_control->initJointConfiguration();
            std::cout << "WWCONTROL: reset deployment joints" << std::endl;
        }*/
        /*else if (!resetDepJoints)
        {
            last_resetDepJoints = resetDepJoints;
        }*/
        if(resetDepJoints)
        {
            wheelwalking_control->initJointConfiguration();
            wheelwalking_control->setSpeed(0.02);
            wheelwalking_control->setOffsetSpeed(0);
            wheelwalking_control->setStepLength(0.06);
        }
    }

    if (_joystick_commands.read(joystick_commands) == RTT::NewData)
        evaluateJoystickCommands(joystick_commands);

    if (_joint_readings.read(joint_readings) == RTT::NewData)
        evaluateJointReadings(joint_readings);

    std::vector<double> position_commands;
    std::vector<double> velocity_commands;
    wheelwalking_control->getJointCommands(position_commands, velocity_commands);

    if (position_commands != last_position_commands || velocity_commands != last_velocity_commands)
//    if (differentCommands())
    {
//        std::cout << "Wheel Walking Control: Update Hook: New command sent to joint dispatcher" << std::endl;
        base::commands::Joints joint_commands = assembleJointCommands(position_commands, velocity_commands);
        /*std::cout << "Wheel Walking Control: Positions -> ";
        for (uint i = 0; i<position_commands.size(); i++)
            std::cout << position_commands[i] << " ";
        std::cout  << std::endl;
        std::cout << "Wheel Walking Control: Velocities -> ";
        for (uint i = 0; i<velocity_commands.size(); i++)
            std::cout << velocity_commands[i] << " ";
        std::cout  << std::endl;*/
        _joint_commands.write(joint_commands);
        last_position_commands=position_commands;
        last_velocity_commands=velocity_commands;
    }
}

bool Task::differentCommands()
{
    double epsilon = 0.01;
    for (int i=0;i<NUMBER_OF_ACTIVE_JOINTS;i++)
    {
        //std::cout << "position " << i << ": " << position_commands[i] << " velocity " << i << ": " << velocity_commands[i] << " abs_dif: " << std::abs(position_commands[i]-last_position_commands[i]) << std::endl;
        //std::cout << "last position " << i << ": " << last_position_commands[i] << " last velocity " << i << ": " << last_velocity_commands[i] << " abs_dif: " << std::abs(velocity_commands[i]-last_velocity_commands[i]) << std::endl;
        if ( (std::abs(position_commands[i]-last_position_commands[i])>epsilon) || (std::abs(velocity_commands[i]-last_velocity_commands[i])>epsilon) )
        {
            return true;
        }
    }
    return false;
}

void Task::evaluateJoystickCommands(const controldev::RawCommand joystick_commands)
{
    if (first_iteration)
    {
        last_button_values = joystick_commands.buttons.elements;
        last_axes_values = joystick_commands.axes.elements;
        first_iteration = false;
    }

    if (joystick_commands.buttons[6] == 1 && last_button_values[6] == 0)    //BTN_TL (left bottom)
    {
        kill_switch = true;

		wheelwalking_control->stopMotion();

        std::cout << "Kill switch engaged." << std::endl;
    }
    else if (joystick_commands.buttons[7] == 1 && last_button_values[7] == 0)    //BTN_TR (right bottom)
    {
        kill_switch = false;

		wheelwalking_control->startMotion();

        std::cout << "Kill switch disengaged." << std::endl;
    }

    if (joystick_commands.buttons[11] == 1 && last_button_values[11] == 0)	//BTN_START (right push button)
    {
    	wheelwalking_control->selectNextGait();
    }
    else if (joystick_commands.buttons[0] == 1 && last_button_values[0] == 0) //BTN_A (blue)
    {
		wheelwalking_control->selectMode(0);
    }
    else if (joystick_commands.buttons[1] == 1 && last_button_values[1] == 0) //BTN_B (green)
    {
		wheelwalking_control->selectMode(1);
    }
    else if (joystick_commands.buttons[2] == 1 && last_button_values[2] == 0) //BTN_C (red)
    {
		wheelwalking_control->selectMode(2);
    }
    else if (joystick_commands.buttons[3] == 1 && last_button_values[3] == 0) //BTN_X (yellow)
    {
		wheelwalking_control->selectMode(3);
    }
    else if (joystick_commands.buttons[9] == 1 && last_button_values[9] == 0) //BTN_TR2 (start)
    {
		wheelwalking_control->selectMode(4);
    }
	else if (joystick_commands.buttons[8] == 1 && last_button_values[8] == 0)	//BTN_TL2 (back)
	{
		wheelwalking_control->initJointConfiguration();
	}

    if (joystick_commands.buttons[5] == 1 && last_button_values[5] == 0) //BTN_Z (right top)
    {
        if ((step_length + 1) * STEP_LENGTH_FACTOR <= MAX_STEP_LENGTH)
        {
            step_length++;
            std::cout << "Set step length to " << step_length * STEP_LENGTH_FACTOR << " m." << std::endl;
        }
    }
    else if (joystick_commands.buttons[4] == 1 && last_button_values[4] == 0) //BTN_Y (left top)
    {
        if ((step_length - 1) * STEP_LENGTH_FACTOR >= MIN_STEP_LENGTH)
        {
            step_length--;
            std::cout << "Set step length to " << step_length * STEP_LENGTH_FACTOR << " m." << std::endl;
        }
    }

    if (joystick_commands.axes[5] == 1 && last_axes_values[5] == 0 && discrete_speed_mode)     //ABS_HAT0Y (dpad)
    {
        if ((discrete_speed + 1) * DISCRETE_SPEED_FACTOR <= MAX_SPEED)
        {
            discrete_speed++;
            std::cout << "Set walking speed to " << discrete_speed * DISCRETE_SPEED_FACTOR << " m/s." << std::endl;
        }
    }
    else if (joystick_commands.axes[5] == -1 && last_axes_values[5] == 0 && discrete_speed_mode)     //ABS_HAT0Y (dpad)
    {
        if ((discrete_speed - 1) * DISCRETE_SPEED_FACTOR >= -MAX_SPEED)
        {
            discrete_speed--;
            std::cout << "Set walking speed to " << discrete_speed * DISCRETE_SPEED_FACTOR << " m/s." << std::endl;
        }
    }

    if (joystick_commands.axes[4] == 1 && last_axes_values[4] == 0)     //ABS_HAT0X (dpad)
    {
        if ((offset_speed + 1) * OFFSET_SPEED_FACTOR <= MAX_OFFSET_SPEED)
        {
            offset_speed++;
            std::cout << "Set offset speed to " << offset_speed * OFFSET_SPEED_FACTOR << " m/s." << std::endl;
        }
    }
    else if (joystick_commands.axes[4] == -1 && last_axes_values[4] == 0)     //ABS_HAT0X (dpad)
    {
        if ((offset_speed - 1) * OFFSET_SPEED_FACTOR >= -MAX_OFFSET_SPEED)
        {
            offset_speed--;
            std::cout << "Set offset speed to " << offset_speed * OFFSET_SPEED_FACTOR << " m/s." << std::endl;
        }
    }

    if (joystick_commands.buttons[10] == 1 && last_button_values[10] == 0)    //BTN_SELECT (left push button)
    {
        if (!discrete_speed_mode)
        {
            discrete_speed_mode = true;
	    discrete_speed = 0;
            std::cout << "Switched to discrete walking speed mode." << std::endl;
            std::cout << "Set walking speed to " << discrete_speed * DISCRETE_SPEED_FACTOR << " m/s." << std::endl;
        }
        else
        {
            discrete_speed_mode = false;
            std::cout << "Switched to continuous walking speed mode." << std::endl;
        }
    }

    if (discrete_speed_mode)
        wheelwalking_control->setSpeed(discrete_speed * DISCRETE_SPEED_FACTOR);
    else
        wheelwalking_control->setSpeed(joystick_commands.axes[1] * MAX_SPEED);   //ABS_Y (left analog) - sign might have to be switched!!

    wheelwalking_control->setOffsetSpeed(offset_speed * OFFSET_SPEED_FACTOR);
    wheelwalking_control->setStepLength(step_length * STEP_LENGTH_FACTOR);

    last_button_values = joystick_commands.buttons.elements;
    last_axes_values = joystick_commands.axes.elements;
}

void Task::evaluateJointReadings(const base::samples::Joints joint_readings)
{
    // Joint order in position_readings & velocity_readings: [left_passive, right_passive, rear_passive, fl_walking, fr_walking, ml_walking, mr_walking, rl_walking, rr_walking, fl_steer, fr_steer, rl_steer, rr_steer, fl_drive, fr_drive, ml_drive, mr_drive, rl_drive, rr_drive]

    std::vector<double> position_readings;
    std::vector<double> velocity_readings;

    position_readings.resize(NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_ACTIVE_JOINTS);
    velocity_readings.resize(NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_ACTIVE_JOINTS);

    base::JointState current_joint;

    for (int i = 0; i < NUMBER_OF_PASSIVE_JOINTS + NUMBER_OF_ACTIVE_JOINTS; i++)
    {
        current_joint = joint_readings[joint_readings_names[i]];
        //std::cout << "Joint: " << joint_readings_names[i] << " -> Position:  " << current_joint.position << std::endl;

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
