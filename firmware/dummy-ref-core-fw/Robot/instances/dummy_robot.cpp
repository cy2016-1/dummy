#include "communication.hpp"
#include "dummy_robot.h"

/**
 * Calculates the absolute maximum value of an array of 6 floating-point numbers and returns it.
 *
 * @param _joints the array of 6 floating-point numbers
 * @param _index a reference to the variable that will store the index of the maximum value
 *
 * @return the absolute maximum value of the array
 *
 * @throws None
 */
inline float AbsMaxOf6(DOF6Kinematic::Joint6D_t _joints, uint8_t &_index)
{
    float max = -1;
    for (uint8_t i = 0; i < 6; i++)
    {
        if (abs(_joints.a[i]) > max)
        {
            max = abs(_joints.a[i]);
            _index = i;
        }
    }

    return max;
}


/**
 * Constructor for the DummyRobot class.
 *
 * @param _hcan Pointer to the CAN_HandleTypeDef structure.
 *
 * @return None.
 *
 * @throws None.
 */
DummyRobot::DummyRobot(CAN_HandleTypeDef* _hcan) :
    hcan(_hcan)
{
    motorJ[ALL] = new CtrlStepMotor(_hcan, 0, false, 1, -180, 180);
    motorJ[1] = new CtrlStepMotor(_hcan, 1, true, 30, -170, 170);
    motorJ[2] = new CtrlStepMotor(_hcan, 2, false, 30, -73, 90);
    motorJ[3] = new CtrlStepMotor(_hcan, 3, true, 30, 35, 180);
    motorJ[4] = new CtrlStepMotor(_hcan, 4, false, 24, -180, 180);
    motorJ[5] = new CtrlStepMotor(_hcan, 5, true, 30, -120, 120);
    motorJ[6] = new CtrlStepMotor(_hcan, 6, true, 50, -720, 720);
    hand = new DummyHand(_hcan, 7);

    dof6Solver = new DOF6Kinematic(0.109f, 0.035f, 0.146f, 0.115f, 0.052f, 0.072f);
}


/**
 * Destructor for the DummyRobot class.
 *
 * Deletes the motorJ array and hand object, as well as the dof6Solver object.
 *
 * @throws None
 */
DummyRobot::~DummyRobot()
{
    for (int j = 0; j <= 6; j++)
        delete motorJ[j];

    delete hand;
    delete dof6Solver;
}


/**
 * Initializes the DummyRobot.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void DummyRobot::Init()
{
    SetCommandMode(DEFAULT_COMMAND_MODE);
    SetJointSpeed(DEFAULT_JOINT_SPEED);
}


/**
 * Reboots the DummyRobot.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void DummyRobot::Reboot()
{
    motorJ[ALL]->Reboot();
    osDelay(500); // waiting for all joints done
    HAL_NVIC_SystemReset();
}


/**
 * Moves the joints of the DummyRobot.
 *
 * @param _joints the target joint positions
 *
 * @throws ErrorType if an error occurs while moving the joints
 */
void DummyRobot::MoveJoints(DOF6Kinematic::Joint6D_t _joints)
{
    for (int j = 1; j <= 6; j++)
    {
        motorJ[j]->SetAngleWithVelocityLimit(_joints.a[j - 1] - initPose.a[j - 1],
                                             dynamicJointSpeeds.a[j - 1]);
    }
}


/**
 * Moves the robot to the specified joint positions.
 *
 * @param _j1 The position of joint 1.
 * @param _j2 The position of joint 2.
 * @param _j3 The position of joint 3.
 * @param _j4 The position of joint 4.
 * @param _j5 The position of joint 5.
 * @param _j6 The position of joint 6.
 *
 * @return True if the robot successfully moves to the target joint positions, false otherwise.
 *
 * @throws None.
 */
bool DummyRobot::MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6)
{
    DOF6Kinematic::Joint6D_t targetJointsTmp(_j1, _j2, _j3, _j4, _j5, _j6);
    bool valid = true;

    for (int j = 1; j <= 6; j++)
    {
        if (targetJointsTmp.a[j - 1] > motorJ[j]->angleLimitMax ||
            targetJointsTmp.a[j - 1] < motorJ[j]->angleLimitMin)
            valid = false;
    }

    if (valid)
    {
        DOF6Kinematic::Joint6D_t deltaJoints = targetJointsTmp - currentJoints;
        uint8_t index;
        float maxAngle = AbsMaxOf6(deltaJoints, index);
        float time = maxAngle * (float) (motorJ[index + 1]->reduction) / jointSpeed;
        for (int j = 1; j <= 6; j++)
        {
            dynamicJointSpeeds.a[j - 1] =
                abs(deltaJoints.a[j - 1] * (float) (motorJ[j]->reduction) / time * 0.1f); //0~10r/s
        }

        jointsStateFlag = 0;
        targetJoints = targetJointsTmp;

        return true;
    }

    return false;
}


+/**
+ * 在6D空间中将机器人移动到指定的位置和姿态。
+ *
+ * @param _x 目标位置的x坐标。
+ * @param _y 目标位置的y坐标。
+ * @param _z 目标位置的z坐标。
+ * @param _a 目标姿态绕x轴的旋转。
+ * @param _b 目标姿态绕y轴的旋转。
+ * @param _c 目标姿态绕z轴的旋转。
+ *
+ * @return 如果机器人成功移动到目标位置，则为true；否则为false。
+ *
+ * @throws None
+ */
bool DummyRobot::MoveL(float _x, float _y, float _z, float _a, float _b, float _c)
{
    DOF6Kinematic::Pose6D_t pose6D(_x, _y, _z, _a, _b, _c);
    DOF6Kinematic::IKSolves_t ikSolves{};
    DOF6Kinematic::Joint6D_t lastJoint6D{};

    dof6Solver->SolveIK(pose6D, lastJoint6D, ikSolves);

    bool valid[8];
    int validCnt = 0;

    for (int i = 0; i < 8; i++)
    {
        valid[i] = true;

        for (int j = 1; j <= 6; j++)
        {
            if (ikSolves.config[i].a[j - 1] > motorJ[j]->angleLimitMax ||
                ikSolves.config[i].a[j - 1] < motorJ[j]->angleLimitMin)
            {
                valid[i] = false;
                continue;
            }
        }

        if (valid[i]) validCnt++;
    }

    if (validCnt)
    {
        float min = 1000;
        uint8_t indexConfig = 0, indexJoint = 0;
        for (int i = 0; i < 8; i++)
        {
            if (valid[i])
            {
                for (int j = 0; j < 6; j++)
                    lastJoint6D.a[j] = ikSolves.config[i].a[j];
                DOF6Kinematic::Joint6D_t tmp = currentJoints - lastJoint6D;
                float maxAngle = AbsMaxOf6(tmp, indexJoint);
                if (maxAngle < min)
                {
                    min = maxAngle;
                    indexConfig = i;
                }
            }
        }

        return MoveJ(ikSolves.config[indexConfig].a[0], ikSolves.config[indexConfig].a[1],
                     ikSolves.config[indexConfig].a[2], ikSolves.config[indexConfig].a[3],
                     ikSolves.config[indexConfig].a[4], ikSolves.config[indexConfig].a[5]);
    }

    return false;
}

/**
 * Updates the joint angles for the DummyRobot.
 *
 * @throws ErrorType description of error
 */
void DummyRobot::UpdateJointAngles()
{
    motorJ[ALL]->UpdateAngle();
}


/**
 * Update the joint angles of the DummyRobot.
 *
 * @throws None
 */
void DummyRobot::UpdateJointAnglesCallback()
{
    for (int i = 1; i <= 6; i++)
    {
        currentJoints.a[i - 1] = motorJ[i]->angle + initPose.a[i - 1];

        if (motorJ[i]->state == CtrlStepMotor::FINISH)
            jointsStateFlag |= (1 << i);
        else
            jointsStateFlag &= ~(1 << i);
    }
}


/**
 * Sets the joint speed of the DummyRobot.
 *
 * @param _speed the speed value to set for the joint speed
 *
 * @throws None
 */
void DummyRobot::SetJointSpeed(float _speed)
{
    if (_speed < 0)_speed = 0;
    else if (_speed > 100) _speed = 100;

    jointSpeed = _speed * jointSpeedRatio;
}


/**
 * Sets the joint acceleration for the DummyRobot.
 *
 * @param _acc the desired joint acceleration value
 *
 * @throws ErrorType if an error occurs while setting the joint acceleration
 */
void DummyRobot::SetJointAcceleration(float _acc)
{
    if (_acc < 0)_acc = 0;
    else if (_acc > 100) _acc = 100;

    for (int i = 1; i <= 6; i++)
        motorJ[i]->SetAcceleration(_acc / 100 * DEFAULT_JOINT_ACCELERATION_BASES.a[i - 1]);
}


/**
 * Calibrates the home offset of the DummyRobot.
 *
 * Disables FixUpdate but does not disable motors. Moves the joints to an L-Pose
 * [precisely]. Applies the home offset the first time. Goes to the resting
 * pose. Applies the home offset the second time. Sets the current limits for
 * motors 2 and 3. Reboots the robot.
 *
 * @throws ErrorType description of error
 */
void DummyRobot::CalibrateHomeOffset()
{
    // Disable FixUpdate, but not disable motors
    isEnabled = false;
    motorJ[ALL]->SetEnable(true);

    // 1.Manually move joints to L-Pose [precisely]
    // ...
    motorJ[2]->SetCurrentLimit(0.5);
    motorJ[3]->SetCurrentLimit(0.5);
    osDelay(500);

    // 2.Apply Home-Offset the first time
    motorJ[ALL]->ApplyPositionAsHome();
    osDelay(500);

    // 3.Go to Resting-Pose
    initPose = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    currentJoints = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    Resting();
    osDelay(500);

    // 4.Apply Home-Offset the second time
    motorJ[ALL]->ApplyPositionAsHome();
    osDelay(500);
    motorJ[2]->SetCurrentLimit(1);
    motorJ[3]->SetCurrentLimit(1);
    osDelay(500);

    Reboot();
}


/**
 * Homing the dummy robot.
 *
 * @throws ErrorType description of error
 */
void DummyRobot::Homing()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(10);

    MoveJ(0, 0, 90, 0, 0, 0);
    MoveJoints(targetJoints);
    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSpeed);
}


/**
 * Resting function that sets the joint speed to 10, moves the robot to the
 * rest pose, moves the robot to the target joints, and waits until the robot
 * finishes moving. Finally, it sets the joint speed back to the last speed.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void DummyRobot::Resting()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(10);

    MoveJ(REST_POSE.a[0], REST_POSE.a[1], REST_POSE.a[2],
          REST_POSE.a[3], REST_POSE.a[4], REST_POSE.a[5]);
    MoveJoints(targetJoints);
    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSpeed);
}


/**
 * Sets the enable/disable state of the DummyRobot.
 *
 * @param _enable the new enable/disable state
 *
 * @throws ErrorType description of error
 */
void DummyRobot::SetEnable(bool _enable)
{
    motorJ[ALL]->SetEnable(_enable);
    isEnabled = _enable;
}


/**
 * Updates the 6D pose of the joint in the dummy robot.
 *
 * @throws ErrorType description of error
 */
void DummyRobot::UpdateJointPose6D()
{
    dof6Solver->SolveFK(currentJoints, currentPose6D);
    currentPose6D.X *= 1000; // m -> mm
    currentPose6D.Y *= 1000; // m -> mm
    currentPose6D.Z *= 1000; // m -> mm
}


/**
 * Checks if the DummyRobot is currently moving.
 *
 * @return True if the DummyRobot is moving, false otherwise.
 */
bool DummyRobot::IsMoving()
{
    return jointsStateFlag != 0b1111110;
}


/**
 * Returns whether the DummyRobot is enabled or not.
 *
 * @return true if the DummyRobot is enabled, false otherwise.
 */
bool DummyRobot::IsEnabled()
{
    return isEnabled;
}


/**
 * Set the command mode of the DummyRobot.
 *
 * @param _mode the command mode to set
 *
 * @throws ErrorType if the _mode is invalid
 */
void DummyRobot::SetCommandMode(uint32_t _mode)
{
    if (_mode < COMMAND_TARGET_POINT_SEQUENTIAL ||
        _mode > COMMAND_MOTOR_TUNING)
        return;

    commandMode = static_cast<CommandMode>(_mode);

    switch (commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            jointSpeedRatio = 1;
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_LOW);
            break;
        case COMMAND_CONTINUES_TRAJECTORY:
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_HIGH);
            jointSpeedRatio = 0.3;
            break;
        case COMMAND_MOTOR_TUNING:
            break;
    }
}


/**
 * Constructor for the DummyHand class.
 *
 * @param _hcan pointer to the CAN_HandleTypeDef object
 * @param _id   the ID of the node
 *
 * @return void
 *
 * @throws None
 */
DummyHand::DummyHand(CAN_HandleTypeDef* _hcan, uint8_t
_id) :
    nodeID(_id), hcan(_hcan)
{
    txHeader =
        {
            .StdId = 0,
            .ExtId = 0,
            .IDE = CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = 8,
            .TransmitGlobalTime = DISABLE
        };
}


/**
 * Sets the angle of the DummyHand.
 *
 * @param _angle the angle to set, must be between 0 and 30 (inclusive)
 *
 * @throws ErrorType if an error occurs during setting the angle
 */
void DummyHand::SetAngle(float _angle)
{
    if (_angle > 30)_angle = 30;
    if (_angle < 0)_angle = 0;

    uint8_t mode = 0x02;
    txHeader.StdId = 7 << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_angle;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


/**
 * Sets the maximum current for the DummyHand.
 *
 * @param _val the maximum current value to set
 *
 * @throws ErrorType if there is an error setting the maximum current
 */
void DummyHand::SetMaxCurrent(float _val)
{
    if (_val > 1)_val = 1;
    if (_val < 0)_val = 0;

    uint8_t mode = 0x01;
    txHeader.StdId = 7 << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


/**
 * Sets the enable/disable flag for the DummyHand.
 *
 * @param _enable a boolean value to enable or disable the DummyHand
 *
 * @throws None
 */
void DummyHand::SetEnable(bool _enable)
{
    if (_enable)
        SetMaxCurrent(maxCurrent);
    else
        SetMaxCurrent(0);
}


/**
 * Pushes a command to the command handler.
 *
 * @param _cmd The command to be pushed.
 *
 * @return The available space in the command FIFO after pushing the command.
 *
 * @throws None.
 */
uint32_t DummyRobot::CommandHandler::Push(const std::string &_cmd)
{
    osStatus_t status = osMessageQueuePut(commandFifo, _cmd.c_str(), 0U, 0U);
    if (status == osOK)
        return osMessageQueueGetSpace(commandFifo);

    return 0xFF; // failed
}


/**
 * Stops the robot immediately by disabling its movement, clearing the FIFO, and resetting the target joints.
 *
 * @throws ErrorType description of error
 */
void DummyRobot::CommandHandler::EmergencyStop()
{
    context->MoveJ(context->currentJoints.a[0], context->currentJoints.a[1], context->currentJoints.a[2],
                   context->currentJoints.a[3], context->currentJoints.a[4], context->currentJoints.a[5]);
    context->MoveJoints(context->targetJoints);
    context->isEnabled = false;
    ClearFifo();
}


/**
 * Pop a command from the command FIFO with an optional timeout.
 *
 * @param timeout the maximum time to wait for a command (in milliseconds)
 *
 * @return the command that was popped from the FIFO as a string
 *
 * @throws osErrorTimeout if no command is received within the timeout period
 */
std::string DummyRobot::CommandHandler::Pop(uint32_t timeout)
{
    osStatus_t status = osMessageQueueGet(commandFifo, strBuffer, nullptr, timeout);

    return std::string{strBuffer};
}


/**
 * Get the amount of free space in the command FIFO.
 *
 * @return The number of free slots available in the command FIFO.
 *
 * @throws None
 */
uint32_t DummyRobot::CommandHandler::GetSpace()
{
    return osMessageQueueGetSpace(commandFifo);
}


/**
 * Parses a command and performs corresponding actions based on the command mode.
 *
 * @param _cmd the command to be parsed
 *
 * @return the space in the commandFifo after parsing the command
 *
 * @throws None
 */
uint32_t DummyRobot::CommandHandler::ParseCommand(const std::string &_cmd)
{
    uint8_t argNum;

    switch (context->commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_CONTINUES_TRAJECTORY:
            if (_cmd[0] == '>' || _cmd[0] == '&')
            {
                float joints[6];
                float speed;

                if (_cmd[0] == '>')
                    argNum = sscanf(_cmd.c_str(), ">%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                    joints + 3, joints + 4, joints + 5, &speed);
                if (_cmd[0] == '&')
                    argNum = sscanf(_cmd.c_str(), "&%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                    joints + 3, joints + 4, joints + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                }
                // Trigger a transmission immediately, in case IsMoving() returns false
                context->MoveJoints(context->targetJoints);

                while (context->IsMoving() && context->IsEnabled())
                    osDelay(5);
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            } else if (_cmd[0] == '@')
            {
                float pose[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                                pose + 3, pose + 4, pose + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                }
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            }

            break;

        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            if (_cmd[0] == '>' || _cmd[0] == '&')
            {
                float joints[6];
                float speed;

                if (_cmd[0] == '>')
                    argNum = sscanf(_cmd.c_str(), ">%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                    joints + 3, joints + 4, joints + 5, &speed);
                if (_cmd[0] == '&')
                    argNum = sscanf(_cmd.c_str(), "&%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                    joints + 3, joints + 4, joints + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                }
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            } else if (_cmd[0] == '@')
            {
                float pose[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                                pose + 3, pose + 4, pose + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                }
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            }
            break;

        case COMMAND_MOTOR_TUNING:
            break;
    }

    return osMessageQueueGetSpace(commandFifo);
}


/**
 * Clears the command FIFO.
 *
 * @throws ErrorType description of error
 */
void DummyRobot::CommandHandler::ClearFifo()
{
    osMessageQueueReset(commandFifo);
}


/**
 * Sets the tuning flag for the DummyRobot's TuningHelper.
 *
 * @param _flag the tuning flag to be set
 *
 * @throws ErrorType description of error (if applicable)
 */
void DummyRobot::TuningHelper::SetTuningFlag(uint8_t _flag)
{
    tuningFlag = _flag;
}


/**
 * Updates the tick of the DummyRobot TuningHelper.
 *
 * @param _timeMillis The time in milliseconds.
 *
 * @throws None
 */
void DummyRobot::TuningHelper::Tick(uint32_t _timeMillis)
{
    time += PI * 2 * frequency * (float) _timeMillis / 1000.0f;
    float delta = amplitude * sinf(time);

    for (int i = 1; i <= 6; i++)
        if (tuningFlag & (1 << (i - 1)))
            context->motorJ[i]->SetAngle(delta);
}


/**
 * Set the frequency and amplitude of the DummyRobot.
 *
 * @param _freq the desired frequency value
 * @param _amp the desired amplitude value
 *
 * @throws ErrorType if an error occurs while setting the frequency and amplitude
 */
void DummyRobot::TuningHelper::SetFreqAndAmp(float _freq, float _amp)
{
    if (_freq > 5)_freq = 5;
    else if (_freq < 0.1) _freq = 0.1;
    if (_amp > 50)_amp = 50;
    else if (_amp < 1) _amp = 1;

    frequency = _freq;
    amplitude = _amp;
}
