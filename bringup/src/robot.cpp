/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/07
 *
 * <h2><center>&copy; COPYRIGHT 2021 DOBOT CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#include <ros/ros.h>
#include <ros/param.h>
#include <bringup/robot.h>

MG400Robot::MG400Robot(ros::NodeHandle& nh, std::string name)
    : ActionServer<FollowJointTrajectoryAction>(nh, std::move(name), false)
    , goal_{}
    , control_nh_(nh)
    , trajectory_duration_(1.0)
{
    index_ = 0;
    memset(goal_, 0, sizeof(goal_));
}

MG400Robot::~MG400Robot()
{
    ROS_INFO("~MG400Robotot");
}

void MG400Robot::init()
{
    std::string ip = control_nh_.param<std::string>("robot_ip_address", "192.168.5.1");

    trajectory_duration_ = control_nh_.param("trajectory_duration", 0.3);
    ROS_INFO("trajectory_duration : %0.2f", trajectory_duration_);

    commander_ = std::make_shared<CR5Commander>(ip);
    commander_->init();

    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/EnableRobot", &MG400Robot::enableRobot, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/bringup/srv/DisableRobot", &MG400Robot::disableRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ClearError", &MG400Robot::clearError, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ResetRobot", &MG400Robot::resetRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SpeedFactor", &MG400Robot::speedFactor, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/User", &MG400Robot::user, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Tool", &MG400Robot::tool, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/RobotMode", &MG400Robot::robotMode, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/PayLoad", &MG400Robot::payload, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/DO", &MG400Robot::DO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/DOExecute", &MG400Robot::DOExecute, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ToolDO", &MG400Robot::toolDO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ToolDOExecute", &MG400Robot::toolDOExecute, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/AO", &MG400Robot::AO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/AOExecute", &MG400Robot::AOExecute, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/AccJ", &MG400Robot::accJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/AccL", &MG400Robot::accL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SpeedJ", &MG400Robot::speedJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SpeedL", &MG400Robot::speedL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Arch", &MG400Robot::arch, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/CP", &MG400Robot::cp, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/LimZ", &MG400Robot::limZ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SetArmOrientation", &MG400Robot::setArmOrientation, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/PowerOn", &MG400Robot::powerOn, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/RunScript", &MG400Robot::runScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/StopScript", &MG400Robot::stopScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/PauseScript", &MG400Robot::pauseScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ContinueScript", &MG400Robot::continueScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SetSafeSkin", &MG400Robot::setSafeSkin, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SetObstacleAvoid", &MG400Robot::setObstacleAvoid, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SetCollisionLevel", &MG400Robot::setCollisionLevel, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/EmergencyStop", &MG400Robot::emergencyStop, this));

    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/MovJ", &MG400Robot::movJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/MovL", &MG400Robot::movL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/JointMovJ", &MG400Robot::jointMovJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Jump", &MG400Robot::jump, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/RelMovJ", &MG400Robot::relMovJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/RelMovL", &MG400Robot::relMovL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Arc", &MG400Robot::arc, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Circle", &MG400Robot::circle, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ServoJ", &MG400Robot::servoJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ServoP", &MG400Robot::servoP, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Sync", &MG400Robot::sync, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/StartTrace", &MG400Robot::startTrace, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/StartPath", &MG400Robot::startPath, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/bringup/srv/StartFCTrace", &MG400Robot::startFCTrace, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/MoveJog", &MG400Robot::moveJog, this));

    registerGoalCallback(boost::bind(&MG400Robot::goalHandle, this, _1));
    registerCancelCallback(boost::bind(&MG400Robot::cancelHandle, this, _1));
    start();
}

void MG400Robot::feedbackHandle(const ros::TimerEvent& tm,
                              ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    control_msgs::FollowJointTrajectoryFeedback feedback;

    double current_joints[6];
    getJointState(current_joints);

    for (uint32_t i = 0; i < 6; i++)
    {
        feedback.joint_names.push_back(std::string("joint") + std::to_string(i + 1));
        feedback.actual.positions.push_back(current_joints[i]);
        feedback.desired.positions.push_back(goal_[i]);
    }

    handle.publishFeedback(feedback);
}

void MG400Robot::moveHandle(const ros::TimerEvent& tm,
                          ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    control_msgs::FollowJointTrajectoryGoalConstPtr trajectory = handle.getGoal();

    if (index_ < trajectory->trajectory.points.size())
    {
        auto point = trajectory->trajectory.points[index_].positions;
        double tmp[6];
        for (uint32_t i = 0; i < 6; i++)
        {
            tmp[i] = point[i] * 180.0 / 3.1415926;
        }

        commander_->servoJ(tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5]);
        index_++;
    }
    else
    {
#define OFFSET_VAL 0.01
        double current_joints[6];
        getJointState(current_joints);
        if ((current_joints[0] >= goal_[0] - OFFSET_VAL) && (current_joints[0] <= goal_[0] + OFFSET_VAL) &&
            (current_joints[1] >= goal_[1] - OFFSET_VAL) && (current_joints[1] <= goal_[1] + OFFSET_VAL) &&
            (current_joints[2] >= goal_[2] - OFFSET_VAL) && (current_joints[2] <= goal_[2] + OFFSET_VAL) &&
            (current_joints[3] >= goal_[3] - OFFSET_VAL) && (current_joints[3] <= goal_[3] + OFFSET_VAL) &&
            (current_joints[4] >= goal_[4] - OFFSET_VAL) && (current_joints[4] <= goal_[4] + OFFSET_VAL) &&
            (current_joints[5] >= goal_[5] - OFFSET_VAL) && (current_joints[5] <= goal_[5] + OFFSET_VAL))
        {
            timer_.stop();
            movj_timer_.stop();
            handle.setSucceeded();
        }
    }
}

void MG400Robot::goalHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    index_ = 0;
    for (uint32_t i = 0; i < 6; i++)
    {
        goal_[i] = handle.getGoal()->trajectory.points[handle.getGoal()->trajectory.points.size() - 1].positions[i];
    }
    timer_ = control_nh_.createTimer(ros::Duration(1.0), boost::bind(&MG400Robot::feedbackHandle, this, _1, handle));
    movj_timer_ = control_nh_.createTimer(ros::Duration(trajectory_duration_),
                                          boost::bind(&MG400Robot::moveHandle, this, _1, handle));
    timer_.start();
    movj_timer_.start();
    handle.setAccepted();
}

void MG400Robot::cancelHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    timer_.stop();
    movj_timer_.stop();
    handle.setSucceeded();
}

void MG400Robot::getJointState(double* point)
{
    commander_->getCurrentJointStatus(point);
}

bool MG400Robot::isEnable() const
{
    return commander_->isEnable();
}

bool MG400Robot::isConnected() const
{
    return commander_->isConnected();
}

void MG400Robot::getToolVectorActual(double* val)
{
    commander_->getToolVectorActual(val);
}

/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  dashboard
 *----------------------------------------------------------------------------------------------------------------------
 */

bool MG400Robot::enableRobot(bringup::EnableRobot::Request& request, bringup::EnableRobot::Response& response)
{
    try
    {
        commander_->enableRobot();
        response.res = 0;
        return true;
    }
    catch (const std::exception& err)
    {
        commander_->clearError();
        response.res = -1;
        return false;
    }
}

bool MG400Robot::disableRobot(bringup::DisableRobot::Request& request,
                            bringup::DisableRobot::Response& response)
{
    try
    {
        commander_->disableRobot();
        response.res = 0;
        return true;
    }
    catch (const std::exception& err)
    {
        commander_->clearError();
        response.res = -1;
        return false;
    }
}

bool MG400Robot::clearError(bringup::ClearError::Request& request, bringup::ClearError::Response& response)
{
    try
    {
        commander_->clearError();
        response.res = 0;
        return true;
    }
    catch (const std::exception& err)
    {
        commander_->clearError();
        response.res = -1;
        return false;
    }
}

bool MG400Robot::resetRobot(bringup::ResetRobot::Request& request, bringup::ResetRobot::Response& response)
{
    try
    {
        commander_->resetRobot();
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::speedFactor(bringup::SpeedFactor::Request& request, bringup::SpeedFactor::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SpeedFactor(%d)", request.ratio);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::user(bringup::User::Request& request, bringup::User::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "User(%d)", request.index);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::tool(bringup::Tool::Request& request, bringup::Tool::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Tool(%d)", request.index);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::robotMode(bringup::RobotMode::Request& request, bringup::RobotMode::Response& response)
{
    try
    {
        const char *cmd = "RobotMode()";
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::payload(bringup::PayLoad::Request& request, bringup::PayLoad::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "PayLoad(%0.3f, %0.3f)", request.weight, request.inertia);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::DO(bringup::DO::Request& request, bringup::DO::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "DO(%d, %d)", request.index, request.status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::DOExecute(bringup::DOExecute::Request& request, bringup::DOExecute::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "DO(%d, %d)", request.index, request.status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::toolDO(bringup::ToolDO::Request& request, bringup::ToolDO::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "ToolDO(%d, %d)", request.index, request.status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::toolDOExecute(bringup::ToolDOExecute::Request& request, bringup::ToolDOExecute::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "ToolDOExecute(%d, %d)", request.index, request.status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::AO(bringup::AO::Request& request, bringup::AO::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AO(%d, %d)", request.index, request.status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::AOExecute(bringup::AOExecute::Request& request, bringup::AOExecute::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AO(%d, %0.3f)", request.index, static_cast<float>(request.value));
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::accJ(bringup::AccJ::Request& request, bringup::AccJ::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AccJ(%d)", request.r);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::accL(bringup::AccL::Request& request, bringup::AccL::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AccL(%d)", request.r);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::speedJ(bringup::SpeedJ::Request& request, bringup::SpeedJ::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SpeedJ(%d)", request.r);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::speedL(bringup::SpeedL::Request& request, bringup::SpeedL::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SpeedL(%d)", request.r);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::arch(bringup::Arch::Request& request, bringup::Arch::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Arch(%d)", request.index);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::cp(bringup::CP::Request& request, bringup::CP::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "CP(%d)", request.r);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::limZ(bringup::LimZ::Request& request, bringup::LimZ::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "LimZ(%d)", request.value);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::setArmOrientation(bringup::SetArmOrientation::Request& request, bringup::SetArmOrientation::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetArmOrientation(%d,%d,%d,%d)", request.LorR, request.UorD, request.ForN, request.Config6);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::powerOn(bringup::PowerOn::Request& request, bringup::PowerOn::Response& response)
{
    try
    {
        const char* cmd = "PowerOn()";
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::runScript(bringup::RunScript::Request& request, bringup::RunScript::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "RunScript(%s)", request.projectName.c_str());
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::stopScript(bringup::StopScript::Request& request, bringup::StopScript::Response& response)
{
    try
    {
        const char *cmd = "StopScript()";
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::pauseScript(bringup::PauseScript::Request& request, bringup::PauseScript::Response& response)
{
    try
    {
        const char *cmd = "PauseScript()";
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::continueScript(bringup::ContinueScript::Request& request, bringup::ContinueScript::Response& response)
{
    try
    {
        const char *cmd = "ContinueScript()";
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::setSafeSkin(bringup::SetSafeSkin::Request& request, bringup::SetSafeSkin::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetSafeSkin(%d)", request.status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::setObstacleAvoid(bringup::SetObstacleAvoid::Request& request, bringup::SetObstacleAvoid::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetObstacleAvoid(%d)", request.status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::setCollisionLevel(bringup::SetCollisionLevel::Request& request, bringup::SetCollisionLevel::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetCollisionLevel(%d)", request.level);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::emergencyStop(bringup::EmergencyStop::Request& request, bringup::EmergencyStop::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "EmergencyStop()");
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}


/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  real time
 *----------------------------------------------------------------------------------------------------------------------
 */

bool MG400Robot::movJ(bringup::MovJ::Request& request, bringup::MovJ::Response& response)
{
    try
    {
        commander_->movJ(request.x, request.y, request.z, request.a, request.b, request.c);
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::movL(bringup::MovL::Request& request, bringup::MovL::Response& response)
{
    try
    {
        commander_->movL(request.x, request.y, request.z, request.a, request.b, request.c);
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::servoJ(bringup::ServoJ::Request& request, bringup::ServoJ::Response& response)
{
    try
    {
        commander_->servoJ(request.j1, request.j2, request.j3, request.j4, request.j5, request.j6);
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::jump(bringup::Jump::Request& request, bringup::Jump::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Jump(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request.offset1, request.offset2, request.offset3,
                request.offset4, request.offset5, request.offset6);
        commander_->realSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::arc(bringup::Arc::Request& request, bringup::Arc::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Arc(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request.x1,
                request.y1, request.z1, request.rx1, request.ry1, request.rz1, request.x2, request.y2, request.z2,
                request.rx2, request.ry2, request.rz2);
        commander_->realSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::circle(bringup::Circle::Request& request, bringup::Circle::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Circle(%d, %0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)",
                request.count, request.x1, request.y1, request.z1, request.rx1, request.ry1, request.rz1, request.x2,
                request.y2, request.z2, request.rx2, request.ry2, request.rz2);
        commander_->realSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::servoP(bringup::ServoP::Request& request, bringup::ServoP::Response& response)
{
    try
    {
        commander_->servoP(request.x, request.y, request.z, request.a, request.b, request.c);
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::relMovJ(bringup::RelMovJ::Request& request, bringup::RelMovJ::Response& response)
{
    try
    {
        commander_->relMovJ(request.offset1, request.offset2, request.offset3, request.offset4, request.offset5,
                            request.offset6);
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::relMovL(bringup::RelMovL::Request& request, bringup::RelMovL::Response& response)
{
    try
    {
        commander_->relMovL(request.x, request.y, request.z);
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::jointMovJ(bringup::JointMovJ::Request& request, bringup::JointMovJ::Response& response)
{
    try
    {
        commander_->jointMovJ(request.j1, request.j2, request.j3, request.j4, request.j5, request.j6);
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::sync(bringup::Sync::Request& request, bringup::Sync::Response& response)
{
    try
    {
        const char* cmd = "Sync()";
        commander_->realSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::startTrace(bringup::StartTrace::Request& request, bringup::StartTrace::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "StartTrace(%s)", request.trace_name.c_str());
        commander_->realSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::startPath(bringup::StartPath::Request& request, bringup::StartPath::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "StartPath(%s,%d,%d)", request.trace_name.c_str(), request.const_val, request.cart);
        commander_->realSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::startFCTrace(bringup::StartFCTrace::Request& request,
                            bringup::StartFCTrace::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "StartFCTrace(%s)", request.trace_name.c_str());
        commander_->realSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::moveJog(bringup::MoveJog::Request& request, bringup::MoveJog::Response& response)
{
    try
    {
        commander_->moveJog(request.axisID);
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

