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
#include <mg400_bringup/robot.h>

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

    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/EnableRobot", &MG400Robot::enableRobot, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/mg400_bringup/srv/DisableRobot", &MG400Robot::disableRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/ClearError", &MG400Robot::clearError, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/ResetRobot", &MG400Robot::resetRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/SpeedFactor", &MG400Robot::speedFactor, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/User", &MG400Robot::user, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/Tool", &MG400Robot::tool, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/RobotMode", &MG400Robot::robotMode, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/PayLoad", &MG400Robot::payload, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/DO", &MG400Robot::DO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/DOExecute", &MG400Robot::DOExecute, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/ToolDO", &MG400Robot::toolDO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/ToolDOExecute", &MG400Robot::toolDOExecute, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/AO", &MG400Robot::AO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/AOExecute", &MG400Robot::AOExecute, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/AccJ", &MG400Robot::accJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/AccL", &MG400Robot::accL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/SpeedJ", &MG400Robot::speedJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/SpeedL", &MG400Robot::speedL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/Arch", &MG400Robot::arch, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/CP", &MG400Robot::cp, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/LimZ", &MG400Robot::limZ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/SetArmOrientation", &MG400Robot::setArmOrientation, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/PowerOn", &MG400Robot::powerOn, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/RunScript", &MG400Robot::runScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/StopScript", &MG400Robot::stopScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/PauseScript", &MG400Robot::pauseScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/ContinueScript", &MG400Robot::continueScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/SetSafeSkin", &MG400Robot::setSafeSkin, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/SetObstacleAvoid", &MG400Robot::setObstacleAvoid, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/SetCollisionLevel", &MG400Robot::setCollisionLevel, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/EmergencyStop", &MG400Robot::emergencyStop, this));

    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/MovJ", &MG400Robot::movJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/MovL", &MG400Robot::movL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/JointMovJ", &MG400Robot::jointMovJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/Jump", &MG400Robot::jump, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/RelMovJ", &MG400Robot::relMovJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/RelMovL", &MG400Robot::relMovL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/Arc", &MG400Robot::arc, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/Circle", &MG400Robot::circle, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/ServoJ", &MG400Robot::servoJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/ServoP", &MG400Robot::servoP, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/Sync", &MG400Robot::sync, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/StartTrace", &MG400Robot::startTrace, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/StartPath", &MG400Robot::startPath, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/mg400_bringup/srv/StartFCTrace", &MG400Robot::startFCTrace, this));
    server_tbl_.push_back(control_nh_.advertiseService("/mg400_bringup/srv/MoveJog", &MG400Robot::moveJog, this));

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

int MG400::robotStatus() const
{
    return commander_->robotStatus();
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

bool MG400Robot::enableRobot(mg400_bringup::EnableRobot::Request& request, mg400_bringup::EnableRobot::Response& response)
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

bool MG400Robot::disableRobot(mg400_bringup::DisableRobot::Request& request,
                            mg400_bringup::DisableRobot::Response& response)
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

bool MG400Robot::clearError(mg400_bringup::ClearError::Request& request, mg400_bringup::ClearError::Response& response)
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

bool MG400Robot::resetRobot(mg400_bringup::ResetRobot::Request& request, mg400_bringup::ResetRobot::Response& response)
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

bool MG400Robot::speedFactor(mg400_bringup::SpeedFactor::Request& request, mg400_bringup::SpeedFactor::Response& response)
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

bool MG400Robot::user(mg400_bringup::User::Request& request, mg400_bringup::User::Response& response)
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

bool MG400Robot::tool(mg400_bringup::Tool::Request& request, mg400_bringup::Tool::Response& response)
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

bool MG400Robot::robotMode(mg400_bringup::RobotMode::Request& request, mg400_bringup::RobotMode::Response& response)
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

bool MG400Robot::payload(mg400_bringup::PayLoad::Request& request, mg400_bringup::PayLoad::Response& response)
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

bool MG400Robot::DO(mg400_bringup::DO::Request& request, mg400_bringup::DO::Response& response)
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

bool MG400Robot::DOExecute(mg400_bringup::DOExecute::Request& request, mg400_bringup::DOExecute::Response& response)
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

bool MG400Robot::toolDO(mg400_bringup::ToolDO::Request& request, mg400_bringup::ToolDO::Response& response)
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

bool MG400Robot::toolDOExecute(mg400_bringup::ToolDOExecute::Request& request, mg400_bringup::ToolDOExecute::Response& response)
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

bool MG400Robot::AO(mg400_bringup::AO::Request& request, mg400_bringup::AO::Response& response)
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

bool MG400Robot::AOExecute(mg400_bringup::AOExecute::Request& request, mg400_bringup::AOExecute::Response& response)
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

bool MG400Robot::accJ(mg400_bringup::AccJ::Request& request, mg400_bringup::AccJ::Response& response)
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

bool MG400Robot::accL(mg400_bringup::AccL::Request& request, mg400_bringup::AccL::Response& response)
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

bool MG400Robot::speedJ(mg400_bringup::SpeedJ::Request& request, mg400_bringup::SpeedJ::Response& response)
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

bool MG400Robot::speedL(mg400_bringup::SpeedL::Request& request, mg400_bringup::SpeedL::Response& response)
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

bool MG400Robot::arch(mg400_bringup::Arch::Request& request, mg400_bringup::Arch::Response& response)
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

bool MG400Robot::cp(mg400_bringup::CP::Request& request, mg400_bringup::CP::Response& response)
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

bool MG400Robot::limZ(mg400_bringup::LimZ::Request& request, mg400_bringup::LimZ::Response& response)
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

bool MG400Robot::setArmOrientation(mg400_bringup::SetArmOrientation::Request& request, mg400_bringup::SetArmOrientation::Response& response)
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

bool MG400Robot::powerOn(mg400_bringup::PowerOn::Request& request, mg400_bringup::PowerOn::Response& response)
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

bool MG400Robot::runScript(mg400_bringup::RunScript::Request& request, mg400_bringup::RunScript::Response& response)
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

bool MG400Robot::stopScript(mg400_bringup::StopScript::Request& request, mg400_bringup::StopScript::Response& response)
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

bool MG400Robot::pauseScript(mg400_bringup::PauseScript::Request& request, mg400_bringup::PauseScript::Response& response)
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

bool MG400Robot::continueScript(mg400_bringup::ContinueScript::Request& request, mg400_bringup::ContinueScript::Response& response)
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

bool MG400Robot::setSafeSkin(mg400_bringup::SetSafeSkin::Request& request, mg400_bringup::SetSafeSkin::Response& response)
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

bool MG400Robot::setObstacleAvoid(mg400_bringup::SetObstacleAvoid::Request& request, mg400_bringup::SetObstacleAvoid::Response& response)
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

bool MG400Robot::setCollisionLevel(mg400_bringup::SetCollisionLevel::Request& request, mg400_bringup::SetCollisionLevel::Response& response)
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

bool MG400Robot::emergencyStop(mg400_bringup::EmergencyStop::Request& request, mg400_bringup::EmergencyStop::Response& response)
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

bool MG400Robot::movJ(mg400_bringup::MovJ::Request& request, mg400_bringup::MovJ::Response& response)
{
    try
    {
        commander_->movJ(request.x, request.y, request.z, request.r);
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

bool MG400Robot::movL(mg400_bringup::MovL::Request& request, mg400_bringup::MovL::Response& response)
{
    try
    {
        commander_->movL(request.x, request.y, request.z, request.r);
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

bool MG400Robot::servoJ(mg400_bringup::ServoJ::Request& request, mg400_bringup::ServoJ::Response& response)
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

bool MG400Robot::jump(mg400_bringup::Jump::Request& request, mg400_bringup::Jump::Response& response)
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

bool MG400Robot::arc(mg400_bringup::Arc::Request& request, mg400_bringup::Arc::Response& response)
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

bool MG400Robot::circle(mg400_bringup::Circle::Request& request, mg400_bringup::Circle::Response& response)
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

bool MG400Robot::servoP(mg400_bringup::ServoP::Request& request, mg400_bringup::ServoP::Response& response)
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

bool MG400Robot::relMovJ(mg400_bringup::RelMovJ::Request& request, mg400_bringup::RelMovJ::Response& response)
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

bool MG400Robot::relMovL(mg400_bringup::RelMovL::Request& request, mg400_bringup::RelMovL::Response& response)
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

bool MG400Robot::jointMovJ(mg400_bringup::JointMovJ::Request& request, mg400_bringup::JointMovJ::Response& response)
{
    try
    {
        commander_->jointMovJ(request.j1, request.j2, request.j3, request.j4);
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

bool MG400Robot::sync(mg400_bringup::Sync::Request& request, mg400_bringup::Sync::Response& response)
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

bool MG400Robot::startTrace(mg400_bringup::StartTrace::Request& request, mg400_bringup::StartTrace::Response& response)
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

bool MG400Robot::startPath(mg400_bringup::StartPath::Request& request, mg400_bringup::StartPath::Response& response)
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

bool MG400Robot::startFCTrace(mg400_bringup::StartFCTrace::Request& request,
                            mg400_bringup::StartFCTrace::Response& response)
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

bool MG400Robot::moveJog(mg400_bringup::MoveJog::Request& request, mg400_bringup::MoveJog::Response& response)
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

