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

#include <string>
#include <memory>
#include <ros/ros.h>
#include <mg400_bringup/commander.h>
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <mg400_bringup/EnableRobot.h>
#include <mg400_bringup/DisableRobot.h>
#include <mg400_bringup/ClearError.h>
#include <mg400_bringup/ResetRobot.h>
#include <mg400_bringup/SpeedFactor.h>
#include <mg400_bringup/User.h>
#include <mg400_bringup/Tool.h>
#include <mg400_bringup/RobotMode.h>
#include <mg400_bringup/PayLoad.h>
#include <mg400_bringup/DO.h>
#include <mg400_bringup/DOExecute.h>
#include <mg400_bringup/ToolDO.h>
#include <mg400_bringup/ToolDOExecute.h>
#include <mg400_bringup/AO.h>
#include <mg400_bringup/AOExecute.h>
#include <mg400_bringup/AccJ.h>
#include <mg400_bringup/AccL.h>
#include <mg400_bringup/SpeedJ.h>
#include <mg400_bringup/SpeedL.h>
#include <mg400_bringup/Arch.h>
#include <mg400_bringup/CP.h>
#include <mg400_bringup/LimZ.h>
#include <mg400_bringup/SetArmOrientation.h>
#include <mg400_bringup/PowerOn.h>
#include <mg400_bringup/RunScript.h>
#include <mg400_bringup/StopScript.h>
#include <mg400_bringup/PauseScript.h>
#include <mg400_bringup/ContinueScript.h>
//#include <mg400_bringup/GetHoldRegs.h>
//#include <mg400_bringup/SetHoldRegs.h>
#include <mg400_bringup/SetSafeSkin.h>
#include <mg400_bringup/SetObstacleAvoid.h>

#include <mg400_bringup/SetCollisionLevel.h>
#include <mg400_bringup/EmergencyStop.h>

#include <mg400_bringup/MovJ.h>
#include <mg400_bringup/MovL.h>
#include <mg400_bringup/Jump.h>
#include <mg400_bringup/Arc.h>
#include <mg400_bringup/Sync.h>
#include <mg400_bringup/Circle.h>
#include <mg400_bringup/ServoJ.h>
#include <mg400_bringup/StartTrace.h>
#include <mg400_bringup/StartPath.h>
#include <mg400_bringup/StartFCTrace.h>
#include <mg400_bringup/MoveJog.h>
#include <mg400_bringup/ServoP.h>
#include <mg400_bringup/RelMovJ.h>
#include <mg400_bringup/RelMovL.h>
#include <mg400_bringup/JointMovJ.h>
#include <mg400_bringup/RobotStatus.h>

#include <sensor_msgs/JointState.h>

using namespace actionlib;
using namespace control_msgs;

/**
 * MG400Robot
 */
class MG400Robot : protected ActionServer<FollowJointTrajectoryAction>
{
private:
    double goal_[6];
    uint32_t index_;
    ros::Timer timer_;
    ros::Timer movj_timer_;
    double trajectory_duration_;
    ros::NodeHandle control_nh_;
    std::shared_ptr<CR5Commander> commander_;
    std::vector<ros::ServiceServer> server_tbl_;

public:
    /**
     * Ctor
     * @param nh node handle
     * @param name topic
     */
    MG400Robot(ros::NodeHandle& nh, std::string name);

    /**
     * MG400Robot
     */
    ~MG400Robot() override;

    /**
     * init
     */
    void init();

    /**
     * getJointState
     * @param point
     */
    void getJointState(double* point);

    /**
     * getToolVectorActual
     * @param val value
     */
    void getToolVectorActual(double* val);

    /**
     * isEnable
     * @return ture enable, otherwise false
     */
    bool isEnable() const;

    /**
     * isConnected
     * @return ture connected, otherwise false
     */
    bool isConnected() const;

protected:
    bool enableRobot(mg400_bringup::EnableRobot::Request& request, mg400_bringup::EnableRobot::Response& response);
    bool disableRobot(mg400_bringup::DisableRobot::Request& request, mg400_bringup::DisableRobot::Response& response);
    bool clearError(mg400_bringup::ClearError::Request& request, mg400_bringup::ClearError::Response& response);
    bool resetRobot(mg400_bringup::ResetRobot::Request& request, mg400_bringup::ResetRobot::Response& response);
    bool speedFactor(mg400_bringup::SpeedFactor::Request& request, mg400_bringup::SpeedFactor::Response& response);
    bool user(mg400_bringup::User::Request& request, mg400_bringup::User::Response& response);
    bool tool(mg400_bringup::Tool::Request& request, mg400_bringup::Tool::Response& response);
    bool robotMode(mg400_bringup::RobotMode::Request& request, mg400_bringup::RobotMode::Response& response);
    bool payload(mg400_bringup::PayLoad::Request& request, mg400_bringup::PayLoad::Response& response);
    bool DO(mg400_bringup::DO::Request& request, mg400_bringup::DO::Response& response);
    bool DOExecute(mg400_bringup::DOExecute::Request& request, mg400_bringup::DOExecute::Response& response);
    bool toolDO(mg400_bringup::ToolDO::Request& request, mg400_bringup::ToolDO::Response& response);
    bool toolDOExecute(mg400_bringup::ToolDOExecute::Request& request, mg400_bringup::ToolDOExecute::Response& response);
    bool AO(mg400_bringup::AO::Request& request, mg400_bringup::AO::Response& response);
    bool AOExecute(mg400_bringup::AOExecute::Request& request, mg400_bringup::AOExecute::Response& response);
    bool accJ(mg400_bringup::AccJ::Request& request, mg400_bringup::AccJ::Response& response);
    bool accL(mg400_bringup::AccL::Request& request, mg400_bringup::AccL::Response& response);
    bool speedJ(mg400_bringup::SpeedJ::Request& request, mg400_bringup::SpeedJ::Response& response);
    bool speedL(mg400_bringup::SpeedL::Request& request, mg400_bringup::SpeedL::Response& response);
    bool arch(mg400_bringup::Arch::Request& request, mg400_bringup::Arch::Response& response);
    bool cp(mg400_bringup::CP::Request& request, mg400_bringup::CP::Response& response);
    bool limZ(mg400_bringup::LimZ::Request& request, mg400_bringup::LimZ::Response& response);
    bool setArmOrientation(mg400_bringup::SetArmOrientation::Request& request,
                           mg400_bringup::SetArmOrientation::Response& response);
    bool powerOn(mg400_bringup::PowerOn::Request& request, mg400_bringup::PowerOn::Response& response);
    bool runScript(mg400_bringup::RunScript::Request& request, mg400_bringup::RunScript::Response& response);
    bool stopScript(mg400_bringup::StopScript::Request& request, mg400_bringup::StopScript::Response& response);
    bool pauseScript(mg400_bringup::PauseScript::Request& request, mg400_bringup::PauseScript::Response& response);
    bool continueScript(mg400_bringup::ContinueScript::Request& request, mg400_bringup::ContinueScript::Response& response);
    //    bool getHoldRegs(mg400_bringup::SpeedFactor::Request& request, mg400_bringup::SpeedFactor::Response& response);
    //    bool setHoldRegs(mg400_bringup::SpeedFactor::Request& request, mg400_bringup::SpeedFactor::Response& response);
    bool setSafeSkin(mg400_bringup::SetSafeSkin::Request& request, mg400_bringup::SetSafeSkin::Response& response);
    bool setObstacleAvoid(mg400_bringup::SetObstacleAvoid::Request& request, mg400_bringup::SetObstacleAvoid::Response& response);
    //    bool getTraceStartPose(mg400_bringup::SpeedFactor::Request& request, mg400_bringup::SpeedFactor::Response& response);
    //    bool getPathStartPose(mg400_bringup::SpeedFactor::Request& request, mg400_bringup::SpeedFactor::Response& response);
    //    bool positiveSolution(mg400_bringup::SpeedFactor::Request& request, mg400_bringup::SpeedFactor::Response& response);
    //    bool inverseSolution(mg400_bringup::SpeedFactor::Request& request, mg400_bringup::SpeedFactor::Response& response);
    bool setCollisionLevel(mg400_bringup::SetCollisionLevel::Request& request,
                           mg400_bringup::SetCollisionLevel::Response& response);
    //    bool handleTrajPoints(mg400_bringup::SpeedFactor::Request& request, mg400_bringup::SpeedFactor::Response& response);
    //    bool getSixForceData(mg400_bringup::SpeedFactor::Request& request, mg400_bringup::SpeedFactor::Response& response);
    //    bool getAngle(mg400_bringup::SpeedFactor::Request& request, mg400_bringup::SpeedFactor::Response& response);
    //    bool getPose(mg400_bringup::SpeedFactor::Request& request, mg400_bringup::SpeedFactor::Response& response);
    bool emergencyStop(mg400_bringup::EmergencyStop::Request& request, mg400_bringup::EmergencyStop::Response& response);

    bool movJ(mg400_bringup::MovJ::Request& request, mg400_bringup::MovJ::Response& response);
    bool movL(mg400_bringup::MovL::Request& request, mg400_bringup::MovL::Response& response);
    bool jointMovJ(mg400_bringup::JointMovJ::Request& request, mg400_bringup::JointMovJ::Response& response);
    bool jump(mg400_bringup::Jump::Request& request, mg400_bringup::Jump::Response& response);
    bool relMovJ(mg400_bringup::RelMovJ::Request& request, mg400_bringup::RelMovJ::Response& response);
    bool relMovL(mg400_bringup::RelMovL::Request& request, mg400_bringup::RelMovL::Response& response);
    // bool MovLIO(mg400_bringup::RelMovL::Request& request, mg400_bringup::RelMovL::Response& response);
    // bool MovJIO(mg400_bringup::RelMovL::Request& request, mg400_bringup::RelMovL::Response& response);
    bool arc(mg400_bringup::Arc::Request& request, mg400_bringup::Arc::Response& response);
    bool circle(mg400_bringup::Circle::Request& request, mg400_bringup::Circle::Response& response);
    bool servoJ(mg400_bringup::ServoJ::Request& request, mg400_bringup::ServoJ::Response& response);
    bool servoP(mg400_bringup::ServoP::Request& request, mg400_bringup::ServoP::Response& response);
    bool sync(mg400_bringup::Sync::Request& request, mg400_bringup::Sync::Response& response);
    bool startTrace(mg400_bringup::StartTrace::Request& request, mg400_bringup::StartTrace::Response& response);
    bool startPath(mg400_bringup::StartPath::Request& request, mg400_bringup::StartPath::Response& response);
    bool startFCTrace(mg400_bringup::StartFCTrace::Request& request, mg400_bringup::StartFCTrace::Response& response);
    bool moveJog(mg400_bringup::MoveJog::Request& request, mg400_bringup::MoveJog::Response& response);

private:
    void feedbackHandle(const ros::TimerEvent& tm,
                        actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void moveHandle(const ros::TimerEvent& tm, actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void goalHandle(actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void cancelHandle(actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
};
