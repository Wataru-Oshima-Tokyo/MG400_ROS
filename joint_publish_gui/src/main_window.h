/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 *
 * <h2><center>&copy; COPYRIGHT 2021 Dobot CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#pragma once

#include <QWidget>
#include <ros/ros.h>

QT_BEGIN_NAMESPACE
namespace Ui
{
class MainWindow;
}
QT_END_NAMESPACE

/**
 *
 */
class MainWindow : public QWidget
{
    Q_OBJECT

private:
    ros::Timer tm_;
    Ui::MainWindow* ui_;
    ros::NodeHandle& nh_;
    ros::Publisher joint_state_pub_;

    double j1_;
    double j2_1_;
    double j2_2_;
    double j3_;
    double j3_1_;
    double j3_2_;
    double j4_1_;
    double j4_2_;
    double j5_;

public:
    explicit MainWindow(ros::NodeHandle& nh, QWidget* parent = nullptr);
    ~MainWindow() override;

private:
    void j1ValueChange(int value);
    void j2ValueChange(int value);
    void j3ValueChange(int value);
    void j4ValueChange(int value);
    void publishJointStates(const ros::TimerEvent& evt);
};
