//
// Created by zhangran on 2021/10/20.
//

// You may need to build the project (run Qt uic code generator) to get "ui_main_windowa.h" resolved

#include "main_window.h"
#include "ui_main_window.h"

#include <sensor_msgs/JointState.h>

MainWindow::MainWindow(ros::NodeHandle& nh, QWidget* parent) : QWidget(parent), nh_(nh), ui_(new Ui::MainWindow)
{
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 100);
    tm_ = nh_.createTimer(ros::Rate(50), &MainWindow::publishJointStates, this);
    tm_.start();
    ui_->setupUi(this);

    ui_->j1_txt->setText("0");
    ui_->j2_txt->setText("0");
    ui_->j3_txt->setText("0");
    ui_->j4_txt->setText("0");
    j1_ = 0.0;
    j2_1_ = 0.0;
    j2_2_ = 0.0;
    j3_ = 0.0;
    j3_1_ = 0.0;
    j3_2_ = 0.0;
    j4_1_ = 0.0;
    j4_2_ = 0.0;
    j5_ = 0.0;
    connect(ui_->j1_slider, &QSlider::valueChanged, this, &MainWindow::j1ValueChange);
    connect(ui_->j2_slider, &QSlider::valueChanged, this, &MainWindow::j2ValueChange);
    connect(ui_->j3_slider, &QSlider::valueChanged, this, &MainWindow::j3ValueChange);
    connect(ui_->j4_slider, &QSlider::valueChanged, this, &MainWindow::j4ValueChange);
}

MainWindow::~MainWindow()
{
    delete ui_;
}

void MainWindow::publishJointStates(const ros::TimerEvent& evt)
{
    sensor_msgs::JointState msg;

    msg.header.stamp = ros::Time::now();
    msg.name.push_back("j1");
    msg.name.push_back("j2_1");
    msg.name.push_back("j2_2");
    msg.name.push_back("j3_1");
    msg.name.push_back("j3_2");
    msg.name.push_back("j4_1");
    msg.name.push_back("j4_2");
    msg.name.push_back("j5");
    msg.position.push_back(j1_);
    msg.position.push_back(j2_1_);
    msg.position.push_back(j2_2_);
    msg.position.push_back(j3_1_);
    msg.position.push_back(j3_2_);
    msg.position.push_back(j4_1_);
    msg.position.push_back(j4_2_);
    msg.position.push_back(j5_);
    joint_state_pub_.publish(msg);
}

void MainWindow::j1ValueChange(int value)
{
    char str[50];
    j1_ = value * 0.0314;
    sprintf(str, "%0.3f", j1_);
    ui_->j1_txt->setText(str);
}

void MainWindow::j2ValueChange(int value)
{


    {
        j2_1_ = value * 0.0314;
        j2_2_ = value * 0.0314;
        j3_2_ = -value * 0.0314;

//        j4_1_ = (j3_ - j2_1_);
    }


    j3_1_ = j3_ - j2_1_;

    char str[50];
    sprintf(str, "%0.3f", j2_1_);
    ui_->j2_txt->setText(str);
}

void MainWindow::j3ValueChange(int value)
{
    {
        j3_ = value * 0.0314;
        j3_1_ = j3_ - j2_1_;
        j4_2_ = j3_;
        j4_1_ = -j3_;
    }

    char str[50];
    sprintf(str, "%0.3f", j3_);
    ui_->j3_txt->setText(str);
}

void MainWindow::j4ValueChange(int value)
{
    char str[50];
    j5_ = value * 0.0314;
    sprintf(str, "%0.3f", j5_);
    ui_->j4_txt->setText(str);
}
