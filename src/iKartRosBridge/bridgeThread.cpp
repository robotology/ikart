#include "bridgeThread.h"

void BridgeThread::setHome(double x, double y, double angle)
{
    mutex_home.wait();
    ikart_home.x=x;
    ikart_home.y=y;
    ikart_home.t=angle;
    mutex_home.post();
}

void BridgeThread::setHome()
{
    mutex_home.wait();
    ikart_home.x=ikart_current_position.x;
    ikart_home.y=ikart_current_position.y;
    ikart_home.t=ikart_current_position.t;
    mutex_home.post();
}

void BridgeThread::getHome(double &x, double &y, double &angle)
{
    mutex_home.wait();
    x=ikart_home.x;
    y=ikart_home.y;
    angle=ikart_home.t;
    mutex_home.post();
}

int BridgeThread::setGoal(double x, double y, double angle)
{
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "home";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = y;
    goal.target_pose.pose.position.x = -x;
    goal.target_pose.pose.orientation.w = -angle;

    ROS_INFO("Sending goal");
    ac->cancelAllGoals();
    ac->sendGoal(goal);
    return 0;
}

int BridgeThread::getGoal(double &x, double &y, double &angle)
{
    return 0;
}

void BridgeThread::getLocalizedPos(double &x, double &y, double &angle)
{
    mutex_localiz.wait();
    x=ikart_current_position.x;
    y=ikart_current_position.y;
    angle=ikart_current_position.t;
    mutex_localiz.post();
}

void BridgeThread::printStats()
{
    fprintf (stdout,"Bridge thread timeouts: %d\n",timeout_counter);
}
