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

    ac->cancelAllGoals();
    ac->sendGoal(goal);
    return 0;
}

int BridgeThread::navigationStop()
{
    ac->cancelAllGoals();
    return 0;    
}

string BridgeThread::getNavigationStatus()
{
    string s = "UNKNOWN";
    int i = ac->getState();
    switch (i)
    {
        case actionlib::SimpleClientGoalState::PENDING:
        s = "PENDING"; break;
        case actionlib::SimpleClientGoalState::ACTIVE:
        s = "ACTIVE"; break;
        case actionlib::SimpleClientGoalState::PREEMPTED:
        s = "PREEMPTED"; break;
        case actionlib::SimpleClientGoalState::SUCCEEDED:
        s = "SUCCEEDED"; break;
        case actionlib::SimpleClientGoalState::ABORTED:
        s = "ABORTED"; break;
        case actionlib::SimpleClientGoalState::REJECTED:
        s = "REJECTED"; break;
        case actionlib::SimpleClientGoalState::PREEMPTING:
        s = "PREEMPTING"; break;
        case actionlib::SimpleClientGoalState::RECALLING:
        s = "RECALLING"; break;
        case actionlib::SimpleClientGoalState::RECALLED:
        s = "RECALLED"; break;
        case actionlib::SimpleClientGoalState::LOST:
        s = "LOST"; break;     
    }
    return s;
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
