//
// Created by eldar on 05.03.2020.
//

//#include "gpio_jetson_service/gpio_srv.h"
//bool serviceHandler(gpio_jetson_service::gpio_srv::Request &request, gpio_jetson_service::gpio_srv::Response)
//{
//
//}

//int main(int argc, char **argv)
//{
//    ros::init(argc, argv, "gpio_jetson_server");
//    return 0;
//}

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include "gpio_jetson_service/MoveAction.h"
#include "actionlib/server/simple_action_server.h"
#include "commands.hpp"


class MoveAction
{
protected:
    ros::NodeHandle nodeHandle;
    actionlib::SimpleActionServer<gpio_jetson_service::MoveAction> actionServer;
    gpio_jetson_service::MoveFeedback actionFeedback;
    gpio_jetson_service::MoveResult actionResult;


    std::string action_name;
    int goal{};
    int progress{};

public:
    MoveAction(std::string &name) :
            actionServer(nodeHandle, name, boost::bind(&MoveAction::executeCB, this, _1), false), action_name(name)
    {
        actionServer.registerPreemptCallback(boost::bind(&MoveAction::preemptCB, this));
        actionServer.start();
    }

    virtual ~MoveAction() = default;

    void preemptCB()
    {
        ROS_WARN("%s got preempted!", action_name.c_str());
        actionResult.result = false;
        actionServer.setPreempted(actionResult, "I got preempted!");
    }

    void executeCB(const gpio_jetson_service::MoveGoalConstPtr &goal)
    {
        if (!actionServer.isActive() || actionServer.isPreemptRequested()) return;
        ROS_INFO("%s is processing the goal %s", action_name.c_str(), MoveCommands::commandName(goal->command).c_str());

    }
};