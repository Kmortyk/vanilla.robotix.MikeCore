//
// Created by eldar on 20.04.2020.
//

#include "ros/ros.h"
#include "../../gpio_jetson_service/include/gpio_jetson_service/commands.hpp"
#include "pcl_ros/point_cloud.h"
#include "gpio_jetson_service/gpio_srv.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include "tf/transform_listener.h"
#include "inference/Bboxes.h"

#define IMAGE_WIDTH 300
#define IMAGE_HEIGHT 300
#define FAULT 20

enum ROBOT_STATES {
    FREE_RIDE, AVOID_OBSTACLE, STUCK, DETECTING_OBJECT, TARGETING_OBJECT, TARGETING_OBJECT_WAIT_RESULTS, MOVE_TO_OBJECT, STOP
} robot_state;

// Free check
bool backward, left, forward, right, forward_ride;
bool inference_at_this_spin, robot_stuck;

// Average on sides of robot
float backward_m = 0, left_m = 0, forward_m = 0, right_m = 0;

// Last position and angle of robot
double x = 0, y = 0, r = 0;
double stuck_x, stuck_y, stuck_r;
double last_x, last_y, last_r;

double transform_time_sec;
ros::ServiceClient gpio_client;
tf::TransformListener* transformListener;
tf::StampedTransform transform_bot;

// TODO remove it
float image_middle_x, image_middle_y;

// State where to go
int min_left_right = 0;

// Time when the last chage was applied
ros::Time last_change_state;

// Time when last stuck detected loop passed
ros::Time last_stuck_detected_loop;

// Counter of spins from last change state
unsigned int spins_last_change_state;

uint8_t last_move_command;

bool first_loop = true;

void gpio_command(const uint8_t command) {
    if (command != last_move_command && command != MoveCommands::FULL_STOP) {
        gpio_jetson_service::gpio_srv service;
        service.request.command = MoveCommands::FULL_STOP;
        gpio_client.call(service);
    }
    gpio_jetson_service::gpio_srv service2;
    service2.request.command = command;
    gpio_client.call(service2);
    last_move_command = command;
}

void change_robot_state(ROBOT_STATES state) {
    ROS_INFO("Robot state changed to %i", state);
    gpio_command(MoveCommands::FULL_STOP);
    robot_state = state;
    last_change_state = ros::Time::now();
    spins_last_change_state = 0;
}

void inferenceCallback(const inference::BboxesConstPtr &bboxes) {
    if(bboxes->bboxes.empty()) {
        return;
    }
//    ROS_WARN("Bboxes got! Size: %lu", bboxes->bboxes.size());
    inference::Bbox bbox;
    if (bboxes->bboxes.size() > 1) {
        float max_score = 0;
        unsigned long max_score_index = 0;
        for (unsigned long i = 0; i < bboxes->bboxes.size(); i++) {
//            ROS_WARN("Object %s with score %f.", bboxes->bboxes[i].label.c_str(), bboxes->bboxes[i].score);
            if (max_score < bboxes->bboxes[i].score) {

                if (bboxes->bboxes[i].label != "bottle") continue;

                max_score = bboxes->bboxes[i].score;
                max_score_index = i;
            }
        }
        bbox = bboxes->bboxes[max_score_index];
    } else bbox = bboxes->bboxes[0];

    if (bbox.label != "bottle") return;

    // it sets to false in main after every spin
    inference_at_this_spin = true;

    ros::Time time_now = ros::Time::now();

    if (robot_state == FREE_RIDE) {
        change_robot_state(DETECTING_OBJECT);
    }

    if (robot_state == DETECTING_OBJECT && last_change_state.toSec() + 1 < time_now.toSec()) {
        change_robot_state(TARGETING_OBJECT);
    }

    if (robot_state == TARGETING_OBJECT_WAIT_RESULTS) {
        if (last_change_state.toSec() + 0.3 < time_now.toSec())
            change_robot_state(TARGETING_OBJECT);
        else
            gpio_command(MoveCommands::FULL_STOP);
    }

    if (!(robot_state == TARGETING_OBJECT || robot_state == MOVE_TO_OBJECT))
        return;

//    ROS_WARN("Selected object %s with score %f and (%f,%f,%f,%f).", bbox.label.c_str(), bbox.score, bbox.x_min, bbox.y_min, bbox.x_max, bbox.y_max);
    float x1 = bbox.x_min;
    float y1 = bbox.y_min;
    float x2 = bbox.x_max;
    float y2 = bbox.y_max;

    float object_center_x = (x1 + x2) / 2;
    float object_center_y = (y1 + y2) / 2;
//    ROS_WARN("Object center (%f,%f).", object_center_x, object_center_y);

    // 0 - not need
    // 1 - object on the left side
    // 2 - object on the right side
    short need_to_correction = 0;

    if (object_center_x < image_middle_x - FAULT) {
        need_to_correction = 1;
    }

    if (object_center_x > image_middle_x + FAULT) {
        need_to_correction = 2;
    }

    if (robot_state == MOVE_TO_OBJECT) {
        if (need_to_correction)
            change_robot_state(TARGETING_OBJECT_WAIT_RESULTS);
        else {
            if ((y1 < 5 || y1 == IMAGE_HEIGHT - 5) && (y2 < 5 || y2 == IMAGE_HEIGHT)) {
                // Robot reached the target object
                change_robot_state(STOP);
            } else
                gpio_command(MoveCommands::FORWARD_MIDDLE);
        }
    } else {
        // robot state = TARGETING_OBJECT
        if (!need_to_correction) {
            change_robot_state(MOVE_TO_OBJECT);
            return;
        }
        if (last_change_state.toSec() + 0.3 < time_now.toSec()) {
            change_robot_state(TARGETING_OBJECT_WAIT_RESULTS);
            return;
        }
        //It is targeting object state
        switch (need_to_correction) {
            case 1:
                gpio_command(MoveCommands::RIGHT_FORWARD_MIDDLE);
                break;
            case 2:
                gpio_command(MoveCommands::LEFT_FORWARD_MIDDLE);
                break;
            default:
                ROS_ERROR("State of need_to_correction doesn't exist!");
        }
    }
}

void ydLidarPointsCallback(const sensor_msgs::LaserScanConstPtr& message) {
    float backward_lm = 0, left_lm = 0, forward_lm = 0, right_lm = 0;
    for (int i = 0; i < 719; ++i) {
         if (i > 270 && i < 450) {
            backward_lm += message->ranges[i] > 0 ? message->ranges[i] : 1;
         } else
         if (i > 90 && i < 270) {
            left_lm += message->ranges[i] > 0 ? message->ranges[i] : 1;
         } else
         if (i > 630 || i < 90) {
            forward_lm += message->ranges[i] > 0 ? message->ranges[i] : 1;
         } else
         if (i > 450 && i < 630) {
            right_lm += message->ranges[i] > 0 ? message->ranges[i] : 1;
         }
    }
    backward_m = backward_lm / 180;
    left_m = left_lm / 180;
    forward_m = forward_lm / 180;
    right_m = right_lm / 180;
    left = right = backward = forward = false;
    for (int i = 0; i < 720; i++) {
        if (message->ranges[i] > 0 && message->ranges[i] <= 0.3f) {
            if (!backward && i > 270 && i < 450) {
                //ROS_WARN("Backward obstacle");
                //ROS_INFO("Range %f", message->ranges[i]);
                backward = true;
                return;
            } else
            if (!left && i > 90 && i < 270) {
                //ROS_WARN("Left obstacle");
                //ROS_INFO("Range %f", message->ranges[i]);
                left = true;
                return;
            } else
            if (!forward && (i > 630 || i < 90)) {
//                ROS_WARN("Forward obstacle");
                //ROS_INFO("Range %f", message->ranges[i]);
                forward = true;
                return;
            } else
            if (!forward_ride && (i > 675 || i < 45)) {
//                ROS_WARN("Forward obstacle");
                //ROS_INFO("Range %f", message->ranges[i]);
                forward_ride = true;
                return;
            } else
            if (!right && i > 450 && i < 630) {
                //ROS_WARN("Right obstacle");
                //ROS_INFO("Range %f", message->ranges[i]);
                right = true;
                return;
            }
        }
    }
}

void move_to_the_side() {
    switch (min_left_right) {
        case 0:
            gpio_command(MoveCommands::RIGHT_FORWARD_MIDDLE);
            break;
        case 1:
            gpio_command(MoveCommands::LEFT_FORWARD_MIDDLE);
            break;
        default:
            ROS_ERROR("State of move to side doesn't exist!");
    }
}

void movement() {
    ros::Time time_now = ros::Time::now();
    if ((robot_state == FREE_RIDE || robot_state == AVOID_OBSTACLE ) && robot_stuck && last_change_state.toSec() + 0.5 < time_now.toSec()) {
        stuck_x = x;
        stuck_y = y;
        stuck_r = r;
        change_robot_state(STUCK);
    }
    if (robot_state == FREE_RIDE && forward_ride) {
        min_left_right = left_m <= right_m ? 0 : 1;
        change_robot_state(AVOID_OBSTACLE);
    }
    if (robot_state == FREE_RIDE) {
        gpio_command(MoveCommands::FORWARD_FAST);
        return;
    }
    if (robot_state == AVOID_OBSTACLE) {
        if (forward_ride) {
            change_robot_state(FREE_RIDE);
            return;
        }
        move_to_the_side();
        return;
    }
    if (robot_state == STUCK) {
        if (std::abs(x - stuck_x) < 5 && std::abs(y - stuck_y) < 5) {
            if (robot_stuck && last_change_state.toSec() + 0.5 < time_now.toSec()) {
                move_to_the_side();
                return;
            }
            gpio_command(MoveCommands::BACKWARD_FAST);
            return;
        }
        if (std::abs(r - stuck_r) < 30) {
            if (robot_stuck && last_change_state.toSec() + 0.5 < time_now.toSec()) {
                gpio_command(MoveCommands::BACKWARD_FAST);
                return;
            }
            move_to_the_side();
            return;
        }
        change_robot_state(FREE_RIDE);
    }
}

void stuck_detect() {
    try {
        transformListener->waitForTransform("base_link", "map", ros::Time(0), ros::Duration(1.0));
        transformListener->lookupTransform("base_link", "map", ros::Time(0), transform_bot);
    } catch (tf2::LookupException &exception) {
        ROS_ERROR("error: %s", exception.what());
        return;
    }
    //ROS_WARN("Wait for transform passed");

    x = transform_bot.getOrigin().x();
    y = transform_bot.getOrigin().y();

    double roll, pitch, yaw;
    transform_bot.getBasis().getRPY(roll, pitch, yaw);

    r = yaw * 180.0 / M_PI;

    if (first_loop) {
        last_x = x;
        last_y = y;
        last_r = r;
    }

    if (last_stuck_detected_loop.toSec() + 0.5 < ros::Time::now().toSec()) {
        double dX = std::abs(last_x - x);
        double dY = std::abs(last_y - y);
        double dR = std::abs(last_r - r);

        robot_stuck = false;

        if ((robot_state == FREE_RIDE && dX < 0.15 && dY < 0.15) ||
            (robot_state == AVOID_OBSTACLE && dR < 1.5) ||
            (robot_state == STUCK && dX < 0.15 && dY < 0.15 && dR < 1.5)) {

            robot_stuck = true;
        }
        last_stuck_detected_loop = ros::Time::now();
    }


//    ROS_WARN("dX = %f dY = %f dR = %f", dX, dY, dR);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "movement");
    image_middle_x = IMAGE_WIDTH / 2.0;
    image_middle_y = IMAGE_HEIGHT / 2.0;
    ros::NodeHandle nodeHandle;
    //sleep(5);
    transform_time_sec = ros::Time::now().toSec();
    transformListener = new tf::TransformListener(nodeHandle);
    ros::Subscriber ydlidarPointsSub =
            nodeHandle.subscribe<sensor_msgs::LaserScan>("/scan", 1, ydLidarPointsCallback);
    ros::Subscriber inferenceSub =
            nodeHandle.subscribe<inference::Bboxes>("/bboxes", 1, inferenceCallback);
    gpio_client = nodeHandle.serviceClient<gpio_jetson_service::gpio_srv>("gpio_jetson_service");
    change_robot_state(FREE_RIDE);
    last_stuck_detected_loop = ros::Time::now();
    while (ros::ok()) {
        ros::spinOnce();
        ros::Time time_now = ros::Time::now();
        stuck_detect();
        if (robot_state == DETECTING_OBJECT && last_change_state.toSec() + 1.5 < time_now.toSec()) {
            // False alarm, return robot to free ride state
            change_robot_state(FREE_RIDE);
        }
        if ((robot_state == TARGETING_OBJECT || robot_state == MOVE_TO_OBJECT || robot_state == TARGETING_OBJECT_WAIT_RESULTS)
        && !inference_at_this_spin && last_change_state.toSec() + 1 < time_now.toSec()) {
            // Robot lost the object when target or move to him, return to free ride
            change_robot_state(FREE_RIDE);
        }
        if (robot_state == FREE_RIDE || robot_state == AVOID_OBSTACLE || robot_state == STUCK) {
            movement();
        }
        inference_at_this_spin = false;
        spins_last_change_state++;
        first_loop = false;
    }
    gpio_command(MoveCommands::FULL_STOP);
    sleep(1);
    return EXIT_SUCCESS;
}