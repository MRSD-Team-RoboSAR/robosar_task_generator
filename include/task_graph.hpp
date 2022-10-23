// Created by Indraneel on 22/10/22

#ifndef TASK_GRAPH_H
#define TASK_GRAPH_H

#include <mutex>
#include <string>
#include <queue>
#include <vector>

// ROS
#include <ros/ros.h>
#include "robosar_messages/pose_graph.h"
#include <geometry_msgs/Pose.h>


class TaskGraph {

public:
    TaskGraph();

    class TaskVertex {
    public:
        TaskVertex() : pose_(), neighbors_() {}
        TaskVertex(int id, geometry_msgs::Pose pose) : id_(id), pose_(pose), neighbors_() {}

        int id_;
        geometry_msgs::Pose pose_;
        std::vector<int> neighbors_;
    };

private:
    void incomingGraph(const robosar_messages::pose_graphConstPtr& new_graph);

    std::map<int,int> id_to_index_;
    std::vector<TaskVertex> V_;
    ros::NodeHandle nh_;
    ros::Subscriber graph_sub_;
    std::mutex mtx;
};

#endif //TASK_GRAPH_H