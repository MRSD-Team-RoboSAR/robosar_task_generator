// Created by Indraneel on 22/10/22

#ifndef TASK_GRAPH_H
#define TASK_GRAPH_H

#include <mutex>
#include <string>
#include <queue>
#include <vector>
#include <thread>
#include <condition_variable>

// ROS
#include <ros/ros.h>
#include "robosar_messages/pose_graph.h"
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>


class TaskGraph {

public:
    TaskGraph();

    class TaskVertex {
    public:
        TaskVertex() : pose_(), neighbors_() {}
        TaskVertex(int id, geometry_msgs::Pose pose) : id_(id), pose_(pose), neighbors_(), info_updated_(false) {}

        int id_;
        geometry_msgs::Pose pose_;
        std::vector<int> neighbors_;
        bool info_updated_;
    };

private:
    void incomingGraph(const visualization_msgs::MarkerArrayConstPtr& new_graph);
    void coverageTaskGenerator();

    std::map<int,int> id_to_index_;
    std::vector<TaskVertex> V_;
    ros::NodeHandle nh_;
    ros::Subscriber graph_sub_;
    std::mutex mtx;
    bool new_data_rcvd_;
    std::thread node_thread_;
    std::condition_variable cv_;
};

#endif //TASK_GRAPH_H