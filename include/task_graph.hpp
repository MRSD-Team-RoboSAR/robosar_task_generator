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
#include <nav_msgs/OccupancyGrid.h>


class TaskGraph {

public:
    TaskGraph();
    ~TaskGraph();

    class TaskVertex {
    public:
        TaskVertex() : pose_(), neighbors_() {}
        TaskVertex(int id, geometry_msgs::Pose pose, bool is_coverage_node) 
                : id_(id), pose_(pose), neighbors_(), info_updated_(true), is_coverage_node_(is_coverage_node_) {}

        float get_info_gain_radius() { return info_gain_radius_; };
        int id_;
        geometry_msgs::Pose pose_;
        std::vector<int> neighbors_;
        bool info_updated_;
        float info_gain_radius_;
        bool is_coverage_node_;

    };

private:
    void incomingGraph(const visualization_msgs::MarkerArrayConstPtr& new_graph);
    void coverageTaskGenerator();
    void initMarkers(void);
    void visualizeMarkers(void);
    float informationGain(std::pair<float, float> &x);
    int gridValue(std::pair<float, float> &Xp);
    void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void filterCoveragePoints(std::pair<float, float> x_new, float info_radius, int id);
    bool isValidCoveragePoint(std::pair<float, float> x_new, float info_radius, int id);

    visualization_msgs::Marker marker_points, marker_line, marker_coverage_area, marker_points_coverage;
    visualization_msgs::MarkerArray marker_coverage_area_array;
    ros::NodeHandle nh_;
    ros::Subscriber graph_sub_;
    ros::Subscriber map_sub_;
    ros::Publisher marker_coverage_area_pub_;
    ros::Publisher marker_pub_;
    nav_msgs::OccupancyGrid mapData_;

    std::map<int,int> id_to_index_;
    std::vector<TaskVertex> V_;
    std::mutex mtx;
    bool new_data_rcvd_;
    std::thread node_thread_;
    std::condition_variable cv_;
    std::string frame_id_;
};

#endif //TASK_GRAPH_H