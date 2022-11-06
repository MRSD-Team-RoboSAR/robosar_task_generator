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
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include "robosar_messages/task_graph_getter.h"
#include "robosar_messages/task_graph_setter.h"
#include "functions.h"
#include "mtrand.h"
#include "rrt_utils/RRT.h"

class TaskGraph
{

public:
    TaskGraph();
    ~TaskGraph();

    class TaskVertex
    {
    public:
        TaskVertex() : pose_(), neighbors_() {}
        TaskVertex(int id, geometry_msgs::Pose pose, bool is_coverage_node)
            : id_(id), pose_(pose), neighbors_(), info_updated_(true), is_coverage_node_(is_coverage_node_), is_visited_(false), is_allocated_(false) {}

        float get_info_gain_radius() { return info_gain_radius_; };
        int id_;
        geometry_msgs::Pose pose_;
        std::vector<int> neighbors_;
        bool info_updated_;
        float info_gain_radius_;
        bool is_coverage_node_;
        bool is_visited_;
        bool is_allocated_;
        RRT rrt_;
    };

private:
    bool taskGraphServiceCallback(robosar_messages::task_graph_getter::Request &req,
                                  robosar_messages::task_graph_getter::Response &res);

    bool taskGraphSetterServiceCallback(robosar_messages::task_graph_setter::Request &req,
                                        robosar_messages::task_graph_setter::Response &res);

    void incomingGraph(const visualization_msgs::MarkerArrayConstPtr &new_graph);
    void coverageTaskGenerator();
    void initMarkers(void);
    void visualizeMarkers(void);
    float informationGain(std::pair<float, float> &x);
    int gridValue(std::pair<float, float> &Xp);
    void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void filterCoveragePoints(std::pair<float, float> x_new, float info_radius, int id);
    bool isValidCoveragePoint(std::pair<float, float> x_new, float info_radius, int id);

    // RRT functions
    void expandRRT(const ros::TimerEvent&);
    std::pair<float, float> pixelsToMap(int x_pixel, int y_pixel);

    visualization_msgs::Marker marker_points, marker_line, marker_coverage_area, marker_points_coverage;
    visualization_msgs::MarkerArray marker_coverage_area_array;
    ros::NodeHandle nh_;
    ros::Subscriber graph_sub_;
    ros::Subscriber map_sub_;
    ros::Publisher marker_coverage_area_pub_;
    ros::Publisher marker_pub_;
    nav_msgs::OccupancyGrid mapData_;
    ros::ServiceServer task_graph_service_;
    ros::ServiceServer task_setter_service_;
    std_msgs::ColorRGBA color_coverage_, color_allocated_, color_visited_;

    std::map<int, int> id_to_index_;
    std::vector<TaskVertex> V_;
    std::mutex mtx;
    bool new_data_rcvd_;
    std::thread node_thread_;
    std::condition_variable cv_;
    std::string frame_id_;

    // RRT related variables
    double rrt_expansion_period_s_;
    ros::Timer rrt_expansion_timer_;
    MTRand drand; // double in [0, 1) generator, already init
};

#endif // TASK_GRAPH_H