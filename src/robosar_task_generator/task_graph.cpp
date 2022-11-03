// Created by Indraneel on 22/10/22

#include "task_graph.hpp"


TaskGraph::TaskGraph() : nh_(""), new_data_rcvd_(false) {
    // TODO

    graph_sub_ = nh_.subscribe("/slam_toolbox/karto_graph_visualization", 1, &TaskGraph::incomingGraph, this);

    ROS_INFO("Task Graphing!");
    node_thread_ = std::thread(&TaskGraph::coverageTaskGenerator, this);
}

inline bool AreSame(double a, double b)
{
    return fabs(a - b) < std::numeric_limits<double>::epsilon();
}


void TaskGraph::incomingGraph(const visualization_msgs::MarkerArrayConstPtr& new_graph)
{
  std::lock_guard<std::mutex> guard(mtx);

  // Go through all the markers in the array and add them to the graph
  for (auto marker : new_graph->markers)
  {
    // Check if the marker is a node
    if (marker.type == visualization_msgs::Marker::SPHERE)
    {
      // Check if the node already exists
      if (id_to_index_.find(marker.id) == id_to_index_.end())
      {
        // Add the node to the graph
        TaskVertex new_vertex(marker.id, marker.pose);
        V_.push_back(new_vertex);
        id_to_index_[marker.id] = V_.size() - 1;
        new_data_rcvd_ = true;
      }
      else {
        // Check if there is a change in the pose
        TaskVertex graph_vertex = V_[id_to_index_[marker.id]];
        if(!AreSame(graph_vertex.pose_.position.x, marker.pose.position.x) || 
            !AreSame(graph_vertex.pose_.position.y, marker.pose.position.y)) {
          // Update the pose
          graph_vertex.pose_ = marker.pose;
          graph_vertex.info_updated_ = true;
          V_[id_to_index_[marker.id]] = graph_vertex;
          new_data_rcvd_ = true;
        }
      }
    }
    // Check if the marker is an edge
    else if (marker.type == visualization_msgs::Marker::ARROW)
    {
      ROS_WARN("Edge marker found");
    }
  }

  ROS_WARN("New size of graph: %ld", V_.size());

}

void TaskGraph::coverageTaskGenerator() {
  
}