// Created by Indraneel on 22/10/22

#include "task_graph.hpp"

TaskGraph::TaskGraph() : nh_("") {
    // TODO

    graph_sub_ = nh_.subscribe("pose_graph", 1, &TaskGraph::incomingGraph, this);
}

void TaskGraph::incomingGraph(const robosar_messages::pose_graphConstPtr& new_graph)
{
  std::lock_guard<std::mutex> guard(mtx);

}