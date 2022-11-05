// Created by Indraneel on 22/10/22

#include "task_graph.hpp"


// Coverage planner parameters
#define COV_MIN_INFO_GAIN_RADIUS_M 0.5
#define COV_PERCENT_COVERAGE_OVERLAP 0.75f // TODO
#define COV_MIN_DIST_BETWEEN_COVERAGE_POINTS 1.0f // TODO
#define COV_MAX_INFO_GAIN_RADIUS_M 5.0 // Should reflect sensor model
std::vector<std::vector<int>> bfs_prop_model = {{-1 , 0}, {0 , -1}, {0 , 1}, {1 , 0}};

TaskGraph::TaskGraph() : nh_(""), new_data_rcvd_(false), frame_id_("map") {
    // TODO

    graph_sub_ = nh_.subscribe("/slam_toolbox/karto_graph_visualization", 1, &TaskGraph::incomingGraph, this);
    map_sub_ = nh_.subscribe("/map", 100, &TaskGraph::mapCallBack, this);
    marker_coverage_area_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/task_graph/coverage_area", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/task_graph/points", 10);

    // advertise task graph services
    task_graph_service_ = nh_.advertiseService("/robosar_task_generator/task_graph_getter", &TaskGraph::taskGraphServiceCallback, this); 
    task_setter_service_ = nh_.advertiseService("/robosar_task_generator/task_graph_setter", &TaskGraph::taskGraphSetterServiceCallback, this);  

    initMarkers();
    ROS_INFO("Task Graphing!");
    node_thread_ = std::thread(&TaskGraph::coverageTaskGenerator, this);
}


bool TaskGraph::taskGraphServiceCallback(robosar_messages::task_graph_getter::Request &req,
                                   robosar_messages::task_graph_getter::Response &res) {
    // TODO
    // Go through all the vertices and add unvisited coverage nodes to the response
    for (auto &v : V_) {
        if (v.is_coverage_node_ && !v.is_visited_) {
            res.task_ids.push_back(v.id_);
            res.points.push_back(v.pose_.position);
            res.task_types.push_back(robosar_messages::task_graph_getter::Response::COVERAGE);
            v.is_allocated_ = true;
        }
    }

    return true;
}

bool TaskGraph::taskGraphSetterServiceCallback(robosar_messages::task_graph_setter::Request &req,
                                      robosar_messages::task_graph_setter::Response &res) {
    // TODO
    // Go through all visited tasks and mark them as visited
    for(const auto &finished_task_: req.finished_task_ids) {
        V_[id_to_index_[finished_task_]].is_visited_ = true;
    }

    return true;
}

// Subscribers callback functions---------------------------------------
void TaskGraph::mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    mapData_ = *msg;
}

TaskGraph::~TaskGraph() {
    new_data_rcvd_ = true;
    cv_.notify_one();
    node_thread_.join();
}

inline bool AreSame(double a, double b)
{
    return fabs(a - b) < std::numeric_limits<double>::epsilon();
}


void TaskGraph::initMarkers()
{
    // visualizations  points and lines..
    marker_points.header.frame_id = frame_id_;
    marker_line.header.frame_id = frame_id_;
    marker_points.header.stamp = ros::Time(0);
    marker_line.header.stamp = ros::Time(0);

    marker_points.ns = marker_line.ns = "markers";
    marker_points.id = 0;
    marker_line.id = 1;

    marker_points.type = marker_points.POINTS;
    marker_line.type = marker_line.LINE_LIST;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker_points.action = marker_points.ADD;
    marker_line.action = marker_line.ADD;
    marker_points.pose.orientation.w = 1.0;
    marker_line.pose.orientation.w = 1.0;
    marker_line.scale.x = 0.03;
    marker_line.scale.y = 0.03;
    marker_points.scale.x = 0.3;
    marker_points.scale.y = 0.3;

    marker_line.color.r = 9.0 / 255.0;
    marker_line.color.g = 91.0 / 255.0;
    marker_line.color.b = 236.0 / 255.0;
    marker_points.color.r = 255.0 / 255.0;
    marker_points.color.g = 0.0 / 255.0;
    marker_points.color.b = 0.0 / 255.0;
    marker_points.color.a = 1.0;
    marker_line.color.a = 1.0;
    marker_points.lifetime = ros::Duration();
    marker_line.lifetime = ros::Duration();

    // marker for coverage points
    marker_points_coverage = marker_points;
    marker_points_coverage.color.r = 0.0;
    marker_points_coverage.color.g = 255.0 / 255.0;
    marker_points_coverage.color.b = 255.0 / 255.0;

    // marker for visited coverage points
    marker_points_cov_visited = marker_points;
    marker_points_cov_visited.color.g = 0.0;
    marker_points_cov_visited.color.r = 255.0 / 255.0;
    marker_points_cov_visited.color.b = 255.0 / 255.0;

    // marker for allocated coverage points
    marker_points_cov_allocated = marker_points;
    marker_points_cov_allocated.color.b = 0.0;
    marker_points_cov_allocated.color.r = 255.0 / 255.0;
    marker_points_cov_allocated.color.g = 255.0 / 255.0;

    // marker for coverage area
    marker_coverage_area = marker_points;
    marker_coverage_area.type = marker_coverage_area.CYLINDER;
    marker_coverage_area.color.r = 0.0;
    marker_coverage_area.color.g = 0.0;
    marker_coverage_area.color.b = 255.0 / 255.0;
    marker_coverage_area.color.a = 0.2;
    marker_coverage_area.scale.z = 0.01;

}

void TaskGraph::incomingGraph(const visualization_msgs::MarkerArrayConstPtr& new_graph)
{
  std::lock_guard<std::mutex> guard(mtx);
  bool local_graph_update = false;

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
        TaskVertex new_vertex(marker.id, marker.pose, false);
        V_.push_back(new_vertex);
        id_to_index_[marker.id] = V_.size() - 1;
        local_graph_update = true;
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
          local_graph_update = true;
        }
      }
    }
    // Check if the marker is an edge
    else if (marker.type == visualization_msgs::Marker::ARROW)
    {
      ROS_WARN("Edge marker found");
    }
  }

  // Inform main thread
  if(local_graph_update) {
    ROS_WARN("New size of graph: %ld", V_.size());
    new_data_rcvd_ = true;
    cv_.notify_one();
  }

}

void TaskGraph::coverageTaskGenerator() {
  
  while(ros::ok()) {
    std::unique_lock<std::mutex> lk(mtx);
    cv_.wait(lk, [this]{return new_data_rcvd_;});

    ROS_WARN("Running coverage task generator");

    /** ==================== Iterate through all the vertices and update information gain ===================== */
    for(auto& vertex : V_) {
      if(vertex.info_updated_) {
        ROS_WARN("Vertex %d has been updated", vertex.id_);

        std::pair<float,float> vertex_pos = std::make_pair(vertex.pose_.position.x, vertex.pose_.position.y);
        vertex.info_gain_radius_ = informationGain(vertex_pos);
        vertex.info_updated_ = false;
      }
    }

    /** ======================= Iterate again and update coverage points ================================ */
    for(auto& vertex : V_) {
//      if(vertex.info_updated_) {

        // Check if valid coverage point
        std::pair<float,float> vertex_pos = std::make_pair(vertex.pose_.position.x, vertex.pose_.position.y);
        if(vertex.is_allocated_ || vertex.is_visited_ || isValidCoveragePoint(vertex_pos, vertex.get_info_gain_radius() , vertex.id_)) {
          vertex.is_coverage_node_ = true;
        } else {
          vertex.is_coverage_node_ = false;
        }

        vertex.info_updated_ = false;
//      }
    }

    /**** ====================== Naive filtering : remove coverage points that are too close to each other ======== */
    for(auto& vertex : V_) {
      if(vertex.is_coverage_node_ && !vertex.is_visited_ && !vertex.is_allocated_) {
        std::pair<float,float> vertex_pos = std::make_pair(vertex.pose_.position.x, vertex.pose_.position.y);
        filterCoveragePoints(vertex_pos, vertex.get_info_gain_radius(), vertex.id_);
      }
    }

    // visualise coverage points
    visualizeMarkers();

    new_data_rcvd_ = false;
    lk.unlock();
  }
}

float Norm(float x1, float y1, float x2, float y2)
{
    return pow((pow((x2 - x1), 2) + pow((y2 - y1), 2)), 0.5);
}

float TaskGraph::informationGain(std::pair<float, float> &x) {
  
  // Local variables
  float info_gain_radius_m = 0.0;
  bool end_of_bfs = false;
  // BFS around this point to find distance to nearest obstacle

  // Local variables
  std::queue<std::pair<int,int>> queue;
  std::set<int> visited;

  //Convert to pixel and add it to queue
  std::pair<int,int> start_ind = std::make_pair((int)((x.first-mapData_.info.origin.position.x)/mapData_.info.resolution), 
                                                (int)((x.second-mapData_.info.origin.position.y)/mapData_.info.resolution));
  queue.push(start_ind);
  
  while(!queue.empty()&& !end_of_bfs) {
    std::pair<int,int> current = queue.front();
    queue.pop();

    //if already visited, continue to next iteration
    if(visited.find((current.first + current.second*mapData_.info.width)) != visited.end()) 
      continue;
    // insert into visited
    else
      visited.insert(current.first + current.second*mapData_.info.width);

    // Check if reached end of BFS radius
    //convert to world coordinates
    std::pair<float,float> current_position = std::make_pair(mapData_.info.origin.position.x + mapData_.info.resolution*(float)(current.first), 
                                  mapData_.info.origin.position.y + mapData_.info.resolution*(float)(current.second));

    float dist = Norm(current_position.first,current_position.second, 
                        x.first,x.second);
    if(dist > info_gain_radius_m) {
      info_gain_radius_m = dist;
    }
    if(dist > COV_MAX_INFO_GAIN_RADIUS_M) {
      //ROS_INFO("Outside radius. Exiting obstacle search");
      end_of_bfs = true;
    }

    // Check if obstacle
    if(gridValue(current_position) == 100 || gridValue(current_position) == -1) {
      end_of_bfs = true;
    }

    //expand node
    for(const auto& dir : bfs_prop_model) {
      std::pair<int,int> neighbour_ind = std::make_pair(current.first + dir[0], current.second + dir[1]);
      //Boundary checks
      if (neighbour_ind.first >= 0 && neighbour_ind.first < mapData_.info.width && 
          neighbour_ind.second >= 0 && neighbour_ind.second < mapData_.info.height ) {
            queue.push(neighbour_ind);
      }
      else
        end_of_bfs = true;
    }
  }

   return info_gain_radius_m;
 }


bool TaskGraph::isValidCoveragePoint(std::pair<float, float> x_new, float info_radius, int id)  {    
    
    // Local variables
    bool valid_node = true;
    std::vector<int> to_remove_indices;

    // parameter check
    if(info_radius < COV_MIN_INFO_GAIN_RADIUS_M) {
        return false;
    }

    int ind=0;
    for (auto j = V_.begin(); j != V_.end(); j++) {

        // Dont compare with yourself
        if(j->id_ == id) {
             ind++;
            continue;
        }

        // eliminate node which is more than info_radius away from x_new
        float max_info_radius = std::max(info_radius, j->get_info_gain_radius());
        if(fabs(x_new.first-j->pose_.position.x) > max_info_radius 
            || fabs(x_new.second-j->pose_.position.y) > max_info_radius) {
            ind++;
            continue;
        }

        // TODO should we check if it is a coverage node?
        float inter_node_dist = Norm(j->pose_.position.x, j->pose_.position.y,
                                        x_new.first, x_new.second); 
        // If there is an overlap (either of the centers is inside the other circle)
        // keep the node with the largest radius
        // TODO can control per cent overlap
        if((info_radius > inter_node_dist || j->get_info_gain_radius() > inter_node_dist)) {

            if(j->is_visited_ || j->is_allocated_) {
                valid_node = false;
                break;
            }

            if(info_radius < j->get_info_gain_radius()) {

                //std::cout<<j->get_info_gain_radius()<<" "<<info_radius<<" "<<" "<<inter_node_dist<<" "<<id<<" "<<j->id_<<std::endl;
                valid_node = false;
                break;
            }
            else {
                to_remove_indices.push_back(ind);
            }
        }

        ind++;
    }

    if(valid_node) {
        for (int ind : to_remove_indices) {
            V_[ind].is_coverage_node_ = false;
        }
    }
    return valid_node;
}

void TaskGraph::filterCoveragePoints(std::pair<float, float> x_new, float info_radius, int id) {

  // Find closest coverage node
  float closest_coverage_node_dist = std::numeric_limits<float>::max();
  int closest_coverage_node_id = -1;
  for (auto k = V_.begin(); k != V_.end(); k++) {
    if(k->id_ == id) {
      continue;
    }
    if(k->is_coverage_node_) {
      float inter_node_dist = Norm(k->pose_.position.x, k->pose_.position.y,
                                    x_new.first, x_new.second); 
      if(inter_node_dist<closest_coverage_node_dist) {
        closest_coverage_node_dist = inter_node_dist;
        closest_coverage_node_id = k->id_;
      }
    }
  }

   // Disable smaller coverage node
  if(closest_coverage_node_dist < COV_MIN_DIST_BETWEEN_COVERAGE_POINTS) {
    
    if(V_[id_to_index_[closest_coverage_node_id]].is_allocated_ || V_[id_to_index_[closest_coverage_node_id]].is_visited_ ||
         info_radius < V_[id_to_index_[closest_coverage_node_id]].get_info_gain_radius()) {
      V_[id_to_index_[id]].is_coverage_node_ = false;
    }
    else {
      V_[id_to_index_[closest_coverage_node_id]].is_coverage_node_ = false;
    }
  }

}

 // gridValue function
int TaskGraph::gridValue(std::pair<float, float> &Xp)
{
    float resolution = mapData_.info.resolution;
    float Xstartx = mapData_.info.origin.position.x;
    float Xstarty = mapData_.info.origin.position.y;

    float width = mapData_.info.width;
    std::vector<signed char> Data = mapData_.data;

    // returns grid value at "Xp" location
    // map data:  100 occupied      -1 unknown       0 free
    float indx = (floor((Xp.second - Xstarty) / resolution) * width) + (floor((Xp.first - Xstartx) / resolution));
    int out;
    out = Data[int(indx)];
    return out;
}

 void TaskGraph::visualizeMarkers(void) {

    // visualization
    marker_points_cov_allocated.points.clear();
    marker_points_cov_visited.points.clear();
    marker_points_coverage.points.clear();
    marker_coverage_area_array.markers.clear();

    // Clear old Markers
    visualization_msgs::Marker delete_all;
    delete_all.action = visualization_msgs::Marker::DELETEALL;
    delete_all.id = 0;
    marker_coverage_area_array.markers.push_back(delete_all);
    marker_coverage_area_pub_.publish(marker_coverage_area_array);
    marker_coverage_area_array.markers.clear();

    for (auto j = V_.begin(); j != V_.end(); j++)
    {
        geometry_msgs::Point p;
        p.x = j->pose_.position.x;
        p.y = j->pose_.position.y;
        p.z = 0.0;

        if(j->is_coverage_node_) {
            
            // Create coverage point marker
            if(j->is_visited_) {
                marker_points_cov_visited.points.push_back(p);
            }
            else if(j->is_allocated_) {
                marker_points_cov_allocated.points.push_back(p);
            }
            else {
                marker_points_coverage.points.push_back(p);
            }

            // Create area marker              
            marker_coverage_area.id += 1;
            marker_coverage_area.pose.position.x = p.x;
            marker_coverage_area.pose.position.y = p.y;
            marker_coverage_area.scale.x = 2 * j->get_info_gain_radius();
            marker_coverage_area.scale.y = 2 * j->get_info_gain_radius();
            marker_coverage_area_array.markers.push_back(marker_coverage_area);
        }
    }
    
    marker_pub_.publish(marker_points_cov_allocated);
    marker_pub_.publish(marker_points_cov_visited);
    marker_pub_.publish(marker_points_coverage);
    marker_coverage_area_pub_.publish(marker_coverage_area_array);
 }
