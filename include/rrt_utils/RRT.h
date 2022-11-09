#ifndef RRT_H
#define RRT_H

#include <vector>
#include <queue>
#include <unordered_map>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "functions.h"
#include "Node.h"

class RRT
{
public:
    RRT() : map_to_root_(){};
    RRT(geometry_msgs::Pose root_pose)
    {
        tf2::Quaternion quat_tf;
        tf2::convert(root_pose.orientation, quat_tf);
        tf2::Vector3 pos_tf;
        tf2::convert(root_pose.position, pos_tf);
        map_to_root_ = tf2::Transform(quat_tf, pos_tf);
        root_node_ = add_node(map_to_root_.getOrigin()[0], map_to_root_.getOrigin()[1], -1);
        assert(root_node_);
        root_node_->set_root();
    };
    ~RRT(){};

    std::shared_ptr<Node> get_node(int id)
    {
        if (nodes_.find(id) == nodes_.end())
        {
            ROS_DEBUG("Node ID %d does not exist, cannot get.", id);
            return nullptr;
        }
        return nodes_[id];
    }

    std::shared_ptr<Node> add_node(float x, float y, int parent)
    {
        tf2::Transform root_to_node_tf = map_to_relative(x, y);
        nodes_[next_id_] = std::make_shared<Node>(x, y, root_to_node_tf, next_id_, parent);
        auto parent_node = get_node(parent);
        if (parent_node)
            parent_node->add_child(next_id_);
        return nodes_[next_id_++];
    }

    void remove_node(int id)
    {
        if (nodes_.find(id) == nodes_.end())
        {
            ROS_WARN("Node ID %d does not exist, cannot remove.", id);
            return;
        }
        auto curr_node = get_node(id);
        std::unordered_set<int> children = curr_node->get_children();
        // A leaf node
        if (children.size() == 0)
        {
            if (!curr_node->is_root())
            {
                auto parent = get_node(curr_node->get_parent());
                parent->remove_child(id);
            }
            nodes_.erase(id);
            return;
        }
        // remove children
        for (int child_id : children)
        {
            remove_node(child_id);
        }
        remove_node(id);
    }

    void disable_node(int id)
    {
        ROS_DEBUG("disabling node %d", id);
        if (nodes_.find(id) == nodes_.end())
        {
            ROS_WARN("Node ID %d does not exist, cannot disable.", id);
            return;
        }
        auto curr_node = get_node(id);
        if (curr_node->is_disabled()) {
            ROS_WARN("Node ID %d is already disabled.", id);
            return;
        }
        std::unordered_set<int> active_children = curr_node->get_active_children();
        // A leaf node
        if (active_children.size() == 0)
        {
            if (!curr_node->is_root())
            {
                auto parent = get_node(curr_node->get_parent());
                parent->disable_child(id);
            }
            curr_node->set_disabled(true);
            return;
        }
        // remove children
        for (int child_id : active_children)
        {
            disable_node(child_id);
        }
        disable_node(id);
    }

    std::shared_ptr<Node> get_parent_node(std::shared_ptr<Node> child)
    {
        return get_node(child->get_parent());
    }

    int nearest(float x, float y)
    {
        float min = std::numeric_limits<float>::max();
        int min_index = -1;
        float temp;
        // ROS_WARN("x: %f, y: %f", x, y);

        for (auto j = nodes_.begin(); j != nodes_.end(); j++)
        {
            if (!j->second->is_disabled()){
                temp = Norm(j->second->get_x(), j->second->get_y(), x, y);
                if (temp <= min)
                {
                    min = temp;
                    min_index = j->first;
                }
            }
        }

        return min_index;
    }

    float dijkstra(int src, int dest)
    {
        using iPair = std::pair<float, int>;
        std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>> pq;
        std::vector<float> dist(nodes_.size(), std::numeric_limits<float>::max());
        pq.push({0.0, src});
        dist[src] = 0.0;

        while (!pq.empty())
        {
            int node_id = pq.top().second;
            auto node = get_node(node_id);
            pq.pop();

            // found dest
            if (node_id == dest)
            {
                return dist[dest];
            }

            for (int neighbor : node->get_children())
            {
                float weight = Norm(node->get_x(), node->get_y(), get_node(neighbor)->get_x(), get_node(neighbor)->get_y());
                if (dist[neighbor] > dist[node_id] + weight)
                {
                    dist[neighbor] = dist[node_id] + weight;
                    pq.push({dist[neighbor], neighbor});
                }
            }
            if (!node->is_root())
            {
                int parent_id = node->get_parent();
                float weight = Norm(node->get_x(), node->get_y(), get_node(parent_id)->get_x(), get_node(parent_id)->get_y());
                if (dist[parent_id] > dist[node_id] + weight)
                {
                    dist[parent_id] = dist[node_id] + weight;
                    pq.push({dist[parent_id], parent_id});
                }
            }\
        }
        return dist[dest];
    }

    tf2::Transform map_to_relative(float x, float y)
    {
        tf2::Transform map_to_node = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(x, y, 0.0));
        tf2::Transform root_to_node = map_to_root_.inverse() * map_to_node;
        return root_to_node;
    }

    void update_node_map_coords(int node_id)
    {
        std::shared_ptr<Node> node = get_node(node_id);
        tf2::Transform root_to_node = node->get_rel_tf();
        tf2::Transform map_to_node = map_to_root_ * root_to_node;
        node->set_x(map_to_node.getOrigin()[0]);
        node->set_y(map_to_node.getOrigin()[1]);
    }

    void update_rrt(geometry_msgs::Pose root_pose_new)
    {
        tf2::Quaternion quat_tf;
        tf2::convert(root_pose_new.orientation, quat_tf);
        tf2::Vector3 pos_tf;
        tf2::convert(root_pose_new.position, pos_tf);
        map_to_root_ = tf2::Transform(quat_tf, pos_tf);

        // traverse tree and update node map coords
        for (auto j = nodes_.begin(); j != nodes_.end(); j++)
        {
            update_node_map_coords(j->first);
        }

        return;
    }

    std::unordered_map<int, std::shared_ptr<Node>> nodes_;
    int next_id_ = 0;

private:
    tf2::Transform map_to_root_;
    std::shared_ptr<Node> root_node_ = NULL;
};

#endif // RRT_H