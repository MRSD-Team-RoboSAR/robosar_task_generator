#ifndef NODE_H
#define NODE_H

#include <unordered_set>
#include <tf2/LinearMath/Transform.h>

class Node
{
public:
    Node(float x, float y, tf2::Transform node_to_root, int id, int parent_id)
        : x_(x), y_(y), node_to_root_(node_to_root), id_(id), parent_(parent_id){};
    ~Node(){};

    bool is_root() { return is_root_; }
    void set_root() { is_root_ = true; }
    float get_x() { return x_; }
    float get_y() { return y_; }
    void set_x(float x) { x_ = x; }
    void set_y(float y) { y_ = y; }
    float get_rel_x() { return node_to_root_.getOrigin()[0]; }
    float get_rel_y() { return node_to_root_.getOrigin()[1]; }
    int get_id() { return id_; }
    std::pair<float, float> get_coord() { return std::make_pair(x_, y_); }

    void add_child(int child)
    {
        children_.insert(child);
    }
    void remove_child(int child)
    {
        if (children_.find(child) == children_.end())
        {
            ROS_INFO("Child %d does not exist, cannot remove.", child);
            return;
        }
        children_.erase(child);
    }
    std::unordered_set<int> get_children() { return children_; }
    int get_parent() { return parent_; }

private:
    float x_;
    float y_;
    tf2::Transform node_to_root_;
    int id_;
    const int parent_;
    bool is_root_ = false;
    std::unordered_set<int> children_;
};

#endif // NODE_H