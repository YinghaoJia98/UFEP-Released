#ifndef GRAPHBASE_H_
#define GRAPHBASE_H_
#pragma once

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <glog/logging.h>
typedef Eigen::Vector4d StateVec;

typedef ros::Time ROSTIME;
#define START_TIME(x) (x = ros::Time::now())
#define GET_ELAPSED_TIME(x) (float)((ros::Time::now() - x).toSec())

struct Vertex
{
    Vertex(int v_id, StateVec v_state)
    {
        id = v_id;
        state << v_state[0], v_state[1], v_state[2], v_state[3];
        is_obstacle = false;
    }

    // Unique id for each vertex (root: 0, others: positive number).
    int id;
    // State of the vertex: x, y, z, yaw.
    StateVec state;

    bool is_obstacle;
    double resolution;
};

// Define your custom comparison function
struct setcomp
{
    bool operator()(const Eigen::Vector3d &lhs, const Eigen::Vector3d &rhs) const
    {
        // For example, compare by the sum of all elements
        // You may want to change this to fit your specific needs
        if (abs(lhs[2] - rhs[2]) < 0.5)
        {
            Eigen::Vector2d lhTem_(lhs[0],
                                   lhs[1]);
            Eigen::Vector2d rhTem_(rhs[0],
                                   rhs[1]);
            return lhTem_.norm() < rhTem_.norm();
        }
        else
        {
            return lhs.norm() < rhs.norm();
        }

        // return lhs.norm() < rhs.norm();
    }
};

#define HASH_P 116101
#define MAX_N 10000000000

class VOXEL_LOC
{
public:
    double x, y, z;

    VOXEL_LOC(double vx = 0, double vy = 0, double vz = 0)
        : x(vx), y(vy), z(vz) {}

    bool operator==(const VOXEL_LOC &other) const
    {
        return (x == other.x && y == other.y && z == other.z);
    }
};

// Hash value
namespace std
{
    template <>
    struct hash<VOXEL_LOC>
    {
        double operator()(const VOXEL_LOC &s) const
        {
            using std::hash;
            using std::size_t;
            return ((((int64_t)(1000000 * s.z) * HASH_P) % MAX_N + (int64_t)(1000000 * s.y)) * HASH_P) % MAX_N + (int64_t)(1000000 * s.x);
        }
    };
} // namespace std

#endif /* GRAPHBASE_H_ */
