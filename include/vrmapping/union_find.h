#ifndef UNION_FIND_H_
#define UNION_FIND_H_

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <unordered_map>

class UF
{

public:
    UF(int n);
    void merge(int p, int q);
    bool connected(int p, int q);
    int getcount();
    void addpoint(int newpoint_global_id);
    void removepoint(int newpoint_global_id);
    int find(int x);
    bool IfPointExist(int Id);

    std::unordered_map<int, int> parent;
    std::unordered_map<int, int> size;

private:
    // 连通分量个数
    int count;
    // std::vector<int> parent; // 存储一棵树
    // std::vector<int> size; // 记录树的“重量”
    // int find(int x);
    // std::unordered_map<int, int> parent;
    // std::unordered_map<int, int> size;
};

#endif
