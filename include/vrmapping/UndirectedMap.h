#ifndef UNDIRECTEDMAP_H_
#define UNDIRECTEDMAP_H_

#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>
#include <limits>
#include <algorithm>

class UndirectedMap
{
public:
    UndirectedMap();
    UndirectedMap(std::unordered_map<int, std::unordered_map<int, double>> adjacencyListIn);
    void addVertex(int id);
    void deleteVertex(int id);
    void addEdge(int id1, int id2, double length);
    void deleteEdge(int id1, int id2);
    bool searchVertex(int id);
    int getNumVertices();
    int getNumEdges();
    void getMap(std::unordered_map<int, std::unordered_map<int, double>> &adjacencyListOut);
    std::vector<int> aStar(int start, int goal);
    std::vector<int> dijkstra(int start, int goal);
    std::vector<int> dijkstra(int start,
                              int goal,
                              std::unordered_map<int, double> &IdAndDistance);
    void DijkstraProcess(int start,
                         std::unordered_map<int, int> &IdAndPrevious,
                         std::unordered_map<int, double> &IdAndDistance);

    void GetDijkstraPathFromPrevious(int goal,
                                     std::unordered_map<int, int> IdAndPrevious,
                                     std::vector<int> &IdsRepresentPath);
    void reset();
    std::vector<int> reconstructPath(const std::unordered_map<int, int> &cameFrom, int current);
    std::unordered_map<int, std::unordered_map<int, double>> adjacencyList_;

private:
    int heuristic(int id1, int id2);

    // std::vector<int> reconstructPath(const std::unordered_map<int, int> &cameFrom, int current);
    // std::unordered_map<int, std::unordered_map<int, double>> adjacencyList_;
};

#endif /* UNDIRECTEDMAP_H_ */
