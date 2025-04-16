#ifndef VRGRAPHMANAGER_H_
#define VRGRAPHMANAGER_H_
#include <vrmapping/UndirectedMap.h>
#include <ikd-Tree/ikd_Tree_impl.h>
#include <vrmapping/GraphBase.h>
#include <ros/ros.h>
#include <vrmapping/IdPool.h>
using PointType = ikdTree_PointType_new;
using PointVector = KD_TREE<PointType>::PointVector;
class VRGraphManager
{
public:
  typedef UndirectedMap GraphType;
  VRGraphManager();
  void Initialize();
  int generateVertexID();
  void Reset();
  void ResetWithoutDelete();
  void ResetWithoutDeleteAndIgnoreIdPool();
  void addVertex(Vertex* v);
  void removeVertex(Vertex* v);
  void removeVertices(std::vector<Vertex*> vertices);
  void removeVerticesInBox(std::vector<Vertex*> vertices, Eigen::Vector3d CenterPosition, double BoxXLength,
                           double BoxYLength, double BoxZLength);
  void addEdge(Vertex* v, Vertex* u, double weight);
  void removeEdge(Vertex* v, Vertex* u);
  int getNumVertices();
  int getNumEdges();
  Vertex* getVertex(int id);

  bool getNearestVertex(const StateVec* state, Vertex** v_res);
  bool getNearestVertexInRange(const StateVec* state, double range, Vertex** v_res);
  bool getNearestVertices(const StateVec* state, double range, std::vector<Vertex*>* v_res);
  bool getNearestVerticesInBox(const StateVec* state, double limitx_, double limity_, double limitz_,
                               std::vector<Vertex*>* v_res);

  void getUndirectedMap(std::unordered_map<int, std::unordered_map<int, double>>& MapOut);
  std::unordered_map<int, Vertex*> vertices_map_;
  std::shared_ptr<GraphType> Graph_;
  std::set<Eigen::Vector3d, setcomp> ObstaclesSet_;
  std::shared_ptr<KD_TREE<PointType>> iKdTreePtr_;
  std::shared_ptr<IdPool> IdPool_;

private:
  // std::shared_ptr<KD_TREE<PointType>> iKdTreePtr_;
  // KD_TREE<PointType> *kdtree_ptr;
  // std::shared_ptr<GraphType> Graph_;
  // std::shared_ptr<IdPool> IdPool_;

  // int id_count_;

  int Nearest_Num0 = 1;
  int Nearest_Num1 = 65;

  double max_dist0 = 3.0;
  double max_dist1 = 5.0;
};

#endif /* VRGRAPHMANAGER_H_ */
