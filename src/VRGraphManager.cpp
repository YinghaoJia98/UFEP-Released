#include <vrmapping/VRGraphManager.h>
VRGraphManager::VRGraphManager()
{
    Initialize();
}
void VRGraphManager::Initialize()
{
    iKdTreePtr_.reset(new KD_TREE<PointType>(0.3, 0.6, 0.1));
    // Reset graph.
    Graph_.reset(new GraphType());
    // Vertex mapping.
    IdPool_.reset(new IdPool(200000));
    // Clear memory for all vertex pointers
    if (vertices_map_.size() > 0)
    {
        for (auto IteratorVerticesTem_ : vertices_map_)
        {
            delete IteratorVerticesTem_.second;
        }
    }
    vertices_map_.clear();
    // Other params.
    // id_count_ = -1;
    StateVec state(0, 0, 0, 0);
    Vertex *root_vertex = new Vertex(generateVertexID(), state);
    root_vertex->resolution = 0.16;
    PointVector point_initial_insert;
    PointType point_initial;
    point_initial.x = root_vertex->state.x();
    point_initial.y = root_vertex->state.y();
    point_initial.z = root_vertex->state.z();
    point_initial.id = root_vertex->id;
    // point_initial.data = root_vertex;
    point_initial_insert.push_back(point_initial);
    iKdTreePtr_->Build(point_initial_insert);
    // std::cout << "root_vertex id is " << root_vertex->id << endl;
    vertices_map_[root_vertex->id] = root_vertex;
    if (root_vertex->id == 0)
    {
        Graph_->addVertex(0);
    }
    else
    {
        Graph_->addVertex(root_vertex->id);
    }

    ObstaclesSet_.clear();
    IdPool_->removeId(root_vertex->id);
}
int VRGraphManager::generateVertexID()
{
    // return ++id_count_;
    return IdPool_->extractId();
}
void VRGraphManager::Reset()
{
    Initialize();
}

void VRGraphManager::ResetWithoutDelete()
{
    iKdTreePtr_.reset(new KD_TREE<PointType>(0.3, 0.6, 0.1));
    // Reset graph.
    Graph_.reset(new GraphType());
    // Vertex mapping.
    IdPool_.reset(new IdPool(100000));
    // Clear memory for all vertex pointers
    if (vertices_map_.size() > 0)
    {
        delete vertices_map_[0];
        // for (auto IteratorVerticesTem_ : vertices_map_)
        // {
        //     delete IteratorVerticesTem_.second;
        // }
    }
    vertices_map_.clear();
    // Other params.
    // id_count_ = -1;
    StateVec state(0, 0, 0, 0);
    Vertex *root_vertex = new Vertex(generateVertexID(), state);
    root_vertex->resolution = 0.01;
    PointVector point_initial_insert;
    PointType point_initial;
    point_initial.x = root_vertex->state.x();
    point_initial.y = root_vertex->state.y();
    point_initial.z = root_vertex->state.z();
    point_initial.id = root_vertex->id;
    // point_initial.data = root_vertex;
    point_initial_insert.push_back(point_initial);
    iKdTreePtr_->Build(point_initial_insert);
    // std::cout << "root_vertex id is " << root_vertex->id << endl;
    vertices_map_[root_vertex->id] = root_vertex;
    if (root_vertex->id == 0)
    {
        Graph_->addVertex(0);
    }
    else
    {
        Graph_->addVertex(root_vertex->id);
    }

    ObstaclesSet_.clear();
    IdPool_->removeId(root_vertex->id);
}

void VRGraphManager::ResetWithoutDeleteAndIgnoreIdPool()
{
    iKdTreePtr_.reset(new KD_TREE<PointType>(0.3, 0.6, 0.1));
    // Reset graph.
    Graph_.reset(new GraphType());
    // Vertex mapping.
    // Clear memory for all vertex pointers
    if (vertices_map_.size() > 0)
    {
        delete vertices_map_[0];
    }
    vertices_map_.clear();
    // Other params.
    StateVec state(0, 0, 0, 0);
    Vertex *root_vertex = new Vertex(0, state);
    root_vertex->resolution = 0.01;
    PointVector point_initial_insert;
    PointType point_initial;
    point_initial.x = root_vertex->state.x();
    point_initial.y = root_vertex->state.y();
    point_initial.z = root_vertex->state.z();
    point_initial.id = root_vertex->id;
    // point_initial.data = root_vertex;
    point_initial_insert.push_back(point_initial);
    iKdTreePtr_->Build(point_initial_insert);
    // std::cout << "root_vertex id is " << root_vertex->id << endl;
    vertices_map_[root_vertex->id] = root_vertex;
    if (root_vertex->id == 0)
    {
        Graph_->addVertex(0);
    }
    else
    {
        Graph_->addVertex(root_vertex->id);
    }
    ObstaclesSet_.clear();
}

void VRGraphManager::addVertex(Vertex *v)
{
    PointVector point_insert;
    PointType new_point;
    new_point.x = v->state.x();
    new_point.y = v->state.y();
    new_point.z = v->state.z();
    new_point.id = v->id;
    // new_point.data = v;
    point_insert.push_back(new_point);
    iKdTreePtr_->Add_Points(point_insert, false);
    Graph_->addVertex(v->id);
    vertices_map_[v->id] = v;
    if (v->is_obstacle)
    {
        Eigen::Vector3d ObstacleTem_(v->state[0],
                                     v->state[1],
                                     v->state[2]);
        ObstaclesSet_.insert(ObstacleTem_);
    }
}
void VRGraphManager::removeVertex(Vertex *v)
{
    if (v->is_obstacle)
    {
        Eigen::Vector3d ObstacleTem_(v->state[0],
                                     v->state[1],
                                     v->state[2]);
        ObstaclesSet_.erase(ObstacleTem_);
    }
    PointVector point_delete;
    PointType old_point;
    old_point.x = v->state.x();
    old_point.y = v->state.y();
    old_point.z = v->state.z();
    vector<float> PointDist;
    iKdTreePtr_->Nearest_Search(old_point, 1, point_delete, PointDist, 0.01);
    if (point_delete.size() == 1)
    {
        iKdTreePtr_->Delete_Points(point_delete);
        Graph_->deleteVertex(v->id);
        vertices_map_.erase(v->id);
        IdPool_->addId(v->id);
        delete v;
    }
    else
    {
        ROS_ERROR("When trying remove vertex, size of point_delete is %ld", point_delete.size());
        iKdTreePtr_->Delete_Points(point_delete);
        Graph_->deleteVertex(v->id);
        vertices_map_.erase(v->id);
        IdPool_->addId(v->id);
        delete v;
    }
}

void VRGraphManager::removeVertices(std::vector<Vertex *> vertices)
{
    PointVector points_delete;
    points_delete.clear();
    for (size_t i = 0; i < vertices.size(); i++)
    {
        if (vertices[i]->is_obstacle)
        {
            Eigen::Vector3d ObstacleTem_(vertices[i]->state[0],
                                         vertices[i]->state[1],
                                         vertices[i]->state[2]);
            ObstaclesSet_.erase(ObstacleTem_);
        }
        PointVector point_delete;
        point_delete.clear();
        PointType old_point;
        old_point.x = vertices[i]->state.x();
        old_point.y = vertices[i]->state.y();
        old_point.z = vertices[i]->state.z();
        vector<float> PointDist;
        iKdTreePtr_->Nearest_Search(old_point, 1, point_delete, PointDist, 0.01);
        points_delete.insert(points_delete.end(), point_delete.begin(), point_delete.end());

        Graph_->deleteVertex(vertices[i]->id);
        vertices_map_.erase(vertices[i]->id);
        IdPool_->addId(vertices[i]->id);
    }
    iKdTreePtr_->Delete_Points(points_delete);
    for (size_t i = 0; i < vertices.size(); i++)
    {
        delete vertices[i];
    }
}

void VRGraphManager::removeVerticesInBox(std::vector<Vertex *> vertices,
                                         Eigen::Vector3d CenterPosition,
                                         double BoxXLength,
                                         double BoxYLength,
                                         double BoxZLength)
{
    for (size_t i = 0; i < vertices.size(); i++)
    {
        if (vertices[i]->is_obstacle)
        {
            Eigen::Vector3d ObstacleTem_(vertices[i]->state[0],
                                         vertices[i]->state[1],
                                         vertices[i]->state[2]);
            ObstaclesSet_.erase(ObstacleTem_);
        }
        Graph_->deleteVertex(vertices[i]->id);
        vertices_map_.erase(vertices[i]->id);
        IdPool_->addId(vertices[i]->id);
    }
    std::vector<BoxPointType> Boxes;
    Boxes.clear();
    BoxPointType BoxPoint;
    BoxPoint.vertex_min[0] = CenterPosition[0] - BoxXLength;
    BoxPoint.vertex_min[1] = CenterPosition[1] - BoxYLength;
    BoxPoint.vertex_min[2] = CenterPosition[2] - BoxZLength;
    BoxPoint.vertex_max[0] = CenterPosition[0] + BoxXLength;
    BoxPoint.vertex_max[1] = CenterPosition[1] + BoxYLength;
    BoxPoint.vertex_max[2] = CenterPosition[2] + BoxZLength;
    Boxes.push_back(BoxPoint);
    iKdTreePtr_->Delete_Point_Boxes(Boxes);
    for (size_t i = 0; i < vertices.size(); i++)
    {
        delete vertices[i];
    }
}

void VRGraphManager::addEdge(Vertex *v, Vertex *u, double weight)
{
    Graph_->addEdge(v->id, u->id, weight);
    // edge_map_[v->id].push_back(std::make_pair(u->id, weight));
    // edge_map_[u->id].push_back(std::make_pair(v->id, weight));
}

void VRGraphManager::removeEdge(Vertex *v, Vertex *u)
{
    Graph_->deleteEdge(v->id, u->id);
}

int VRGraphManager::getNumVertices()
{
    return Graph_->getNumVertices();
}
int VRGraphManager::getNumEdges()
{
    return Graph_->getNumEdges();
}

Vertex *VRGraphManager::getVertex(int id)
{
    // TODO Add a judgment of if the id exist.
    if (vertices_map_.find(id) == vertices_map_.end())
    {
        ROS_ERROR("Vertex ID %d does not exist in the graph.", id);
        return NULL;
    }
    else
    {
        return vertices_map_[id];
    }
}

bool VRGraphManager::getNearestVertex(const StateVec *state, Vertex **v_res)
{
    if (getNumVertices() <= 0)
    {
        return false;
    }

    PointType point_target;
    point_target.x = state->x();
    point_target.y = state->y();
    point_target.z = state->z();

    PointVector search_result;

    vector<float> PointDist;

    iKdTreePtr_->Nearest_Search(point_target, Nearest_Num0, search_result, PointDist, max_dist0);

    if (search_result.size() <= 0)
    {
        return false;
    }

    PointType point_out;
    point_out = search_result[0];
    if (vertices_map_.find(point_out.id) == vertices_map_.end())
    {
        ROS_ERROR("[VRGrahManager_Info]: Can not get the nearest vertex as the id is not exist in vertices map.");
        return false;
    }
    *v_res = vertices_map_[point_out.id];
    //*v_res = (Vertex *)point_out.data;
    return true;
}

bool VRGraphManager::getNearestVertexInRange(const StateVec *state, double range,
                                             Vertex **v_res)
{
    if (getNumVertices() <= 0)
    {
        return false;
    }
    PointType point_target;
    point_target.x = state->x();
    point_target.y = state->y();
    point_target.z = state->z();

    PointVector search_result;
    vector<float> PointDist;
    iKdTreePtr_->Nearest_Search(point_target, Nearest_Num0, search_result, PointDist, range);

    if (search_result.size() <= 0)
    {
        return false;
    }
    PointType point_out;
    point_out = search_result[0];
    if (vertices_map_.find(point_out.id) == vertices_map_.end())
    {
        ROS_ERROR("[VRGrahManager_Info]: Can not get the nearest vertex as the id is not exist in vertices map.");
        return false;
    }
    *v_res = vertices_map_[point_out.id];
    //*v_res = (Vertex *)point_out.data;

    Eigen::Vector3d dist;
    dist << state->x() - (*v_res)->state.x(), state->y() - (*v_res)->state.y(),
        state->z() - (*v_res)->state.z();
    if (dist.norm() > range)
    {
        return false;
    }
    return true;
}

bool VRGraphManager::getNearestVertices(const StateVec *state, double range,
                                        std::vector<Vertex *> *v_res)
{
    // Notice that this might include the same vertex in the result.
    // if that vertex is added to the tree before.
    // Use the distance 0 or small threshold to filter out.

    if (getNumVertices() <= 0)
    {
        return false;
    }
    PointType point_target;
    point_target.x = state->x();
    point_target.y = state->y();
    point_target.z = state->z();

    PointVector search_result;

    vector<float> PointDist;

    iKdTreePtr_->Nearest_Search(point_target, Nearest_Num1, search_result, PointDist, range);

    if (search_result.size() <= 0)
    {
        return false;
    }

    PointType point_out;
    v_res->clear();
    for (size_t i = 0; i < search_result.size(); ++i)
    {
        point_out = search_result[i];
        if (vertices_map_.find(point_out.id) == vertices_map_.end())
        {
            ROS_ERROR("[VRGrahManager_Info]: Can not get the nearest vertex as the id is not exist in vertices map.");
            return false;
        }
        Vertex *new_neighbor = vertices_map_[point_out.id];
        // Vertex *new_neighbor = (Vertex *)point_out.data;
        v_res->push_back(new_neighbor);
    }
    return true;
}

bool VRGraphManager::getNearestVerticesInBox(const StateVec *state, double limitx_, double limity_,
                                             double limitz_, std::vector<Vertex *> *v_res)
{
    if (getNumVertices() <= 0)
    {
        return false;
    }
    PointVector search_result;
    BoxPointType BoxPoint;
    BoxPoint.vertex_min[0] = state->x() - limitx_;
    BoxPoint.vertex_min[1] = state->y() - limity_;
    BoxPoint.vertex_min[2] = state->z() - limitz_;
    BoxPoint.vertex_max[0] = state->x() + limitx_;
    BoxPoint.vertex_max[1] = state->y() + limity_;
    BoxPoint.vertex_max[2] = state->z() + limitz_;

    iKdTreePtr_->Box_Search(BoxPoint, search_result);

    if (search_result.size() <= 0)
    {
        return false;
    }
    PointType point_out;
    v_res->clear();
    for (size_t i = 0; i < search_result.size(); ++i)
    {
        point_out = search_result[i];
        if (vertices_map_.find(point_out.id) == vertices_map_.end())
        {
            ROS_ERROR("[VRGrahManager_Info]: Can not get the nearest vertex as the id is not exist in vertices map.");
            return false;
        }
        Vertex *new_neighbor = vertices_map_[point_out.id];
        // Vertex *new_neighbor = (Vertex *)point_out.data;
        v_res->push_back(new_neighbor);
    }

    return true;
}

void VRGraphManager::getUndirectedMap(std::unordered_map<int, std::unordered_map<int, double>> &MapOut)
{
    std::unordered_map<int, std::unordered_map<int, double>> MapMiddle;
    Graph_->getMap(MapMiddle);
    MapOut = MapMiddle;
}
