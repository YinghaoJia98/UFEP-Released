#include <vrmapping/UndirectedMap.h>
UndirectedMap::UndirectedMap()
{
    adjacencyList_.clear();
}

UndirectedMap::UndirectedMap(std::unordered_map<int, std::unordered_map<int, double>> adjacencyListIn)
{
    adjacencyList_ = adjacencyListIn;
}

void UndirectedMap::addVertex(int id)
{
    if (adjacencyList_.find(id) == adjacencyList_.end())
    {
        adjacencyList_[id] = std::unordered_map<int, double>();
    }
    else
    {
        std::cout << "Vertex with ID " << id << " already exists." << std::endl;
    }
}

void UndirectedMap::deleteVertex(int id)
{
    if (adjacencyList_.find(id) != adjacencyList_.end())
    {
        // Delete the vertex from adjacency lists of other vertices
        for (auto &vertex : adjacencyList_)
        {
            vertex.second.erase(id);
        }

        // Delete the vertex from the main adjacency list
        adjacencyList_.erase(id);
    }
    else
    {
        std::cout << "[UndirectedMap_INFO]: Vertex with ID " << id << " does not exist." << std::endl;
    }
}

void UndirectedMap::addEdge(int id1, int id2, double length)
{
    if ((adjacencyList_.find(id1) != adjacencyList_.end()) &&
        (adjacencyList_.find(id2) != adjacencyList_.end()))
    {
        adjacencyList_[id1][id2] = length;
        adjacencyList_[id2][id1] = length;
    }
    else
    {
        std::cout << "//////////ERROR//////////" << std::endl
                  << "[UndirectedMap_Info]: One or both vertices do not exist." << std::endl;
        // std::cout << "One or both vertices do not exist." << std::endl;
    }
}

void UndirectedMap::deleteEdge(int id1, int id2)
{
    if (adjacencyList_.find(id1) != adjacencyList_.end() && adjacencyList_.find(id2) != adjacencyList_.end())
    {
        adjacencyList_[id1].erase(id2);
        adjacencyList_[id2].erase(id1);
    }
    else
    {
        std::cout << "One or both vertices do not exist." << std::endl;
    }
}

bool UndirectedMap::searchVertex(int id)
{
    return adjacencyList_.find(id) != adjacencyList_.end();
}

int UndirectedMap::getNumVertices()
{
    return adjacencyList_.size();
}

int UndirectedMap::getNumEdges()
{
    int count = 0;
    for (const auto &vertex : adjacencyList_)
    {
        count += vertex.second.size();
    }
    return count / 2; // Divide by 2 since it's an undirected map
}
void UndirectedMap::getMap(std::unordered_map<int, std::unordered_map<int, double>> &adjacencyListOut)
{
    adjacencyListOut = adjacencyList_;
}

std::vector<int> UndirectedMap::aStar(int start, int goal)
{
    std::unordered_map<int, int> gScore;
    std::unordered_map<int, int> fScore;
    std::unordered_map<int, int> cameFrom;
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> openSet;

    gScore[start] = 0;
    fScore[start] = heuristic(start, goal);
    openSet.push({fScore[start], start});

    while (!openSet.empty())
    {
        int current = openSet.top().second;
        openSet.pop();

        if (current == goal)
        {
            return reconstructPath(cameFrom, goal);
        }

        for (const auto &neighbor : adjacencyList_[current])
        {
            int neighborID = neighbor.first;
            double neighborDist = neighbor.second;

            int tentativeGScore = gScore[current] + neighborDist;
            if (gScore.find(neighborID) == gScore.end() || tentativeGScore < gScore[neighborID])
            {
                cameFrom[neighborID] = current;
                gScore[neighborID] = tentativeGScore;
                fScore[neighborID] = tentativeGScore + heuristic(neighborID, goal);
                openSet.push({fScore[neighborID], neighborID});
            }
        }
    }

    // No path found
    return std::vector<int>();
}

std::vector<int> UndirectedMap::dijkstra(int start, int goal)
{
    std::unordered_map<int, double> distance;
    std::unordered_map<int, int> previous;
    std::unordered_set<int> unvisited;

    for (const auto &vertex : adjacencyList_)
    {
        distance[vertex.first] = std::numeric_limits<double>::max();
        previous[vertex.first] = -1;
        unvisited.insert(vertex.first);
    }

    distance[start] = 0;

    int CountMaxTem_ = unvisited.size();
    int CountTem_ = 0;
    while ((!unvisited.empty()) && (CountTem_ < CountMaxTem_))
    {
        CountTem_++;
        int current = -1;
        double minDist = std::numeric_limits<double>::max();

        for (const auto &vertex : unvisited)
        {
            if (distance[vertex] < minDist)
            {
                current = vertex;
                minDist = distance[vertex];
            }
        }

        if (current == goal)
        {
            return reconstructPath(previous, goal);
        }

        unvisited.erase(current);

        for (const auto &neighbor : adjacencyList_[current])
        {
            int neighborID = neighbor.first;
            double neighborDist = neighbor.second;

            double alt = distance[current] + neighborDist;
            if (alt < distance[neighborID])
            {
                distance[neighborID] = alt;
                previous[neighborID] = current;
            }
        }
    }

    // No path found
    return std::vector<int>();
}

std::vector<int> UndirectedMap::dijkstra(int start,
                                         int goal,
                                         std::unordered_map<int, double> &IdAndDistance)
{
    std::unordered_map<int, double> distance;
    std::unordered_map<int, int> previous;
    std::unordered_set<int> unvisited;

    for (const auto &vertex : adjacencyList_)
    {
        distance[vertex.first] = std::numeric_limits<double>::max();
        previous[vertex.first] = -1;
        unvisited.insert(vertex.first);
    }

    distance[start] = 0;

    int CountMaxTem_ = unvisited.size();
    int CountTem_ = 0;
    while ((!unvisited.empty()) && (CountTem_ < CountMaxTem_))
    {
        CountTem_++;
        int current = -1;
        double minDist = std::numeric_limits<double>::max();

        for (const auto &vertex : unvisited)
        {
            if (distance[vertex] < minDist)
            {
                current = vertex;
                minDist = distance[vertex];
            }
        }

        if (current == goal)
        {
            IdAndDistance.clear();
            IdAndDistance = distance;
            return reconstructPath(previous, goal);
        }

        unvisited.erase(current);

        for (const auto &neighbor : adjacencyList_[current])
        {
            int neighborID = neighbor.first;
            double neighborDist = neighbor.second;

            double alt = distance[current] + neighborDist;
            if (alt < distance[neighborID])
            {
                distance[neighborID] = alt;
                previous[neighborID] = current;
            }
        }
    }

    // No path found
    return std::vector<int>();
}

void UndirectedMap::DijkstraProcess(int start,
                                    std::unordered_map<int, int> &IdAndPrevious,
                                    std::unordered_map<int, double> &IdAndDistance)
{
    std::unordered_map<int, double> distance;
    std::unordered_map<int, int> previous;
    std::unordered_set<int> unvisited;

    for (const auto &vertex : adjacencyList_)
    {
        distance[vertex.first] = std::numeric_limits<double>::max();
        previous[vertex.first] = -1;
        unvisited.insert(vertex.first);
    }

    distance[start] = 0;
    int CountMaxTem_ = unvisited.size();
    int CountTem_ = 0;
    while ((!unvisited.empty()) && (CountTem_ < CountMaxTem_))
    {
        CountTem_++;
        int current = -1;
        double minDist = std::numeric_limits<double>::max();

        for (const auto &vertex : unvisited)
        {
            if (distance[vertex] < minDist)
            {
                current = vertex;
                minDist = distance[vertex];
            }
        }

        unvisited.erase(current);

        for (const auto &neighbor : adjacencyList_[current])
        {
            int neighborID = neighbor.first;
            double neighborDist = neighbor.second;

            double alt = distance[current] + neighborDist;
            if (alt < distance[neighborID])
            {
                distance[neighborID] = alt;
                previous[neighborID] = current;
            }
        }
    }
    IdAndPrevious.clear();
    IdAndPrevious = previous;
    IdAndDistance.clear();
    IdAndDistance = distance;
    return;
}

void UndirectedMap::GetDijkstraPathFromPrevious(int goal,
                                                std::unordered_map<int, int> IdAndPrevious,
                                                std::vector<int> &IdsRepresentPath)
{
    IdsRepresentPath.clear();
    IdsRepresentPath = reconstructPath(IdAndPrevious, goal);
    return;
}

int UndirectedMap::heuristic(int id1, int id2)
{
    // Replace with your own heuristic function
    return 0;
}

std::vector<int> UndirectedMap::reconstructPath(const std::unordered_map<int, int> &cameFrom, int current)
{
    std::vector<int> path;
    while (current != -1)
    {
        path.push_back(current);
        current = cameFrom.at(current);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void UndirectedMap::reset()
{
    adjacencyList_.clear();
}