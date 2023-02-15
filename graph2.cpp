#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>

/**
 * @brief An Edge structure
 *
 * @tparam T - type of the vertices that the edge connects. %T has to comply with the type of graph
 */
template <typename T>
class Edge
{
public:
    T u;
    T v;
    int weight;
    Edge() { weight = 0; }
    Edge(T u, T v, int weight)
    {
        this->u = u;
        this->v = v;
        this->weight = weight;
    }
    void makeEdge(T vertex1, T vertex2, int weight = 0);
    bool doesConnect(const T &vertex1, const T &vertex2) const;
};

/**
 * @brief Makes an edge with given vertex pairs and an weight
 *
 * @tparam T
 * @param vertex1 - First vertex label
 * @param vertex2 - Second vertex label
 * @param weight - Weight of the edge. 0 by default
 */
template <typename T>
void Edge<T>::makeEdge(T vertex1, T vertex2, int weight)
{
    this->u = vertex1;
    this->v = vertex2;
    this->weight = weight;
}

/**
 * @brief Check whether this edges connects the given two vertices
 *
 * @tparam T
 * @param vertex1
 * @param vertex2
 * @return true - if the edge connects the vertices
 * @return false - if the edge does not connect the vertices
 */
template <typename T>
bool Edge<T>::doesConnect(const T &vertex1, const T &vertex2) const
{
    return (this->u == vertex1 && this->v == vertex2) || (this->u == vertex2 && this->v == vertex1);
}

/**
 * @brief Graph data structure
 *
 * @tparam T
 */
template <typename T>
class Graph
{
private:
    std::unordered_map<T, std::vector<std::pair<T, int>>> adjList;
    std::unordered_set<T> visited;

public:
    Graph() {}
    Graph(std::vector<Edge<T>> const &edges, int n);
    void addEdge(Edge<T> &edge);
    void addVertex(T vertex);
    void DFS(T startVertex);
    void BFS(T startVertex);

private:
    void DFSPrivate(T startVertex);
    void BFSPrivate(T startVertex);
};

template <typename T>
Graph<T>::Graph(std::vector<Edge<T>> const &edges, int n)
{
    for (auto edge : edges)
    {
        adjList[edge.u].push_back({edge.v, edge.weight});
        adjList[edge.v].push_back({edge.u, edge.weight});
    }
}

template <typename T>
void Graph<T>::addEdge(Edge<T> &edge)
{
    adjList[edge.u].push_back({edge.v, edge.weight});
    adjList[edge.v].push_back({edge.u, edge.weight});
}

template <typename T>
void Graph<T>::addVertex(T vertex)
{
    adjList.insert({vertex, {}});
}

template <typename T>
void Graph<T>::DFSPrivate(T startVertex)
{
    visited.insert(startVertex);
    std::cout << startVertex << " ";
    for (auto u : adjList[startVertex])
    {
        if (visited.count(u.first) == 0)
            DFSPrivate(u.first);
    }
}

template <typename T>
void Graph<T>::DFS(T startVertex)
{
    DFSPrivate(startVertex);
    visited.clear();
}

template <typename T>
void Graph<T>::BFSPrivate(T startVertex)
{
    std::queue<T> q;
    visited.insert(startVertex);
    q.push(startVertex);
    while (!q.empty())
    {
        auto v = q.front();
        q.pop();
        std::cout << v << " ";
        for (auto u : adjList[v])
        {
            if (visited.count(u.first) == 0)
            {
                visited.insert(u.first);
                q.push(u.first);
            }
        }
    }
}

template <typename T>
void Graph<T>::BFS(T startVertex)
{
    BFSPrivate(startVertex);
    visited.clear();
}

int main()
{
    Graph<int> g;
    g.addVertex(0);
    g.addVertex(1);
    g.addVertex(2);
    g.addVertex(3);

    Edge<int> e0;
    e0.makeEdge(0, 1, 33);
    Edge<int> e1(1, 2, 43);
    Edge<int> e2(1, 3, 56);
    Edge<int> e3(2, 3, 53);

    g.addEdge(e0);
    g.addEdge(e1);
    g.addEdge(e3);

    g.DFS(0);
    std::cout << std::endl;
    g.BFS(0);

    return 0;
}
