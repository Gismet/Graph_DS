#include <iostream>
#include <vector>
#include <queue>

/*Some terminology*/
/**
 * Graph - A data structure that consists of a set of vertices and a set of edges connecting vertices
 * Vertex - A node in a graph
 * Edge - a pair of vertices representing a connection between two nodes in a graph
 * Undirected graph - A graph in which edges have no direction
 * Directed graph(aka digraph) - A graph in which each edge is directed from one vertex to another or the same vertex
 *
 */

/*
G =  {      0
        1
      / | \
     2  3  4
      \    /
       6  5
   }

    Graph G(V, E);

 a set of vertices V(G) of the above graph =  {0,1,2,3,4,5,6};
 a set of edges (written as pair of vertices) = {{1,2}, {1,3}, {1,4}, {2,6}, {4,5}}

 representation:
  - adjacency matrix
  - adjacency list
  - edge list(not common)
*/

// A struct to store edges
struct Edge
{
    // source and destination of edge
    int src, dest;
};

// A simple graph implementation using adjacency list representation
class Graph
{
public:
    // a vector of vectors to store adjacency list of the graph
    std::vector<std::vector<int>> adjList;

    // graph constructor
    Graph(std::vector<Edge> &edges, int numOfVertices)
    {
        // resize adjList to hold all vertices
        adjList.resize(numOfVertices); // a vector of size numOfVertices

        for (auto &edge : edges)
        {
            /*for example, say, the edge is {1,2},  then adjList[1].push_back(2) means a connection between vertex 1 and vertex 2*/
            adjList[edge.src].push_back(edge.dest);
            adjList[edge.dest].push_back(edge.src); // for directed graphs, comment out this line of code
        }
    }
};

/*
 * Depth-first search - an algorithm to travers a graph or a tree data structure.
 * Start at a vertex(root in case of a tree), explore each branch as possible as depth.
 * You can implement DFS iteratively by using a stack.
 */

// dfs recursive procedure
void DFS(Graph const &graph, int startVertex, std::vector<bool> &visited)
{
    // mark the current vertex (startVertex) as visited
    visited[startVertex] = true; // if,for example, the vertex is 1 that we start at, then we do visited[1] = true so as to avoid visiting it again

    // process the current vertex. This could be any process. Let's just print it out
    std::cout << startVertex << " ";

    // repeat the above for every edge (startVertex, u)
    for (int u : graph.adjList[startVertex])
    {
        // if the vertex has not been visited yet, call DFS on it
        if (!visited[u])
        {
            DFS(graph, u, visited);
        }
    }
}

/**
 * Bread-first search - another traversal algorithm for graphs and trees
 * Start at a vertex, visit every adjacent vertices(aka neighboring vertices) before moving to the next level
 * (This is referred to as level order traversal in trees)
 * Bread-first search uses a queue data structure
 */

// BFS iterative procedure
void BFS(Graph const &graph, int startVertex, std::vector<bool> &visited)
{
    // first let's create a queue
    std::queue<int> q;
    // mark startVertex visited
    visited[startVertex] = true;
    // push(enqueue) it to the queue
    q.push(startVertex);

    // repeat untill queue is empty
    while (!q.empty())
    {
        // dequeue front vertex
        startVertex = q.front();
        // pop it off the queue
        q.pop();
        // process the vertex
        std::cout << startVertex << " ";

        // repeat for every edge (startVertex, u)
        for (int u : graph.adjList[startVertex])
        {
            // if the vertex has not been visited yet, visit it
            if (!visited[u])
            {
                visited[u] = true; // mark it as visited
                q.push(u);         // push to the queue
            }
        }
    }
};

// test

int main()
{
    // a vector of edges for the above graph
    std::vector<Edge> edges = {{1, 2},
                               {1, 3},
                               {1, 4},
                               {2, 6},
                               {4, 5}};
    // vertex 0 is a single vertex

    // number of vertices labelled from 0 to 6
    int n = 7;
    // initialize a graph
    Graph graph(edges, n);

    // we need this to keep track of whether a vertes is visited or not
    std::vector<bool> visited(n, false); // assign every element false

    // Call DFS/BFS on each unvisited vertex so to cover all connected component(a subgraph in which each vertex is reachable from any other vertex)
    for (int i = 0; i < n; i++)
    {
        // if vertex i is not visited
        if (visited[i] == false)
        {
            DFS(graph, i, visited);
        }
    }

    return 0;
}
