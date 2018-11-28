#include <stdio.h>
#include <limits.h>
#include<stdbool.h>
#include <grafo.h>



// Number of vertices in the graph


// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int minDistance(int dist[], bool sptSet[],Grafo* g)
{
   // Initialize min value
   int min = INT_MAX, min_index;

   for (int v = 0; v < g->tam; v++)
     if (sptSet[v] == false && dist[v] <= min)
         min = dist[v], min_index = v;

   return min_index;
}

// A utility function to print the constructed distance array
int printSolution(int dist[], int n)
{
   qDebug() << "Vertex   Distance from Source\n";
   for (int i = 0; i < n; i++)
      printf("%d         %d\n", i, dist[i]);
}

// Function that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
void dijkstra(Grafo* g, int src)
{
     vector<vector<int>> graph = g->nodos;
     int dist[g->tam];     // The output array.  dist[i] will hold the shortest
                      // distance from src to i

     bool sptSet[g->tam]; // sptSet[i] will be true if vertex i is included in shortest
                     // path tree or shortest distance from src to i is finalized

     // Initialize all distances as INFINITE and stpSet[] as false
     for (int i = 0; i < g->tam; i++)
        dist[i] = INT_MAX, sptSet[i] = false;

     // Distance of source vertex from itself is always 0
     dist[src] = 0;

     // Find shortest path for all vertices
     for (int count = 0; count < g->tam-1; count++)
     {
       // Pick the minimum distance vertex from the set of vertices not
       // yet processed. u is always equal to src in the first iteration.
       int u = minDistance(dist, sptSet,g);

       // Mark the picked vertex as processed
       sptSet[u] = true;

       // Update dist value of the adjacent vertices of the picked vertex.
       for (int v = 0; v < g->tam; v++)

         // Update dist[v] only if is not in sptSet, there is an edge from
         // u to v, and total weight of path from src to  v through u is
         // smaller than current value of dist[v]
         if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX && dist[u]+graph[u][v] < dist[v])
            dist[v] = dist[u] + graph[u][v];
     }

     // print the constructed distance array
     printSolution(dist, g->tam);
}



// A utility function to find the vertex with
// minimum key value, from the set of vertices
// not yet included in MST
int minKey(int key[], bool mstSet[],int V)
{
// Initialize min value
int min = INT_MAX, min_index;

for (int v = 0; v < V; v++)
    if (mstSet[v] == false && key[v] < min)
        min = key[v], min_index = v;

return min_index;
}

// A utility function to print the
// constructed MST stored in parent[]
int printMST(int parent[], int n, vector<vector<int>> graph)
{
printf("Edge \tWeight\n");
for (int i = 1; i < n; i++)
    printf("%d - %d \t%d \n", parent[i], i, graph[i][parent[i]]);
}

// Function to construct and print MST for
// a graph represented using adjacency
// matrix representation
void primMST(Grafo* g)
{
    vector<vector<int>> graph = g->nodos;
    // Array to store constructed MST
    int parent[g->tam];
    // Key values used to pick minimum weight edge in cut
    int key[g->tam];
    // To represent set of vertices not yet included in MST
    bool mstSet[g->tam];

    // Initialize all keys as INFINITE
    for (int i = 0; i < g->tam; i++)
        key[i] = INT_MAX, mstSet[i] = false;

    // Always include first 1st vertex in MST.
    // Make key 0 so that this vertex is picked as first vertex.
    key[0] = 0;
    parent[0] = -1; // First node is always root of MST

    // The MST will have V vertices
    for (int count = 0; count < g->tam-1; count++)
    {
        // Pick the minimum key vertex from the
        // set of vertices not yet included in MST
        int u = minKey(key, mstSet,g->tam);

        // Add the picked vertex to the MST Set
        mstSet[u] = true;

        // Update key value and parent index of
        // the adjacent vertices of the picked vertex.
        // Consider only those vertices which are not
        // yet included in MST
        for (int v = 0; v < g->tam; v++)

        // graph[u][v] is non zero only for adjacent vertices of m
        // mstSet[v] is false for vertices not yet included in MST
        // Update the key only if graph[u][v] is smaller than key[v]
        if (graph[u][v] && mstSet[v] == false && graph[u][v] < key[v])
            parent[v] = u, key[v] = graph[u][v];
    }

    // print the constructed MST
    printMST(parent, g->tam, graph);
}


// Find set of vertex i
int find(int i,int* parent)
{
    while (parent[i] != i)
        i = parent[i];
    return i;
}

void union1(int i, int j,int* parent)
{
    int a = find(i,parent);
    int b = find(j,parent);
    parent[a] = b;
}


void kruskalMST(Grafo* g)
{
    int parent[g->tam];
    vector<vector<int>> cost = g->nodos;
    int mincost = 0; // Cost of min MST.

    // Initialize sets of disjoint sets.
    for (int i = 0; i < g->tam; i++)
        parent[i] = i;

    // Include minimum weight edges one by one
    int edge_count = 0;
    while (edge_count < g->tam - 1) {
        int min = INT_MAX, a = -1, b = -1;
        for (int i = 0; i < g->tam; i++) {
            for (int j = 0; j < g->tam; j++) {
                if (find(i,parent) != find(j,parent) && cost[i][j] < min) {
                    min = cost[i][j];
                    a = i;
                    b = j;
                }
            }
        }

        union1(a, b,parent);
        printf("Edge %d:(%d, %d) cost:%d \n",
               edge_count++, a, b, min);
        mincost += min;
    }
    printf("\n Minimum cost= %d \n", mincost);
}
