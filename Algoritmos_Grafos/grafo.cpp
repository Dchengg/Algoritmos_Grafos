
#include "grafo.h"

Grafo::Grafo(int pTam)
{
    for(int i = 0; i < pTam; i++){
        vector<int> n(pTam);
        nodos.push_back(n);
    }
    tam = pTam;
}

// A utility function to add an edge in an
// undirected graph.
void Grafo::addEdge(int u, int v,int dist)
{
    nodos[u][v]= dist;
    nodos[v][u] = dist;
}

// A utility function to print the adjacency list
// representation of graph
void Grafo::printGraph()
{
    for (int v = 0; v < tam; v++)
    {
        qDebug()<<"\n Adjacency list of vertex "<< v << "\n head ";
        for (auto x : nodos[v])
           qDebug() << "-> " << x;
        qDebug()<<("\n");
    }
}
