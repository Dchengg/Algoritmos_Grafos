#ifndef GRAFO_H
#define GRAFO_H
#include<bits/stdc++.h>
#include <QDebug>
#include <vector>
using namespace std;



class Grafo
{
public:
    Grafo(int pTam);
    int tam;
    vector<vector<int>> nodos;
    void addEdge(int u, int v,int dist);
    void printGraph();

};

#endif // GRAFO_H
