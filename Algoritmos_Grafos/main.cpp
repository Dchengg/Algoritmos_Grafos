#include <QCoreApplication>
#include <algoritmos.cpp>
#include <QFile>


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    while(1){
        qDebug()<<"ingrese el path del archivo que contiene la matriz : ";
        string input;
        std::cin >> input;
        QString path =  QString::fromStdString(input);
        QFile* file =  new QFile();
        file->setFileName(path);
        if(file->exists()){
            if(!file->open(QIODevice::ReadWrite)) {
                qDebug()<<"No se pudo abrir el archivo ";
            }else{
                QTextStream in(file);
                int fila = 0;
                qDebug()<<"Ingrese las filas de la matriz ";
                int tam;
                std::cin >> tam;
                Grafo* g =  new Grafo(tam);
                while(!in.atEnd()) {
                    QString line = in.readLine();
                    QStringList edge =  line.split(",");
                    for(int col = 0; col<edge.size();col++)
                        g->addEdge(fila, col ,edge[col].toInt());
                    fila++;
                }
                while(!in.atEnd()) {
                    QString line = in.readLine();
                    QStringList edge =  line.split(",");
                    for(int col = 0; col<edge.size();col++)
                        g->addEdge(fila, col ,edge[col].toInt());
                    fila++;
                }
                g->printGraph();
                qDebug()<< "------Dijkstra---------";
                dijkstra(g,0);
                qDebug()<< "------Prim---------";
                primMST(g);
                qDebug()<< "------Kruskal---------";
                kruskalMST(g);

            }
        }else
            qDebug()<<"El archivo ingresado no existe";
    }


    return a.exec();
}
