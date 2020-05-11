/**
  *  Escuela Superior de Ingeniería y Tecnología
  *  Grado en Ingeniería Informática
  *  Asignatura: Diseño y análisis de algoritmos
  *  Curso: 3º
  *  Práctica 8: Max-mean dispersion problem.
  *  @author: Manuel Andrés Carrera Galafate <alu0101132020@ull.edu.es>
  *  @since: 23/04/2020
  *  @desc: Programa principal.
  *  @references:
  *               Explicación del problema y la solución:
  *               https://drive.google.com/file/d/1btbyAPJNfy-9oQ9as22bmagi6o1XhHGn/view
  * 
  *
  *  @version:
  *              20/04/2020 - Creación (primera versión) del código.
  */

#include <iostream>
#include <stdio.h>
#include <cstring>
#include "../include/vector_S.hpp"
#include "../include/set-of-vectors.hpp"

int main(int argc, char* argv[]){
  if (argc == 2) {
    srand(time(NULL));
    std::string graphString = argv[1];
  //   Graph myGraph(graphString);
  //   // myGraph.printGraph();
  //   int numberOfIterations = 5;
  //   for (int currentIteration = 0; currentIteration < numberOfIterations; currentIteration++) {
  //     GreedyConstructiveAlgorithm myAlg(myGraph);
  //     auto start = std::chrono::high_resolution_clock::now();
  //     Solution sol1 = myAlg.resolve();
  //     auto stop = std::chrono::high_resolution_clock::now();
  //     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  //     std::cout << "Greedy Constructive:\t";
  //     sol1.print();
  //     std::cout << "\t" << currentIteration << "\t" << duration.count() << "\n";
  //   }

  //   for (int currentIteration = 0; currentIteration < numberOfIterations; currentIteration++) {
  //     GreedyAlgorithm myAlg2(myGraph);
  //     auto start = std::chrono::high_resolution_clock::now();
  //     Solution sol2 = myAlg2.resolve();
  //     auto stop = std::chrono::high_resolution_clock::now();
  //     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  //     std::cout << "Greedy:\t";
  //     sol2.print();
  //     std::cout << "\t" << currentIteration << "\t" << duration.count() << "\n";
  //   }

  //   for (int currentIteration = 0; currentIteration < numberOfIterations; currentIteration++) {
  //     GraspAlgorithm myAlg3(myGraph);
  //     auto start = std::chrono::high_resolution_clock::now();
  //     Solution sol3 = myAlg3.resolve();
  //     auto stop = std::chrono::high_resolution_clock::now();
  //     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  //     std::cout << "Grasp:\t";
  //     sol3.print();
  //     std::cout << "\t" << currentIteration << "\t" << duration.count() << "\n";
  //   }

  //   for (int currentIteration = 0; currentIteration < numberOfIterations; currentIteration++) {
  //     MultiStartAlgorithm myAlg4(myGraph);
  //     auto start = std::chrono::high_resolution_clock::now();
  //     Solution sol4 = myAlg4.resolve();
  //     auto stop = std::chrono::high_resolution_clock::now();
  //     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  //     std::cout << "Multiarranque:\t";
  //     sol4.print();
  //     std::cout << "\t" << currentIteration << "\t" << duration.count() << "\n";
  //   }

    
  //   for (int currentIteration = 0; currentIteration < numberOfIterations; currentIteration++) {
  //     VNSAlgorithm myAlg5(myGraph);
  //     auto start = std::chrono::high_resolution_clock::now();
  //     Solution sol5 = myAlg5.resolve();
  //     auto stop = std::chrono::high_resolution_clock::now();
  //     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  //     std::cout << "VNS:\t";
  //     sol5.print();
  //     std::cout << "\t" << currentIteration << "\t" << duration.count() << "\n";
  //   }
  //   for (int currentIteration = 0; currentIteration < numberOfIterations; currentIteration++) {
  //     ModificactionAlgorithm myAlg6(myGraph);
  //     auto start = std::chrono::high_resolution_clock::now();
  //     Solution sol6 = myAlg6.resolve();
  //     auto stop = std::chrono::high_resolution_clock::now();
  //     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  //     std::cout << "VNS-GRASP:\t";
  //     sol6.print();
  //     std::cout << "\t" << currentIteration << "\t" << duration.count() << "\n";
  //   }
  // } else {
  //   std::cout << "Introduzca 1 y solo 1 cadena como parámetros.\n";
  // }
  // std::vector<double> vectorsito;
  // vectorsito.push_back(3.6);
  // vectorsito.push_back(3.9);
  // vectorsito.push_back(4.7);
  // Vector_S myVec(vectorsito, 3);
  // std::cout << myVec.getIdentifier() << "\t" << myVec.getValueOfComponent(2) << "\n";
    SetOfVectors mySet(graphString);
    mySet.printVectors();
  }
}

