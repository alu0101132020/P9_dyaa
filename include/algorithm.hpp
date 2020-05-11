/**
  *  Escuela Superior de Ingeniería y Tecnología
  *  Grado en Ingeniería Informática
  *  Asignatura: Diseño y análisis de algoritmos
  *  Curso: 3º
  *  Práctica 8: Max-mean dispersion problem.
  *  @author: Manuel Andrés Carrera Galafate <alu0101132020@ull.edu.es>
  *  @since: 23/04/2020
  *  @desc: Clase algoritmo para la implementación del max-mean problem
  *  @references:
  *               Explicación del problema y la solución:
  *               https://drive.google.com/file/d/1btbyAPJNfy-9oQ9as22bmagi6o1XhHGn/view
  * 
  *
  *  @version:
  *              25/04/2020 - Creación (primera versión) del código.
  */

#include "graph.hpp"
#include "solution.hpp"
#include <chrono>
#include "solution.hpp"
#pragma once

class MaxMeanAlgorithm {
protected:
  Graph _workingGraph;
  Solution _bestSolution;
  std::chrono::time_point<std::chrono::system_clock> timeOfBestSolution;
  std::vector<std::pair<Solution, std::chrono::time_point<std::chrono::system_clock>>> solutions;
public:
  // Constructor que recibe un grafo con el que se va a trabajar
  MaxMeanAlgorithm(Graph workingGraph);
  // Método virtual que tendrán todas las clases heredadas redefinido para resolver
  // el problema según los distintos algoritmos.
  virtual Solution resolve() = 0;
  Graph getGraph();
  void setGraph(Graph workingGraph);
};