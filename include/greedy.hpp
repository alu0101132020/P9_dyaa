/**
  *  Escuela Superior de Ingeniería y Tecnología
  *  Grado en Ingeniería Informática
  *  Asignatura: Diseño y análisis de algoritmos
  *  Curso: 3º
  *  Práctica 8: Max-mean dispersion problem.
  *  @author: Manuel Andrés Carrera Galafate <alu0101132020@ull.edu.es>
  *  @since: 23/04/2020
  *  @desc: Clase constructivo voraz, que es uno de los algoritmos que resuelve el max-mean problem.
  *  El algoritmo concretamente busca en cada iteración la distancia de arista mayor que haya entre
  *  cualquier nodo no solución y un nodo solución, coge todos los nodos no solución que están a esa
  *  distancia y en cada iteración añade 1.
  *  @references:
  *               Explicación del problema y la solución:
  *               https://drive.google.com/file/d/1btbyAPJNfy-9oQ9as22bmagi6o1XhHGn/view
  * 
  *
  *  @version:
  *              25/04/2020 - Creación (primera versión) del código.
  */

#include "algorithm.hpp"
#include <algorithm>
#pragma once

class GreedyAlgorithm : public MaxMeanAlgorithm {
private:
  // Función que busca todos los pares de nodos que tienen distancias
  // positivas entre sí.
  std::vector<std::pair<double, std::pair<int, int>>> getPosibleStarts();
  // Algoritmo greedy principal. En cada iteración busca la mayor distancia
  // de arista existente (no el valor medio de distancias al nodo si no solo
  // la mayor distancia a un nodo desde cualquiera de la solución) entre todos
  // los nodos no incluidos en la solución. A continuación se trata de añadir
  // un nodo cualquiera de los que cumplan esta condición viendo si la condición
  // de que el valor medio al añadir el nodo sea superior. Esto ocurre hasta que
  // no se añada ningún elemento más. Esta es una versión obviamente peor que
  // la del otro algoritmo greedy.
  std::pair<double, std::vector<int>> construct(std::vector<int> solutionArray, double currentDistance);
public:
  GreedyAlgorithm(Graph workingGraph);
  // Interfaz común entre los distintos algoritmos y función principal.
  Solution resolve();
};