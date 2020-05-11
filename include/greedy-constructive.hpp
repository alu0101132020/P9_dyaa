/**
  *  Escuela Superior de Ingeniería y Tecnología
  *  Grado en Ingeniería Informática
  *  Asignatura: Diseño y análisis de algoritmos
  *  Curso: 3º
  *  Práctica 8: Max-mean dispersion problem.
  *  @author: Manuel Andrés Carrera Galafate <alu0101132020@ull.edu.es>
  *  @since: 23/04/2020
  *  @desc: Clase constructivo voraz, que es uno de los algoritmos que resuelve el max-mean problem
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

class GreedyConstructiveAlgorithm : public MaxMeanAlgorithm {
private:
  // Función que busca todos los pares de nodos que tienen la máxima distancia
  // del grafo entre dos nodos entre sí.
  std::vector<std::pair<double, std::pair<int, int>>> getPosibleStarts();
  // Función que construye la solución. En cada iteración añade el nodo que
  // desde los nodos de la solución actual añade en promedio una mayor distancia.
  // Cuando no añade ningún nodo nuevo se da por concluida la búsqueda.
  std::pair<double, std::vector<int>> construct(std::vector<int> solutionArray, double currentDistance);
public:
  GreedyConstructiveAlgorithm(Graph workingGraph);
  // Interfaz común entre los distintos algoritmos y función principal.
  Solution resolve();
};