/**
  *  Escuela Superior de Ingeniería y Tecnología
  *  Grado en Ingeniería Informática
  *  Asignatura: Diseño y análisis de algoritmos
  *  Curso: 3º
  *  Práctica 8: Max-mean dispersion problem.
  *  @author: Manuel Andrés Carrera Galafate <alu0101132020@ull.edu.es>
  *  @since: 23/04/2020
  *  @desc: Clase solución para la implementación del max-mean problem
  *  @references:
  *               Explicación del problema y la solución:
  *               https://drive.google.com/file/d/1btbyAPJNfy-9oQ9as22bmagi6o1XhHGn/view
  * 
  *
  *  @version:
  *              20/04/2020 - Creación (primera versión) del código.
  */
#include <vector>
#include <iostream>
#pragma once

class Solution {
private:
  double _score;
  std::vector<int> _solutionNodes;
public:
  // Constructor por defecto
  Solution();
  // Constructor por valor doble
  Solution(double solutionValue, std::vector<int> );
  // Getter del valor de la solución almacenada
  double getSolutionScore();
  // Getter de los nodos que componen la solución.
  std::vector<int> getSolutionNodes();
  // Setter de un valor de solución
  void setSolutionScore(double solutionToSet);
  // Setter del conjunto de nodos que compone la solución.
  void setSolutionNodes(std::vector<int> solutionNodes);
  // Imprimir solución.
  void print();
};