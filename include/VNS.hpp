/**
  *  Escuela Superior de Ingeniería y Tecnología
  *  Grado en Ingeniería Informática
  *  Asignatura: Diseño y análisis de algoritmos
  *  Curso: 3º
  *  Práctica 8: Max-mean dispersion problem.
  *  @author: Manuel Andrés Carrera Galafate <alu0101132020@ull.edu.es>
  *  @since: 23/04/2020
  *  @desc: Clase VNS, que es uno de los algoritmos que resuelve el max-mean problem
  *  @references:
  *               Explicación del problema y la solución:
  *               https://drive.google.com/file/d/1btbyAPJNfy-9oQ9as22bmagi6o1XhHGn/view
  * 
  *
  *  @version:
  *              27/04/2020 - Creación (primera versión) del código.
  */

#include "algorithm.hpp"
#include <algorithm>
#pragma once

class VNSAlgorithm : public MaxMeanAlgorithm {
private:
  // Función que busca todos los pares de nodos que tienen distancias
  // positivas entre sí.
  std::vector<std::pair<double, std::pair<int, int>>> preProcess();
  // Parte constructiva del algoritmo. Se replica lo hecho en GRASP.
  // Función que construye la solución inicial. Durante un número aleatorio de iteraciones,
  // en cada paso añade un nodo del conjunto LRC que es el conjunto de las n-mejores 
  // soluciones (kelementsOfLRC).
  std::pair<double, std::vector<int>> construct(std::vector<int> solutionArray, double currentDistance, int kElementsOfLRC);
  // Una vez se tiene la solución inicial se postprocesa, es decir, se hace una búsqueda local con algoritmo greedy o ansioso.
  // El booleano indica true si realizaremos búsqueda greedy o 0 si ansiosa.
  std::pair<double, std::vector<int>> postProcess(double currentDistance, std::vector<int> solutionArray, bool greedyOrAnxious);
  // Búsqueda greedy a partir de la solución actual
  std::pair<double, std::vector<int>> constructGreedy(double currentDistance, std::vector<int> solutionArray);
  // Búsqueda ansiosa a partir de la solución actual.
  std::pair<double, std::vector<int>> constructAnxious(double currentDistance, std::vector<int> solutionArray);
  // Función que busca una nueva solución en la vecindad. La vecindad se define como soluciones qu están
  // a una distanca concreta de nuestra solución actual. En nuestro caso hemos decidido que la distancia
  // de estos entornos venga definida por la inclusión de un nodo nuevo o en la exclusión de algún nodo 
  // ya integrado en la solución. El parámetro neighborhoodDistance indica cuántos de estos elementos se
  // añadirán o excluirán respectivamente. 
  std::pair<double, std::vector<int>> shake(std::vector<int> solutionArray, double currentDistance, int neighborhoodDistance);
public:
  VNSAlgorithm(Graph workingGraph);
  // Interfaz común entre los distintos algoritmos y función principal.
  Solution resolve();
};