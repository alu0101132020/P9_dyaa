#include <vector>
#include <fstream>
#include "Vector_S.hpp"
#include <string>
#include <iostream>
#pragma once

class SetOfVectors
{
private:
  // Vector de vectores que conforman el problema.
  std::vector<Vector_S> _vectors;
  int _numberOfVectors;
  int _sizeOfVectors;
public:
  // Constructor mediante un vector de vectores
  SetOfVectors(std::vector<Vector_S> vectors, int sizeOfVectors);
  // Constructor a partir de un fichero de entrada
  SetOfVectors(std::string inputFileName);
  // Retorna el número de vectores
  int getNumberOfVectors();
  // Retorna el tamaño de los vectores
  int getSizeOfVectors();
  // Retorna el vector de vectores
  const std::vector<Vector_S>& getVectors() const;
  // Retorna el vector concreto pasado por índice
  const Vector_S& getVector(int vectorIndex) const;
  // Cambia el vector de vectores.
  void setVectors(std::vector<Vector_S> vectors);
  // Cambia el vector de un índice concreto.
  void setVectorOfIndex(int vectorIndex, Vector_S vector);
  // Imprime los vectores
  void printVectors();
};