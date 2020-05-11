#include "../include/set-of-vectors.hpp"

SetOfVectors::SetOfVectors(std::vector<Vector_S> vectors, int sizeOfVectors) : _vectors(vectors),
 _numberOfVectors(vectors.size()), _sizeOfVectors(sizeOfVectors) {}

SetOfVectors::SetOfVectors(std::string inputFileName) {
  std::ifstream inputFile;
  inputFile.open(inputFileName);
  if (!inputFile) {
    std::cout << "Unable to open file\n";
    exit(1); // terminate with error
  } else {
    // Obtenemos el número de vectores y su tamaño e inicializamos el conjunto sin valores.
    inputFile >> _numberOfVectors;
    inputFile >> _sizeOfVectors;
    for (int currentVectorIndex = 0; currentVectorIndex < _numberOfVectors; currentVectorIndex++) {
      std::vector<double> dummyVector;
      double data;
      for (int currentElementOfVector = 0; currentElementOfVector < _sizeOfVectors; currentElementOfVector++) {
        inputFile >> data;
        dummyVector.push_back(data);
      }
      _vectors.push_back(Vector_S(dummyVector, currentVectorIndex));
    }
  }  
  inputFile.close();
}

int SetOfVectors::getNumberOfVectors() {
  return _numberOfVectors;
}

int SetOfVectors::getSizeOfVectors() {
  return _sizeOfVectors;
}

const std::vector<Vector_S>& SetOfVectors::getVectors() const {
  return _vectors;
}

const Vector_S& SetOfVectors::getVector(int vectorIndex) const {
  return _vectors[vectorIndex];
}

void SetOfVectors::setVectors(std::vector<Vector_S> vectors) {
  _vectors = vectors;
}

void SetOfVectors::setVectorOfIndex(int vectorIndex, Vector_S vector) {
  _vectors[vectorIndex] = vector;
}

void SetOfVectors::printVectors() {
  for (int currentVector = 0; currentVector < _numberOfVectors; currentVector++) {
    for (int currentElementOfVector = 0; currentElementOfVector < _sizeOfVectors; currentElementOfVector++) {
      std::cout << _vectors[currentVector].getValueOfComponent(currentElementOfVector) << "\t";
    }
    std::cout << "\n";
  }
}