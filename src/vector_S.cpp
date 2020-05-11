#include "../include/vector_S.hpp"

Vector_S::Vector_S() : _identifier(0), _components(0) {}

Vector_S::Vector_S(std::vector<double> components, int identifier) : _identifier(identifier),
_components(components) {}

const int& Vector_S::getIdentifier() const {
  return _identifier;
}

const std::vector<double>& Vector_S::getComponents() const {
  return _components;
}

const double& Vector_S::getValueOfComponent(int componentIndex) const {
  return _components[componentIndex];
}

void Vector_S::setIdentifier(int identifier) {
if (identifier >= 0) {
  _identifier = identifier;
  } else {
    throw "Error at setting an identifier";
  }
}

void Vector_S::setComponents(std::vector<double> components) {
  _components = components;
}

void Vector_S::setValueOnComponent(int componentIndex, double value) {
  _components[componentIndex] = value;
}