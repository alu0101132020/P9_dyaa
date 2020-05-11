/**
  *  Escuela Superior de Ingeniería y Tecnología
  *  Grado en Ingeniería Informática
  *  Asignatura: Diseño y análisis de algoritmos
  *  Curso: 3º
  *  Práctica 9: Maximum diversity problem.
  *  @author: Manuel Andrés Carrera Galafate <alu0101132020@ull.edu.es>
  *  @since: 11/05/2020
  *  @desc: Clase vector_S para la implementación del max-mean problem
  *  @references:
  *  @version:
  *              11/05/2020 - Creación (primera versión) del código.
  */

#include <vector>
#pragma once

class Vector_S {
private:
  // Parámetros que definen un vector, los cuales son el id del vector y un conjunto
  // de aristas a otros nodos.
  int _identifier;
  std::vector<double> _components;
public:
  // Constructor por defecto.
  Vector_S();
  // Constructor con parámetros.
  Vector_S(std::vector<double> components, int identifier = -1);
  // getters y setters

  // Retorna el id del nodo.
  const int& getIdentifier() const;
  // Retorna el vector de aristas completo.
  const std::vector<double>& getComponents() const;
  // getter que devuelve el coste de la component i-ésima pasada por parámetro.
  const double& getValueOfComponent(int componentIndex) const;
  // Cambia el valor del id del nodo.
  void setIdentifier(int identifier);
  // Cambia el vector de aristas completo.
  void setComponents(std::vector<double> components);
  // Cambia el coste de una componente concreta.
  void setValueOnComponent(int componentIndex, double value);
};