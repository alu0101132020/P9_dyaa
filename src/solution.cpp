#include "../include/solution.hpp"

Solution::Solution() : _score(0) {}

Solution::Solution(double solutionValue, std::vector<int> solutionNodes) : _score(solutionValue),
_solutionNodes(solutionNodes) {}

double Solution::getSolutionScore() {
  return _score;
}

void Solution::setSolutionScore(double solutionToSet) {
  _score = solutionToSet;
}

std::vector<int> Solution::getSolutionNodes() {
  return _solutionNodes;
}

void Solution::setSolutionNodes(std::vector<int> solutionNodes) {
  _solutionNodes = solutionNodes;
}

void Solution::print() {
  for (int currentNode = 0; currentNode < _solutionNodes.size(); currentNode++) {
    std::cout << _solutionNodes[currentNode] << "\t";
  }
  std::cout << "\t with val: " << _score;
}