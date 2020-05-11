#include "../include/algorithm.hpp"

MaxMeanAlgorithm::MaxMeanAlgorithm(Graph workingGraph) : _workingGraph(workingGraph) {}

Graph MaxMeanAlgorithm::getGraph() {
  return _workingGraph;
}

void MaxMeanAlgorithm::setGraph(Graph workingGraph) {
  _workingGraph = workingGraph;
  _bestSolution.setSolutionScore(0);
  _bestSolution.setSolutionNodes(std::vector(1, 0));
}
