#include "../include/greedy.hpp"

GreedyAlgorithm::GreedyAlgorithm(Graph workingGraph) : 
  MaxMeanAlgorithm(workingGraph) {}

Solution GreedyAlgorithm::resolve() {
  // Obtenemos todas las parejas de nodos que tienen la máxima afinidad del grafo.
  std::vector<std::pair<double, std::pair<int, int>>> posibleInitialSolution = getPosibleStarts();
  // Escogemos aleatoriamente una de estas parejas.
  std::pair<double, std::pair<int, int>> initialSolution = posibleInitialSolution[std::rand() % posibleInitialSolution.size()];
  std::vector<int> solutionArray;
  double currentDistance = initialSolution.first;
  // Incluimos los dos nodos iniciales de la arista en la solución.
  solutionArray.push_back(initialSolution.second.first);
  solutionArray.push_back(initialSolution.second.second);

  std::pair<double, std::vector<int>> solution = construct(solutionArray, currentDistance);

  return Solution(solution.first  / solution.second.size(), solution.second);
}

std::vector<std::pair<double, std::pair<int, int>>> GreedyAlgorithm::getPosibleStarts() {
  Graph copyOfGraph = getGraph();
  double bestSolution = -500.0;
  int indexOfFirstNodeOfSolution;
  int indexOfSecondNodeOfSolution;
  std::vector<std::pair<double, std::pair<int, int>>> posibleInitialSolutions;
  for (int currentNodeOfGraph = 0; currentNodeOfGraph < copyOfGraph.getNumberOfNodes(); currentNodeOfGraph++) {
    for (int currentEdgeOfNode = 0; currentEdgeOfNode < copyOfGraph.getNode(currentNodeOfGraph).getEdges().size(); currentEdgeOfNode++) {
      // Si el mejor valor es inferior al valor actual actualizamos el valor y borramos el conjunto de nodos que cumplen la condición.
      if (bestSolution < copyOfGraph.getNode(currentNodeOfGraph).getCostOfEdge(currentEdgeOfNode)) {
        posibleInitialSolutions.clear();
        bestSolution = copyOfGraph.getNode(currentNodeOfGraph).getCostOfEdge(currentEdgeOfNode);
        indexOfFirstNodeOfSolution = currentNodeOfGraph;
        indexOfSecondNodeOfSolution = copyOfGraph.getNode(currentNodeOfGraph).getEdges()[currentEdgeOfNode].getDestination();
      }
      // Si la solución actual tiene el mismo valor que la mejor solución incluimos la pareja.
      if (bestSolution == copyOfGraph.getNode(currentNodeOfGraph).getCostOfEdge(currentEdgeOfNode)) {
        indexOfFirstNodeOfSolution = currentNodeOfGraph;
        indexOfSecondNodeOfSolution = copyOfGraph.getNode(currentNodeOfGraph).getEdges()[currentEdgeOfNode].getDestination();
        posibleInitialSolutions.push_back(std::pair<double, std::pair<int, int>> (bestSolution, std::pair<int, int> (indexOfFirstNodeOfSolution, indexOfSecondNodeOfSolution)));
      }
    }
  }
  return posibleInitialSolutions;
}

std::pair<double, std::vector<int>> GreedyAlgorithm::construct(std::vector<int> solutionArray, double currentDistance) {
  int numberOfElementsOnSolution = 0;
  // Mientras sigamos incluyendo elementos en las iteraciones seguimos ejecutando el código.
  do {
    // Obtenemos el número de elementos incluidos en la solución y su media.
    numberOfElementsOnSolution = solutionArray.size();
    double md = currentDistance / numberOfElementsOnSolution;
    // Array de elementos que cumplen la distancia de la nueva solución.
    double maxEdge = -500;
    std::vector<int> indexOfNewNodes;
    std::vector<double> costsToNodes(getGraph().getNumberOfNodes());
    for (int currentNodeToCheck = 0; currentNodeToCheck < solutionArray.size(); currentNodeToCheck++) {
      std::vector<Edge> edgesToCheck = getGraph().getNode(solutionArray[currentNodeToCheck]).getEdges();
      for (int currentEdge = 0; currentEdge < edgesToCheck.size(); currentEdge++) {
        if ((std::find(solutionArray.begin(), solutionArray.end(), edgesToCheck[currentEdge].getDestination()) == solutionArray.end())) {
          // Sumamos la distancia de la arista al nodo actual.
          costsToNodes[edgesToCheck[currentEdge].getDestination()] += edgesToCheck[currentEdge].getCost();
          // Comprobamos si hay una solución mejor que la actual. Si la hay
          // reseteamos el array de soluciones.
          if (maxEdge < edgesToCheck[currentEdge].getCost()) {
            indexOfNewNodes.clear();
            maxEdge = edgesToCheck[currentEdge].getCost();
          }
          // Si el nuevo elemento cumple la distancia se añade al conjunto
          // de elementos candidatos para añadirse a la solución.
          if (maxEdge == edgesToCheck[currentEdge].getCost()) {
            indexOfNewNodes.push_back(edgesToCheck[currentEdge].getDestination());
          }
        }
      }
    }
  // Una vez hemos obtenido la nueva distancia máxima, comprobamos si esta al
  // añadirse mejoraría la solución.
    int randIndex = indexOfNewNodes[std::rand() % indexOfNewNodes.size()];
    if (((costsToNodes[randIndex] + currentDistance) / (solutionArray.size() + 1)) > md) {
      currentDistance += costsToNodes[randIndex];
      solutionArray.push_back(randIndex);
    }
  } while (numberOfElementsOnSolution != solutionArray.size());
  return std::pair<double, std::vector<int>> (currentDistance, solutionArray);
}