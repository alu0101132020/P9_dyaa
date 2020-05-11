#include "../include/multi-start.hpp"

MultiStartAlgorithm::MultiStartAlgorithm(Graph workingGraph) : 
  MaxMeanAlgorithm(workingGraph) {}

Solution MultiStartAlgorithm::resolve() {
  // Obtenemos todas las parejas de nodos que tienen distancia positiva entre sí.
  std::vector<std::pair<double, std::pair<int, int>>> posibleInitialSolution = preProcess();
  int iterationsWithNoImprove = -1;
  for (int currentSolution = 0; currentSolution < 1000; currentSolution++) {
    iterationsWithNoImprove++;
    // Escogemos aleatoriamente una de estas parejas.

    std::pair<double, std::pair<int, int>> initialSolution = posibleInitialSolution[std::rand() % posibleInitialSolution.size()];
    std::vector<int> solutionArray;
    double currentDistance = initialSolution.first;
    // Incluimos los dos nodos iniciales de la arista en la solución.
    solutionArray.push_back(initialSolution.second.first);
    solutionArray.push_back(initialSolution.second.second);

    int kElementsOfLRC = 3;

    std::pair<double, std::vector<int>> solution = construct(solutionArray, currentDistance);
    std::pair<double, std::vector<int>> solution2 = postProcess(solution.first, solution.second, false);
    if (_bestSolution.getSolutionScore() < (solution2.first / solution2.second.size())) {
      _bestSolution.setSolutionNodes(solution2.second);
      _bestSolution.setSolutionScore(solution2.first / solution2.second.size());
      iterationsWithNoImprove = 0;
    }
    if (iterationsWithNoImprove == 200) {
      break;
    }
  }
  return _bestSolution;
}

std::vector<std::pair<double, std::pair<int, int>>> MultiStartAlgorithm::preProcess() {
  Graph copyOfGraph = getGraph();
  int indexOfFirstNodeOfSolution;
  int indexOfSecondNodeOfSolution;
  std::vector<std::pair<double, std::pair<int, int>>> posibleInitialSolutions;
  for (int currentNodeOfGraph = 0; currentNodeOfGraph < copyOfGraph.getNumberOfNodes(); currentNodeOfGraph++) {
    for (int currentEdgeOfNode = 0; currentEdgeOfNode < copyOfGraph.getNode(currentNodeOfGraph).getEdges().size(); currentEdgeOfNode++) {
      // Si la pareja de nodos tiene un valor positivo lo incluimos como posible solución inicial
      if (0 < copyOfGraph.getNode(currentNodeOfGraph).getCostOfEdge(currentEdgeOfNode)) {
        double currentSolution = copyOfGraph.getNode(currentNodeOfGraph).getCostOfEdge(currentEdgeOfNode);
        indexOfFirstNodeOfSolution = currentNodeOfGraph;
        indexOfSecondNodeOfSolution = copyOfGraph.getNode(currentNodeOfGraph).getEdges()[currentEdgeOfNode].getDestination();
        posibleInitialSolutions.push_back(std::pair<double, std::pair<int, int>> (currentSolution, std::pair<int, int> (indexOfFirstNodeOfSolution, indexOfSecondNodeOfSolution)));
      }
    }
  }
  return posibleInitialSolutions;
}

std::pair<double, std::vector<int>> MultiStartAlgorithm::construct(std::vector<int> solutionArray, double currentDistance) {
  int numberOfElementsOnSolution = 0;
  int iteration = 0;
  int numberRandomOfIterations = std::rand() % getGraph().getNumberOfNodes();
  // Número aleatorio de iteraciones que se repetirá el proceso
  do {
    // Obtenemos el número de elementos incluidos en la solución y su media.
    numberOfElementsOnSolution = solutionArray.size();
    double md = currentDistance / numberOfElementsOnSolution;
    // Array de distancia a los distintos nodos fuera de la solución (Los que estén dentro acabarán a 0)
    std::vector<double> costToNodesOutOfSolution(getGraph().getNumberOfNodes(), 0);
    // Para cada elemento de la solución actual recorremos sus aristas.
    for (int currentNodeOfCurrentSolution = 0; currentNodeOfCurrentSolution < solutionArray.size(); currentNodeOfCurrentSolution++) {
      std::vector<Edge> edgesToCheck = getGraph().getNode(solutionArray[currentNodeOfCurrentSolution]).getEdges();
      for (int currentEdge = 0; currentEdge < edgesToCheck.size(); currentEdge++) {
        // Comprobamos que el elemento al que va la arista no esté ya en la solución
        if ((std::find(solutionArray.begin(), solutionArray.end(), edgesToCheck[currentEdge].getDestination()) == solutionArray.end())) {
          costToNodesOutOfSolution[edgesToCheck[currentEdge].getDestination()] += edgesToCheck[currentEdge].getCost();
        }
      }
    }
    // Nos quedamos solo con los costes e índices de los nodos no-incluidos en la solución.
    std::vector<std::pair<int, double>> candidatesForNextElement;
    for (int currentNode = 0; currentNode < costToNodesOutOfSolution.size(); currentNode++) {
      if ((std::find(solutionArray.begin(), solutionArray.end(), currentNode) == solutionArray.end())) {
        candidatesForNextElement.push_back(std::pair<int, double> (currentNode, costToNodesOutOfSolution[currentNode]));
      }
    }

    // Añadimos un elemento aleatorio del LRC a la solución.
    if (candidatesForNextElement.size() > 0) {
      int randIndex = (std::rand() % candidatesForNextElement.size());
      currentDistance += candidatesForNextElement[randIndex].second;
      solutionArray.push_back(candidatesForNextElement[randIndex].first);
    }
    iteration++;
  } while (iteration < numberRandomOfIterations);
  return std::pair<double, std::vector<int>> (currentDistance, solutionArray);
}

std::pair<double, std::vector<int>> MultiStartAlgorithm::postProcess(double currentDistance, std::vector<int> solutionArray, bool greedyOrAnxious) {
  std::pair <double, std::vector<int>> solution;
  // GreedyOrAnxious = true es hacer greedy
  if (greedyOrAnxious) {
    solution = constructGreedy(currentDistance, solutionArray);
  } else {
    solution = constructAnxious(currentDistance, solutionArray);
  }
  return solution;
}

std::pair<double, std::vector<int>> MultiStartAlgorithm::constructGreedy(double currentDistance, std::vector<int> solutionArray) {
  if (solutionArray.size() < getGraph().getNumberOfNodes()) {
    int numberOfElementsOnSolution = 0;
    // Mientras sigamos incluyendo elementos en las iteraciones seguimos ejecutando el código.
    do {
      // Obtenemos el número de elementos incluidos en la solución y su media.
      numberOfElementsOnSolution = solutionArray.size();
      double md = currentDistance / numberOfElementsOnSolution;
      // Array de elementos que cumplen la distancia de la nueva solución.
      std::vector<int> costToNodesOutOfSolution(getGraph().getNumberOfNodes(), 0);

      // Para cada elemento de la solución actual recorremos sus aristas.
      for (int currentNodeOfCurrentSolution = 0; currentNodeOfCurrentSolution < solutionArray.size(); currentNodeOfCurrentSolution++) {
        std::vector<Edge> edgesToCheck = getGraph().getNode(solutionArray[currentNodeOfCurrentSolution]).getEdges();
        for (int currentEdge = 0; currentEdge < edgesToCheck.size(); currentEdge++) {
          // Comprobamos que el elemento al que va la arista no esté ya en la solución
          if ((std::find(solutionArray.begin(), solutionArray.end(), edgesToCheck[currentEdge].getDestination()) == solutionArray.end())) {
            costToNodesOutOfSolution[edgesToCheck[currentEdge].getDestination()] += edgesToCheck[currentEdge].getCost();
          }
        }
      }
      double maxEdge = -500;
      std::vector<int> indexOfNewNodes;
      for (int currentNodeToCheck = 0; currentNodeToCheck < costToNodesOutOfSolution.size(); currentNodeToCheck++) {
        if (std::find(solutionArray.begin(), solutionArray.end(), currentNodeToCheck) == solutionArray.end()) {
            // Comprobamos si hay una solución mejor que la actual. Si la hay
            // reseteamos el array de soluciones.
          if (maxEdge < costToNodesOutOfSolution[currentNodeToCheck]) {
            indexOfNewNodes.clear();
            maxEdge = costToNodesOutOfSolution[currentNodeToCheck];
          }
          // Si el nuevo elemento cumple la distancia se añade al conjunto
          // de elementos candidatos para añadirse a la solución.
          if (maxEdge == costToNodesOutOfSolution[currentNodeToCheck]) {
            indexOfNewNodes.push_back(currentNodeToCheck);
          }
        }
      }
      // Una vez hemos obtenido la nueva distancia máxima, comprobamos si esta al
      // añadirse mejoraría la solución.
      if (((maxEdge + currentDistance) / (solutionArray.size() + 1)) > md) {
        currentDistance = maxEdge + currentDistance;
        solutionArray.push_back(indexOfNewNodes[std::rand() % indexOfNewNodes.size()]);
      }
    } while (numberOfElementsOnSolution != solutionArray.size());
  }
  return std::pair<double, std::vector<int>> (currentDistance, solutionArray);
}

std::pair<double, std::vector<int>> MultiStartAlgorithm::constructAnxious(double currentDistance, std::vector<int> solutionArray) {
  if (solutionArray.size() < getGraph().getNumberOfNodes()) {
    int numberOfElementsOnSolution = 0;
    // Mientras sigamos incluyendo elementos en las iteraciones seguimos ejecutando el código.
    do {
      // Obtenemos el número de elementos incluidos en la solución y su media.
      numberOfElementsOnSolution = solutionArray.size();
      double md = currentDistance / numberOfElementsOnSolution;
      // Array de elementos que cumplen la distancia de la nueva solución.
      std::vector<int> costToNodesOutOfSolution(getGraph().getNumberOfNodes(), 0);

      // Para cada elemento de la solución actual recorremos sus aristas.
      for (int currentNodeOfCurrentSolution = 0; currentNodeOfCurrentSolution < solutionArray.size(); currentNodeOfCurrentSolution++) {
        std::vector<Edge> edgesToCheck = getGraph().getNode(solutionArray[currentNodeOfCurrentSolution]).getEdges();
        for (int currentEdge = 0; currentEdge < edgesToCheck.size(); currentEdge++) {
          // Comprobamos que el elemento al que va la arista no esté ya en la solución
          if ((std::find(solutionArray.begin(), solutionArray.end(), edgesToCheck[currentEdge].getDestination()) == solutionArray.end())) {
            costToNodesOutOfSolution[edgesToCheck[currentEdge].getDestination()] += edgesToCheck[currentEdge].getCost();
          }
        }
      }
      for (int currentNodeToCheck = 0; currentNodeToCheck < costToNodesOutOfSolution.size(); currentNodeToCheck++) {
        if (std::find(solutionArray.begin(), solutionArray.end(), currentNodeToCheck) == solutionArray.end()) {
          // Comprobamos si hay una solución mejor que la actual. 
          // Si la hay directamente la añadimos a la solución y rompemos el for
          // ya que al ser ansioso nos quedaremos con la primera solución que mejore
          // nuestro resultado
          if (((costToNodesOutOfSolution[currentNodeToCheck] + currentDistance) / (solutionArray.size() + 1)) > md) {
            currentDistance = costToNodesOutOfSolution[currentNodeToCheck] + currentDistance;
            solutionArray.push_back(currentNodeToCheck);
            break;
          }
        }
      }
    } while (numberOfElementsOnSolution != solutionArray.size());
  }
  return std::pair<double, std::vector<int>> (currentDistance, solutionArray);
}