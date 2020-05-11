#include "../include/modification.hpp"

ModificactionAlgorithm::ModificactionAlgorithm(Graph workingGraph) : 
  MaxMeanAlgorithm(workingGraph) {}

Solution ModificactionAlgorithm::resolve() {
  // Obtenemos todas las parejas de nodos que tienen distancia positiva entre sí.
  std::vector<std::pair<double, std::pair<int, int>>> posibleInitialSolution = preProcess();
    std::pair<double, std::pair<int, int>> initialSolution = posibleInitialSolution[std::rand() % posibleInitialSolution.size()];
    // Escogemos aleatoriamente una de estas parejas.

    std::vector<int> solutionArray;
    double currentDistance = initialSolution.first;
    // Incluimos los dos nodos iniciales de la arista en la solución.
    solutionArray.push_back(initialSolution.second.first);
    solutionArray.push_back(initialSolution.second.second);
    // Tamaño de LRC
    int kElementsOfLRC = 2;
    // Hacemos una construcción de tipo GRASP.
    std::pair<double, std::vector<int>> solution = construct(solutionArray, currentDistance, kElementsOfLRC);
    // Búsqueda local para nuestra solución actual.
    std::pair<double, std::vector<int>> solution2 = postProcess(solution.first, solution.second, true);
    double bestValue = solution2.first / solution2.second.size();
    std::vector<int> bestVector = solution2.second;
    // A partir de aquí haremos en bucle haciendo el siguiente proceso: Buscaremos una solución
    // En la vecindad a partir de nuestra primera solución. A esta solución vecina le aplicaremos búsqueda local
    // y si el resultado es mejor haremos que esta búsqueda local sea nuestra nueva solución principal.
    // En caso de que no sea mejor, buscaremos en una vecindad mayor. Este proceso se repetirá hasta que
    // encontremos un ópitmo local que sea la mejor solución para 3 vecindades (Es un valor umbral de vecindad
    // escogido). Esta solución obtenida sería nuestro óptimo local a comprar con la solución final.
    int currentIteration = 0;
    while (currentIteration < 3) {
      std::pair<double, std::vector<int>> solution3 = shake(bestVector, bestValue, currentIteration);
      std::pair<double, std::vector<int>> solution4 = postProcess(solution3.first, solution3.second, true);
      if (solution4.first / solution4.second.size() > bestValue) {
        bestValue = solution4.first / solution4.second.size();
        bestVector = solution4.second;
        currentIteration = 0;
      } else {
        currentIteration++;
      }
    }
    _bestSolution.setSolutionNodes(bestVector);
    _bestSolution.setSolutionScore(bestValue);
  return _bestSolution;
}

std::vector<std::pair<double, std::pair<int, int>>> ModificactionAlgorithm::preProcess() {
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

std::pair<double, std::vector<int>> ModificactionAlgorithm::construct(std::vector<int> solutionArray, double currentDistance, int kElementsOfLRC) {
  int numberOfElementsOnSolution = 0;
  int iteration = 0;
  // Número aleatorio de iteraciones que se repetirá el proceso
  int numberRandomOfIterations = std::rand() % getGraph().getNumberOfNodes();
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

    // Una vez hemos calculado la distancia a todos los nodos nos quedamos con los k-elementos mejores.
    std::vector<std::pair<int, double>> bestNewNodes(kElementsOfLRC);
    for (int i = 0; i < bestNewNodes.size(); i++) {
      bestNewNodes[i].second = -500;
    }
    for (int currentNodeToCheck = 0; currentNodeToCheck < costToNodesOutOfSolution.size(); currentNodeToCheck++) {
      std::pair<int,double> substitutionElement(currentNodeToCheck, costToNodesOutOfSolution[currentNodeToCheck]);
      if ((std::find(solutionArray.begin(), solutionArray.end(), currentNodeToCheck) == solutionArray.end())) {
        for (int currentTopNode = bestNewNodes.size() - 1; currentTopNode >= 0 ; currentTopNode--){
          if (bestNewNodes[currentTopNode].second < substitutionElement.second) {
            std::pair<int, double> auxiliarElement = substitutionElement;
            substitutionElement.first = bestNewNodes[currentTopNode].first;
            substitutionElement.second = bestNewNodes[currentTopNode].second;
            bestNewNodes[currentTopNode] = auxiliarElement;
          }
        }
      }
    }

    // // Filtramos de las mejores soluciones aquellas que cumplen la factibilidad.
    // std::vector<std::pair<int, double>> indexOfNewNodes;
    // for (int currentTopNode = 0; currentTopNode < bestNewNodes.size(); currentTopNode++) {
    //   if (((bestNewNodes[currentTopNode].second + currentDistance) / (solutionArray.size() + 1)) > md) {
    //     indexOfNewNodes.push_back(std::pair(bestNewNodes[currentTopNode].first, bestNewNodes[currentTopNode].second));
    //   }
    // }

    // Añadimos un elemento aleatorio del LRC a la solución.
    if (bestNewNodes.size() > 0) {
      int randIndex = (std::rand() % bestNewNodes.size());
      currentDistance += bestNewNodes[randIndex].second;
      solutionArray.push_back(bestNewNodes[randIndex].first);
    }
    iteration++;
  } while (iteration < numberRandomOfIterations);
  return std::pair<double, std::vector<int>> (currentDistance, solutionArray);
}

std::pair<double, std::vector<int>> ModificactionAlgorithm::postProcess(double currentDistance, std::vector<int> solutionArray, bool greedyOrAnxious) {
  std::pair <double, std::vector<int>> solution;
  // GreedyOrAnxious = true es hacer greedy
  if (greedyOrAnxious) {
    solution = constructGreedy(currentDistance, solutionArray);
  } else {
    solution = constructAnxious(currentDistance, solutionArray);
  }
  return solution;
}

std::pair<double, std::vector<int>> ModificactionAlgorithm::constructGreedy(double currentDistance, std::vector<int> solutionArray) {
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

std::pair<double, std::vector<int>> ModificactionAlgorithm::constructAnxious(double currentDistance, std::vector<int> solutionArray) {
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

std::pair<double, std::vector<int>> ModificactionAlgorithm::shake(std::vector<int> solutionArray, double currentDistance, int neighborhoodDistance) {
  std::pair<double, std::vector<int>> neighbor;
  bool addOrSubstractElemenet = std::rand() % 2;
  for (int currentShake = 0; currentShake < neighborhoodDistance; currentShake++) {
    // Si está a true añadimos elementos.
    if (addOrSubstractElemenet) {
      std::vector<int> elementsNotInSol;
      // Buscamos lo elementos que no estén ya en la solución.
      for (int currentNode = 0; currentNode < getGraph().getNumberOfNodes(); currentNode++) {
        if ((std::find(solutionArray.begin(), solutionArray.end(), currentNode) == solutionArray.end())) {
          elementsNotInSol.push_back(currentNode);
        }
      }
      // Comprobamos que no se pueda añadir ningún nodo nuevo
      if (elementsNotInSol.size () > 0){
        int elementToAdd = elementsNotInSol[std::rand() % elementsNotInSol.size()];
        double newDistance = 0;
        for (int currentNodeOnSolution = 0; currentNodeOnSolution < solutionArray.size(); currentNodeOnSolution++) {
          newDistance += getGraph().getNode(solutionArray[currentNodeOnSolution]).getCostOfEdge(elementToAdd);
        }
        currentDistance += newDistance;
        solutionArray.push_back(elementToAdd);
      }
    } else {
      if (solutionArray.size() > 0) {
        int randIndex = std::rand() % solutionArray.size();
        int elementToDelete = solutionArray[randIndex];
        double newDistance = 0;
        for (int currentNodeOnSolution = 0; currentNodeOnSolution < solutionArray.size(); currentNodeOnSolution++) {
          newDistance += getGraph().getNode(solutionArray[currentNodeOnSolution]).getCostOfEdge(elementToDelete);
        }
        currentDistance -= newDistance;
        solutionArray.erase(solutionArray.begin() + randIndex);
      }

    }
  }
  neighbor.first = currentDistance;
  neighbor.second = solutionArray;
  return neighbor;
}