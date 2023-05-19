import numpy as np
from collections import deque
from queue import PriorityQueue

goalState = [0, 0, 0]
initState = np.array([3, 3, 1])
generalMaxDepth = 20


def getValidNextStates(currentState):
    """ this function return list of possible next states according to the current state that was received """

    nextStates = []

    possibleMoves = np.array([[2, 0, 1], [1, 1, 1], [1, 0, 1], [0, 2, 1], [0, 1, 1]])

    for move in possibleMoves:

        if currentState[2] == 1:
            newState = currentState - move

        else:
            newState = currentState + move

        if is_valid(newState):
            nextStates.append(newState.tolist())

    return nextStates


def is_valid(newState):
    """ this function checks if the next new state is a valid state according to the problem conditions """

    if newState[0] < 0 or newState[1] < 0:
        return False

    if newState[0] > 3 or newState[1] > 3:
        return False

    if newState[1] > newState[0] > 0:
        return False

    if 3 - newState[1] > 3 - newState[0] > 0:
        return False

    return True


def BFS():
    """ Breadth-first search Algorithm """

    # using as queue to store frontier nodes
    frontier = deque()

    explored = set()

    frontier.append((initState.tolist(), [initState.tolist()]))

    while frontier:

        node, nodePath = frontier.popleft()

        explored.add(tuple(node))

        # get next legal states for the current state
        nextLegalStates = getValidNextStates(node)

        for childState in nextLegalStates:

            # calculate child path
            childPath = nodePath + [childState]

            # the goal test
            if childState == goalState:
                return len(explored), childPath

            if tuple(childState) not in explored and childState not in [item[0] for item in frontier]:
                frontier.append((childState, childPath))

    return None


def IDDFS():
    """ Iterative deepening depth-first search Algorithm """

    totalExplored = 0

    for depth in range(generalMaxDepth):

        exploredNumber, path, indication = depthLimitedSearch(depth)

        totalExplored += exploredNumber

        # solution was found
        if indication == "successes":
            return totalExplored, path

    return None


def depthLimitedSearch(maxDepth):

    depth = 0

    # using as stack to store frontier nodes
    frontier = deque()

    explored = set()

    frontier.append((initState.tolist(), [initState.tolist()], depth))

    while frontier:

        node, nodePath, currentDepth = frontier.pop()

        explored.add(tuple(node))

        # the goal test
        if node == goalState:
            print(f"the solution found in depth {currentDepth}")

            return len(explored), nodePath, "successes"

        # we reach to the depth limit for some specific state
        if currentDepth >= maxDepth:
            continue

        # get next legal states for the current state
        nextLegalStates = getValidNextStates(np.array(node))

        for childState in nextLegalStates:

            # calculate child path
            childPath = nodePath + [childState]

            frontier.append((childState, childPath, currentDepth + 1))

    # indicate that we reach to the max depth, the algorithm will finish with failure
    return len(explored), [], "failure"


def heuristicFunc(currentState):

    # h(n) = (cannibals in left bank + missionaries in left side) / 2
    return (currentState[0] + currentState[1]) / 2


def GBFS():
    """ Greedy best-first search Algorithm """

    # using as priority queue to store frontier nodes
    frontier = PriorityQueue()

    explored = set()

    frontier.put((heuristicFunc(initState), initState.tolist(), [initState.tolist()]))

    while frontier:
        _, node, nodePath = frontier.get()

        explored.add(tuple(node))

        # get next legal states for the current state
        nextLegalStates = getValidNextStates(node)

        for childState in nextLegalStates:

            # calculate child path
            childPath = nodePath + [childState]

            # the goal test
            if childState == goalState:
                return len(explored), childPath

            # when child state not explored yet, and he is not already in the frontier, we add him to the frontier
            if tuple(childState) not in explored and childState not in [item[1] for item in frontier.queue]:
                frontier.put((heuristicFunc(childState), childState, childPath))

    return None


def AStar():
    """  A* Algorithm """

    frontier = PriorityQueue()

    explored = set()

    frontier.put((heuristicFunc(initState), initState.tolist(), [initState.tolist()]))

    while not frontier.empty():
        _, node, nodePath = frontier.get()

        explored.add(tuple(node))

        # get next legal states for the current state
        nextLegalStates = getValidNextStates(node)

        for childState in nextLegalStates:

            # calculate child path
            childPath = nodePath + [childState]

            # the goal test
            if childState == goalState:
                return len(explored), childPath

            if tuple(childState) not in explored:
                childCost = heuristicFunc(childState) + len(nodePath)

                update = False

                # Create a temporary queue
                tempQueue = PriorityQueue()

                # check if child state is already in the frontier, replace the priority of him if she is better
                for currentPriority, element, elementPath in frontier.queue:
                    if childState == element and childCost < currentPriority:
                        update = True

                    tempQueue.put(childCost, element, elementPath)

                if update:
                    frontier = tempQueue

                else:
                    frontier.put((childCost, childState, childPath))

    return None

if __name__ == '__main__':
    exploredStates, path = BFS()
    print(f"BFS:\nsolution path: {path}\nnumber of explored states: {exploredStates}\n")

    print("IDDFS:")
    exploredStates, path = IDDFS()
    print(f"solution path: {path}\nnumber of explored states: {exploredStates}\n")

    exploredStates, path = GBFS()
    print(f"GBFS:\nsolution path: {path}\nnumber of explored states: {exploredStates}\n")

    exploredStates, path = AStar()
    print(f"AStar:\nsolution path: {path}\nnumber of explored states: {exploredStates}\n")
