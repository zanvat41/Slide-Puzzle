import sys
import queue
import time
import math

class puzzleNode:
    def __init__(self, cur, prev, path, n, h):
        self.cur = cur
        self.prev = prev
        self.path = path
        self.n = n
        self.h = h

    # len(path) is g(state), while h_?(cur, n) is h(state)
    # cost f(state) = g(state) + h (state)
    def __eq__(self, other):
        if h == 2:
            return len(self.path) + h_misplaced(self.cur, self.n) == len(other.path) + h_misplaced(other.cur, other.n)
        else:
            return len(self.path) + h_distance(self.cur, self.n) == len(other.path) + h_distance(other.cur, other.n)

    def __ne__(self, other):
        if h == 2:
            return len(self.path) + h_misplaced(self.cur, self.n) != len(other.path) + h_misplaced(other.cur, other.n)
        else:
            return len(self.path) + h_distance(self.cur, self.n) != len(other.path) + h_distance(other.cur, other.n)

    def __lt__(self, other):
        if h == 2:
            return len(self.path) + h_misplaced(self.cur, self.n) < len(other.path) + h_misplaced(other.cur, other.n)
        else:
            return len(self.path) + h_distance(self.cur, self.n) < len(other.path) + h_distance(other.cur, other.n)

    def __gt__(self, other):
        if h == 2:
            return len(self.path) + h_misplaced(self.cur, self.n) > len(other.path) + h_misplaced(other.cur, other.n)
        else:
            return len(self.path) + h_distance(self.cur, self.n) > len(other.path) + h_distance(other.cur, other.n)

    def __le__(self, other):
        if h == 2:
            return len(self.path) + h_misplaced(self.cur, self.n) <= len(other.path) + h_misplaced(other.cur, other.n)
        else:
            return len(self.path) + h_distance(self.cur, self.n) <= len(other.path) + h_distance(other.cur, other.n)

    def __ge__(self, other):
        if h == 2:
            return len(self.path) + h_misplaced(self.cur, self.n) >= len(other.path) + h_misplaced(other.cur, other.n)
        else:
            return len(self.path) + h_distance(self.cur, self.n) >= len(other.path) + h_distance(other.cur, other.n)


# moves = UP, RIGHT, DOWN, LEFT
moves = [[-1, 0], [0, 1], [1, 0], [0, -1]]

# record of states explored
stateExp = 0

# The functions below are copied from puzzleGenerator.py
def isPositionLegal(board, x, y):
    n = len(board)
    return ((x >= 0) and (x < n) and (y >= 0) and (y < n))


def nextPos(x,y, move):
    nextX = x + move[0]
    nextY = y + move[1]

    return nextX, nextY


def possibleMoves(board):

    global moves
    x, y = findGap(board)

    res = []
    for mv in moves:
        x2, y2 = nextPos(x, y, mv)
        if isPositionLegal(board, x2, y2):
            res.append(mv)

    return res


# This function was modified so that it returns a board/state
# after move was performed on board provided
def moveGap(board, move, n):

    x, y = findGap(board)
    x2, y2 = nextPos(x, y, move)

    clone = []
    for i in range(n):
        clone.append([])
        for j in range(n):
            clone[i].append(board[i][j])

    tmp = clone[x][y]
    clone[x][y] = clone[x2][y2]
    clone[x2][y2] = tmp
    return clone


def findGap(board):
    for i in range(len(board)):
        for j in range(len(board[i])):
            if board[i][j] == 0:
                return i,j
    return -1, -1

# The functions above are copied from puzzleGenerator.py


# Admissible heuristics 1: number of misplaced tiles
def h_misplaced(board, n):
    result = 0
    for i in range(n):
        for j in range(n):
            if board[i][j] != n * i + j + 1 and board[i][j] != 0:
                result += 1
    return result


# Admissible heuristics 2: sum of distances of the tiles to their end positions
def h_distance(board, n):
    result = 0
    for i in range(n):
        for j in range(n):
            if board[i][j] != 0:
                # Calculate the end position of board[i][j]
                j1 = int((board[i][j] - 1) % n)
                i1 = int((board[i][j] - 1)/n)

                result += abs(i - i1) + abs(j - j1)
    return result


# Graph-search version of A* algorithm
# Modified from the psuedocode in lecture slide
def A_solver(board, goal, n, h):
    global stateExp
    explored = []
    # Sort frontier in terms of path cost.
    frontier = queue.PriorityQueue()
    frontier.put(puzzleNode(board, None, [], n, h))
    while not frontier.empty():
        # the top of the queue.
        current = frontier.get()
        # goal test when node expands
        if current.cur == goal:
            return current.path
        if not current.cur in explored:
            explored.append(current.cur)
            stateExp += 1
            actions = possibleMoves(current.cur)
            for action in actions:
                new = moveGap(current.cur, action, n)
                new_path = []
                for item in current.path:
                    new_path.append(item)
                new_path.append(action)
                new_node = puzzleNode(new, current.cur, new_path, n, h)
                frontier.put(new_node)


# IDA* algorithm
# Modified from the psuedocode on Wikipedia: https://en.wikipedia.org/wiki/Iterative_deepening_A*
def IDA_solver(board, goal, n, h):
    if h == 2:
        bound = h_misplaced(board, n)
    else:
        bound = h_distance(board, n)
    path = []
    path.append(board)
    while True:
        t = IDA_search(path, 0, n, h, bound, goal)
        if t == "FOUND":
            return path, bound
        if t == math.inf:
            return [], math.inf
        bound = t


def IDA_search(path, g, n, h, bound, goal):
    node = path[len(path) - 1]
    if h == 2:
        f = g + h_misplaced(node, n)
    else:
        f = g + h_distance(board, n)

    if f > bound:
        return f

    if node == goal:
        return "FOUND"

    min = math.inf
    actions = possibleMoves(node)
    successors = []
    for action in actions:
        succ = moveGap(node, action, n)
        successors.append(succ)
    for succ in successors:
        if not succ in path:
            path.append(succ)
            global stateExp
            stateExp += 1
            t = IDA_search(path, g + 1, n, h, bound, goal)
            if t == "FOUND":
                return "FOUND"
            if t < min:
                min = t
            path.pop()

    return min


# A function that convert a list of board states to a list of moves
def state2move(l):
    if len(l) < 2:
        return []
    result = []
    x, y = findGap(l[0])
    for i in range(1, len(l)):
        x1, y1 = findGap(l[i])
        move = [x1 - x, y1 - y]
        result.append(move)
        x = x1
        y = y1
    return result


if __name__ == '__main__':

    a = 0
    n = 0
    h = 0
    in_file = ''
    out_file = ''

    if len(sys.argv) == 6:
        a = int(sys.argv[1])
        n = int(sys.argv[2])
        h = int(sys.argv[3])
        in_file = open(sys.argv[4], 'r')
        out_file = open(sys.argv[5], 'w')
    else:
        print('Wrong number of arguments. Usage:\npython puzzleSolver.py <#Algorithm> <N> <H> <INPUT_FILE_PATH><OUTPUT_FILE_PATH>')
        sys.exit()

    if a!= 1 and a != 2:
        print('Illegal argument: <#Algorithm> should be 1 or 2')
        sys.exit()

    if n!= 3 and n != 4:
        print('Illegal argument: <N> should be 3 or 4')
        sys.exit()

    if h!= 1 and h != 2:
        print('Illegal argument: <H> should be 1 or 2')
        sys.exit()

    # Create goal state board
    goal = []
    for i in range(n):
        goal.append([])
        for j in range(n):
            if (n * i + j + 1) == n * n:
                goal[i].append(0)
            else:
                goal[i].append(n * i + j + 1)

    # Read the input puzzle
    board = []
    i = 0
    for line in in_file:
        if i < n:
            line1 = line.split(',')
            board.append([])
            for j in range(n):
                if line1[j] == '' or line1[j] == '\n':
                    board[i].append(0)
                else:
                    board[i].append(int(line1[j]))
            i += 1

    print("start: " + str(board))
    print("goal " + str(goal))

    # Run solver and get runtime
    startTime = time.time()

    if a == 1:
        solution = A_solver(board, goal, n, h)
    else:
        solution, bound= IDA_solver(board, goal, n, h)
        print(bound)

    print('The Algorithm took {0} second !'.format(time.time() - startTime))
    if(a==1):
        print('depth:'+str(len(solution)))
    else:
        print('depth:' + str(len(solution) - 1))
    global stateExp
    print('states:'+str(stateExp))

    # Convert list of states to list of moves
    if a == 2:
        solution = state2move(solution)

    print("solution: " + str(solution))

    # Write to output file
    for i in range(len(solution)):
        item = solution[i]
        if item == [-1, 0]:
            out_file.write("U")
        elif item == [1, 0]:
            out_file.write("D")
        elif item == [0, 1]:
            out_file.write("R")
        elif item == [0, -1]:
            out_file.write("L")
        if i < len(solution) - 1:
            out_file.write(",")

    out_file.write("\n")

    in_file.close()
    out_file.close()