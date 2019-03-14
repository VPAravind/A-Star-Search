from heapq import *
from math import *
import time
'''
Code which implements A* Search with 4 different heuristics and prints out the performance metrics to the console.
'''
class Node:
    '''
    The class which stores information about the co-ordinates and the costs to each node in the maze
    '''
    def __init__(self, x, y, cost = 0, heuristicCost = 0, totalCost = 0):
        '''
        Initialize the x and y coordinate along the costs.
        :param x: x coordinate
        :param y: y coordinate
        :param cost: path cost
        :param heuristicCost: estimated heuristic cost
        :param totalCost: total cost = path cost + heuristic cost
        '''
        self.x = x
        self.y = y
        self.predecessor = None
        self.cost = cost
        self.heuristicCost = heuristicCost
        self.totalCost = totalCost

    def heuristics(self, target, option, constant):
        '''
        Heuristic function to calculate the heuristics for the given node in the maze
        :param target: target node
        :param option: option on which heuristic to choose
        :param constant:
        :return:
        '''
        if option == 1:
            return constant * (abs(self.x - target.x) + abs(self.y - target.y))
        elif option == 2:
            return constant * (sqrt(pow((self.x - target.x),2) + pow((self.y - target.y),2)))
        elif option ==3 :
            return constant * (max(abs(self.x - target.x), abs(self.y - target.y)))
        elif option == 4:
            return constant * ((abs(self.x - target.x)/(abs(self.x) + abs(target.x))) + (abs(self.y - target.y)/(abs(self.y) + abs(target.y))))

class Astar:
    '''
    The Astar class implements the A* algorithm and it's utility functions
    '''
    def __init__(self, rows, cols, option, heuristicConstant):
        '''
        Initializes the parameters needed for the algorithm.
        :param rows: number of rows
        :param cols: number of columns
        :param option: option for heuristic
        :param heuristicConstant:
        '''
        self.heuristic_constant = heuristicConstant
        self.option = option
        self.rows = rows
        self.cols = cols
        self.row = [-1, 0, 0, 1]
        self.col = [0, -1, 1, 0]
        self.visited = set()
        self.nodes = []
        self.to_be_visited = []
        heapify(self.to_be_visited)
        self.target = None
        self.src = None


    # def __hash__(self, Node):
    #     return hash(Node)

    def nodesInMaze(self, src, target):
        '''
        Given a 2D matrix representation of all the nodes in the maze. Sets the source node and target node
        :param src: source node
        :param target: target node
        :return:
        '''
        for i in range(self.rows):
            self.nodes.append([])
            for j in range(self.cols):
                self.nodes[i].append(Node(i,j))
        self.src = src
        self.target = target
        self.nodes[src.x][src.y] = src
        self.nodes[target.x][target.y] = target

    def isValid(self, maze,node):
        '''
        Method to check if a particular node can be visited.
        :param maze: Maze
        :param node: Node that we want to visit
        :return: boolean
        '''
        if maze[node.x][node.y] == 0 and node not in self.visited:
            return True
        else:
            return False



    def getNeighbour(self, node, maze):
        '''
        This method gets all the adjacent nodes of the current node
        :param node: Current node
        :param maze: 2d matrix representation of the Maze
        :return: list of all the adjacent nodes
        '''
        neighbours = []
        for k in range(len(self.row)):
            adj_row = node.x + self.row [k]
            adj_col = node.y + self.col[k]
            if adj_row >=0 and adj_col>=0 and adj_row < self.rows and adj_col < self.cols:
                neighbours.append(self.nodes[adj_row][adj_col])
        return neighbours

    def set_neighbour(self, current, neighbour):
        '''
        Sets the cost values and parents of an adjacent node.
        :param current: Current node
        :param neighbour: adjacent node
        :return:
        '''
        neighbour.cost = current.cost + 10
        neighbour.heuristicCost = neighbour.heuristics(self.target, self.option, self.heuristic_constant)
        neighbour.totalCost = neighbour.cost + neighbour.heuristicCost
        neighbour.predecessor = current

    def display(self):
        '''
        Displays the coordinates of the path
        :return:
        '''
        cell = self.target
        path = [(cell.x, cell.y)]
        while cell.predecessor is not self.src:
            cell = cell.predecessor
            path.append((cell.x, cell.y))

        path.append((self.src.x, self.src.y))
        path.reverse()
        print('Total steps in the path:', len(path))
        return path

    def astar_search(self, maze, src, target):
        '''
        Implements the A* algorithm.
        :param maze: Maze
        :param src: Source Node
        :param target: Target Node
        :return: Boolean
        '''
        count = 1
        steps = 0
        self.nodesInMaze(src,target)
        heappush(self.to_be_visited, (src.totalCost, count, src))

        while len (self.to_be_visited):
            totalCost, tie, current = heappop(self.to_be_visited)
            steps += 1
            # print(steps)
            self.visited.add(current)
            if current is self.target:
                print( self.display())
                print("Visited Nodes: ", steps)
                print("Generated Nodes: ",count)
                return True

            neighbours = self.getNeighbour(current, maze)
            # print(neighbours)
            for neighbour in neighbours:
                count+=1
                # print('visiting', neighbour.x , neighbour.y)

                if self.isValid(maze, neighbour):
                    if neighbour in self.to_be_visited:
                        if neighbour.g > current.g + 10:
                            self.set_neighbour(current, neighbour)
                    else:
                        self.set_neighbour(current, neighbour)
                        heappush(self.to_be_visited, (neighbour.totalCost, count, neighbour))
        return False



def main():
    text_file = 'five.txt'
    opt = input('Do you want to test with your own input file?(y/n) ')
    if opt == 'y':
        print('The input file must be a text file with only binary numbers that are space separated in a m x n matrix format ')
        text_file = input('Enter your text file with the .txt extension: ')
    else:
        print ('Default file: five.txt will be used, make sure it is available')

    list1 = list(map(str.split, open(text_file)))
    maze = [[int(list1[i][j]) for j in range(len(list1[i]))] for i in range(len(list1))]
    bool = True
    while (bool):
        startTime = time.time()
        print(
            '-----------------------------------------------------------------------------------------------------------------------------------------------')
        option = int(input(
            "Select your heuristics 1. Manhattan Heuristics, 2. Euclidian Heuristics, 3. Diagonal Heuristics,4. Canberra Heuristics "))
        print(
            '-----------------------------------------------------------------------------------------------------------------------------------------------')

        rows = len(maze)
        cols = len(maze[0])
        constant = rows * cols
        target = Node(rows - 1, cols - 1)
        src = Node(0, 0)
        astar = Astar(rows, cols, option, constant)
        astar.astar_search(maze, src, target)
        print(" -------- Time taken in %s seconds --------" % (time.time() - startTime))

        choice = input('Do you want to test with another heuristic?(y/n) ')
        if choice.strip() == 'n':
            bool == False
            break


if __name__ == "__main__":
    main()
