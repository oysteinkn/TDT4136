## This code solves all parts - 1, 2 and 3 ##

class Node:
  def __init__(self, t, x, y):
    self.x = x          # X coordinate
    self.y = y          # Y coordinate
    self.f = -1         # f-value
    self.g = 0          # g-cost
    self.t = t          # Type of node (grass, road etc.)
    self.parent = None  # Parent node

costs = {'w':100, 'm':50, 'f':10, 'g':5, 'r':1, '.':1, '#':0, 'A':1, 'B':1}

#Read the map into a matrix of nodes and find a and b
def readfile(f):
  matrix = []
  with open(f) as fileobj:
    y = 0
    for line in fileobj:
      x = 0
      row = []
      for ch in line:
        if ch == "\n": continue
        row.append(Node(ch, x, y))
        if ch == 'A': a = row[x]    # Start node
        elif ch == 'B': b = row[x]  # Goal node
        x = x+1
      matrix.append(row)
      y = y+1
  return (matrix, a, b)

#Find neighboring nodes that are not walls
def get_neighbors(matrix, node):
  mx, my = len(matrix), len(matrix[0])            # Max indexes for a neighbor
  x, y = node.x, node.y
  dirs = [(x, y+1), (x+1, y), (x, y-1), (x-1, y)] # Possible directions - top, right, bottom, left
  possible_neighbors = []
  for d in dirs:                                  # Check if neighbours are on matrix and not a wall
    if 0 <= d[0] < my and 0<= d[1] < mx and costs[matrix[d[1]][d[0]].t] > 0: #Cost 0 implies wall
      possible_neighbors.append(matrix[d[1]][d[0]])
  return possible_neighbors

#Manhattan is used for heuristics
def manhattan(curr, goal):
  return abs(goal.x - curr.x) + abs(goal.y - curr.y)

#Helper function for sorting by first element in tuple
def getKey(item):
  return item[0]

#Calculate path from goal to start
def get_path(a, b):
  path, cost, curr = [], 0, b.parent
  while curr != a:                                  # Trace parents back to a
    path.append((curr.x, curr.y))                   # Append all parents
    cost, curr = cost + costs[curr.t], curr.parent  # Calculate cost so far and set new current
  print("Cost: ", cost) 
  return path

#A* implementation
def astar(matrix, a, b):
  o, c = [], []     #Open and closed sets
  o.append((0, a))  #Add the start node to the open set

  while o:
    curr = o.pop()[1]                     # Get node with highest priority
    if curr in c: continue                # Don't care about closed nodes - continue

    if curr == b:                         # Check if we have reached goal node
      return (get_path(a, curr), o, c)    # Return path, open set and closed set

    neighbors = get_neighbors(matrix, curr)
    for n in neighbors:                   # Iterate neighbors
      g = curr.g + costs[n.t]             # Neighbor g-cost

      if n not in c or g < n.g:           # Update node if new node or shorter path
        n.parent = curr                   # Set parent
        n.g = g                           # Set new g-cost
        n.f = n.g + manhattan(n, b)       # Set new f-cost
        if n not in c:                    
          o.append((n.f, n))              # Add unvisited nodes to open set
        o.sort(reverse=True, key=getKey)  # Sort to keep highest priority on top (helper function used here)
    c.append(curr)                        # Visited node is now closed

#BFS implementation
def bfs(matrix, a, b):
  o, c = [], []             # Open and closed sets - Open treated as queue
  o.append(a)               # Add the start node to the open set

  while o:
    curr = o.pop(0)         # First-in first-out in BFS
    if curr in c: continue  # Don't care about closed nodes

    if curr == b:           # Check if we have reached goal node, b
      return (get_path(a, curr), o, c)  # Return path, open set and closed set

    neighbors = get_neighbors(matrix, curr)
    for n in neighbors:     # Iterate neighbors
      if n not in c:        # Update node if new node
        n.parent = curr     # Set parent
        o.append(n)         # Add unvisited nodes to open set
    c.append(curr)          # Visited node is now closed

#Dijkstra implementation
def dijkstra(matrix, a, b):
  o, c = [], []                     # Open and closed sets
  o.append((0, a))                  # Add the start node to the open set

  while o:
    curr = o.pop()[1]               # Highest priority is always on top of open
    if curr in c: continue          # Don't care about closed nodes
    
    if curr == b:                   # Check if we have reached goal node, b
      return (get_path(a, curr), o, c)  # Return path, open set and closed set

    neighbors = get_neighbors(matrix, curr)
    for n in neighbors:             # Iterate neighbors
      g = curr.g + costs[n.t]       # Neighbor g-cost
      if n not in c or g < n.g:     # Update node if new node or shorter path
        n.parent = curr             # Set parent
        n.g = g                     # Set new g-cost
        if n not in c:          
          o.append((n.g, n))        # Add unvisited to open set
        o.sort(reverse=True, key=getKey) #Sort again with new costs, to keep highest priority on top (helper function used here)
    c.append(curr)              # Visited node is now closed

#Prints matrix with found path
def print_matrix(matrix, path, a, b, o, c, sets):
  if type(o[0]) is tuple:         # Convert closed set to Nodes if it is made of tuples
    for i in range(0, len(o)):
      o[i] = o[i][1]

  for n in path:                  # Set type to O for nodes in path
      matrix[n[1]][n[0]].t = 'O'

  if sets:                        # If open and closed set should be printed
    for row in matrix:
      r = ""
      for col in row:             # Find out what the node should be printed as
        if col == a: r = r + 'A'
        elif col == b: r = r + 'B'
        elif col.t == 'O': r = r + 'O'
        elif col in c: r = r + 'x'
        elif col in o: r = r + '*'
        else: r = r + col.t
      print(r)
  else:
    for row in matrix:
      r = ""
      for col in row:
        r = r + col.t
      print(r)
  print('\n')

algos = {'A*':astar, 'BFS':bfs, 'Dijkstra':dijkstra} #To be able to call functions from strings

#Runs the desired algo on the desired board and prints the result
def search(alg, board, sets=False):             # sets=False by default - dont show open and closed sets in visualization
  print(alg, '-', board[7:16])                  # Print which algorithm on which board
  args = readfile(board)                        # returns: [0]=matrix, [1]=a, [2]=b
  path = algos[alg](args[0], args[1], args[2])  # returns: [0]=path, [1]=open set, [2]=closed set
  print_matrix(args[0], path[0], args[1], args[2], path[1], path[2], sets) # sets=True if open and closed set are shown

###### PART 1 ######
print('###### Part 1 ######')
search('A*', 'boards/board-1-1.txt')
search('A*', 'boards/board-1-2.txt')
search('A*', 'boards/board-1-3.txt')
search('A*', 'boards/board-1-4.txt')
print('\n')

###### PART 2 ######
print('###### Part 2 ######')
search('A*', 'boards/board-2-1.txt')
search('A*', 'boards/board-2-2.txt')
search('A*', 'boards/board-2-3.txt')
search('A*', 'boards/board-2-4.txt')
print('\n')

###### PART 3 ######
print('###### Part 3 ######')
search('A*', 'boards/board-1-3.txt', sets=True)
search('BFS', 'boards/board-1-3.txt', sets=True)
search('Dijkstra', 'boards/board-1-3.txt', sets=True)

search('A*', 'boards/board-2-4.txt', sets=True)
search('BFS', 'boards/board-2-4.txt', sets=True)
search('Dijkstra', 'boards/board-2-4.txt', sets=True)