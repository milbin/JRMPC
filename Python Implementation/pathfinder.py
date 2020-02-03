import pygame as pg
import math
import random



def game(maxMoves):
    pg.init()

    w = 800
    h = 800
    screen = pg.display.set_mode((w, h))
    pg.display.set_caption("JRMPC")
    clock = pg.time.Clock()
    fps = 1
    rows = 10
    columns = 10
    highest = 100

    #euclidian distance
    def dist(x1, y1, x2, y2):
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)

    #distance factoring in grid layout
    def grid_dist(x1, y1, x2, y2):
        first = dist(x1, y1, x2, y2)
        second = dist(x1, y1, (columns - x2), y2)
        third = dist(x1, y1, x2, (rows - y2))
        fourth = dist(x1, y1, (columns - x2), (rows - y2))
        return max(first, second, third, fourth)

    def rightOf(x, y):
        x = x + 1
        # loops back around if passing thru wall
        if x >= grid.columns:
            x = 0
        return (x, y)
    def leftOf(x, y):
        x = x - 1
        # loops back around if passing thru wall
        if x <= 0:
            x = grid.columns - 1
        return (x, y)
    def upOf(x, y):
        y = y - 1
        # loops back around if passing thru wall
        if y <= 0:
            y = grid.rows - 1
        return (x, y)
    def downOf(x, y):
        y = y + 1
        # loops back around if passing thru wall
        if y >= grid.rows:
            y = 0
        return (x, y)

    def getNeighbors(point): # returns neighbors as [right, down, left, up]

        right = rightOf(point[0], point[1])
        down = downOf(point[0], point[1])
        left = leftOf(point[0], point[1])
        up = upOf(point[0], point[1])

        return [right, down, left, up]
    def standardizePoint(xPoint, yPoint):
        if xPoint < 0:
            xPoint = columns + xPoint
        if xPoint > (columns - 1):
            xPoint = xPoint - columns
        if yPoint < 0:
            yPoint = columns + yPoint
        if yPoint > (rows - 1):
            yPoint = yPoint - rows

        newPoint = (xPoint, yPoint)
        return newPoint

    #any spot on the grid
    class Spot:
        def __init__(self, grid, x, y, points):
            self.grid = grid
            self.x = x
            self.y = y
            self.points = points
        def display(self):
            #scale the points to fit on the screen
            x = self.x * (w/grid.rows)
            y = self.y * (h/grid.columns)
            r = (w/grid.rows)/2

            brightness = 255 - 255 * (self.points/highest) #brightness of color values other than red get scaled with more points, spot on grid gets redder with more points
            key = max(grid.densityField.keys()) #finds highest point density on grid
            highestPoints = grid.densityField[key]  #references which spots are in this area

            # highest density points are drawn blue
            if (self.x, self.y) in highestPoints.values():
                pg.draw.circle(screen, (brightness, brightness, 255), (int(x + r), int(y + r)), int(r), 0)
            # everything else is red
            else:
                pg.draw.circle(screen, (255, brightness, brightness), (int(x + r), int(y + r)), int(r), 0)

            #overlays text displaying how many points the spot had
            font = pg.font.Font('freesansbold.ttf', 10)
            text = font.render("{}".format(self.points), True, (0, 0, 0), None)
            textRect = text.get_rect()
            textRect.center = (int(x+r), int(y+r))
            screen.blit(text, textRect)

    # grid that holds all the spots
    class Grid:
        def __init__(self, rows, columns, distribution="random"):
            if distribution == "random":
                self.grid = {(x, y) : Spot(self, x, y, random.randint(0, highest)) for x in range(0, columns) for y in range(0, rows)} #dictionary
            elif distribution == "gradient":
                self.grid = {}
                radius = 9
                center = (random.randint(radius, columns-radius), random.randint(radius, rows-radius))
                for x in range(0, columns):
                    for y in range(0, rows):
                        pointVal = random.randint(0, int(highest/10))
                        if dist(x, y, center[0], center[1]) <= radius:
                            pointVal = int((((radius-dist(x, y, center[0], center[1]))/radius)**2)*highest)
                        self.grid[(x, y)] = Spot(self, x, y, pointVal)

            elif distribution == "line":
                self.grid = {}
                for x in range(0, columns):
                    for y in range(0, rows):
                        if y == 0:
                            self.grid[(x, y)] = Spot(self, x, y, highest)
                        else:
                            self.grid[(x, y)] = Spot(self, x, y, random.randint(0, int(highest/10)))
            self.rows = rows
            self.columns = columns
            self.densityField = {} #groups spots based on how many points are in a given area, key is amount of points in a given area, value is which spots are in that area

        def findDensity(self, pointRange, pointWithin=None):
            self.densityField = {}
            currentDensity = 0
            currentPoints = {}

            """
            if pointWithin:
                x1 = rightOf(pointWithin[0], 0)[0]
                y1 = downOf(0, pointWithin[1])[1]
                for i1 in range(0, pointRange):
                    x1 = leftOf(x1, 0)[0]
                    for j1 in range(0, pointRange):
                        y1 = upOf(0, y1)[1]
                        for i2 in range(0, pointRange):
                            x2 =
            """
            #iterates through grid
            for x1 in range(0, self.columns - pointRange):
                for y1 in range(0, self.rows - pointRange):

                    #iterates through area, with starting point x1 y1
                    for x2 in range(0, pointRange):
                        x = x1+x2
                        for y2 in range(0, pointRange):
                            y = y1+y2

                            currentDensity += self.spot_at(x, y).points # adds point value of current spot to overall point value of area
                            currentPoints[currentDensity] = (x, y) #adds current spot to list of spots within current area

                    #resets variables and adds current area points and spots to overall dictionary
                    self.densityField[currentDensity] = currentPoints
                    currentDensity = 0
                    currentPoints = {}

        #returns spot at a given coordinate
        def spot_at(self, x, y):
            return self.grid[(x, y)]

        #displays spots
        def display(self):
            for s in self.grid.values():
                s.display()

    #fucking robot
    class Robot:
        def __init__(self, grid, x, y, points):
            self.grid = grid #grid robot is a part of
            self.x = x
            self.y = y
            self.points = points #points robot has accumulated
            self.path = []
            self.color = (0, 255, 0)
            self.allPath = {}
            self.totalEnergyCollected = 0

            #right from the getgo steals points from the spot it's sitting on
            self.points += grid.spot_at(self.x, self.y).points
            grid.spot_at(self.x, self.y).points = 0

        # finds coordinate to the right of the robot
        def right(self):
            y = self.y
            x = self.x + 1
            #loops back around if passing thru wall
            if x >= self.grid.columns:
                x = 0
            return (x, y)

        # finds coordinate to the left of the robot
        def left(self):
            y = self.y
            x = self.x - 1
            # loops back around if passing thru wall
            if self.x <= 0:
                x = grid.columns - 1
            return (x, y)

        # finds coordinate to the up of the robot
        def up(self):
            x = self.x
            y = self.y - 1
            # loops back around if passing thru wall
            if self.y <= 0:
                y = grid.rows - 1
            return (x, y)

        # finds coordinate to the down of the robot
        def down(self):
            x = self.x
            y =  self.y + 1
            # loops back around if passing thru wall
            if y >= grid.rows:
                y = 0
            return (x, y)

        #moves the robot in set direction (for testing)
        def move(self, movementType, iterations=1):
            if movementType == "Astar":
                #finds highest density area
                targetField = self.grid.densityField[max(self.grid.densityField.keys())]

                #x and y coordinates of spots within that area
                xvals = [spot[0] for spot in targetField.values()]
                yvals = [spot[1] for spot in targetField.values()]

                #if the target field has been reached, switch over to nearest neighbor
                if (self.x, self.y) in targetField.values():
                    self.path = []
                    self.x, self.y = self.nearest_neighbor(iterations)
                    if self.grid.spot_at(self.x, self.y).points == 0:
                        self.grid.findDensity(3)
                #if not, continue through path
                else:
                    target = ((max(xvals)+min(xvals))/2, (max(yvals)+min(yvals))/2)
                    if not self.path:
                        self.path = self.Astar(target, self.x, self.y, [], [])
                    else:
                        self.x, self.y = self.path[0]
                        self.path.remove(self.path[0])
            elif movementType == "nearestNeighbor":
                if len(self.path) == 0:
                    self.path = self.nearest_neighbor(iterations)
                self.x, self.y = self.path[0][0], self.path[0][1]
                self.path.remove(self.path[0])

            elif movementType == "dijkstra":
                if len(self.path) == 0:
                    self.path = self.Dijkstra(iterations)
                self.x, self.y = self.path[0][0], self.path[0][1]
                self.path.remove(self.path[0])
            #self.x, self.y = self.up()
            self.points += grid.spot_at(self.x, self.y).points
            grid.spot_at(self.x, self.y).points = 0

        # implementation of A* algorithm that takes into account high spots along the path
        def Astar(self, target, x, y, traversed, path):

            traversed.append((x, y))  # current spot has been reached and is now out of bounds for future positions
            neighbors = [rightOf(x, y), leftOf(x, y), upOf(x, y), downOf(x, y)]  # possible future positions
            dict = {}  # will store cost of path as key, and position as value

            for neighbor in neighbors:
                # if target has been reached, return the path that took it there
                if neighbor == target:
                    # print(path)
                    return path

                # only calculate cost of position if it has not been traversed before
                if neighbor not in traversed:
                    Gcost = dist(neighbor[0], neighbor[1], x, y)  # dist away from current spot
                    Hcost = dist(neighbor[0], neighbor[1], target[0], target[1])  # dist away from target
                    Pcost = (Gcost + Hcost) * (self.grid.spot_at(neighbor[0], neighbor[1]).points/highest) * 0.25 #factor of how high a point is in that spot
                    Fcost = Gcost + Hcost - Pcost
                    dict[Fcost] = neighbor  # store relationship between position and cost of moving there

            # proceed to order future positions in order of least costly to most
            ordered = list(dict.keys())
            ordered.sort()
            for i in range(0, len(ordered)):
                ordered[i] = dict[ordered[i]]  # final list just contains all possible positions, but is ordered

            # go through these positions in order, repeating the process until completion
            for neighbor in ordered:
                path.append((neighbor[0], neighbor[1]))  # current spot is now part of path
                return self.Astar(target, neighbor[0], neighbor[1], traversed, path)

        def nearest_neighbor(self, iterations, Xrange=None, Yrange=None):
            currentIteration = 1
            x, y = self.x, self.y
            paths = {0:[(x, y)]}
            pathID = 0
            pathEnergies = {0:0}

            #iterate as many times as specified
            while currentIteration <= iterations:
                newPaths = {} #new paths charted out
                #iterate through paths already charted out

                for value in paths.keys():
                    path = paths[value] #current path being charted out
                    neighbors = [leftOf(path[-1][0], path[-1][1]), rightOf(path[-1][0], path[-1][1]), upOf(path[-1][0], path[-1][1]), downOf(path[-1][0], path[-1][1])] #neighbors of latest position in path

                    #iterate through each neighbor
                    for neighbor in neighbors:
                        if Xrange and Yrange:
                            if neighbor[0] not in Xrange or neighbor[1] not in Yrange:
                                continue
                        #only review this neighbor if it goes over new ground
                        if (neighbor[0], neighbor[1]) not in path:
                            newVal = pathEnergies[value] + self.grid.spot_at(neighbor[0], neighbor[1]).points #more points are added as it goes over new ground
                            # this new position is added to the overall path
                            newPath = path.copy()
                            newPath.append((neighbor[0], neighbor[1]))
                            newPaths[pathID] = newPath
                            pathEnergies[pathID] = newVal
                            pathID += 1

                #update variables
                paths = newPaths.copy()
                currentIteration += 1

            self.allPath = paths.copy()

            bestPathID = 0
            for (key,value) in pathEnergies.items():
                if max(pathEnergies.values()) == value:
                    bestPathID = key
            bestPath = paths[bestPathID]
            self.totalEnergyCollected += pathEnergies[bestPathID]
            bestPath.remove(bestPath[0])
            return bestPath

        def Dijkstra(self, scanningRadius):  # returns best path
            # find points on the perimeter of the scanning radius circle --------------------------------------------------
            pointsOnPerimeter = []
            for i in range(scanningRadius):
                pointsOnPerimeter.append(standardizePoint(self.x + i, self.y + scanningRadius - i))  # first quadrant arc from top waypoint to right waypoint
                pointsOnPerimeter.append(standardizePoint(self.x - i, self.y + scanningRadius - i))  # second quadrant arc from top waypoint to left waypoint
                pointsOnPerimeter.append(standardizePoint(self.x - scanningRadius + i, self.y - i))  # third quadrant arc from top waypoint to left waypoint
                pointsOnPerimeter.append(standardizePoint(self.x + scanningRadius - i, self.y - i))  # fourth quadrant arc from right waypoint to bottom waypoint

            # create a new list of all points within the robots scanning radius --------------------------------------------
            grid = []
            pointsList = []
            for xCord in range(scanningRadius * 2 + 1):
                for yCord in range(scanningRadius * 2 + 1):
                    xPoint = self.x - scanningRadius + xCord
                    yPoint = self.y - scanningRadius + yCord
                    # this will take the coordinate system wrap around the board
                    point = standardizePoint(xPoint, yPoint)
                    grid.append(point)
                    pointsList.append(self.grid.spot_at(point[0], point[1]).points)

            # initialization stuff -----------------------------------------------------------------------------------------
            startingPoint = (self.x, self.y)
            distanceFromStart = {}
            previousVertex = {}
            visited = []
            unVisited = grid.copy()

            # set all distances to infinity and initlize previous distance dict with default keys
            for point in unVisited:
                distanceFromStart[point] = 1000000000000  # 1 billion, number must be very large because distance from starting point to all other points is unkown at this point
                previousVertex[point] = (-1, -1)  # default value
            distanceFromStart[startingPoint] = 0

            # find the highest point since we will be subtracting the value of all other points from this in order to find the cost to move to a square
            highestPoint = max(pointsList)

            # point visit loop --------------------------------------------------------------------------------------------
            while len(unVisited) > 0:
                currentPoint = (-1, -1)  # placeholder
                # find min value in distanceFromStart dict
                minValue = 10000000000000  # 10 billion
                for (key, value) in distanceFromStart.items():
                    if (value < minValue) and (key not in visited):
                        currentPoint = key
                # iterate through neighbors
                neighbors = getNeighbors(currentPoint)  # returns a list of points in [(x,y), (x,y), (x,y), (x,y)] format
                for neighbor in neighbors:
                    if neighbor in grid:  # check that neighbor is not out of bounds
                        distance = distanceFromStart[currentPoint] + (highestPoint - self.grid.spot_at(neighbor[0], neighbor[1]).points)
                        if distance < distanceFromStart[neighbor]:
                            distanceFromStart[neighbor] = distance
                            previousVertex[neighbor] = currentPoint
                visited.append(currentPoint)
                unVisited.remove(currentPoint)

            # while loop has exited meaning that the shortest path has been found -----------------------------------------
            shortestPathWeight = 1000000000000  # 1 billion
            bestPointToTravelTo = (-1, -1)
            for point in pointsOnPerimeter:
                if distanceFromStart[point] < shortestPathWeight:
                    bestPointToTravelTo = point
                    shortestPathWeight = distanceFromStart[point]
            bestPath = [bestPointToTravelTo]

            # work backwards from best point to find optimal path
            point = bestPointToTravelTo
            for unused in range(scanningRadius):
                bestPath.append(previousVertex[point])
                point = previousVertex[point]
            bestPath.pop()  # removes last item from list since that is the current point we are at
            bestPath = list(reversed(bestPath))
            print(distanceFromStart)
            self.allPath = {0:visited}
            return bestPath





        #draws the robot on screen
        def display(self):
            for path in self.allPath.values():
                for coord in path:
                    if coord in self.path:
                        # scales point to fit onto screen
                        x = coord[0] * (w / grid.columns)
                        y = coord[1] * (h / grid.rows) - 20
                        r = (w / grid.rows) / 2

                        pg.draw.circle(screen, (100, 100, 255), (int(x + r), int(y + r)), int(r / 4), 0)
                    else:
                        # scales point to fit onto screen
                        x = coord[0] * (w / grid.columns)
                        y = coord[1] * (h / grid.rows) - 20
                        r = (w / grid.rows) / 2

                        pg.draw.circle(screen, (0, 0, 0), (int(x + r), int(y + r)), int(r / 4), 0)

            #scales point to fit onto screen
            x = self.x * (w / grid.columns)
            y = self.y * (h / grid.rows)
            r = (w / grid.rows) / 2

            pg.draw.circle(screen, self.color, (int(x) + int(r), int(y) + int(r)), int(r), 0)

    grid = Grid(rows, columns, distribution="random")
    grid.findDensity(5)
    robot1 = Robot(grid, 5, 5, 0)
    robot2 = Robot(grid, 8, 8, 0)
    robot2.color = (127, 0, 127)
    robot3 = Robot(grid, 0, 0, 0)
    robot3.color = (127, 127, 0)

    running = True
    pause = False
    moveNumber = 0
    while running:
        #cancels loop if user exits screen
        for event in pg.event.get():
            if event.type == pg.QUIT:
                return [robot1.points, robot2.points, robot3.points]

            if event.type == pg.KEYDOWN:
                if event.key == pg.K_SPACE:
                    if pause:
                        pause = False
                    else:
                        pause = True

        screen.fill((0, 0, 0)) #screen black

        grid.display()  # displays spots
        if not pause:
            robot1.move("nearestNeighbor", iterations=1) # moves bot
            robot2.move("dijkstra", iterations=4) # moves bot
            #robot3.move("nearestNeighbor", iterations=6) # moves bot
            if moveNumber >= maxMoves:
                return [robot1.points, robot2.points, robot3.points]

            moveNumber += 1

        robot1.display()  # displays bot
        robot2.display()  # displays bot
        #robot3.display()  # displays bot

        #updates frame
        pg.display.update()
        clock.tick(fps)

numberOfTests = 100  # set to zero to run no tests
maxMoves = 100
numberOfGamesToRun = 1
robot1TotalPoints = 0
robot2TotalPoints = 0
robot3TotalPoints = 0

if numberOfTests > 0:
    numberOfGamesToRun = numberOfTests
for testNumber in range(numberOfGamesToRun):
    try:
        returnString = game(maxMoves)
        print('Game Number ' + str(testNumber) + ": Robot 1: {}, Robot 2: {}, Robot 3: {}".format(returnString[0], returnString[1], returnString[2]))
        robot1TotalPoints += returnString[0]
        robot2TotalPoints += returnString[1]
        robot3TotalPoints += returnString[2]
        print("TOTALS: Robot 1: {}, Robot 2: {}, Robot 3: {}".format(robot1TotalPoints, robot2TotalPoints,
                                                                     robot3TotalPoints))
    except:
        pass

print("TOTALS: Robot 1: {}, Robot 2: {}, Robot 3: {}".format(robot1TotalPoints, robot2TotalPoints, robot3TotalPoints))
'''
ISSUES:
 - Dijkstra only goes to squares on the perimeter
 - some values within the grid are still 1Bn




'''



