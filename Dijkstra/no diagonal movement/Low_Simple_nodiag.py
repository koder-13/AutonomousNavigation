# IMAGE Task_1_Low_Simple.png
# Dijkstra with ONLY straight movement NO diagonal movement allowed

import time
import numpy as np
import cv2 as cv

__all__ = [cv]

# Start time
begin = time.time()

# declare openList and closedList as global variables for storing the nodes that have been processed
openList = []
closedList = []

# Reading the image, then upscaling to 1000x1000 pixels
img = cv.imread('/home/ramona/Desktop/IIT KGP/AGV/Software/Task 1/Task_1_Low_Simple.png', 1)
cv.imshow('orig_image', img)
img = cv.resize(img, (1000, 1000), interpolation=cv.INTER_AREA)
cv.imshow('image resized after reading', img)
cv.waitKey(0)
cv.destroyAllWindows()


# to find the coordinates of the pixel given the rgb colour value
def find_pixel_coord(b, g, r):
    color = (b, g, r)
    loc = np.argwhere(img == color)
    x = (loc[0][0]).item()
    y = (loc[0][1]).item()
    return x, y


# If the node is an obstacle, return true otherwise false
# Obstacles = rgb(255,255,255)
def is_obstacle(x, y):
    b, g, r = img[x, y]
    if b == 255 and g == 255 and r == 255:
        return True
    else:
        return False


# Defines node as objects of Makenode class
# Attributes are parent, coord, g representing cost to reach the node from the start where cost = travel distance
class MakeNode:

    def __init__(self, parent, coord):
        self.parent = parent
        self.coord = coord
        self.g = 0

    def __eq__(self, other):
        return self.coord == other.coord


# To give nodes in openList a light blue colour
def colorOpen(l):
    for node in l:
        img[node.coord[0]:(node.coord[0] + 10), node.coord[1]:(node.coord[1] + 10)] = (255, 144, 30)


# To give nodes in closedList a yellow colour
def colorClosed(l):
    for node in l:
        img[node.coord[0]:(node.coord[0] + 10), node.coord[1]:(node.coord[1] + 10)] = (0, 215, 255)


# Function to find path implementing Dijkstra algorithm
def dijkstra(startCoord, endCoord):
    # Make start and end nodes
    start = MakeNode(None, startCoord)
    end = MakeNode(None, endCoord)

    # Add start node to openList and make current= start node
    openList.append(start)
    current = MakeNode(start, startCoord)

    # Loop until no nodes in openList(no path)
    while len(openList) > 0:

        # Search lowest gcost node and make it the current node
        current = openList[0]
        j = 0
        for i, minCost in enumerate(openList):
            if minCost.g < current.g:
                current = minCost
                j = i

        # Remove current from openList and add it to closedList
        openList.pop(j)
        closedList.append(current)

        # If we reached the goal node, backtrack the path by going to the parent of the previous node
        if current == end:
            x = current.parent
            path = []
            while x.parent is not None:
                path.append(x.coord)
                x = x.parent
            path = path[::-1]

            # Change rgb values of each node accordingly
            colorOpen(openList)
            colorClosed(closedList)
            for coord in path:
                img[coord[0]:(coord[0] + 10), coord[1]:(coord[1] + 10)] = (112, 25, 25)

            img[start.coord[0]:(start.coord[0] + 10), start.coord[1]:(start.coord[1] + 10)] = (113, 204, 45)
            img[end.coord[0]:(end.coord[0] + 10), end.coord[1]:(end.coord[1] + 10)] = (60, 76, 231)
            return path

        # Check adjacent squares from the steps - diagonal and straight movement
        for step in [(0, -10), (0, 10), (-10, 0), (10, 0)]:

            # Get adjacent squares coordinates
            coord = (current.coord[0] + step[0], current.coord[1] + step[1])

            # Check if they are within range
            if coord[0] >= 1000 or coord[0] < 0 or coord[1] >= 1000 or coord[1] < 0:
                continue

            # Node should be walkable - not obstacle
            if is_obstacle(coord[0], coord[1]):
                continue

            # Go to next node if this one is in closedList
            check = 0
            for closedNode in closedList:
                if coord == closedNode.coord:
                    check = 1
                    break
            if check == 1:
                continue

            child = MakeNode(None, coord)

            # Record the gcost
            child.g = current.g + 10

            # Check if it is in openList - if it is then is this a better path?
            check = 0
            for node in openList:
                if coord == node.coord:
                    check = 1
                    if child.g < node.g:
                        node.g = child.g
                        node.parent = current
            if check == 1:
                continue

            # Add to open list
            child.parent = current
            openList.append(child)


# Find coordinates corresponding to start and goal nodes
startCoord = find_pixel_coord(45, 204, 113)
endCoord = find_pixel_coord(231, 76, 60)

path = dijkstra(startCoord, endCoord)

# Record run time of program
stop = time.time()
timeOfProgram6 = stop - begin
print("Run time of program implementing Dijkstra is:", timeOfProgram6)

cv.imshow('output image', img)
cv.waitKey(0)
cv.destroyAllWindows()
cv.imwrite('/home/ramona/Desktop/IIT KGP/AGV/Software/Task 1/Task_1_Low_Simple_ds_result.png', img)