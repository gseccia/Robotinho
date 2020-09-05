#!/usr/bin/env python2
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import numpy as np
from tf import transformations
from math import cos,sin

class MinPriorityQueue():

    def __init__(self,lenght):
        self.heap = [None for i in range(lenght)]
        self.elements = {}
        self.last_element = -1

    def push(self,priority,element):
        if element[1] in self.elements:
            self.update(priority,self.elements[element[1]])
        else:
            self.last_element += 1
            self.heap[self.last_element] = [priority,element,self.last_element]
            self.elements[element[1]] = self.last_element
            self.__upheap(self.heap[self.last_element])

    def empty(self):
        return self.last_element == -1

    def pop(self):
        self.__swap(self.heap[0],self.heap[self.last_element])
        self.elements[self.heap[0][1][1]] = 0

        node = self.heap[self.last_element]
        self.last_element -= 1
        self.__downheap(self.heap[0])
        del self.elements[node[1][1]]

        return node

    def update(self,priority,node_index):
        self.heap[node_index][0] = priority
        parent = self.__getParent(self.heap[node_index])


        if parent is not None and priority < parent[0]:
            self.__upheap(self.heap[node_index])
        else:
            self.__downheap(self.heap[node_index])

    def __swap(self,node1,node2):

        index1 = node1[2]
        index2 = node2[2]

        self.heap[index1] = node2
        self.heap[index1][2] = index1

        self.heap[index2] = node1
        self.heap[index2][2] = index2

        
    def __downheap(self,node):
        childs = self.__getChilds(node)
        if childs is not None:
            child = childs[0]
            if len(childs) > 1 and childs[0] > childs[1]:
                child = childs[1]
            if node[0] > child[0]:
                
                self.__swap(self.heap[node[2]],self.heap[child[2]])
                

                self.elements[node[1][1]] = node[2]
                self.elements[child[1][1]] = child[2]

                self.__downheap(self.heap[node[2]])
        
    
    def __upheap(self,node):
        parent = self.__getParent(node)
        if parent is not None:
            if node[0] < parent[0]:
                self.__swap(self.heap[node[2]],self.heap[parent[2]])
                
                self.elements[node[1][1]] = node[2]
                self.elements[parent[1][1]] = parent[2]
                
                self.__upheap(self.heap[node[2]])
        

    
    def __getChilds(self,node):
        index = node[2]
        if self.last_element >= 2*index + 2:
            return [self.heap[2*index + 1],self.heap[2*index + 2]]
        elif self.last_element >= 2*index + 1:
            return [self.heap[2*index + 1]]
        else:
            return None
    
    def __getParent(self,node):
        index = node[2]
        return self.heap[(index - 1)//2] if (index - 1)//2 >= 0 else None

class Tile:
    TILE_BUSY = 0
    TILE_FREE = 1

    def __init__(self,x,y,tileProbability):
        self.x = x
        self.y = y

        self.status = Tile.TILE_FREE
        self.ballProbabilityPresence = tileProbability
    
    def isFree(self):
        return self.status ==  Tile.TILE_FREE
    
    def setBusy(self):
        self.status = Tile.TILE_BUSY
    
    def setFree(self):
        self.status = Tile.TILE_FREE
    
    def setProbability(self,prob):
        self.ballProbabilityPresence = prob
    
    def getProbability(self):
        return self.ballProbabilityPresence
    
    def __repr__(self):
        return str(self.x)+","+str(self.y)+" -> "+ ("FREE" if self.isFree() else "BUSY")

class Map:

    def __init__(self):
        self.verteces = {}
        self.edges = {}
        self.modified = False

        self.vertical_dims = [-24,25]
        self.horizontal_dims = [-12,13]

        self.maxProbabilityTiles = MinPriorityQueue( ((self.vertical_dims[1] - self.vertical_dims[0]) // 2) *((self.horizontal_dims[1] - self.horizontal_dims[0]) // 2) )

        for i in range(self.vertical_dims[0],self.vertical_dims[1]):
            for j in range(self.horizontal_dims[0],self.horizontal_dims[1]):
                self.verteces[str(i)+"|"+str(j)] = Tile(i,j, 1.0 / float( (self.vertical_dims[1] - self.vertical_dims[0])*(self.horizontal_dims[1] - self.horizontal_dims[0]) ))
                if self.vertical_dims[0]//2 <= i <= self.vertical_dims[1]//2 and self.horizontal_dims[0]//2 <= i <= self.horizontal_dims[1]//2:
                    self.maxProbabilityTiles.push(-self.verteces[str(i)+"|"+str(j)].getProbability(),str(i)+"|"+str(j))

        for k,tile in self.verteces.items():
            availableTile = []
            for i in range(-1,2):
                for j in range(-1,2):
                    if i != 0 or j != 0:
                        available_x = tile.x + i
                        available_y = tile.y + j

                        # Scoraggio traiettorie a zig zag nel breve termine
                        w = 2 if i != 0 and j != 0 else 1

                        if self.vertical_dims[0] <= available_x < self.vertical_dims[1] and self.horizontal_dims[0] <= available_y < self.horizontal_dims[1]:
                            availableTile.append((str(available_x)+"|"+str(available_y),w))

            self.edges[tile] = availableTile
            
            self.current_x = 0
            self.current_y = 0
            self.current_theta = 0

            self.image_pub = rospy.Publisher("/map_image",Image, queue_size = 1)
            self.bridge = CvBridge()


    def setTileBusy(self,x,y,convert = True):
        if convert:
            tx,ty = self.getTileCoords(x,y)
        else:
            tx,ty = x,y
        
        if str(tx)+"|"+str(ty) in self.verteces and self.verteces[str(tx)+"|"+str(ty)].isFree():
            self.verteces[str(tx)+"|"+str(ty)].setBusy()
            self.modified = True

    def setTileFree(self,x,y,convert = True):
        if convert:
            tx,ty = self.getTileCoords(x,y)
        else:
            tx,ty = x,y
        
        if str(tx)+"|"+str(ty) in self.verteces and not self.verteces[str(tx)+"|"+str(ty)].isFree():
            self.verteces[str(tx)+"|"+str(ty)].setFree()
            self.modified = True  
    
    def isModified(self):
        return self.modified
    
    def getTileCoords(self,x,y):
        scale_x = (self.vertical_dims[1] - self.vertical_dims[0] - 1) // 12
        scale_y = (self.horizontal_dims[1] - self.horizontal_dims[0] - 1) // 6

        x = int(x * scale_x) if x < self.vertical_dims[1] else self.vertical_dims[1] - 1
        y = int(y * scale_y) if y < self.horizontal_dims[1] else self.horizontal_dims[1] - 1
        
        return x,y

    def resetMap(self):
        for v in self.verteces:
            self.verteces[v].setFree()

    def getAdiacentVerteces(self,vertex):
        adList = []
        for v,w in self.edges[vertex]:
            if self.verteces[v].isFree():
                adList.append((w,v))
        return adList
    
    def __str__(self):
        out_str = "          "
        for j in range(self.horizontal_dims[0],self.horizontal_dims[1]):
            out_str += ("| {0:4.2f}".format(-j / 4.0) if -j < 0 else "|  {0:4.2f}".format(-j / 4.0))
        out_str += "\n"

        for i in range(self.vertical_dims[0],self.vertical_dims[1]):
            out_str += ("| {0:4.3f} ".format(-i / 4.0) + " " if -i < 0 else "|  {0:4.3f} ".format(-i / 4.0) + " ")
            for j in range(self.horizontal_dims[0],self.horizontal_dims[1]):
                out_str += "|" + (" FREE " if self.verteces[str(-i)+"|"+str(-j)].isFree() else " BUSY ")
            out_str += "|\n"
        return out_str
    
    def bestPath(self,startVertex,endVertex,verbose = False):
        visited = {}
        path = {}

        distance = {}

        for vertex in self.verteces:
            distance[vertex] = float("inf")

        pq = MinPriorityQueue(len(distance))
        pq.push(0,(None,startVertex))

        distance[startVertex] = 0
        path[startVertex] = (0,None)

        while not pq.empty():
            currentDist, element, _ = pq.pop()

            previousVertex,currentVertex = element

            if currentVertex == endVertex:
                break

            for w,vertex in self.getAdiacentVerteces(self.verteces[currentVertex]):
                if currentDist + w < distance[vertex]:
                    distance[vertex] = currentDist + w
                    
                    pq.push(distance[vertex],(currentVertex,vertex))

                    path[vertex] = (distance[vertex],currentVertex)

            if verbose:
                out_str = "   EST    "
                for j in range(self.horizontal_dims[0],self.horizontal_dims[1]):
                    out_str += ("| {0:4.2f}".format(-j / 4.0) if -j < 0 else "|  {0:4.2f}".format(-j / 4.0))
                out_str += "\n"

                for i in range(self.vertical_dims[0],self.vertical_dims[1]):
                    out_str += ("| {0:4.3f} ".format(-i / 4.0) + " " if -i < 0 else "|  {0:4.3f} ".format(-i / 4.0) + " ")
                    for j in range(self.horizontal_dims[0],self.horizontal_dims[1]):
                        if distance[str(-i)+"|"+str(-j)] == float("inf"):
                            out_str += "| +INF "
                        else:
                            out_str += "|" + ("{0:6d}".format(distance[str(-i)+"|"+str(-j)]) if self.verteces[str(-i)+"|"+str(-j)].isFree() else " BUSY ")
                    out_str += "|\n"
                print(out_str)
                print("CURRENT EVALUATION: ",currentVertex,endVertex,currentVertex==endVertex)

                time.sleep(1)

        finalpath = []
        if endVertex not in path:
            print("STUCK: ",startVertex,endVertex)
            # print(str(self))
            self.resetMap()

            return None
        else:
            dist,vertex = path[endVertex]
            finalpath.append(self.verteces[endVertex])
            while vertex is not None:
                finalpath.append(self.verteces[vertex])
                dist,vertex = path[vertex]
            finalpath.reverse()

        return finalpath

    def getBestTilePath(self,startPose,targetPose):
        sx,sy = self.getTileCoords(startPose.x,startPose.y)
        tx,ty = self.getTileCoords(targetPose.x,targetPose.y)

        print("Start Pose: ",startPose.x,startPose.y)
        print("End Pose: ",targetPose.x,targetPose.y)

        print("Start: ",sx,sy)
        print("End: ",tx,ty)
        path = self.bestPath(str(sx)+"|"+str(sy),str(tx)+"|"+str(ty))

        i = 0
        while i < 10 and path is None:
            path = self.bestPath(str(sx)+"|"+str(sy),str(tx)+"|"+str(ty))
            i += 1
        if path is None:
            path = []

        scale_x = (self.vertical_dims[1] - self.vertical_dims[0] -1) // 12
        scale_y = (self.horizontal_dims[1] - self.horizontal_dims[0] -1) // 6

        coords = []
        for point in path:
            x = float(point.x) / scale_x
            y = float(point.y) / scale_y
            coords.append((x,y))
        
        self.modified = False
        return coords
    
    def updateProbableBallPosition(self,x,y,probability,convert = True):
        if convert:
            tx,ty = self.getTileCoords(x,y)
        else:
            tx,ty = x,y

        self.maxProbabilityTiles.push(-probability,str(tx)+"|"+str(ty))

    def getMostProbableBallPath(self,startPose):
        if self.maxProbabilityTiles.empty():
            return None
        else:
            while not self.maxProbabilityTiles.empty():
                prob,tile,_ = self.maxProbabilityTiles.pop()

                sx,sy = self.getTileCoords(startPose.x,startPose.y)

                if self.verteces[tile].isFree():
                    path = self.bestPath(str(sx)+"|"+str(sy),tile,True)

                    scale_x = (self.vertical_dims[1] - self.vertical_dims[0] - 1) // 12
                    scale_y = (self.horizontal_dims[1] - self.horizontal_dims[0] -1) // 6

                    coords = []
                    for point in path:
                        x = float(point.x) / scale_x
                        y = float(point.y) / scale_y
                        coords.append((x,y))
                    
                    self.modified = False
                    return coords

def positionUpdate(msg):
    global current_x, current_y, current_theta

    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    
    roll,pitch,theta = transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
    current_theta = theta

def mapUpdate(msg):
    occupancy_grid = np.reshape(msg.data,(3,5))

    busy_x = current_x + 0.15 * cos(current_theta)
    busy_y = current_y + 0.15 * sin(current_theta)

    max_value = 20

    if min(max(0, max_value-int(np.sum(occupancy_grid[2][1:4])*max_value)), max_value) < 15:
        mappa.setTileBusy(busy_x, busy_y)
    else:
        mappa.setTileFree(busy_x, busy_y)

if __name__ == "__main__":
    rospy.init_node("Map_NODE")
    mappa = Map()
    current_x = 0
    current_y = 0
    current_theta = 0

    positionSubscriber = rospy.Subscriber("/robot1/odom",Odometry,positionUpdate)
    occupancy_grid = rospy.Subscriber("/occupancy_grid",Float32MultiArray,mapUpdate)

    w = mappa.horizontal_dims[1] - mappa.horizontal_dims[0] - 1
    h = mappa.vertical_dims[1] - mappa.vertical_dims[0] - 1
    cv_image =  np.zeros((h,w,3),dtype=np.uint8)

    while not rospy.is_shutdown():
        mappa.current_x = current_x
        mappa.current_y = current_y
        mappa.current_theta = current_theta
        
        x,y = mappa.getTileCoords(mappa.current_x,mappa.current_y)

        for i in range(mappa.vertical_dims[0],mappa.vertical_dims[1]):
            for j in range(mappa.horizontal_dims[0],mappa.horizontal_dims[1]):
                if mappa.verteces[str(-i)+"|"+str(-j)].isFree():
                    cv_image[i + mappa.vertical_dims[0]][j + mappa.horizontal_dims[0]][:] = (255,255,255)
                else:
                    cv_image[i + mappa.vertical_dims[0]][j + mappa.horizontal_dims[0]][:] = (0,0,0)
        
        cv_image[-x + mappa.vertical_dims[0]][-y + mappa.horizontal_dims[0]][:] = (0,0,255)

        ros_image = mappa.bridge.cv2_to_imgmsg(cv_image)
        mappa.image_pub.publish(ros_image)    


