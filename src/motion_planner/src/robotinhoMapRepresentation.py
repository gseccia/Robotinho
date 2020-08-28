#!/usr/bin/env python2

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

    def __init__(self,x,y):
        self.x = x
        self.y = y

        self.status = Tile.TILE_FREE
    
    def isFree(self):
        return self.status ==  Tile.TILE_FREE
    
    def setBusy(self):
        self.status =  Tile.TILE_BUSY
    
    def __repr__(self):
        return str(self.x)+","+str(self.y)+" -> "+ ("FREE" if self.isFree() else "BUSY")

class Map:

    def __init__(self):
        self.verteces = {}
        self.edges = {}
        self.modified = False

        for i in range(-12,12):
            for j in range(-6,6):
                self.verteces[str(i)+"|"+str(j)] = (Tile(i,j))

        for k,tile in self.verteces.items():
            availableTile = []
            for i in range(-1,2):
                for j in range(-1,2):
                    if i != 0 or j != 0:
                        available_x = tile.x + i
                        available_y = tile.y + j

                        if -12 <= available_x <= 11 and -6 <= available_y <= 5:
                            availableTile.append(str(available_x)+"|"+str(available_y))

            self.edges[tile] = availableTile

    def setTileBusy(self,x,y):
        tx,ty = self.__getTileCoords(x,y)
        self.verteces[str(tx)+"|"+str(ty)].setBusy()
        self.modified = True
    
    def isModified(self):
        return self.modified
    
    def __getTileCoords(self,x,y):
        tileX = x - int(x) > 0.5
        tileY = y - int(y) > 0.5
        
        return int(x) + tileX,int(y) + tileY
    
    def getAdiacentVerteces(self,vertex):
        adList = []
        for v in self.edges[vertex]:
            if self.verteces[v].isFree():
                adList.append(v)
        return adList

    def __str__(self):
        out_str = "    "
        for j in range(-6,6):
            out_str += "|  {0:2d}  ".format(j)
        out_str = "\n"

        for i in range(-12,12):
            out_str += "| {0:3d} ".format(i) + " "
            for j in range(-6,6):
                out_str += "|" + (" FREE " if self.verteces[str(i)+"|"+str(j)].isFree() else " BUSY ")
            out_str += "|\n"
        return out_str
    
    def bestPath(self,startVertex,endVertex):
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

            for vertex in self.getAdiacentVerteces(self.verteces[currentVertex]):
                if currentDist + 1 < distance[vertex]:
                    distance[vertex] = currentDist + 1
                    
                    pq.push(distance[vertex],(currentVertex,vertex))

                    path[vertex] = (distance[vertex],currentVertex)
        

        finalpath = []
        dist,vertex = path[endVertex]
        finalpath.append(self.verteces[endVertex])
        while vertex is not None:
            finalpath.append(self.verteces[vertex])
            dist,vertex = path[vertex]
        finalpath.reverse()

        return finalpath

    def getBestTilePath(self,startPose,targetPose):
        sx,sy = self.__getTileCoords(startPose.x,startPose.y)
        tx,ty = self.__getTileCoords(targetPose.x,targetPose.y)

        path = self.bestPath(str(sx)+"|"+str(sy),str(tx)+"|"+str(ty))

        coords = []
        for point in path:
            x = point.x // 2 + 0.5*(point.x % 2)
            y = point.y // 2 + 0.5*(point.y % 2)
            coords.append((x,y))
        
        self.modified = False
        return coords
        


if __name__ == "__main__":
    mappa = Map()

    """for i,vertex in enumerate(mappa.verteces):
        print(i,mappa.verteces[vertex])
    for i,vertex in enumerate(mappa.edges):
        print(vertex," : ",mappa.edges[vertex])"""

    mappa.verteces["1|1"].setBusy()
    
    mappa.bestPath("0|0","2|2")



    
    
    


