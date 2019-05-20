import cv2
import numpy as np
import MathModule as MM
import time
from multiprocessing import Process, RawValue
from collections import deque
import copy

#program odpowiadajacy za planiwanie sciezki kulki
class PathPlanner:
    
    obstacle_map_size = 40    #rozmiar mapy przeszkod
    obstacle_map_update_delta = 4    #co ile sekund odswiezana ma byc mapa przeszkod?
    path_sub_update_delta = 0.3    #co ile sekund aktualizowac podsciezke?
    
    def __init__(self):
        print("PathPlanner object created")
        
        self.obstacle_map = None
        self.path = None
        self_path_last_index = 0
        self.proximity_map = np.zeros((PathPlanner.obstacle_map_size, PathPlanner.obstacle_map_size)) #tablica 2D z kosztem bliskosci wykrytych przeszkod
        
        self.ball_pos_x = RawValue('f', 0.5)
        self.ball_pos_y = RawValue('f', 0.5)
        self.target_pos_x = RawValue('f', 0.25)
        self.target_pos_y = RawValue('f', 0.25)
        self.path_x = RawValue('f', 0.5)
        self.path_y = RawValue('f', 0.5)
        
    def setBallPosition(self, pos):
        self.ball_pos_x.value = pos[1]
        self.ball_pos_y.value = pos[0]
        
    def setTargetPosition(self, pos):
        self.target_pos_x.value = pos[1]
        self.target_pos_y.value = pos[0]
        
    def getPathTarget(self):
        return (self.path_x.value, self.path_y.value)
        
    def startProcessing(self, _frame_array):
        print("Starting PathPlanner process")
        self.process = Process(target=PathPlanner.doPlanning, args=(self,_frame_array))
        self.process.daemon = True
        self.process.start()
        
    def stopProcessing(self):
        print("Stopping PathPlanner process")
        self.process.terminate()
        
    def doPlanning(self, _frame_array):
        obstacle_map_update_time = 0.0
        path_sub_update_time = 0.0
        while True:
            if time.perf_counter() - obstacle_map_update_time >= PathPlanner.obstacle_map_update_delta:
                obstacle_map_update_time = time.perf_counter()
                PathPlanner.updateObstacleMap(self, _frame_array)
                
            if time.perf_counter() - path_sub_update_time >= PathPlanner.path_sub_update_delta:
                path_sub_update_time = time.perf_counter()
                PathPlanner.UpdateSubPath(self)
        
    #aktualizuje bitmape przeszkod
    def updateObstacleMap(self, _frame_array):
        frame = np.frombuffer(_frame_array, dtype=np.int32)
        frame = np.clip(frame, 0, 255).astype('uint8').reshape((PathPlanner.obstacle_map_size, PathPlanner.obstacle_map_size))
        #cv2.imshow("Map", frame)
        frame = cv2.inRange(frame, 100, 255)
        #kernel = np.ones((2,2), np.uint8)
        #frame = cv2.dilate(frame, kernel, iterations=1)
        self.obstacle_map = frame
        
        #aktualizacja mapy bliskosci przeszkod
        self.proximity_map.fill(0)
        size = PathPlanner.obstacle_map_size - 1
        sides = ((1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1))
        for x in range(1, size):
            for y in range(1, size):
                if frame[x, y] > 0:
                    for side in sides:
                        self.proximity_map[x + side[0], y + side[1]] += 1
        
        #np.clip(self.proximity_map, 0, 1, self.proximity_map)
        self.proximity_map *= 0#5000
        
        #aktualizacja glownej sciezki
        start = (round(self.ball_pos_x.value * PathPlanner.obstacle_map_size), round(self.ball_pos_y.value * PathPlanner.obstacle_map_size))
        end = (round(self.target_pos_x.value * PathPlanner.obstacle_map_size), round(self.target_pos_y.value * PathPlanner.obstacle_map_size))
        self.path = PathPlanner.a_star(self, start, end)
        self.path_last_index = len(self.path)-1
        
    #aktualizuje podsciezke przy uzyciu algorytmu A*
    def UpdateSubPath(self):
        if self.path == None: return None
        
        ball_pos = (self.ball_pos_x.value, self.ball_pos_y.value)
        path = self.path
        start = (round(ball_pos[0] * PathPlanner.obstacle_map_size), round(ball_pos[1] * PathPlanner.obstacle_map_size))
        end = path[self.path_last_index]
        
        #wyszukiwanie binarne najdlaszego punktu na sciezce, do ktorego da sie dojsc w linii prostej
        x = 0
        y = self.path_last_index
        center = 0
        index = 0
        while x <= y:
            center = (x + y) // 2
            if not PathPlanner.Raycast(self, start, path[center]):
                index = center
                x = center + 1
            else: y = center - 1
        
        end = (end[0] / PathPlanner.obstacle_map_size, end[1] / PathPlanner.obstacle_map_size)
        dist = 0.13 * MM.clamp(4 * MM.magnitude(ball_pos[0] - end[0], ball_pos[1] - end[1]), 0.4, 1)
        #print(str(MM.magnitude(ball_pos[0] - self.target_pos_x.value, ball_pos[1] - self.target_pos_y.value)))
        
        vec2go = MM.normalized(path[index][0] - start[0], path[index][1]- start[1])    #wektor docelowego ruchu kulki
        mag = MM.magnitude(0.5 - ball_pos[0], 0.5 - ball_pos[1])    #odleglosc kulki od srodka plyty
        vec2center = ((0.5 - ball_pos[0]) / mag, (0.5 - ball_pos[1]) / mag)    #wektor z pozycji kulki do srodka plyty
        edgeReluctance = 0.012 / (0.6 - min(mag, 0.5))
        print(edgeReluctance)
        
        self.path_x.value = vec2go[1] * dist + ball_pos[1] + vec2center[1] * edgeReluctance
        self.path_y.value = vec2go[0] * dist + ball_pos[0] + vec2center[0] * edgeReluctance
            
        frame = copy.copy(self.obstacle_map)
        #frame = np.uint8(frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        
        #DEBUG
        #for p in path:
        #    if PathPlanner.isPointWithinMap(self, p):
        #        frame[p[0], p[1]] = [255, 255, 0]
            
        PathPlanner.PaintRay(self, start, path[index], frame)
        frame = cv2.resize(frame, (200, 200), interpolation=cv2.INTER_NEAREST)
        
        cv2.imshow("PathPlanner frame", frame)
        key = cv2.waitKey(1) & 0xFF
        
    #sprawdza, czy punkt wewnatrz mapy przeszkod
    def isPointWithinMap(self, point):
        size = self.obstacle_map_size
        return point[0] >= 0 and point[0] < size and point[1] >= 0 and point[1] < size
        
    #algorytm A* wyznaczajacy sciezke z punktu A do B
    def a_star(self, A, B):
        start = B
        end = A
        movement = ((1, 0), (-1, 0), (0, 1), (0, -1))
        #movement = ((1, 0), (-1, 0), (0, 1), (0, -1), (-1, -1), (-1, 1), (1, 1), (1, -1))
        
        que = MM.PriorityQueue()
        que.push(start, 0)
        
        visited_from = {}
        cost = {}
        
        visited_from[start] = None
        visited_from[end] = None
        cost[start] = 0
        
        #timeStart = time.perf_counter()
        while not que.empty():
            v = que.pop()
            if v == end: break
            
            new_cost = cost[v] + 1
            for move in movement:
                nx = v[0] + move[0]
                ny = v[1] + move[1]
                
                if PathPlanner.isPointWithinMap(self, (nx, ny)) and self.obstacle_map[nx, ny] == 0:
                    u = (nx, ny)
                    if u not in cost or new_cost < cost[u]:
                        cost[u] = new_cost
                        center = PathPlanner.obstacle_map_size // 2
                        que.push(u, new_cost + MM.sqrMagnitude(v[0] - u[0], v[1] - u[1]) + self.proximity_map[u[0], u[1]] + int(MM.sqrMagnitude(center - u[0], center - u[1])))
                        visited_from[u] = v
        
        path = []
        if visited_from[end] != None:
            v = end
            while v != start:
                path.append(v)
                v = visited_from[v]
            path.append(start)
        else:
            time.sleep(0.05)
            path.append(end)
        
        return path
    
    #sprawdza, czy promien przecina pole, na ktorym znajduje sie przeszkoda
    def Raycast(self, origin, end):
        obstacle_map = self.obstacle_map
        if not PathPlanner.isPointWithinMap(self, origin) or not PathPlanner.isPointWithinMap(self, end): return False    #jesli punkt startowy jest poza mapa
        if origin == end: return obstacle_map[origin[0], origin[1]]    #jesli promien jest punktem
        
        vec = (end[0] - origin[0], end[1] - origin[1])
        flipped = False    #czy wspolrzedne w ukladzie sa zamienione miejscami? (x; y) -> (y; x)
        if abs(vec[1]) > abs(vec[0]):
            #jesli nachylenie wektora jest wieksze niz 45 stopni
            #uklad wspolrzednych 'obracany jest' o 90 stopni
            vec = (vec[1], vec[0])
            origin = (origin[1], origin[0])
            end = (end[1], end[0])
            flipped = True
        
        dir = vec[1]/vec[0] #wspolczynnik kierunkowy promienia
        offset = origin[1] - dir * origin[0]    #skladnik 'b' w funkcji y = dir*x + b; przechodzi ona przez 'origin'
        
        #znaleznienie najbardziej lewego i prawego punktu promienia
        if origin[0] >= end[0]:
            left = end[0]
            right = origin[0]
        else:
            left = origin[0]
            right = end[0]
            
        #przejscie po wszystkich punktach mapy przeszkod nalezacych do promienia i sprawdzenie, czy ktorys z nich jest przeszkada
        if not flipped:
            for x in range(left, right+1):
                y = round(dir * x + offset)
                #print("Checked (" + str(x) + ", " + str(y) + ")") 
                if obstacle_map[x, y] > 0:
                    return True
        else:
            for x in range(left, right+1):
                y = round(dir * x + offset)
                #print("Checked (" + str(y) + ", " + str(x) + ")") 
                if obstacle_map[y, x] > 0:
                    return True
                
        return False
    
    #DEBUG
    def PaintRay(self, origin, end, frame):
        obstacle_map = self.obstacle_map
        if not PathPlanner.isPointWithinMap(self, origin) or not PathPlanner.isPointWithinMap(self, end): return    #jesli punkt startowy jest poza mapa
        if origin == end:
            frame[origin[0], origin[1]] = [0, 255, 0]
            return
            
        
        vec = (end[0] - origin[0], end[1] - origin[1])
        flipped = False    #czy wspolrzedne w ukladzie sa zamienione miejscami? (x; y) -> (y; x)
        if abs(vec[1]) > abs(vec[0]):
            #jesli nachylenie wektora jest wieksze niz 45 stopni
            #uklad wspolrzednych 'obracany jest' o 90 stopni
            vec = (vec[1], vec[0])
            origin = (origin[1], origin[0])
            end = (end[1], end[0])
            flipped = True
        
        dir = vec[1]/vec[0] #wspolczynnik kierunkowy promienia
        offset = origin[1] - dir * origin[0]    #skladnik 'b' w funkcji y = dir*x + b; przechodzi ona przez 'origin'
        
        #znaleznienie najbardziej lewego i prawego punktu promienia
        if origin[0] >= end[0]:
            left = end[0]
            right = origin[0]
        else:
            left = origin[0]
            right = end[0]
            
        #przejscie po wszystkich punktach mapy przeszkod nalezacych do promienia i sprawdzenie, czy ktorys z nich jest przeszkada
        if not flipped:
            for x in range(left, right+1):
                y = round(dir * x + offset)
                frame[x, y] = [0, 255, 0]
        else:
            for x in range(left, right+1):
                y = round(dir * x + offset)
                frame[y, x] = [0, 255, 0]