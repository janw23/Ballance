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
    path_sub_update_delta = 0.1    #co ile sekund aktualizowac podsciezke?
    
    def __init__(self):
        print("PathPlanner object created")
        
        self.obstacle_map = None
        self.path = None
        self_path_last_index = 0
        self.proximity_map = np.zeros((PathPlanner.obstacle_map_size, PathPlanner.obstacle_map_size)) #tablica 2D z kosztem bliskosci wykrytych przeszkod
        
        self.path_position = 0.0   #aktualna pozycja na sciezce
        self.path_speed = 0.2 * PathPlanner.obstacle_map_size    #predkosc przechodzenia sciezki
        
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
        frame = cv2.inRange(frame, 90, 255)
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
        self.proximity_map *= 5000
        
        #aktualizacja glownej sciezki
        start = PathPlanner.FromUnitaryToMapSpace((self.ball_pos_x.value, self.ball_pos_y.value), self.obstacle_map_size)
        end = PathPlanner.FromUnitaryToMapSpace((self.target_pos_x.value, self.target_pos_y.value), self.obstacle_map_size)
        self.path = PathPlanner.a_star(self, start, end)
        
        self.path_last_index = len(self.path)-1
        self.path_position = 0.0
        
    #aktualizuje podsciezke przy uzyciu algorytmu A*
    def UpdateSubPath(self):
        if self.path == None: return None
        
        ball_pos = (self.ball_pos_x.value, self.ball_pos_y.value)
        path = self.path
        
        index = int(self.path_position)
        A = PathPlanner.FromMapToUnitarySpace(path[index])
        
        frame = copy.copy(self.obstacle_map)
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        
        if self.path_last_index > 0:
            B = PathPlanner.FromMapToUnitarySpace(path[index+1])
            dist = MM.distance(A, B)
            mant = self.path_position - index
            
            PathPlanner.PaintRay(self, PathPlanner.FromUnitaryToMapSpace(ball_pos, self.obstacle_map_size), path[index+1], frame)
            if not PathPlanner.Raycast(self, PathPlanner.FromUnitaryToMapSpace(ball_pos, self.obstacle_map_size), path[index+1]):
                print("False")
                self.path_position += self.path_speed * PathPlanner.path_sub_update_delta / (dist * PathPlanner.obstacle_map_size)
            if self.path_position >= self.path_last_index: self.path_position = self.path_last_index - 0.00001
            
            target_y = MM.lerp(A[0], B[0], mant)
            target_x = MM.lerp(A[1], B[1], mant)
        else:
            target_y = A[0]
            target_x = A[1]
            
        #print(target_x)
        #print(target_y)
        #print("")
        
        self.path_x.value = target_x
        self.path_y.value = target_y
            
        
        
        #DEBUG
        for p in path:
            if PathPlanner.isPointWithinMap(self, p):
                frame[p[0], p[1]] = [255, 255, 0]
            
        frame = cv2.resize(frame, (200, 200), interpolation=cv2.INTER_NEAREST)
        
        cv2.imshow("PathPlanner frame", frame)
        key = cv2.waitKey(1) & 0xFF
        
    #zmienia uklad odniesienia z mapy przeszkod na jednostkowy
    def FromMapToUnitarySpace(point):
        return (point[0] / PathPlanner.obstacle_map_size, point[1] / PathPlanner.obstacle_map_size)
    
    #zmienia uklad odniesienia z jednostkowego na mape przeszkod
    def FromUnitaryToMapSpace(point, size):
        return (round(point[0] * size), round(point[1] * size))
        
    #sprawdza, czy punkt wewnatrz mapy przeszkod
    def isPointWithinMap(self, point):
        size = self.obstacle_map_size
        return point[0] >= 0 and point[0] < size and point[1] >= 0 and point[1] < size
        
    #algorytm A* wyznaczajacy sciezke z punktu A do B
    def a_star(self, A, B):
        start = B
        end = A
        #movement = ((1, 0), (-1, 0), (0, 1), (0, -1))
        movement = ((1, 0), (-1, 0), (0, 1), (0, -1), (-1, -1), (-1, 1), (1, 1), (1, -1))
        
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
            i = 0
            for move in movement:
                nx = v[0] + move[0]
                ny = v[1] + move[1]
                
                i += 1
                if i == 5: new_cost += 0.4
                
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