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
    path_sub_update_delta = 0.2    #co ile sekund aktualizowac podsciezke?
    path_sub_length = 8    #dlugosc podsciezki w celu optymalizacji
    
    def __init__(self):
        print("PathPlanner object created")
        
        self.obstacle_map = None
        self.path = None
        self.path_sub_index = 0 #punkt na glownej sciezce, ktory ma zostac osiagniety przez podsciezke
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
        frame = cv2.inRange(frame, 50, 255)
        #frame = cv2.dilate(frame, None, iterations=1)
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
        start = (int(round(self.ball_pos_x.value * PathPlanner.obstacle_map_size)), int(round(self.ball_pos_y.value * PathPlanner.obstacle_map_size)))
        end = (int(round(self.target_pos_x.value * PathPlanner.obstacle_map_size)), int(round(self.target_pos_y.value * PathPlanner.obstacle_map_size)))
        self.path = PathPlanner.a_star(self, start, end)
        self.path_sub_index = min(len(self.path)-1, PathPlanner.path_sub_length)
        
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
                
                if nx >= 0 and nx < PathPlanner.obstacle_map_size and ny >= 0 and ny < PathPlanner.obstacle_map_size and self.obstacle_map[nx, ny] == 0:
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
        
    #aktualizuje podsciezke przy uzyciu algorytmu A*
    def UpdateSubPath(self):
        if self.path == None: return None
        
        start = (int(round(self.ball_pos_x.value * PathPlanner.obstacle_map_size)), int(round(self.ball_pos_y.value * PathPlanner.obstacle_map_size)))
        end = self.path[self.path_sub_index]
        subpath = PathPlanner.a_star(self, start, end)
        
        #sprawdzanie, czy docelowy podpunkt zostal osiagniety wiec trzeba wyznaczyc kolejny
        if len(subpath) <= 4:
            self.path_sub_index = min(len(self.path)-1, self.path_sub_index + PathPlanner.path_sub_length)
        
        index = min(len(subpath)-1, 2)
        index2 = min(len(subpath)-1, 1)
        vec = MM.normalized((subpath[index][0] + subpath[index2][0])*0.5 - start[0], (subpath[index][1] + subpath[index2][1])*0.5 - start[1])
        self.path_x.value = vec[1] * 0.08 + float(start[1]) / PathPlanner.obstacle_map_size
        self.path_y.value = vec[0] * 0.08 + float(start[0]) / PathPlanner.obstacle_map_size
            
        frame = copy.copy(self.obstacle_map)
        #frame = np.uint8(frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        for p in subpath:
            if p[0] >= 0 and p[0] < PathPlanner.obstacle_map_size and p[1] >= 0 and p[1] < PathPlanner.obstacle_map_size:
                frame[p[0], p[1]] = [255, 255, 0]
        if start[0] >= 0 and start[0] < PathPlanner.obstacle_map_size and start[1] >= 0 and start[1] < PathPlanner.obstacle_map_size:
            frame[start[0], start[1]] = [0, 255, 0]
            
        frame = cv2.resize(frame, (200, 200), interpolation=cv2.INTER_NEAREST)
        
        cv2.imshow("PathPlanner frame", frame)
        key = cv2.waitKey(1) & 0xFF
        #print("Path = " + str(path))