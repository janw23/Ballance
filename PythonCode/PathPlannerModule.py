import cv2
import numpy as np
import MathModule as MM
import time

#program odpowiadajacy za planiwanie sciezki kulki
class PathPlanner:
    
    #rozmiar mapy przeszkod
    obstacle_map_size = 50
    
    def __init__(self):
        print("PathPlanner object created")
        
        self.frame_array = None #klatka z kamery, na podstawie ktorej maja byc wykrywane przeszkody
        self.obstacle_map = None
        self.path = None
        
        self.ball_pos = (0.5, 0.5)
        self.target_pos = (0.1, 0.1)
        
    #aktualizuje bitmape przeszkod
    def updateObstacleMap(self):
        if self.frame_array is None: return
        
        frame = np.frombuffer(self.frame_array, dtype=np.int32)
        frame = np.clip(frame, 0, 255).astype('uint8').reshape((PathPlanner.obstacle_map_size, PathPlanner.obstacle_map_size))
        frame = cv2.inRange(frame, 70, 255)
        self.obstacle_map = frame
        
        frame = cv2.resize(frame, (200, 200))
        cv2.imshow("PathPlanner frame", frame)
        key = cv2.waitKey(1) & 0xFF
        
    #ustawia RawArray z ktorego ma byc pobierana klatka z kamery
    def setFrameArray(self, _frame_array):
        self.frame_array = _frame_array
        
    #aktualizuje sciezke przy uzyciu algorytmu A*
    def UpdatePath(self):
        start = tuple(int(round(x * (PathPlanner.obstacle_map_size -1))) for x in self.ball_pos)
        end = tuple(int(round(x * (PathPlanner.obstacle_map_size - 1))) for x in self.target_pos)
        movement = ((1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1))
        
        que = MM.PriorityQueue()
        que.push(start, 0)
        
        visited_from = {}
        cost = {}
        
        visited_from[start] = None
        cost[start] = 0
        
        while not que.empty():
            v = que.pop()
            if v == end: break
            
            for move in movement:
                nx = start[0] + move[0]
                ny = start[1] + move[1]
                
                if nx >= 0 and nx < PathPlanner.obstacle_map_size and ny >= 0 and ny < PathPlanner.obstacle_map_size and self.obstacle_map[nx, ny] != 0:
                    new_cost = cost[v] + 1
                    u = (nx, ny)
                    if u not in cost or new_cost < cost[u]:
                        cost[u] = new_cost
                        que.push(u, new_cost + MM.sqrMagnitude(v[0] - u[0], v[1] - u[1]))
                        visited_from[u] = v
                        
        path = []
        if visited_from[end] != None:
            v = end
            while v != start:
                path.append(v)
                v = visited_from[v]
            path.append(start)
            
        self.path = path