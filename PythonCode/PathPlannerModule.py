import cv2
import numpy as np
import time

#program odpowiadajacy za planiwanie sciezki kulki
class PathPlanner:
    
    obstacle_map_size = 50
    
    def __init__(self):
        print("PathPlanner object created")
        
        self.frame_array = None #klatka z kamery, na podstawie ktorej maja byc wykrywane przeszkody
        self.ball_pos = None
        self.obstacle_map = None
        
    def updateObstacleMap(self):
        if self.frame_array is None: return
        
        frame = np.frombuffer(self.frame_array, dtype=np.int32)
        frame = np.clip(frame, 0, 255).astype('uint8').reshape((PathPlanner.obstacle_map_size, PathPlanner.obstacle_map_size))
        frame = cv2.inRange(frame, 70, 255)
        self.obstacle_map = frame
        
        
        frame = cv2.resize(frame, (200, 200))
        cv2.imshow("PathPlanner frame", frame)
        key = cv2.waitKey(1) & 0xFF
        
    def setFrameArray(self, _frame_array):
        self.frame_array = _frame_array

        
        