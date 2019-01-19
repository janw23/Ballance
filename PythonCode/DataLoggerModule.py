#modul do tworzenia i zapisywania log'ow danych

import numpy as np
import pandas as pd
import os

path = os.path.dirname(os.path.abspath(__file__))    #sciezka, w ktorej znajduje sie program

class DataLogger:
    
    def __init__(self):
        print("DataLogger object created")
        
        #zdefiniowanie nazw kolumn
        self.dataColumns = ["timestamp", "ball_pos_x", "ball_pos_y", "ball_vel_x", "ball_vel_y", "KP", "KI", "KD", "error_x", "error_y", "error_prev_x", "error_prev_y", "error_sum_x", "error_sum_y", "derivative_x", "derivative_y", "servo_actual_x", "servo_actual_y", "servo_target_x", "servo_target_y"]
        self.data2write = []    #wszystkie dane wpisow razem do zapisania
        self.currentRecordData = [None] * len(self.dataColumns)   #dane obecnego wpisu
        
    #funkcja dodajoca wartosc 'value' do kolumny 'name' w obecnym wpisie
    def addRecord(self, name, value):
        self.currentRecordData[self.dataColumns.index(name)] = value
        
    #funckja finalizujaca obecny wpis
    def saveRecord(self):
        self.data2write.append(self.currentRecordData)
        self.currentRecordData = [None] * len(self.dataColumns)
        
    #funkcja zapisujaca obecne dane do arkusza w excelu o nazwie 'name'.xlsx
    def saveToFile(self, name):
        print("Zapisywanie danych DataLog do pliku " + name + ".xlsx")
        
        writer = pd.ExcelWriter(os.path.join(path, name + ".xlsx"), engine="xlsxwriter")
        df = pd.DataFrame(self.data2write, columns=self.dataColumns)
        df.to_excel(writer, index=False, sheet_name="BallanceLog")
        
        #zmiany wizualne arkusza
        worksheet = writer.sheets["BallanceLog"]
        worksheet.set_zoom(90)
        worksheet.set_column('A:T', 15)

        #zapisanie arkusza
        writer.save()
