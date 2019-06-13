#modul do tworzenia i zapisywania log'ow danych

import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt
import multiprocessing
import time

path = os.path.dirname(os.path.abspath(__file__))    #sciezka, w ktorej znajduje sie program

#program do zapisywania zepranych danych w arkuszu excel'a
class DataLogger:
    
    def __init__(self):
        print("DataLogger object created")
        #zdefiniowanie nazw kolumn
        self.dataColumns = ["timestamp", "ball_pos_x", "ball_pos_y", "target_pos_x", "target_pos_y", "KP", "KI", "KD", "error_x", "error_y", "error_prev_x","error_prev_y",
                            "error_sum_x", "error_sum_y", "derivative_x", "derivative_y", "servo_actual_x", "servo_actual_y", "servo_target_x", "servo_target_y"]
        self.data2write = {}    #wszystkie dane wpisow razem do zapisania
        DataLogger.clearData(self)
        self.currentRecordData = [None] * len(self.dataColumns)   #dane obecnego wpisu
        
        plt.style.use('ggplot')
        self.temp = False
        
    #funkcja dodajaca wartosc 'value' do kolumny 'name' w obecnym wpisie
    def addRecord(self, name, value):
        self.currentRecordData[self.dataColumns.index(name)] = value
        
    #funkcja finalizujaca obecny wpis
    def saveRecord(self):
        for i in range(len(self.dataColumns)):
            self.data2write[self.dataColumns[i]].append(self.currentRecordData[i])
            
        self.currentRecordData = [None] * len(self.dataColumns)
        
    #czysci wszystjkie zapisane dane
    def clearData(self):
        for col in self.dataColumns:
            self.data2write[col] = []
        
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
        
    def makePlot(self):
        if self.temp: self.process.terminate()
        self.temp = True
        self.process = multiprocessing.Process(target=DataLogger.makePlot_hidden, args=(self,))
        self.process.start()
        
    def makePlot_hidden(self):
        t = self.data2write['timestamp']
        actual = self.data2write['servo_actual_x']
        target = self.data2write['ball_pos_x']
        
        plt.plot(t, actual, 'r', label='servo')
        plt.plot(t, target, 'b', label='pos')
        plt.legend()
        plt.ylim([-1, 1])
        plt.show(block=False)
        time.sleep(4)
