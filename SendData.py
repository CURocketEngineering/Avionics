"""
This Class will retrieve flight data and from our csv file and print it 
according to the time delta in the file.
"""
import csv
import pandas as pd
import time as tm
import struct
import serial
import psutil

class RetrieveData:
    def __init__(self, filename):
        self.filename = filename
        


    
    """
    retrieve the time from the csv file and return it
    """
    def getTime(self):
        data = pd.read_csv(self.filename)
        time = data['time']
        return time
    """
    retrieve the x acceleration from the csv based on row with matching time and return it
    """
    def getAccelX(self):
        data = pd.read_csv(self.filename)
        accelX = data['accelx']
        return accelX

    """
    retrieve the y acceleration from the csv file and return it
    """
    def getAccelY(self):
        data = pd.read_csv(self.filename)
        accelY = data['accely']
        return accelY
    """
    retrieve the z acceleration from the csv file and return it
    """
    def getAccelZ(self):
        data = pd.read_csv(self.filename)
        accelZ = data['accelz']
        return accelZ
    """
    retrieve the x gyro from the csv file and return it
    """
    def getGyroX(self):
        data = pd.read_csv(self.filename)
        gyroX = data['gyrox']
        return gyroX
    """
    retrieve the y gyro from the csv file and return it
    """
    def getGyroY(self):
        data = pd.read_csv(self.filename)
        gyroY = data['gyroy']
        return gyroY
    def getGyroZ(self):
        data = pd.read_csv(self.filename)
        gyroZ = data['gyroz']
        return gyroZ
    """
    get altitude from the csv file and return it
    """
    def getAltitude(self):
        data = pd.read_csv(self.filename)
        altitude = data['altitude']
        return altitude
    """
    get temperature from the csv file and return it
    """
    def getTemperature(self):
        data = pd.read_csv(self.filename)
        temperature = data['temp']
        return temperature
    
    """
    print data at original time delta by setting a sleep timer based on the time delta. 
    """
    def printData(self):
        time = self.getTime()
        accelX = self.getAccelX()
        accelY = self.getAccelY()
        accelZ = self.getAccelZ()
        gyroX = self.getGyroX()
        gyroY = self.getGyroY()
        gyroZ = self.getGyroZ()
        altitude = self.getAltitude()
        temperature = self.getTemperature()

        #work for mac/windows/linux -> pick usb serial port. ex: COM3
        #send data over serial port
        ser = serial.Serial('placeholder', baudrate=9600, timeout=1)
        
        for i in range(len(time)):
            
            print("Time: ", time[i])

            print("AccelX: ", accelX[i])
            print("AccelY: ", accelY[i])
            print("AccelZ: ", accelZ[i])
            print("GyroX: ", gyroX[i])
            print("GyroY: ", gyroY[i])
            print("GyroZ: ", gyroZ[i])
            print("Altitude: ", altitude[i])
            print("Temperature: ", temperature[i])

            #36 byte packet
            packet = struct.pack('>IfF', time[i], accelX[i], accelY[i], accelZ[i], gyroX[i], gyroY[i], gyroZ[i], altitude[i], temperature[i])
            ser.write(packet)
            
            print("\n")
            if i > 0:
                print("Time Delta: ", time[i] - time[i-1])
                tm.sleep(abs(time[i] - time[i+1]) / 1e9)




#package data into a packet represent as 4 byte floats 8 floats total 1 time stamp also 4 bytes
#9 bytes of data. 
#look into pyserial library - Send on usb port. 
#command data injection mode to martha then send. 
#// path to data on my laptop as an argument. 
#will pull repo to rapsberry pi on hil1 NOT hil2
def main():
    filename = 'data2.csv'
    retrieveData = RetrieveData(filename)
    retrieveData.printData()
# run python script on hil1 rapsberry pi. 
#serial sensor data driver. 
if __name__ == "__main__":
    main()