import csv
import os
import serial
import re

from datetime import datetime

serialPort = None
for filename in os.listdir("/dev"):
    if re.match("tty.usbmodem\d+", filename):
        serialPort = "/dev/" + filename
        break
    # end
# end

if not serialPort:
    print("Bluefruit Sense board is not connected.")
    exit(0)
# end

bandRate = 115200  # In arduino, Serial.begin(band_rate)

# log file
logName = "{}/logs/{}.csv".format(os.getcwd(), datetime.now().strftime("%Y-%m-%d_%H_%M"))
csvFile = open(logName, 'w', newline = '')
csvWriter = csv.writer(csvFile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
csvWriter.writerow(["time"]+["temperature"]+["pressure"]+["humidity"]+["altitude"]+["accel_x"]+["accel_y"]+["accel_z"]+["gyro_x"]+["gyro_y"]+["gyro_z"]+["mic"]+["fan"])

device = serial.Serial(serialPort, bandRate)

while True:
    timeStamp = str(datetime.now())

    data = (device.readline()).decode("utf-8").splitlines()[0].split(',')
    # print(data)
    csvWriter.writerow([timeStamp]+[data[0]]+[data[1]]+[data[2]]+[data[3]]+[data[4]]+[data[5]]+[data[6]]+[data[7]]+[data[8]]+[data[9]]+[data[10]]+[-1])
# end
