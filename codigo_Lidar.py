import os
import ydlidar
import time
import numpy as np
import serial
import math

# ----------- CONFIGURACIï¿½N GENERAL -----------

x_deseado = 0.0 # metros
y_deseado = 0.0  # metros
frepx = 0.0
frepy = 0.0
krep = 0.0008
dmax=0.4
theta_deseado =  0# radianes
RMAX = 10  # Rango mï¿½ximo vï¿½lido (m)

ESP32_PORT = "/dev/ttyUSB2"
ESP32_BAUD = 250000

# ----------- INICIALIZACIï¿½N LIDAR -----------

ydlidar.os_init()
ports = ydlidar.lidarPortList()
port = "/dev/ttyUSB0"
for key, value in ports.items():
    port = value

laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 12)
laser.setlidaropt(ydlidar.LidarPropSampleRate, 10)
laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)


ret = laser.initialize()
if ret:
    ret = laser.turnOn()
    scan = ydlidar.LaserScan()
    # ----------- INICIALIZACIï¿½N SERIAL -----------

    ser = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=1)
    time.sleep(2)

    # ----------- BUCLE PRINCIPAL -----------

    while ret and ydlidar.os_isOk():
        r = laser.doProcessSimple(scan)
        if r:
            distancias = []
            angulos = []
            for point in scan.points:
                #print(len(scan.points))
                dist = point.range
                angle = point.angle
                if (dist is not None and
                    isinstance(dist, (int, float)) and
                    not math.isnan(dist) and
                    not math.isinf(dist) and
                    0.1 < dist < RMAX):
                    distancias.append(dist)
                    angulos.append(angle)
                else:
                    distancias.append(0.0)
                    angulos.append(0.0)
            angulos = angulos[::-1]
            frepx = 0
            frepy = 0
            for i in range(len(distancias)):
                xs_i=distancias[i]*math.cos(angulos[i])
                ys_i=distancias[i]*math.sin(angulos[i])
                di = math.sqrt(xs_i**2 + ys_i**2)
                if di < 0.005:
                    di = 0.005
                if di > dmax:
                    di4 = di**4
                    frepx += -krep * xs_i / di4
                    frepy += -krep * ys_i / di4
            frepx_str = f"{frepx:.2f}"
            frepy_str = f"{frepy:.2f}"
            #distancias_str = ",".join(distancias)
            #print(len(distancias))
            #print(frepx)
            #print(frepy)
            deseados_str = f"{x_deseado:.2f},{y_deseado:.2f},{theta_deseado:.2f}"
            mensaje = f"<{frepx_str},{frepy_str},{deseados_str}>\n"
            ser.write(mensaje.encode('utf-8'))

        else:
            print("Failed to get Lidar Data")

        time.sleep(0.01)

    # ----------- APAGADO -----------

    laser.turnOff()
    ser.close()

laser.disconnecting()