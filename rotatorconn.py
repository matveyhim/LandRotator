import socket
import sys
import glob
import serial
import time
from math import asin, sin, cos, atan2, pi, degrees, radians
from typing import Tuple

XYmode = True # XY or AZ/EL rotator
# XYmode = False
baudrate = 115200
# baudrate = 9600

bind_ip = "0.0.0.0"
bind_port = 4533

AZhome = 0 # AZ / X motor home
ELhome = 0 # EL / Y motor home
SendToHome = False # send rotator to home on startup

minEL = 0
minAZ = -90
maxAZ = 450

az = 0
el = 0

global ser
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ser = serial.Serial()
ser.close()

def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def AE2XY(azimuth: float, elevation: float) -> Tuple[float, float]:
    x = asin(sin(azimuth) * cos(elevation))
    y = -atan2(cos(azimuth) * cos(elevation), sin(elevation))
    
    return (x, y)

def XY2AE(x: float, y: float) -> Tuple[float, float]:
    x0 = sin(x)
    y0 = -sin(y)*cos(x)
    z0 = cos(y)*cos(x)
    
    el = asin(z0)
    az = - atan2(y0, x0)    
    
    return pi/2 + az, el

def setPosition(az, el):
    if XYmode:
        az = radians(az)
        el = radians(el)
        x, y = AE2XY(az, el)

        x = degrees(x)
        y = degrees(y)
    else:
        x = az
        y = el

    resp = 'AZ'+'{:.3f}'.format(round(x,3))+' EL'+'{:.3f}'.format(round(y,3))+'\n'
    ser.write(resp.encode('ascii'))
    # print(resp)

def getPosition():
    ser.write(b"AZ EL")
    resp=ser.readline().decode('ascii')

    space=resp.find(' ') # format: AZXX.xxx ELYY.yyy
    x = float(resp[2:space])
    y = float(resp[space+3:len(resp)-1])

    if XYmode:
        x = radians(x)
        y = radians(y)
        az, el = XY2AE(x, y)

        az = degrees(az)
        el = degrees(el)
    else:
        az = x
        el = y

    if az<0: az = az+360
    return (az, el)

def parse(text):
    Ispace=text.find(' ')
    IIspace=text.find(' ',Ispace+1)
    az=float(text[Ispace:IIspace])
    el=float(text[IIspace+1:len(text)-1])
    return (az, el)

def doComms(client_socket):
    global az,el
    try:    
        request = client_socket.recv(1024)
    except:
        print('Client disconnected on recv')
        request = b''

    text = request.decode('ascii')
    print ("    Received: "+text[:len(text)-1])

    if text.find('p')!=-1:
        az, el = getPosition()
        resp = '{:.3f}'.format(round(az,3))+' '+'{:.3f}'.format(round(el,3))+'\n'

        try:
            client_socket.sendall(resp.encode('ascii')) 
        except:
            print('Client disconnected on send')

        print ("    Sended: "+resp[:len(resp)-1])

    if text.find('P')!=-1:
        az, el = parse(text) # format: P AA.aaa EE.eee
        if az>maxAZ:
            az = az-360
            
        if az<minAZ:
            az = az+360
            
        if el<minEL:
            el = minEL

        setPosition(az, el)

        try:
            client_socket.sendall(b"RPRT 0\n")
        except:
            print('Client disconnected')

        print ("    Sended: RPRT 0") 
    return text

ports = serial_ports()
print(ports)

def search(ports):
    global ser
    for port in ports:
        ser = serial.Serial(port)
        ser.baudrate = baudrate
        ser.timeout = 2
        time.sleep(2)
        print('trying',port)
        ser.write(b"AZ EL")

        try:
            resp=ser.readline().decode('ascii')
            print('ans:',resp[:len(resp)-1])
        except:
            resp=''

        if resp.rfind('AZ')!=-1:
            print('connected to',port)
            ser.timeout = 5
            break
        ser.close()

search(ports)

while ports == [] or ser.isOpen() == False:
    print('Connnect rotator!!!')
    ports=serial_ports()
    print(ports)
    search(ports)
    time.sleep(1)

if SendToHome: ser.write(b"AZ"+str(AZhome)+" EL"+str(ELhome))

server.bind((bind_ip, bind_port))
server.listen()
print ("Listening on %s:%d" % (bind_ip, bind_port))

try:
    while True: # работаем постоянно
        conn, addr = server.accept()
        print("New connection from " + addr[0])
        try:
            while True:
                text=doComms(conn)
                if text=="":
                    break        
        finally:
            conn.close()
finally: server.close()
