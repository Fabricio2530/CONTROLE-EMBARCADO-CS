import serial
import argparse
import time
import logging
import pyvjoy # Windows apenas
import numpy as np

#  j.set_axis(pyvjoy.HID_USAGE_X, 16383)

class MyControllerMap:
    def __init__(self):
        self.button = {'A': 1, 'B':2, 'C':3, 'Menu':4, "Load":5}


class SerialControllerInterface:

    # Protocolo
    # byte 1 -> Botão 1 (estado - Apertado 1 ou não 0)
    # byte 2 -> EOP - End of Packet -> valor reservado 'X'

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.j = pyvjoy.VJoyDevice(1)
        self.incoming = '0'
        self.handshake = False

    def interporla_analog(self, raw_value):

        return int(32767/4085)*(raw_value-10)
    
    def interporla_gyr(self, raw_value):

        return int( (2**15-1) * (raw_value + 180)/360 )

    def update(self):
        while(not(self.handshake)):

            self.incoming = self.ser.read()
            print(self.incoming)
            if (self.incoming == b'K' ):
                self.ser.write('i'.encode("ascii"))
                self.handshake = True
                self.incoming = '0'
         ## Sync protocol
        data = bytearray()
        
        READING = False
        if self.incoming == b'K':
            READING = True
            #logging.debug("Received b'K'")
            self.incoming = self.ser.read()


        while READING:
            if self.incoming == b'X':
                READING = False
                break
            
            data += (self.incoming)
            self.incoming = self.ser.read()
            #logging.debug("Received INCOMING: {}".format(self.incoming))
            
        print("Received DATA: {}".format(data))
        
        if len(data) > 9:
            # Button A
            #print("INDICE 0 DO ARRAY: {}".format(data[0]))
            if (data[0]) == ord('1'):
                #logging.debug("Sending press - Button A")
                self.j.set_button(self.mapping.button['A'], 1)
            elif (data[0]) == ord('0'):
                self.j.set_button(self.mapping.button['A'], 0)
                
            # Button B
            if data[1] == ord('1'):
                #logging.debug("Sending press - Button B")
                self.j.set_button(self.mapping.button['B'], 1)
            elif data[1] == ord('0'):
                self.j.set_button(self.mapping.button['B'], 0)
            
            # Button C
            if data[2] == ord('1'):
                #logging.debug("Sending press - Button C")
                self.j.set_button(self.mapping.button['C'], 1)
            elif data[2] == ord('0'):
                self.j.set_button(self.mapping.button['C'], 0)
            
             # Button Menu
            if data[3] == ord('1'):
                #logging.debug("Sending press - Button Menu")
                self.j.set_button(self.mapping.button['Menu'], 1)
            elif data[3] == ord('0'):
                self.j.set_button(self.mapping.button['Menu'], 0)
            
             # Button Load
            if data[4] == ord('1'):
                #logging.debug("Sending press - Button Load")
                self.j.set_button(self.mapping.button['Load'], 1)
            elif data[4] == ord('0'):
                self.j.set_button(self.mapping.button['Load'], 0)

            # Realiza a leitura do analógico
            x1 = data[5]
            x2 = data[6]
            x = (x2 << 8) | x1
            y1 = data[7]
            y2 = data[8]
            y = (y2 << 8) | y1

            yaw_1 = data[9]
            yaw_2 = data[10]
            yaw_3 = data[11]
            yaw_4 = data[12]
            yaw =  np.int16((yaw_4 << 12) | (yaw_3 << 8) | (yaw_2 << 4) | (yaw_1))
            print(yaw)
            self.j.set_axis(pyvjoy.HID_USAGE_X, self.interporla_analog(x))
            self.j.set_axis(pyvjoy.HID_USAGE_Y, self.interporla_analog(y))
            self.j.set_axis(pyvjoy.HID_USAGE_RX, self.interporla_gyr(yaw))
            #self.j.set_axis(pyvjoy.HID_USAGE_X, 16383)

            

            #total_y = int(chr(ord(x1))) | int(chr(ord(x2)))

            #print("O total em x é: {}\n".format(total_x))
            #print("O total em y é: {}\n".format(total_y))

        self.incoming = self.ser.read()



class DummyControllerInterface:
    def __init__(self):
        self.mapping = MyControllerMap()
        self.j = pyvjoy.VJoyDevice(1)

    def update(self):
        self.j.set_button(self.mapping.button['A'], 1)
        time.sleep(0.1)
        self.j.set_button(self.mapping.button['A'], 0)
        #logging.info("[Dummy] Pressed A button")
        time.sleep(1)


if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=9600)
    argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(args.serial_port, args.controller_interface, args.baudrate))
    if args.controller_interface == 'dummy':
        controller = DummyControllerInterface()
    else:
        controller = SerialControllerInterface(port=args.serial_port, baudrate=args.baudrate)

    while True:
        controller.update()
