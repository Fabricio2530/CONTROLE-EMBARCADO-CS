from gzip import READ
import serial
import argparse
import time
import logging
import pyvjoy # Windows apenas

class MyControllerMap:
    def __init__(self):
        self.button = {'A': 1}
        self.button = {'B': 2}


class SerialControllerInterface:

    # Protocolo

    # Header|Payload|EOP

    # Header: 1 byte
        # Se for "K" começa a ler.
    
    # Payload: 2 bytes.
        # Posição [0]: A -> 1(apertado) e 0(Não apertado) 
        # Posição [1]: B -> 1(apertado) e 0(Não apertado)
    
    # EOP: 1 byte
        # Se for "X" para de ler.

    
    

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.j = pyvjoy.VJoyDevice(1)
        self.incoming = '0'

    def update(self):
        ## Sync protocol
        READING = False
        if self.incoming == b'K':
            READING = True
            self.incoming = self.ser.read()
            data = bytearray()


        while READING:
            if self.incoming == b'X':
                READING = False
                break
            
            self.incoming = self.ser.read()
            data += self.incoming
            logging.debug("Received INCOMING: {}".format(self.incoming))
            
        print("Received DATA: {}".format(data))
       
        
        # Button A
        if data[0] == b'1':
            logging.debug("Sending press - Button A")
            self.j.set_button(self.mapping.button['A'], 1)
        elif data[0] == b'0':
            self.j.set_button(self.mapping.button['A'], 0)

        # Button B
        if data[1] == b'1':
            logging.debug("Sending press - Button B")
            self.j.set_button(self.mapping.button['B'], 1)
        elif data[1] == b'0':
            self.j.set_button(self.mapping.button['B'], 0)

        self.incoming = self.ser.read()


class DummyControllerInterface:
    def __init__(self):
        self.mapping = MyControllerMap()
        self.j = pyvjoy.VJoyDevice(1)

    def update(self):
        self.j.set_button(self.mapping.button['A'], 1)
        time.sleep(0.1)
        self.j.set_button(self.mapping.button['A'], 0)
        logging.info("[Dummy] Pressed A button")
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
