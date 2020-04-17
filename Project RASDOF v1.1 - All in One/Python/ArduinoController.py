'''
PLEASE NOTE: 
This code automatically checks the COM port that the Arduino is connected to. It 
does not constantly check the status of Arduino connection. It assumes that the Arduino is connected to COM5
_____________________________________________________________________________________________________________
FUTURE WORKS:
For code to automatically detect when the Arduino is connected and disconnected

'''



'''
Import Modules
'''
import serial.tools.list_ports
import time
import threading
import serial



'''
Arduino Controller Class
'''

class ArduinoController():

    '''
    Class Initialisation
    ''' 
    def __init__(self, baud_rate=9600, read_timeout=5):
        '''
        1) Automatically find the COM port to which Arduino is connected to
        2) Initializes the serial connection to the Arduino board
        '''
        self.serial_port = self.check_arduino_port()
        self.arduino_connection = serial.Serial(self.serial_port, baud_rate)
        self.arduino_connection.timeout = read_timeout # Timeout for readline()
        print '[Status Message] Serial Connection at', self.arduino_connection, 'is', self.arduino_connection.is_open


    '''
    Class Functions
    '''

    def check_arduino_port(self):
        '''
        Automatically finds Arduino COM port for initialisation
        '''

        try:
            ports_detected = list(serial.tools.list_ports.comports()) # Machine statements
            myports = [tuple(p) for p in list(serial.tools.list_ports.comports())] # shows all port
            print '[Status Message] Connected ports are:', myports


            flagfound = False
            for device in ports_detected:
                if 'Arduino Uno' in device.description:
                    flagfound = True
                    self.serial_port = device[0]
                    print '[Status Message] Using the first detected Arduino. Arduino COM Port is:', self.serial_port
                    
            if not flagfound:
                print '[Status Message] No Arduino Unos detected'
            
        except: # This is the else statement
            print '[Status Message] No Arduino Unos detected' # Other error messages will appear

        return self.serial_port



    def servo_write(self, pin_number, digital_value):
        '''
        Writes the digital_value on pin_number

        How it works:
        Internally sends b'WS{pin_number}:{digital_value}' over the serial
        connection 
        '''
        command = "WS{}:{}".format(str(pin_number),str(digital_value)).encode()
        self.arduino_connection.write(command) 
        print '  Writing Angle', digital_value, 'degrees to Pin Number', pin_number



    # Closes Arduino port
    def close(self):
        '''
        To ensure we are properly closing our connection to the
        Arduino device. 
        '''
        self.arduino_connection.close()
        print '  Arduino Connection closed'



if __name__ == "__main__":
    ArduinoController()
