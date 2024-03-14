import serial 

class Interface:
    def __init__(self, port='/dev/ttyS3', baud_rate=9600):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = serial.Serial(port=self.port,
                                 baudrate=self.baud_rate,
                                 timeout=5.0,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE)
    def set_freq(self, freq):
        if(freq > 999): return 1
        self.ser.write(f"F{freq}".encode('ASCII'))
    
    def set_duty(self, duty):
        self.ser.write(f"F{str(duty).zfill(3)}".encode('ASCII'))

    def read(self):
        self.ser.write("read".encode("ASCII"))
        msg = self.ser.read_until("%".encode("ASCII"))
        print(str(msg))