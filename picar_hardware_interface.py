import serial 

class Interface:
    def __init__(self, port='/dev/ttyS3', baud_rate=9600):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = serial.Serial(port=self.port,
                                 baudrate=self.baud_rate,
                                 timeout=5.0,
                                 bytesize=8,
                                 parity='N',
                                 stopbits=1)
    def set_freq(self, freq):
        if(freq > 999): return 1
        self.ser.write(f"F{freq}")
    
    def set_duty(self, duty):
        self.ser.write(f"D{duty}")

    def read(self):
        self.ser.write("read")
        msg = self.read(4)
        print(str(msg))