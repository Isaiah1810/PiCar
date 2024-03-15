import serial
from picar_hardware_interface import Interface

inter = Interface()
inter.set_freq(200)
inter.set_duty(50)
print(inter.read())
