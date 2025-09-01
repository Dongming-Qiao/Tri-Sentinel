import pyb
from pyb import UART, LED

uart = UART(3, 115200)  # Initialize UART3 with baud rate 115200

led_red = LED(1)    
led_green = LED(2)

sensor_data = [0, 0, 0, 0]  # 四个传感器数据暂存数组

def Receive_Sensor_Data():
    if uart.any():  # If there is any data received
        data = uart.read(6)
        
        if data and len(data) == 6:
            if data[0] == 0xAA and data[5] == 0x55:
                for i in range(4):
                    sensor_data[i] = data[i + 1]

                led_green.toggle()
                return sensor_data.copy()
            else:
                led_red.toggle()
        else:
            led_red.toggle()
    #返回四个传感器数据暂存数组
    return None

def Get_Sensor_Data():
    return sensor_data.copy()
            