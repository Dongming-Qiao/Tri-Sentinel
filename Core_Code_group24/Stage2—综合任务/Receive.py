import pyb
from pyb import UART, LED

uart = UART(3, 115200)  # Initialize UART3 with baud rate 115200

sensor_data = [0, 0, 0, 0]  # 四个传感器数据暂存数组

def Receive_Sensor_Data():
    if uart.any():  # If there is any data received
        receive = uart.read().decode().strip()  # 将接收到的消息提取出来
        print(receive)
        if(receive[4] == '+'):
            sensor_data[0] = 1 if receive[0] == '1' else 0
            sensor_data[1] = 1 if receive[1] == '1' else 0
            sensor_data[2] = 1 if receive[2] == '1' else 0
            sensor_data[3] = 1 if receive[3] == '1' else 0

        else:
            sensor_data[0] = -1 if receive[0] == '1' else 0
            sensor_data[1] = -1 if receive[1] == '1' else 0
            sensor_data[2] = -1 if receive[2] == '1' else 0
            sensor_data[3] = -1 if receive[3] == '1' else 0
    return None

def Get_Sensor_Data():
    return sensor_data.copy()