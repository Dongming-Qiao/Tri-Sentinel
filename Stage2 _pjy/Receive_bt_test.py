import pyb
from pyb import UART, LED

#与uart3串口不同，蓝牙接口波特率为9600
uart = UART(3, 9600)  # Initialize UART3 with baud rate 115200

sensor_data = [0, 0, 0, 0]  # 四个传感器数据暂存数组

def Receive_Sensor_Data():
    """
    现象：
        消息发送后，uart.read()在数据到达前被调用，导致返回None。
    原因：
        蓝牙数据传输有延迟，若代码中uart.read()调用过早，缓冲区可能还没有数据。
        部分模块默认缓冲区较小，若数据未及时读取，可能被覆盖或丢弃。
    """

    # 等待有数据可读时再读取
    while uart.any() == 0:  # 检查缓冲区是否有数据
        pass  # 等待
    data = uart.read()  # 此时大概率能读到数据
    if data!=None:
        print(data)

    """
    理解不了传过来的是什么东西，只能掩耳盗铃了
    up: b'\xf8\x00\xf8'
    left: b'\x00\x80\xf8'
    right: b'\x80x\xf8'
    OK: b'x\x80\xf8'
    down: b'\xf8x\xf8'
    """
    if data==b'\xf8\x00\xf8':
        print("up")
    elif data==b'\x00\x80\xf8':
        print("left")
    elif data==b'\x80x\xf8':
        print("right")
    elif data==b'x\x80\xf8':
        print("OK")
    elif data==b'\xf8x\xf8':
        print("down")

    return None

def Get_Sensor_Data():
    return sensor_data.copy()


while True:
    Receive_Sensor_Data()  # 接收传感器数据
