import pyb
from pyb import UART, LED

# 与uart3串口不同，蓝牙接口波特率为9600
uart = UART(3, 9600)  # Initialize UART3 with baud rate 9600

action = ""


def Receive_Data_bt():
    global action
    """
    现象：
        消息发送后，uart.read()在数据到达前被调用，导致返回None。
    原因：
        蓝牙数据传输有延迟，若代码中uart.read()调用过早，缓冲区可能还没有数据。
        部分模块默认缓冲区较小，若数据未及时读取，可能被覆盖或丢弃。
    """

    # 等待有数据可读时再读取
    # while uart.any() == 0:  # 检查缓冲区是否有数据
    #    pass  # 等待
    data = uart.read()  # 此时大概率能读到数据

    action = ""
    if data != None:
        print(f"Info received from bluetooth as {data}.")
        """
        理解不了传过来的是什么东西，只能掩耳盗铃了
        up: b'\xf8\x00\xf8'
        left: b'\x00\x80\xf8'
        right: b'\x80x\xf8'
        OK: b'x\x80\xf8'
        down: b'\xf8x\xf8'
        """
        if data == b'\xf8\x00\xf8':
            action = "up"
            print("up")
        elif data == b'\x00\x80\xf8':
            action = "left"
            print("left")
        elif data == b'\x80x\xf8':
            action = "right"
            print("right")
        elif data == b'x\x80\xf8':
            action = "OK"
            print("OK")
        elif data == b'\xf8x\xf8':
            action = "down"
            print("down")
        else:
            action = "failed"
            print("Failed to understand info.")
        if action != None:
            print("Action is " + action)

    return None


def Get_Action_bt():
    return action