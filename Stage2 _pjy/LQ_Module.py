"""
# 调用示例,
注意：需了解 所用管脚pin_pwm为定时器timer几的通道chl几

# 从自定义模块LQ_Motor中导入两个自定义电机对象
import LQ_Motor
from LQ_Motor import motor,motor_brushless   # 不使用此句则需要在对象前加 模块名. LQ_Motor.motor(...)

#参数(self, timer, chl, freq, pin_pwm, pin_io)

motor1 = motor(timer=4, chl=3, freq=10000, pin_pwm="P9", pin_io="P24")
motor1.run(cnt)
motor1.run_percent(cnt)   # PWM 百分比 计
motor1.run(cnt)           # 实际脉宽24000满

#LQ 无刷驱动 在50Hz 下1~2ms范围2000~4000, 算上驱动死区可控范围约在 2240 ~3800

M1= motor_brushless(4, 1, 50, "P7")  #参数(self, timer, chl, freq, pin_pwm)
M1.run(2300)

"""
import pyb,time
from pyb import Pin, Timer,ExtInt,ADC

class motor:
    def __init__(self, timer, chl, freq, pin_pwm, pin_io):
        self.io = Pin(pin_io, Pin.OUT_PP, Pin.PULL_NONE)
        self.chl = chl
        self.timer = timer
        self.freq = freq
        self.duty = 0
        self.tim = Timer(self.timer, freq=self.freq)
        self.pwm = self.tim.channel(self.chl, Timer.PWM, pin=Pin(pin_pwm), pulse_width_percent=self.duty)
        self.ad = ADC("P19")

    def run_percent(self, duty):
        duty = int(duty)
        if(duty<0):
            self.duty = -duty
            self.io.value(1)
            self.pwm.pulse_width_percent(self.duty)
        else:
            self.duty = duty
            self.io.value(0)
            self.pwm.pulse_width_percent(self.duty)

    def run(self, duty):
        if(duty<0):
            self.duty = -duty
            self.io.value(1)
            self.pwm.pulse_width(int(self.duty*(695/self.ad.read())))
        else:
            self.duty = duty
            self.io.value(0)
            self.pwm.pulse_width(int(self.duty*(695/self.ad.read())))

class motor_brushless:
    def __init__(self, timer, chl, freq, pin_pwm):
        self.chl = chl
        self.timer = timer
        self.freq = freq
        self.duty = 0
        self.tim = Timer(self.timer, freq=self.freq)
        self.pwm = self.tim.channel(self.chl, Timer.PWM, pin=Pin(pin_pwm), pulse_width_percent=self.duty)

    def run(self, duty):
        duty = int(duty)         #pulse_width只接受int类型的参数
        if(duty<0):      #abs
            self.duty = -duty
        else:
            self.duty = duty
        self.pwm.pulse_width(self.duty) 


#____LQ_Motor end_____

"""
#调用示例

import pyb, time, LQ_Enc
from pyb import Pin, Timer
from LQ_Enc import Enc_AB, Encoder

#初始化  定时器,频率(多久读一次),管脚, 适用于AB和带方向的编码器以及单一脉冲的编码器

#带方向的
Enc1 = Enc_AB(Timer(12, freq=10), Enc_A="P24", Enc_B="P23")

#不带方向的
Enc2 = Encoder(Timer(14, freq=10), Encpin="P23")

while True:
    time.sleep_ms(30)   # 延时多久无所为

    print(Enc1.Get(), Enc2.Get())

"""
class Enc_AB:
    def __init__(self, timer, Enc_A, Enc_B):
        self.pin_A = Pin(Enc_A, Pin.IN, Pin.PULL_DOWN)
        self.pin_B = Pin(Enc_B, Pin.IN, Pin.PULL_DOWN)
        self.counter = 0
        self.cnt = 0
        self.set_callbacks()
        timer.callback(self.Enc_tick)

    def __ENC_cnt(self, line):
        if(self.pin_B.value()):
            self.counter += 1
        else:
            self.counter -= 1

    def Enc_tick(self, tim):
        self.cnt = self.counter
        self.counter = 0
#        print(self.cnt)

    def set_callbacks(self):

        ExtInt(self.pin_A, ExtInt.IRQ_RISING, self.pin_A.PULL_DOWN, self.__ENC_cnt)

    def Get(self):
        return  self.cnt

class Encoder:
    def __init__(self, timer, Encpin):
        self.pin_A = Pin(Encpin, Pin.IN, Pin.PULL_DOWN)
        self.cntr = 0
        self.Ecnt = 0
        self.set_Init()
        timer.callback(self.Enc_tims)

    def ENC_cntr(self, line):
        self.cntr += 1

    def Enc_tims(self, tim):
        self.Ecnt = self.cntr
        self.cntr = 0

    def set_Init(self):
        ExtInt(self.pin_A, ExtInt.IRQ_RISING, self.pin_A.PULL_DOWN, self.ENC_cntr)

    def Get(self):
        return  self.Ecnt
# ______ LQ_Enc  end __________


"""_class_key
"""
class key:
    def __init__(self, key_pin, timeout = 650):
        self.pin = Pin(key_pin, Pin.IN, Pin.PULL_UP)
        self.count = 0
        self.flag = 0
        self.status = 0
        self.timeout = timeout

    def down(self):
        if(self.flag == 0):
            if(self.pin.value() == 0):
                return 1
            else:
                return 0
        else:
            return 0
    def up(self):
        if(self.flag == 0):
            if(self.pin.value() == 0):
                while(self.pin.value() == 0):
                    pass
                return 1
            else:
                return 0
        else:
            return 0

    def value(self):
        return self.pin.value()

    def hold(self):
        if(self.pin.value() == 0):              # 按键按下
            if(self.flag == 0):                 # 判断标志位是否为长按状态下 否-往下执行
                for i in range(self.timeout):
                    self.count = 1              # 按键按下标志为1
                    time.sleep_ms(1)            # 延时
                    if(self.pin.value() == 1):  # 再次判断按键是否按下
                        return 0
                self.flag = 1                   # 长按标志位置1
                return self.flag                # 返回标志位

            else:
                if(self.pin.value() == 1):
                    self.count = 0
                    self.flag = 0
                    return 0
                else:
                    return 1
        else:
            self.count = 0
            self.flag = 0
            return 0
        
class speed_pid:
    def __init__(self,kp,ki,kd):
        self.err = 0
        self.err_last = 0
        self.out = 0
        self.out_last = 0
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def compute(self,speed_expect,speed_real):
        self.err = speed_expect - speed_real
        self.err = (self.err * 0.7) + (self.err_last * 0.3)
        self.out = self.out_last + (self.kp * (self.err - self.err_last)) + self.ki * self.err
        self.out_last = self.out
        self.err_last = self.err
        return self.out

# ______ LQ_key  end __________

