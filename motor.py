#스테퍼 모터
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

def setStep(w1,w2,w3,w4):
    GPIO.output(coil_A_1_pin,w1)
    GPIO.output(coil_A_2_pin,w2)
    GPIO.output(coil_B_1_pin,w3)
    GPIO.output(coil_B_2_pin,w4)

def forward(delay,steps):
    for i in range(steps):
        for j in range(step_count):
            setStep(seq[j][0],seq[j][1],seq[j][2],seq[j][3])
            time.sleep(delay)

def backwards(delay,steps):
    for i in range(steps):
        for j in reversed(range(step_count)):
            setStep(seq[j][0],seq[j][1],seq[j][2],seq[j][3])
            time.sleep(delay)

coil_A_1_pin = 17#1
coil_A_2_pin = 27#2
coil_B_1_pin = 22#3
coil_B_2_pin = 23#4

step_count = 8
seq = list(range(0,step_count))
seq[0] = [1,0,0,0]
seq[1] = [1,1,0,0]
seq[2] = [0,1,0,0]
seq[3] = [0,1,1,0]
seq[4] = [0,0,1,0]
seq[5] = [0,0,1,1]
seq[6] = [0,0,0,1]
seq[7] = [1,0,0,1]
GPIO.setup(coil_A_1_pin,GPIO.OUT)
GPIO.setup(coil_A_2_pin,GPIO.OUT)
GPIO.setup(coil_B_1_pin,GPIO.OUT)
GPIO.setup(coil_B_2_pin,GPIO.OUT)
if __name__=='__main__':
    delay = 0.001           #다음 스텝으로 진행할 때 필요한 시간
    steps = 1000            #모터가 돌아가는 시간
    forward(delay,int(steps))

GPIO.cleanup()

#I2C 멀티제어
from board import SCL, SDA

import busio
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)

pca = PCA9685(i2c)
pca.frequency = 60

duty = 20
pul = duty * 16 *4096 // 100
pca.channels[0].duty_cycle = pul

#BLDC
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
CW_PIN = 23
VOL_PIN = 24
GPIO.setup(CW_PIN, GPIO.OUT)
GPIO.setup(VOL_PIN, GPIO.OUT)

#0, 1으로 방향 조정
GPIO.output(CW_PIN, 0)

#속도(주파수) 조정(저속에서 고속으로)
p = GPIO.PWM(VOL_PIN, 1000)

#점점 속도를 올려 안정적으로 동작하게 함
p.start(0)
time.sleep(1)
p.start(20)
time.sleep(1)
p.start(60) #duty
time.sleep(10)
p.start(0)
input()         #shell에 아무 입력을 넣어주면
GPIO.cleanup()