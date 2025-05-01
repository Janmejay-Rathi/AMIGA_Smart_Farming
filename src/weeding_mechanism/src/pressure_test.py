import Jetson.GPIO as GPIO
from time import sleep

# Working GPIO Pins according to board numbering - 11, 12, 16, 18, 19, 21, 22, 24, 26, 35, 37
if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD)

    pin_1 = 16
    pin_2 = 37
    # pin__2 = 351 = 18 #extend
    # pin #extend
    GPIO.setup(pin_1, GPIO.OUT)
    GPIO.setup(pin_2, GPIO.OUT)

    GPIO.output(pin_1, 1)
    GPIO.output(pin_2, 1)

    # print('turning 1 on-----')
    # GPIO.output(pin_2, 0)
    # sleep(1)
    # print('turning 1 off-----')
    # GPIO.output(pin_2, 1)
    # sleep(1)
    print('turning 2 on-----')
    GPIO.output(pin_2, 0)
    sleep(0.09)
    print('turning 2 off-----')
    GPIO.output(pin_2, 1)
    sleep(0.09)

    GPIO.output(pin_1, 0)
    sleep(0.09)
    print('turning 2 off-----')
    GPIO.output(pin_1, 1)
    sleep(0.09)


