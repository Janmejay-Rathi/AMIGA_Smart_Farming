import Jetson.GPIO as gpio
pin = 33
gpio.setmode(gpio.BOARD)
gpio.setup(pin, gpio.OUT)

gpio.output(pin, 0)