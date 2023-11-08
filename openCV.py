import cv2
import numpy as np
from gpiozero import Servo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
import threading
import asyncio
from concurrent.futures import ProcessPoolExecutor
from concurrent.futures import ThreadPoolExecutor
import RPi.GPIO as GPIO
# Константы
IS_DEBUG = 0
LED_PIN = 4
SERVO_PIN = 14
VIDEO_DEVICE = 0

# Инициализация устройств
from gpiozero import LED
led = LED(LED_PIN)
led_state = False  # текущее состояние LED

if not IS_DEBUG:
	pigpio_factory = PiGPIOFactory()
	servo = Servo(SERVO_PIN, pin_factory=pigpio_factory)

cap = cv2.VideoCapture(VIDEO_DEVICE)

def led_control():
	global led_state
	led_state_old = False
	while True:
		sleep(3)
		print(led_state)
		print(led_state_old)
		if led_state and not led_state_old:
			print("on")
			led.on()
			led_state_old = True
			sleep(10)
		elif not led_state and led_state_old:
			print("off")
			led.off()
			led_state_old = False
			sleep(10)

def camera_loop():
	global led_state
	print("camera_loop")
	while True:
		ret, frame = cap.read()
		print("frame")
		if not ret:
			print("not ret")
			break

		# в пространство цветов HSV
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		# lower boundary RED color range values; Hue (0 - 10)
		lower1 = np.array([0, 100, 20])
		upper1 = np.array([10, 255, 255])

		# upper boundary RED color range values; Hue (160 - 180)
		lower2 = np.array([160,100,20])
		upper2 = np.array([179,255,255])

		lower_mask = cv2.inRange(hsv, lower1, upper1)
		upper_mask = cv2.inRange(hsv, lower2, upper2)
		mask = lower_mask + upper_mask

		# Применение морфологического открытия и закрытия для устранения шума
		kernel = np.ones((5,5),np.uint8)
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
		mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

		# Применение детектора кругов Hough
		circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=1.5, minDist=100, param1=50,param2=30,minRadius=0,maxRadius=0)

		if circles is not None:
			led_state = True  # обновить состояние LED

			# самый большой круг по радиусу
			max_circle = max(circles[0], key=lambda x: x[2])
			x, y, r = max_circle.astype("int")

			center_x = x / frame.shape[1]
			center = (center_x - 0.5) * 2

			if not IS_DEBUG:
				servo.value = center

			# зеленую рамку вокруг обнаруженного объекта
			cv2.rectangle(frame, (x-r, y-r), (x+r, y+r), (0, 255, 0), 4)
		else:
			led_state = False

		cv2.imshow('frame', frame)
		
		sleep(0.1)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cap.release()
	cv2.destroyAllWindows()
	GPIO.cleanup()


# Запуск потока управления LED и цикла камеры как асинхронных задач
with ThreadPoolExecutor(max_workers=2) as executor:
    executor.submit(led_control)
    executor.submit(camera_loop)
