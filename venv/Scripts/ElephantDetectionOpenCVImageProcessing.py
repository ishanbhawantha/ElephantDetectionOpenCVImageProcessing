import numpy as np
import cv2
import time
import RPi.GPIO as GPIO
from subprocess import call
import pygame

GPIO.setmode(GPIO.BOARD)
pygame.mixer.init()
pygame.mixer.music.load("bees.mp3")


def diffImg(t0, t1, t2):
    d1 = cv2.absdiff(t2, t1)
    d2 = cv2.absdiff(t1, t0)
    return cv2.bitwise_and(d1, d2)


Light1 = 40
Light2 = 38
Warn = 36

GPIO.setup(Light1, GPIO.OUT)
GPIO.setup(Light2, GPIO.OUT)
GPIO.setup(Warn, GPIO.OUT)
cap = cv2.VideoCapture(0)
kernel1 = np.ones((3, 3), np.uint8)
kernel2 = np.ones((5, 5), np.uint8)
mode = 0
GPIO.output(Light1, False)
GPIO.output(Light2, False)
GPIO.output(Warn, False)

while (1):
    ret, frame1 = cap.read()
    ret, frame2 = cap.read()
    ret, frame3 = cap.read()

    frame1 = cv2.resize(frame1, (0, 0), fx=0.5, fy=0.5)
    frame2 = cv2.resize(frame2, (0, 0), fx=0.5, fy=0.5)
    frame3 = cv2.resize(frame3, (0, 0), fx=0.5, fy=0.5)

    frame1 = cv2.cvtColor(frame1, cv2.COLOR_RGB2GRAY)
    frame2 = cv2.cvtColor(frame2, cv2.COLOR_RGB2GRAY)
    frame3 = cv2.cvtColor(frame3, cv2.COLOR_RGB2GRAY)

    detector = diffImg(frame1, frame2, frame3)
    retval, detector = cv2.threshold(detector, 5, 255, cv2.THRESH_BINARY)

    detector = cv2.erode(detector, kernel1, iterations=1)
    detector = cv2.dilate(detector, kernel2, iterations=1)

    Tw = 0
    k = 0
    for i in range(detector.shape[0]):
        val = 0
        for j in range(detector.shape[1]):
            val = val + detector[i][j]

        if (val / 255.0) > 5:  # ============================
            Tw = Tw + 1

    k = (Tw / 320.0)
    print k

    if k > 0.35:  # =========================== if we increase we can take big elephants.
        ##print "Warrning!"
        GPIO.output(Light1, True)
        GPIO.output(Light2, True)
        GPIO.output(Warn, True)
        # call(["espeak","-s130 -ven+8 -Z", "Attention, attention. Defense system is activated. Please do not touch the fence."])

        start_time = time.time()
        mode = 1
        pygame.mixer.music.play()
        # pygame.mixer.music.stop()

    if mode == 1:
        stop_time = time.time()
        if (stop_time - start_time) >= 10:  # ===========================
            # call(["espeak","-s130 -ven+8 -Z", "Defense system is going to deactivated."])
            pygame.mixer.music.stop()
            mode = 0
    else:
        GPIO.output(Light1, False)
        GPIO.output(Light2, False)
        GPIO.output(Warn, False)

    cv2.imshow('frame', detector)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

GPIO.cleanup()
cap.release()
cv2.destroyAllWindows()

