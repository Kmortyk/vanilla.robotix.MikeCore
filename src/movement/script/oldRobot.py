#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from time import sleep          # this lets us have a time delay
import OPi.GPIO as GPIO

# 11 - left forward 13 -right forward, 7 left backward, 15 right backward

GPIO.setboard(GPIO.PCPCPLUS)    # Orange Pi PC board
GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)        # set up BOARD BCM numbering

GPIO.setup(35, GPIO.OUT)
GPIO.output(35, 0)

GPIO.setup(37, GPIO.OUT)
GPIO.output(37, 0)

GPIO.setup(33, GPIO.OUT)
GPIO.output(33, 0)

GPIO.setup(31, GPIO.OUT)
GPIO.output(31, 0)

def move():
    GPIO.output(37, 1)
    GPIO.output(33, 1)

def stop():
    GPIO.output(35, 0)
    GPIO.output(37, 0)
    GPIO.output(33, 0)
    GPIO.output(31, 0)


def back():
    GPIO.output(35, 1)
    GPIO.output(31, 1)

def go_out_on_tupik(msg):
    step=0
    stop()
    for i in range(181, 424):
        if(round(msg.ranges[i],2)>0 and round(msg.ranges[i],2)<0.45):
            back()


    flag=0
    while(step<10):
        print('Attention we hit a dead end!!!! Oh noooo!My GOD ')
        step+=1
    while(flag!=1):
        flag=0
        for i in range(181, 303):
            if(round(msg.ranges[i],2)>0 and round(msg.ranges[i],2)<0.45):
                flag=1
                stop()
                print ('Left1')
                left()
                global tupik
                tupik+=1
            #print round(msg.ranges[i],2)

        for i in range(304,424):
            if(round(msg.ranges[i],2)>0 and round(msg.ranges[i],2)<0.45):
                flag=1
                stop()
                print ('Right1')
                right()
                global tupik
                tupik+=1
            #print round(msg.ranges[i],2)
        if(flag==0):
            flag=1
#stop()
#left()

def left():
    GPIO.output(37, 1)
    GPIO.output(31, 1)

def right():
    GPIO.output(33, 1)
    GPIO.output(35, 1)

tupik=0

def callback(msg):

    flag=0

    for i in range(181, 222):
        if(round(msg.ranges[i],2)>0 and round(msg.ranges[i],2)<0.23):
            flag=1
            stop()
            print ('Left')
            left()
            global tupik
            tupik+=1
        #print round(msg.ranges[i],2)
    for i in range(223, 261):
        if(round(msg.ranges[i],2)>0 and round(msg.ranges[i],2)<0.30):
            flag=1
            stop()
            print ('Left')
            left()
            global tupik
            tupik+=1
        #print round(msg.ranges[i],2)
    for i in range(262, 336):
        if(round(msg.ranges[i],2)>0 and round(msg.ranges[i],2)<0.33):
            flag=1
            stop()
            print ('Left')
            left()
            global tupik
            tupik+=1
        #print round(msg.ranges[i],2)
    for i in range(337, 375):
        if(round(msg.ranges[i],2)>0 and round(msg.ranges[i],2)<0.30):
            flag=2
            stop()
            global tupik
            tupik+=1
            print ('Right')
            right()
        #print round(msg.ranges[i],2)
    for i in range(376, 424):
        if(round(msg.ranges[i],2)>0 and round(msg.ranges[i],2)<0.23):
            flag=2
            stop()
            global tupik
            tupik+=1
            print ('Right')
            right()
        #print round(msg.ranges[i],2)

    if(flag==0):
        stop()
        print ('Move')
        move()
        tupik=0
    if(tupik>=400):
        print (tupik)
        go_out_on_tupik(msg)





    '''
    print 'values at 0 degree:'
    print msg.ranges[0]
    print 'values at 90 degree:'
    print msg.ranges[152]
    print 'values at 180 degree:'
    print msg.ranges[303]
    print 'values at 270 degree:'
    print msg.ranges[454]
    '''

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
