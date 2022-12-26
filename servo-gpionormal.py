#servo setup
import RPi.GPIO as GPIO
from time import sleep
GPIO.setmode(GPIO.BOARD) #sets names to board mode, names pin according to nubers

GPIO.setup(3, GPIO.OUT) #output to send pwm signal on
pwm = GPIO.PWM(3, 50)
pwm.start(0)

from random import randint

def SetAngle(angle):
    duty = angle/ 18 + 2
    GPIO.output(3, True)        #turns on pin for output
    pwm.ChangeDutyCycle(duty)                                                                    
    sleep(0.3)
    GPIO.output(3, False)       #turns off pin
    pwm.ChangeDutyCycle(0)       #not continuously sending inputs to the servo


centre_x = randint(0,960)
cam_x = 480
smallest_value = cam_x - 10
largest_value = cam_x + 10
print(centre_x)

SetAngle(90)
a = 90

def main():
  global a
  global centre_x
  
  while centre_x > largest_value or centre_x < smallest_value:
      if centre_x > largest_value:
          a+=15
          print('motor turn left') #to turn left num increase
      elif centre_x < smallest_value:
          a-=15
          print('motor turn right')
      print(a)
      
      if a <= 0 or a >= 180:
          print('out of range')
          break #not sure if this works. could try setting center x to arbitrary working value
      else:
          SetAngle(a) #some issues when = 0, + will only turn once
          #centre_x = [update...]
          centre_x = randint(0,960) #update
          print(centre_x)
main()

pwm.stop()
GPIO.cleanup()
GPIO.setwarnings(False)
#may receive an error that says the selected GPIO channels are already in use if you don't have this line
#does not affect the project