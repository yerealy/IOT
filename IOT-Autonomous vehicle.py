'''
1. importing necessary modules
'''
import cv2
from adafruit_servokit import ServoKit
from jd_deep_lane_detect import JdDeepLaneDetect
from jd_car_motor_l9110 import JdCarMotorL9110
import time

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0
        
    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output 

'''
2. Creating object from classes
  1) Servo handling object from ServoKit class 
  2) Deep learning lane detecting object from JdDeepLaneDetect class 
  3) DC motor handling object form JdCarMotorL9110 class 
'''
# Deep learning detector object
deep_detector = JdDeepLaneDetect("/home/pi/deepThinkCar/models/lane_navigation_final.h5")
# DC motor object
motor = JdCarMotorL9110()
# Servo object 
servo = ServoKit(channels=16)

# PID controller for steering
pid = PID(Kp=0.1, Ki=0.01, Kd=0.05)

'''
3. Creating camera object and setting resolution of camera image
cv2.VideoCapture() function create camera object.  
'''
# Camera object: reading image from camera 
cap = cv2.VideoCapture(0)
# Setting camera resolution as 320x240
cap.set(3, 320)
cap.set(4, 240)

'''
4. Preparing deepThinkCar starting.
Before start driving, we need to adjust servo offset(wheel calibraton) 
and wheel control while motor stop.
It will prevents mis-driving of deepThinkCar. 
'''
# Servo offset. You can get offset from calibration.py
servo_offset = 0
servo.servo[0].angle = 90 + servo_offset

# Prepare real starting 
for i in range(30):
    ret, img_org = cap.read()
    if ret:
        angle_deep, img_angle = deep_detector.follow_lane(img_org)
        if img_angle is None:
            print("can't find lane...")
        else:
            print(angle_deep)
            if angle_deep > 10 and angle_deep < 170:
                servo.servo[0].angle = angle_deep + servo_offset   
                  
            cv2.imshow("img_angle", img_angle)
            cv2.waitKey(1)
    else:
        print("cap error")

'''
5. Starting motor before real driving 
'''
# Start motor 
motor.motor_move_forward(5)

'''
6. Perform real driving1qa
In this part, we perform real OpenCV lane detecting driving.
When you press 'q' key, it stp deepThinkCar.
While on driving, driving is recorded. 
'''
# real driving routine
previous_time = time.time()

while cap.isOpened():
    ret, img_org = cap.read()
    current_time = time.time()
    dt = current_time - previous_time
    previous_time = current_time
    
    if ret:
    # Find lane angle
        angle_deep, img_angle = deep_detector.follow_lane(img_org)
        if img_angle is None:
                print("can't find lane...")
        else:
            print(angle_deep)
            if angle_deep >= 20 and angle_deep <= 160:
                error = angle_deep - (90 + servo_offset)
                control = pid.update(error, dt)
                control_angle = 90 + servo_offset + control
                control_angle = max(0, min(180, control_angle)
                servo.servo[0].angle = control_angle
            
            cv2.imshow("img_angle", img_angle)
    else:
        print("cap error")
                                
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
'''
7. Finishing the driving
Releasing occupied resources
'''   
motor.motor_stop()
cap.release()
cv2.destroyAllWindows()