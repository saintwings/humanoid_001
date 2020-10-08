import numpy as np
import cv2
import subprocess
import imutils

class ObjectDetection():
    def __init__(self, camera_comport, screen_size, robot_state):

        self.camera_comport = camera_comport
        self.screen_size = screen_size
        self.object_position_x = None
        self.object_position_y = None
        self.robot_state = robot_state

        

    def test_camera(self):
        cap = cv2.VideoCapture(self.camera_comport)
        cap.set(3, self.screen_size[0])
        cap.set(4, self.screen_size[1])
        subprocess.call(["v4l2-ctl", "-c", "focus_auto=0"]) ##trun off auto focus##

        while(True):
            # Capture frame-by-frame
            ret, frame = cap.read()


            # Display the resulting frame
            cv2.imshow('frame',frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()

    def color_tracking(self):
        greenLower = (29, 86, 6)
        greenUpper = (64, 255, 255)

        self.cap = cv2.VideoCapture(self.camera_comport)
        self.cap.set(3, self.screen_size[0])
        self.cap.set(4, self.screen_size[1])
        subprocess.call(["v4l2-ctl", "-c", "focus_auto=0"]) ##trun off auto focus##
        subprocess.call(["v4l2-ctl", "-c", "white_balance_temperature_auto=0"]) ##trun off auto white_balance##


        while True:
            ret, frame = self.cap.read()

            blurred = cv2.GaussianBlur(frame, (11,11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, greenLower, greenUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            cv2.imshow('mask',mask)
            cv2.imshow('frame',frame)

            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None

            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                if radius > 50:
                    cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
                    self.object_position_x = int(x)
                    self.object_position_y = int(y)
                else:
                    self.object_position_x = None
                    self.object_position_y = None

            else:
                self.object_position_x = None
                self.object_position_y = None

            cv2.imshow('frame',frame)

            #### report output to server ####
            self.robot_state[2][0] = self.object_position_x
            self.robot_state[2][1] = self.object_position_y
            #### report output to server ####

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything done, release the capture
        self.cap.release()
        cv2.destroyAllWindows()

    def disconnect(self):
        self.cap.release()
        cv2.destroyAllWindows()