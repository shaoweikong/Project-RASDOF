################################################################################
# Copyright (C) 2012-2016 Leap Motion, Inc. All rights reserved.               #
# Leap Motion proprietary and confidential. Not for distribution.              #
# Use subject to the terms of the Leap Motion SDK Agreement available at       #
# https://developer.leapmotion.com/sdk_agreement, or another agreement         #
# between Leap Motion and you, your company or other organization.             #
################################################################################

'''
PLEASE NOTE:
This is the main file. We get the data from LEAP and process it accordingly before sending the
data to Arduino servos

This is a 5DOF arm. The 6th servo does not contribute to any DOF as it only effects the end effector claw mechanism
'''


import sys
sys.path.insert(0, "LeapLib/") # Not Required

import Leap, sys, thread, time
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture
from ArduinoController import * # Unused import serial and import threading in Arduino Class
from kinematics_calculations import * # Not used as of now


# This class is somehow multithreading/ multiprocessing?
class SampleListener(Leap.Listener):

    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']

    prevtime = time.time()
    currenttime = time.time()

    # On LEAP Motion Controller initialisation
    def on_init(self, controller):                          
        self.ac = ArduinoController()                                                       # Instantiate Arduino Controller class
        time.sleep(5)                                                                       # Wait for initialisation

        #Follow PWN digital pins on Arduino
        SERVO_PIN0 = 3                                                                      #
        SERVO_PIN1 = 5                                                                      #
        SERVO_PIN2 = 6                                                                      #
        SERVO_PIN3 = 9                                                                      #
        SERVO_PIN4 = 10                                                                     #
        SERVO_PIN5 = 11                                                                     # End effector Claw 
        #AZIMUTHAL_LIMIT = 180                                   # Motor1 Limit is between 0 and 180. This is a fail safe as normal human range cannot move beyond that.
        AZIMUTHAL_LIMIT = 180
        AZIMUTHAL_LIMIT = 180
        AZIMUTHAL_LIMIT = 180
        AZIMUTHAL_LIMIT = 180
        AZIMUTHAL_LIMIT = 180
        AZIMUTHAL_LIMIT = 180

        time.sleep(1)

        print 'Initialized'


    # When LEAP Motion Controller is connected
    def on_connect(self, controller):
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE)
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP)
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP)
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE)

        print 'Connected'


    # Note: not dispatched when running in a debugger.
    def on_disconnect(self, controller):                                                        # How to reconnect and reestablish connection?
        
        # These motors should be resetted to these default positions
        self.ac.servo_write(self.SERVO_PIN0,90) 
        self.ac.servo_write(self.SERVO_PIN1,90) 
        self.ac.servo_write(self.SERVO_PIN2,90) 
        self.ac.servo_write(self.SERVO_PIN3,90) 
        self.ac.servo_write(self.SERVO_PIN4,90) 
        self.ac.servo_write(self.SERVO_PIN5,90) 

        print 'Disconnected'


    # When user stops LEAP Motion Controller
    def on_exit(self, controller):

        time.sleep(1)

        # These motors should be resetted to these default positions
        self.ac.servo_write(self.SERVO_PIN0,90) 
        self.ac.servo_write(self.SERVO_PIN1,90) 
        self.ac.servo_write(self.SERVO_PIN2,90) 
        self.ac.servo_write(self.SERVO_PIN3,90) 
        self.ac.servo_write(self.SERVO_PIN4,90) 
        self.ac.servo_write(self.SERVO_PIN5,90) 
        self.ac.close()                                                                         # Close Arduino Connection properly

        print 'Exited'


    # Get frame data of controller, parse into inverse kinematics then write to Arduino
    def on_frame(self, controller):

        '''
        Instantiate important variables
        '''
        AZIMUTHAL_LIMIT = 180
        self.currenttime = time.time()                                                              # We determine the rate of data transfer using time.time because the default baud rate is too fast

        

        '''
        Determine what data you want from this block of code
        '''
        try:                                                                                        # We try the following block of code
            
            if self.currenttime-self.prevtime > 0.1:                                                # If computer time difference between times is 10ms
            
                frame = controller.frame()                                                          # Get the most recent frame and report some basic information
                interaction_box = frame.interaction_box


                print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d" % (
                    frame.id, 
                    frame.timestamp, 
                    len(frame.hands),                                                               # This is a list of Hand objects detected in this frame
                    len(frame.fingers))                                                             # This is a list of Finger objects detected in this frame

                
                ######################################################################################################################################
                # CALCULATIONS AND PROCESSING FROM HERE ONWARDS
                # Hand is a point, not direction vector

                '''
                [GET DATA] Getting the relevant data of hands and fingers
                '''
                for hand in frame.hands:                                                                # For each element (hand) in list of Hand objects


                    '''
                   # [HAND]  Get Cartesian Coordinates for Hand to move robot arm. This is for 1st, 2nd and 3rd servo
                    '''
                    handType = "Left hand" if hand.is_left else "Right hand"                            # Determines which hand
                    normalized_point = interaction_box.normalize_point(hand.palm_position,True)         # Instantiate normalised point (palm position) inside interaction box

                    self.XPOS = normalized_point.x
                    self.YPOS = normalized_point.y
                    self.ZPOS = normalized_point.z

                    print "  %s, id %d, position: %s" % (handType, hand.id, hand.palm_position)
                    print "  %s, id %d, x-position: %s" % (handType, hand.id, int(self.XPOS*180) )
                    print "  %s, id %d, y-position: %s" % (handType, hand.id, int(self.YPOS*85) )
                    print "  %s, id %d, z-position: %s" % (handType, hand.id, int(self.ZPOS*180) )

                    

                    '''
                   # [HAND]  Get RPY for 4th and 5th servo
                    '''
                    normal = hand.palm_normal                                                            # Get the hand's normal vector
                    direction = hand.direction                                                           # Get the hand's direction

                    self.ROLL = normal.roll * Leap.RAD_TO_DEG                                            # Roll of hand is in RAD. Convert to DEG
                    self.PITCH = direction.pitch * Leap.RAD_TO_DEG                                       # Pitch of hand is on RAD. Convert to DEG
                    #self.YAW, no yaw

                    # Calculate the hand's pitch, roll, and yaw angles
                    print "  pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (
                        direction.pitch * Leap.RAD_TO_DEG,
                        normal.roll * Leap.RAD_TO_DEG,
                        direction.yaw * Leap.RAD_TO_DEG)




                    '''
                    # [FINGERS] Get Cartesian Coordinates for Fingers to move end effector. This is for 6th servo.
                    '''
                    print 'my fingers =',len(hand.fingers)
                    if len(hand.fingers) >= 2:
                        x1 = hand.fingers[0].tip_position[0]                                            # hand.fingers[0] gives thumb. tip_position[0/1/2] gives x,y,z of thumb                                          
                        y1 = hand.fingers[0].tip_position[1]
                        z1 = hand.fingers[0].tip_position[2]
                        
                        x2 = hand.fingers[1].tip_position[0]                                            # hand.fingers[1] gives index finger. tip_position[0/1/2] gives x,y,z of index finger
                        y2 = hand.fingers[1].tip_position[1]
                        z2 = hand.fingers[1].tip_position[2]
                 
                        r = ( (x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2 )**0.5                               # calc 2norm for difference between vector to thumb and pointer finger

                        dist_norm = r/100.                                                              # perform a crude normalization, may need to change the 100 to something else

                        print 'Finger Tip Distance = ',dist_norm
                        
                
                if dist_norm >= 1:                                                                      # not really a normalized position, sometimes can be more than 1
                    dist_norm=1

                    # I MIGHT NEED RPY
                   ####################################################################################
                    # No plans for now ↓
                   ####################################################################################
                    '''
                    normal = hand.palm_normal                                                   # Get the hand's normal vector
                    direction = hand.direction                                                  # Get the hand's direction

                    # Calculate the hand's pitch, roll, and yaw angles
                    print "  pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (
                        direction.pitch * Leap.RAD_TO_DEG,
                        normal.roll * Leap.RAD_TO_DEG,
                        direction.yaw * Leap.RAD_TO_DEG)


                    # Get fingers
                    for finger in hand.fingers:

                        print "    %s finger, id: %d, length: %fmm, width: %fmm" % (
                            self.finger_names[finger.type],
                            finger.id,
                            finger.length,
                            finger.width)
                            
                        ##
                        The hand’s position is given by its palm position attribute, which provides a vector containing the 3-dimensional coordinates of the palm center point in millimeters from the Leap Motion origin. The hand’s orientation is given by two vectors: the direction, which points from the palm center towards the fingers, and the palm normal, which points out of the palm, perpendicular to the plane of the hand.

                        The movement of the hand is given by the velocity attribute, which is a vector providing the instantaneous motion of the hand in mm/s. You can also get motion factors that translate how a hand has moved between two given frames into translation, rotation, and scaling values.

                        The following code snippet illustrates how to get a Hand object from a frame and access its basic attributes:

                        Hand hand = frame.Hands[0];
                        Vector position = hand.PalmPosition;
                        Vector velocity = hand.PalmVelocity;
                        Vector direction = hand.Direction;


                        # Get finger bones. This is useful for a robot hand as an end effector as a future implementation
                        for b in range(0, 4):
                            bone = finger.bone(b)
                            print "      Bone: %s, start: %s, end: %s, direction: %s" % (
                                self.bone_names[bone.type],
                                bone.prev_joint,
                                bone.next_joint,
                                bone.direction)
                        '''

                    ####################################################################################
                    # No plans for now ^
                    ####################################################################################
                    

                '''
                [PROCESS DATA] Processing the data: Process through IK
                '''
                
                XPOS_servo = abs(145-self.XPOS*145) # 0-Azimuth
                YPOS_servo = abs(85-self.YPOS*85)   # 1-Altitude
                ZPOS_servo = 35+135*self.ZPOS # Wrist Angle
        



                '''
                [WRITE DATA] Writing the processed data to Arduino
                '''
                # Robot end effector position wrt Hand position
                self.a.servo_write(self.PIN2,int(XPOS_servo))                                          # Azimuth
                self.a.servo_write(self.PIN3,int(YPOS_servo))                                          # Altitude
                self.a.servo_write(self.PIN4,int(ZPOS_servo))                                          # Wrist

                # Robot claw end effector wrt distance between the thumb and index finger
                CLAW_SERVO = abs(90-dist_norm*70)
                print 'Claw Angle =',CLAW_SERVO
                self.a.servo_write(self.PIN5,int(CLAW_SERVO))

                # update the old time
                self.oldtime = self.newtime

                # update previous values
                self.previous_angles[0] = XPOS_servo
                self.previous_angles[1] = YPOS_servo
                self.previous_angles[2] = ZPOS_servo
                self.previous_angles[3] = CLAW_SERVO


            else:                                                                                      # If time difference is < 10ms
                pass                                                                                   # Do nothing and wait for 10ms to elapse before we collect the set set of data
       
        except:                                                                                        # For any exceptions, if there is a condition whereby the time difference is NOT > 10ms or < 10ms (i.e. No hands detected)
            print "No hands detected!"                                                                 # There is no hands detected. Print No hands detected.
            pass







'''
Main File Process

Do Not Touch. This runs the controller
'''

def main():

    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)



'''
Run this file
'''
if __name__ == "__main__":
    main()

