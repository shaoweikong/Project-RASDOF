################################################################################
# Copyright (C) 2012-2016 Leap Motion, Inc. All rights reserved.               #
# Leap Motion proprietary and confidential. Not for distribution.              #
# Use subject to the terms of the Leap Motion SDK Agreement available at       #
# https://developer.leapmotion.com/sdk_agreement, or another agreement         #
# between Leap Motion and you, your company or other organization.             #
################################################################################

'''
PLEASE NOTE:
1) This is the main file. We get the data from LEAP and process it accordingly before sending the
   data to Arduino servos

2) File is able to connect to LEAP caa 13/4/20
3) It is very hard for Y to reach 0
4) Make sure script is able to write to Arduino

'''


import sys
#sys.path.insert(0, "LeapLib/")                                                               # Not Required since all the required files are in the same folder
import numpy as np
import Leap, sys, thread, time
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture
from ArduinoController import ArduinoController                                              # Import ArduinoController Class
from kinematics_calculations import InverseKinematics2D                                      # Import only InverseKinematics2D Class



class SampleListener(Leap.Listener):

    prevtime = time.time()
    currenttime = time.time()

    # On LEAP Motion Controller initialisation
    def on_init(self, controller):                          

        self.ac = ArduinoController()                                                         # Instantiate Arduino Controller class
        self.ik2D = InverseKinematics2D()
        time.sleep(5)                                                                         # Wait for initialisation

        #Follow PWM digital pins on Arduino, change pin
        self.SERVO_1 = 1                                                                      # X Axis
        self.SERVO_2 = 2                                                                      # Y and Z Axis
        self.SERVO_3 = 3                                                                      # Y and Z Axis
        self.SERVO_4 = 4                                                                      # Pitch
        self.SERVO_5 = 5                                                                      # Roll
        self.SERVO_6 = 6                                                                      # End effector Claw 
                                                                                           
        # Parameters
        self.l1 = 70.0                                                                        # mm, distance from base to top of servo 1
        self.l2 = 105.0                                                                       # mm, distance between servo 2 and servo 3
        self.l3 = 125.0                                                                       # mm, distance between servo 3 and servo 4
        #self.l4 = 150.0                                                                      # mm, distance between servo 4 and center of end effector
        self.rtd = 180.0/np.pi                                                                # converts radians to degrees
        #self.home = (0.0, 99)                                                                # coordinates of home position in mm
        self.prevXYCoord = [0.5,0.5]                                                          # Replace this with home coordinates for XY
        self.stored_nonZeroXY = [0.1,0.1]                                                     # Random Values
        
        # Serovo Min Limit
        self.SERVO1_MINLIMIT = 0
        self.SERVO2_MINLIMIT = 0
        self.SERVO3_MINLIMIT = 0
        self.SERVO4_MINLIMIT = 0
        self.SERVO5_MINLIMIT = 0
        self.SERVO6_MINLIMIT = 0

        # Servo Max Limit
        self.SERVO1_MAXLIMIT = 180
        self.SERVO2_MAXLIMIT = 145
        self.SERVO3_MAXLIMIT = 145
        self.SERVO4_MAXLIMIT = 145
        self.SERVO5_MAXLIMIT = 145
        self.SERVO6_MAXLIMIT = 145

        time.sleep(1)

        print '  Initialized'


    # When LEAP Motion Controller is connected
    def on_connect(self, controller):
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE)
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP)
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP)
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE)

        print '  Connected'


    # Note: not dispatched when running in a debugger.
    def on_disconnect(self, controller):                                                              # Write Default Home Position Here
        
        # These motors should be resetted to these default positions
        self.ac.servo_write(self.SERVO_1,90) 
        self.ac.servo_write(self.SERVO_2,30) 
        self.ac.servo_write(self.SERVO_3,200) 
        self.ac.servo_write(self.SERVO_4,90) 
        self.ac.servo_write(self.SERVO_5,0) 
        self.ac.servo_write(self.SERVO_6,90) 

        print '  Disconnected'


    # When user stops LEAP Motion Controller
    def on_exit(self, controller):

        time.sleep(1)
        print '  Returning to Home.'
        # These motors should be resetted to these default positions
        self.ac.servo_write(self.SERVO_1,90) 
        self.ac.servo_write(self.SERVO_2,180) 
        self.ac.servo_write(self.SERVO_3,160) 
        self.ac.servo_write(self.SERVO_4,10) 
        self.ac.servo_write(self.SERVO_5,0) 
        self.ac.servo_write(self.SERVO_6,0)     
        self.ac.close()                                                                                # Close Arduino Connection properly

        print '  Exited'


    # Get frame data of controller, parse into inverse kinematics then write to Arduino
    def on_frame(self, controller):
         
        '''
        Instantiate important variables
        '''
        
        self.currenttime = time.time()                                                                  # We determine the rate of data transfer using time.time because the default baud rate is too fast
        
        

        '''
        Determine what data you want from this block of code
        '''
        try:                                                                                            # We try the following block of code
            
            if self.currenttime-self.prevtime > 0.1:                                                    # If computer time difference between times is 10ms
            
                frame = controller.frame()                                                              # Get the most recent frame and report some basic information
                interaction_box = frame.interaction_box


                print '\n  ############'
                print '  # RAW DATA #'
                print '  ############'
                print "  Frame id: %d, timestamp: %d, hands: %d, fingers: %d" % (
                    frame.id, 
                    frame.timestamp, 
                    len(frame.hands),                                                                   # This is a list of Hand objects detected in this frame
                    len(frame.fingers))                                                                 # This is a list of Finger objects detected in this frame

                
                ######################################################################################################################################
                # GET DATA HERE
                ######################################################################################################################################
                # Hand is a point, not direction vector

                '''
                # [GET DATA] Getting the relevant data of hands and fingers
                '''
                for hand in frame.hands:                                                                # For each element (hand) in list of Hand objects


                    '''
                   # 1) [HAND]  Get Cartesian Coordinates for Hand to move robot arm. This is for 1st, 2nd and 3rd servo. Return self.XPOS, self.YPOS, self.ZPOS
                    '''
                    handType = "Left hand" if hand.is_left else "Right hand"                            # Determines which hand
                    normalized_point = interaction_box.normalize_point(hand.palm_position,True)         # Instantiate normalised point (palm position) inside interaction box

                    self.leapX = normalized_point.x                                                     
                    self.leapY = normalized_point.y                                                     # self.leapY is denormalise in ik2D
                    self.leapZ = 1 - normalized_point.z                                                 # self.leapZ is denormalise in ik2D

                    
                    print "  %s, id %d, position: %s" % (handType, hand.id, hand.palm_position)         # I do not know how to map these coordinates into coordinates that the arm can use
                    print "  %s, id %d, x-position: %s" % (handType, hand.id, self.leapX )              # Prints the motor angle range for x axis. Why multiply 180?
                    print "  %s, id %d, y-position: %s" % (handType, hand.id, self.leapY )
                    print "  %s, id %d, z-position: %s" % (handType, hand.id, self.leapZ )



                    '''
                   # 2) [HAND]  Get RPY for 4th and 5th servo. Return self.ROLL and self.PITCH
                    '''
                    normal = hand.palm_normal                                                           # Get the hand's normal vector
                    direction = hand.direction                                                          # Get the hand's direction

                    self.ROLL = normal.roll * Leap.RAD_TO_DEG                                           # Roll of hand is in RAD. Convert to DEG
                    self.PITCH = direction.pitch * Leap.RAD_TO_DEG                                      # Pitch of hand is in RAD. Convert to DEG
                                                                                                        # No yaw

                    # Calculate the hand's pitch, roll, and yaw angles
                    print "  pitch: %f degrees, roll: %f degrees" % (
                        direction.pitch * Leap.RAD_TO_DEG,
                        normal.roll * Leap.RAD_TO_DEG,)



                    '''
                    # 3) [FINGERS] Get Cartesian Coordinates for Fingers to move end effector for 6th servo. Return dist_norm
                    '''
                    print '  Number of fingers detected =',len(hand.fingers)
                    x1 = hand.fingers[0].tip_position[0]                                                # hand.fingers[0] gives thumb. tip_position[0/1/2] gives x,y,z of thumb                                          
                    y1 = hand.fingers[0].tip_position[1]
                    z1 = hand.fingers[0].tip_position[2]
                    
                    x2 = hand.fingers[1].tip_position[0]                                                # hand.fingers[1] gives index finger. tip_position[0/1/2] gives x,y,z of index finger
                    y2 = hand.fingers[1].tip_position[1]
                    z2 = hand.fingers[1].tip_position[2]
                
                    r = ( (x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2 )**0.5                                   # calc 2norm for difference between vector to thumb and pointer finger

                    dist_norm = r/100.                                                                  # perform a crude normalization, may need to change the 100 to something else
                    print '  Finger Tip Distance = ', dist_norm
                    
                
                    ######################################################################################################################################
                    # PROCESS DATA HERE
                    ######################################################################################################################################
                    '''
                    [CALCULATE DATA] Processing the data: Process through IK
                    '''
                    #ik2D = InverseKinematics2D()                                                        # Instantiate InverseKinematics2D class

                    s1_Angle = self.ik2D.angle_for_armZ(self.leapX)
                 
                    #s2_Angle, s3_Angle, stored_updatednonZeroXY = self.ik2D.angle_for_armXY(self.leapZ, self.leapY, self.prevXYCoord, self.stored_nonZeroXY)   # Rechange the input arguments, definition and output
                    s2_Angle, s3_Angle = self.ik2D.angle_for_armXY(self.leapZ, self.leapY)   
                    s3_Angle = s3_Angle * -1                                                             # Changed to positive value. To compensate, we changed the way the servo motor rotates

                    s4_Angle = self.ik2D.angle_for_armPitch(self.PITCH)                                                
                    s5_Angle = self.ik2D.angle_for_armRoll(self.ROLL)
                    s6_Angle = self.ik2D.claw_Angle(dist_norm)                                                         


                    print '\n  ##################'
                    print '  # PROCESSED DATA #'
                    print '  ##################'
                    #print '  Servo 1 Angle = ', int(s1_Angle) #okay
                    #print '  Servo 2 Angle = ', s2_Angle #okay
                    #print '  Servo 3 Angle = ', s3_Angle #okay
                    #print '  Servo 4 Angle = ', s4_Angle #okay
                    #print '  Servo 5 Angle = ', s5_Angle #okay
                    #print '  Servo 6 Angle = ', s6_Angle #okay


                    '''
                    [WRITE DATA] Writing the processed data to Arduino
                    '''

                    # We need to remember that s3 is wrt to s2

                    # Robot end effector position wrt Hand position
                    self.ac.servo_write(self.SERVO_1, int(s1_Angle) )                                       # Uncheck once Arduino handshake is done
                    self.ac.servo_write(self.SERVO_2, int(s2_Angle) )                                       # Uncheck once Arduino handshake is done                                      
                    self.ac.servo_write(self.SERVO_3, int(s3_Angle) )                                       # Uncheck once Arduino handshake is done                                      
                    self.ac.servo_write(self.SERVO_4, int(s4_Angle) )                                       # Uncheck once Arduino handshake is done                                     
                    self.ac.servo_write(self.SERVO_5, int(s5_Angle) )                                       # Uncheck once Arduino handshake is done                                     
                    self.ac.servo_write(self.SERVO_6, int(s6_Angle) )                                       # Uncheck once Arduino handshake is done                                     
                    
                    print '\n  Data Written.'
                    print '\n  Done.'

                    # Update the old time
                    self.oldtime = self.newtime

                    # Update previous coordinates
                    # self.prevXYCoord[0] = self.leapX
                    # self.prevXYCoord[1] = self.leapY
                    # self.prevXYCoord[2] = self.leapZ
                    #self.stored_nonZeroXY = stored_updatednonZeroXY
                    

            else:                                                                                      # If time difference is < 10ms
                pass                                                                                   # Do nothing and wait for 10ms to elapse before we collect the set set of data
       
        except:                                                                                        # For any exceptions, if there is a condition whereby the time difference is NOT > 10ms or < 10ms (i.e. No hands detected)
            print ""                                                                                   # There is no hands detected. Print No hands detected.
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

