'''
PLEASE NOTE:
There are two classes
1) Inverse Kinematics 6 DOF: Actual 6 DOF Inverse Kinematics
2) Inverse Kinematics 2D: 2D Inverse Kinematics for Servo 2 and 3

* We need to account for singularities and how to overcome it. One way to restrict the decimal points
* It is important to note the starting 0 angle for our servos
* Servo 3 angle is wrt to servo 2
* We need to account for leap values from 0 to 1. What if the values are out of range outside and internal? Out of range: Find radius then assume theta 3 is 0. Find theta2
* We need to account for negative angle, clockwise rotation.
'''

import numpy as np
import math



class InverseKinematics6DOF:

    def __init__(self):

        pass
    pass


###################################################
# 2D 3DOF Robot Arm Kinematics
###################################################


class ForwardKinematics2D3DOF:

    pass


class InverseKinematics2D:
    '''
    We want to find 
    1) theta 1 for the XZ plane
    2) theta 2, 3 for the XY plane
    3) theta 4 for pitch
    4) theta 5 for roll
    '''

    def __init__(self):
        '''
        Initialization of any class attributes that has to be used across different methods. This is automatically called
        ''' 

        # Servo Max Angle
        self.S1maxAngle = 180                       # Servo 1 Max Angle
        self.S2maxAngle = 145                       # Servo 2 Max Angle
        self.S3maxAngle = 160                       # Servo 3 Max Angle

        # Parameters
        self.l2 = 105.0                             # mm, distance between motors
        self.l3 = 125.0                             # mm, length of near arm (left)
        #self.l4 = 150.0                            # mm, length of near arm (right)
        self.rtd = 180.0/np.pi                      # converts radians to degrees
        self.largest_Radius = self.l3 + self.l2
        self.smallest_Radius = self.l3
        
        


    def angle_for_armZ(self, nleapX): #theta 1
        '''
        Robot Arm Axes for reference

                   ^ X
                   |    
                   |    .   (Robot Arm Top Profile)
                   |   /
                   |  /
                   | /
            Servo1 |/ theta1
        -----------Y-------------> Z

        where

        Robot Arm Y = LEAP Y
        Robot Arm X = LEAP -Z
        Robot Arm Z = LEAP X
        '''

        # Convert to Robot Arm Axes Frame
        narmZ = nleapX
        s1_Angle = abs( self.S1maxAngle - (narmZ * self.S1maxAngle) )                                                          # In degrees

        return s1_Angle



    #def angle_for_armXY(self, nleapZ, nleapY, prevXYCoord, stored_nonZeroXY): #theta 2 and 3
    def angle_for_armXY(self, nleapZ, nleapY): #theta 2 and 3
        # Zero Check is currently not working
        '''
        Robot Arm Axes for reference

                ^ Y
                |   Servo3  
                |    ________           (Robot Arm Side Profile)
                |   /_| theta3
                |  /
                | /
        Servo2  |/| theta2
                Z-------------> X

        where

        Robot Arm Y = LEAP Y
        Robot Arm X = LEAP -Z
        Robot Arm Z = LEAP X
        '''

        def in_rangeCheck(unarmX,unarmY):
            largeradius_ofPoint = np.sqrt( (unarmX**2) + (unarmY**2) )
            smallradius_ofPoint = np.sqrt( (( unarmX - (-self.l2/(self.l2+self.l3)) )**2) + (unarmY**2) ) # center is shifted

            if largeradius_ofPoint > self.largest_Radius:
                theta = np.arctan(unarmY/unarmX)
                unarmX = self.largest_Radius * np.cos(theta)
                unarmY = self.largest_Radius * np.sin(theta)

            elif smallradius_ofPoint < self.smallest_Radius: 
                theta = np.arctan( unarmY/ ( unarmX - (-self.l2/(self.l2 + self.l3)) ) )
                unarmX = (self.smallest_Radius * np.cos(theta)) + (-self.l2/(self.l2 + self.l3)) # rmb to -105/230
                unarmY = self.smallest_Radius * np.sin(theta)
            
            else:
                unarmX = unarmX
                unarmY = unarmY

            return unarmX,unarmY
        

        def zeroCheck(unarmX,unarmY, prevXYCoord, stored_nonZeroXY):
            if unarmX == 0:
                unarmX = 0.000001
            elif unarmX == 0 & unarmY == 0:
                if prevXYCoord[0] == 0 & prevXYCoord[1] == 0: # If user hands have been in zero state for quite some time
                    unarmX = stored_nonZeroXY[0]              # Use its last know non zero coordinates
                    unarmY = stored_nonZeroXY[1]
                elif prevXYCoord[0] != 0 & prevXYCoord[1] != 0: # If user hand just entered zero state
                    # take previous angles
                    unarmX = prevXYCoord[0]                     # Take the previous non zero coordinates
                    unarmY = prevXYCoord[1]
                    stored_nonZeroXY[0] = prevXYCoord[0]        # Keep the non zero coord for future use in case of if condition
                    stored_nonZeroXY[1] = prevXYCoord[1]
            else:
                unarmX = unarmX
                unarmY = unarmY
            
            return unarmX, unarmY, stored_nonZeroXY



        # Unnormalise the leap values and convert to Robot Arm Axes Frame
        unarmY = nleapY * (self.l2 + self.l3)
        unarmX = nleapZ * (self.l2 + self.l3)      # Uncomment for a fake test
        if unarmX == 0:
                unarmX = 0.000001 #delete this if script does not work
        #unarmX = (1-nleapZ) * (self.l2 + self.l3) 
        print '  old unarmY: ', unarmY
        print '  old unarmX: ', unarmX
        


       # Do a zero check. Unused since Y can never reach 0.
        #unarmX, unarmY, stored_updatednonZeroXY = zeroCheck(unarmX,unarmY, prevXYCoord, stored_nonZeroXY)

        #Do an in range check
        unarmX, unarmY = in_rangeCheck(unarmX, unarmY)
        print '  new unarmY: ', unarmY
        print '  new unarmX: ', unarmX


        # Convert to Robot Arm Axes Frame
        cosTheta3 = round( ( (unarmY**2) + (unarmX**2) - (self.l2**2) - (self.l3**2) ) / ( (2)*(self.l2)*(self.l3) ), 5)       
        print cosTheta3

        s3_radAngle = np.arccos( cosTheta3 )                   
        s3_Angle = np.degrees(s3_radAngle) * -1   
                                          

        s2_radAngle = np.arctan( (unarmY / unarmX) ) + np.arctan( ( (self.l3 * np.sin(s3_radAngle) ) / (self.l2 + self.l3 * np.cos(s3_radAngle) ) ) )
        s2_Angle = np.degrees(s2_radAngle)


        if s2_Angle > 180.0:
            s2_Angle = 180.0

        if s3_Angle > 170.0: # 3sf or 3dp needed
            s3_Angle = 160.0

        #return s2_Angle, s3_Angle, stored_updatednonZeroXY
        return s2_Angle, s3_Angle


    def angle_for_armPitch(self, leapPitch):                                                                # Theta 4 for Servo 4
        s4_Angle = leapPitch
        s4_Angle = s4_Angle + 90        # This is due to the nature of the servo motor angle of rotation (0 to 180). The servo cannot handle negative angles
        return s4_Angle


    def angle_for_armRoll(self, leapRoll):                                                                  # Theta 5 for Servo 5
        s5_Angle = abs(leapRoll)
        return s5_Angle


    def claw_Angle(self,dist_norm):
        s6_Angle = abs(90-dist_norm*70)     # moves only 70, where 0 or 70 is open

        return s6_Angle
        
        
class InverseKinematics2DHome:
    #FIll this up once the angles have been sorted out
    pass



if __name__ == "__main__":
    ik2D = InverseKinematics2D()

    # Servo 1 Test Case
    #s1_Angle = ik2D.angle_for_armZ(162.634) 
    #print(s1_Angle)

    
    # Servo 2 and 3 Test Case. This function accepts normalised values only. Inputs are (leapZ, leapY) or (armX, armY)
    
    # Test 1: X, Y 
    s2_Angle, s3_Angle = ik2D.angle_for_armXY(0.4, 0.6) 
    print(s2_Angle, s3_Angle) 
    



