import numpy as np

'''
# we need to remember that s3 is wrt to s2

#0 < s2 < 180
#0 < s3 < 160


l2 = 105.0
l3 = 125.0
nleapY = 0.3    # change this to try
nleapZ = 0.9   # change this to try
largest_Radius = l3 + l2
smallest_Radius = l3


def in_rangeCheck(unarmX,unarmY):
    largeradius_ofPoint = np.sqrt( (unarmX**2) + (unarmY**2) )
    smallradius_ofPoint = np.sqrt( (( unarmX - (-l2/(l2+l3)) )**2) + (unarmY**2) ) # center is shifted

    if largeradius_ofPoint > largest_Radius:
        theta = np.arctan(unarmY/unarmX)
        unarmX = largest_Radius * np.cos(theta)
        unarmY = largest_Radius * np.sin(theta)

    elif smallradius_ofPoint < smallest_Radius: 
        theta = np.arctan( unarmY/ ( unarmX - (-l2/(l2+l3)) ) )
        unarmX = (smallest_Radius * np.cos(theta)) + (-l2/(l2+l3)) # rmb to -105/230
        unarmY = smallest_Radius * np.sin(theta)
    
    else:
        unarmX = unarmX
        unarmY = unarmY

    return unarmX,unarmY


unarmY = nleapY * 230
#unarmX = (1-nleapZ) * 230
unarmX = nleapZ * 230
print("old unarmX: ", unarmX)
print("old unarmY: ", unarmY)


if unarmX == 0:
    unarmX = 0.000001

unarmX, unarmY = in_rangeCheck(unarmX, unarmY)
print("new unarmX: ", unarmX)
print("new unarmY: ", unarmY)




# We need to be careful with the accuracy of s3_Angle because it has downstream implications s2_Angle. s3_Angle has more restrictions than s2_Angle.
cosTheta3 =   round( ((unarmY**2) + (unarmX**2) - (l2**2) - (l3**2))   /  (2*(l2)*(l3)), 5)
print("cosTheta3: ", cosTheta3)

s3_radAngle = np.arccos( cosTheta3 )         
s3_Angle = np.degrees(s3_radAngle) * -1   
print("s3_Angle: ", s3_Angle)                                                       


s2_radAngle = np.arctan( (unarmY / unarmX) ) + np.arctan( ( (l3 * np.sin(s3_radAngle) ) / (l2 + l3 * np.cos(s3_radAngle) ) ) )
s2_Angle = np.degrees(s2_radAngle)
print("s2_Angle: ", s2_Angle)  


if s3_Angle > 170.0: 
    s3_Angle = 160.0

if s2_Angle > 180.0:
    s2_Angle = 180.0


print(s2_Angle, s3_Angle)

# we need to remember that s3 is wrt to s2
'''

###################################################################################################################################
###################################################################################################################################
###################################################################################################################################
###################################################################################################################################
###################################################################################################################################
###################################################################################################################################

'''
class InverseKinematics2D:


    def __init__(self):


        # Servo Max Angle
        self.S1maxAngle = 180                       # Servo 1 Max Angle
        self.S2maxAngle = 145                       # Servo 2 Max Angle
        self.S3maxAngle = 160                       # Servo 3 Max Angle
        self.S4maxAngle = 145                       # Servo 4 Max Angle
        self.S5maxAngle = 145                       # Servo 5 Max Angle
        self.S6maxAngle = 145                       # Servo 6 Max Angle



        # Parameters
        self.l2 = 105.0                             # mm, distance between motors
        self.l3 = 125.0                             # mm, length of near arm (left)
        #self.l4 = 150.0                            # mm, length of near arm (right)
        self.rtd = 180.0/np.pi                      # converts radians to degrees
        self.largest_Radius = self.l3 + self.l2
        self.smallest_Radius = self.l3


    def angle_for_armXY(self, nleapZ, nleapY): #theta 2 and 3
        

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


        # Unnormalise the leap values and convert to Robot Arm Axes Frame
        unarmY = nleapY * (self.l2 + self.l3)
        #unarmX = (1-nleapZ) * (self.l2 + self.l3)
        unarmX = nleapZ * (self.l2 + self.l3)

        if unarmX == 0:
            unarmX = 0.000001

        unarmX, unarmY = in_rangeCheck(unarmX, unarmY)



        # Convert to Robot Arm Axes Frame
        cosTheta3 = ( (unarmY**2) + (unarmX**2) - (self.l2**2) - (self.l3**2) ) / ( (2)*(self.l2)*(self.l3) )       
        s3_radAngle = np.arccos( cosTheta3 )                   
        s3_Angle = np.degrees(s3_radAngle) * -1   
                                          

        s2_radAngle = np.arctan( (unarmY / unarmX) ) + np.arctan( ( (self.l3 * np.sin(s3_radAngle) ) / (self.l2 + self.l3 * np.cos(s3_radAngle) ) ) )
        s2_Angle = np.degrees(s2_radAngle)


        if s2_Angle > 180.0:
            s2_Angle = 180.0

        if s3_Angle > 170.0: # 3sf or 3dp needed
            s3_Angle = 160.0



        return s2_Angle, s3_Angle




if __name__ == "__main__":
    ik2D = InverseKinematics2D()

    s2_Angle, s3_Angle = ik2D.angle_for_armXY(0, 0) 
    print(s2_Angle, s3_Angle) # This is not possible

'''
x = 0.0
y = 0.0
prevXYCoord = [0,0]

if x == 0 and y == 0:
    print 'this works'
else:
    print 'failed'

prevXYCoord[0] = 1
print prevXYCoord

