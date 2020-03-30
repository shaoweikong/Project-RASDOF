'''
PLEASE NOTE:
Palm Coordinates give the end effector cartesian coordinates
RPY of palm gives end effector angle of rotation
'''

import numpy as np
import math

class CoordinateCalculator:

    def __init__(self):
        '''
        Initialization of any class attributes that has to be used across different methods. This is automatically called
        ''' 

        self.l0 = 60.0 #mm, distance between motors
        self.l11 = 43.0 #mm, length of near arm (left)
        self.l12 = 60.0 #mm, length of near arm (right)
        self.l21 = 95.0 #mm, length of far arm (left)
        self.l22 = 68.0 #mm, length of far arm (right) without extension
        self.l3 = 30.0 #mm, length of extension of far arm (right)
        self.rtd = 180.0/np.pi #converts radians to degrees
        self.range = 50.0 #mm, must be less than l1
        self.home = (0.0, 99) #coordinates of home position in mm
        self.zero = 80.0/self.rtd #radians, angle of the calibration limit. t1: (82.1352, 191.5292), t2: (89.1188, 201.7870)
        self.h1 = 80 #mm, change if necessary

        self.CT_origin_coords = np.array( [ 0, 0, 0 ] )

        self.extrapolationMagnitude = 1 # magnitude to extrapolate costmapVector by

        
        pass


    # Find robot origin, XYZ vector, robot base plane normal, point on robot base plane
    def ComputeRobotParameters(self, points, OMdist = 40, height = 73.9):
        """
        Given three coordinates as input A,B,C where AB = 40, BC = 80 and AC = 100 (A is furthest point, B is right and C is left), compute the robot origin and the normal to the plane 
        OMdist = distance between midpoint of BC and origin in mm
        height = vertical distance between plane of balls and needle holder in mm
        """
        A = np.array(list(points[0]))
        B = np.array(list(points[1]))
        C = np.array(list(points[2]))

        # Get vector normal to CB and CA
        CB = B - C
        CA = A - C
        normal = np.cross(CB, CA)
        normalUnit = normal/np.linalg.norm(normal)

        BCMid = (B + C)/2                   # midpoint of points B and C
        OM = np.cross(CB, normal)           # vector between midpoint of B and C and Origin (incorrect magnitude)
        xUnit = CB/np.linalg.norm(CB)       # unit vector CB is x axis
        yUnit = OM/np.linalg.norm(OM)       # unit vector OM is y axis
        zUnit = normal/np.linalg.norm(normal)
        origin = np.array( tuple(BCMid - OMdist*yUnit + height*zUnit) )
        xUnit  = np.array( tuple(xUnit) )
        yUnit  = np.array( tuple(yUnit) )
        zUnit  = np.array( tuple(zUnit) )

        # output is (3,) array
        return origin, xUnit, yUnit, zUnit, normalUnit, A


    # Find transformation matrix
    def getTransformationMatrix(self, robot_origin_coords, rxUnit, ryUnit, rzUnit):


        # Get translation vector from CT scan image coordinate origin to robot coordinate origin wrt to global frame
        # i.e robot origin = [8,8,8], robot x unit vector is [0.5,0.5,0.5], then robot x unit point( henceforth called point A) is [8.5,8.5,8.5]. 
        # We translate the robot origin points and point A to the CT image frame --> [0,0,0] and [0.5,0.5,0.5] respectively. Robot x unit vector is still [0.5,0.5,0.5]
        translationMatrix = np.array( np.subtract(robot_origin_coords,self.CT_origin_coords) ) # 1x3 matrix
        translationMatrix = np.array( np.transpose( [translationMatrix] ) )

        
        # Get Euler Angle in the form of thetaX, thetaY and thetaZ. 
        '''
        Tails- Bryan Angles is RxRyRz. Since we the robot's unit vectors coordinates,
        
        Default Transformation Matrix => RxRyRz
        | cos(thetaY)cos(thetaZ)                                        -cos(thetaY)sin(thetaZ)                                         sin(thetaY)                 |  
        | sin(thetaX)sin(thetaY)cos(thetaZ) + cos(thetaX)sin(thetaZ)    -sin(thetaX)sin(thetaY)sin(thetaZ) + cos(thetaX)cos(thetaZ)     -sin(thetaX)cos(thetaY)     |   multiplied by
        | -cos(thetaX)sin(thetaY)cos(thetaZ) + sin(thetaA)sin(thetaZ)   cos(thetaX)sin(thetaY)sin(thetaZ) + sin(thetaX)cos(thetaZ)      cos(thetaX)cos(thetaY)      |  
        
        CT Unit X Coordinate, ctX                     Robot Unit X Coordinate, rX
        | 1 |                                         | rXx |
        | 0 |                   gives ==>             | rXy |
        | 0 |                                         | rXz |

        CT Y Coordinate, ctY                          Robot Unit Y Coordinate, rY
        | 0 |                                         | rYx |
        | 1 |                   gives ==>             | rYy |
        | 0 |                                         | rYz |

        CT Z Coordinate, ctZ                          Robot Unit Z Coordinate, rZ
        | 0 |                                         | rZx |
        | 0 |                   gives ==>             | rZy |
        | 1 |                                         | rZz |  

        and since rX, rY and rZ is found, we have nine equations for 3 unknowns (thetax, thetaY, thetaZ). By inspection,

        transformationMatrix * ctZ: sin(thetaY) = rZx
        transformationMatrix * ctY: -cos(thetaY)sin(thetaZ) = rYx
        transformationMatrix * ctZ: -sin(thetaX)cos(thetaY) = rZy

        '''

        rthetaY = np.arcsin( rzUnit[0] )                    # in radian
        rthetaZ = np.arcsin( (ryUnit[0]/-np.cos(rthetaY)) ) # in radian
        rthetaX = np.arcsin( (rzUnit[1]/-np.cos(rthetaY)) ) # in radian

        dthetaX = np.degrees(rthetaX) # in degree. Uncomment and make changes if needed
        dthetaY = np.degrees(rthetaY) # in degree. Uncomment and make changes if needed
        dthetaZ = np.degrees(rthetaZ) # in degree. Uncomment and make changes if needed





        rotX = np.array([   [1,         0,                 0                   ], #3x3
                            [0,         math.cos(rthetaX), -math.sin(rthetaX)  ],
                            [0,         math.sin(rthetaX),  math.cos(rthetaX)  ]
                            ])
                 
                 
                             
        rotY = np.array([   [math.cos(rthetaY),    0,      math.sin(rthetaY)   ], #3x3
                            [0,                    1,      0                   ],
                            [-math.sin(rthetaY),   0,      math.cos(rthetaY)   ]
                            ])
                     
        rotZ = np.array([   [math.cos(rthetaZ),   -math.sin(rthetaZ),     0    ], #3x3
                            [math.sin(rthetaZ),    math.cos(rthetaZ),     0    ],
                            [0,                     0,                    1    ]
                            ])
                         
        
        # Forward rotation X-Y-Z
        rotationMatrix = np.dot(rotX, np.dot( rotY, rotZ ))




        # # Get Transformation Matrix. The robot frame is rotated then translated. So for our transformation matrix, we multiply the translation matrix to the rotation matrix
        filerMat = np.array( [ [ 0, 0, 0, 1 ] ] ) # 1x4. 1 represents scale factor
        transformationMatrix =  np.array( np.concatenate((rotationMatrix, translationMatrix), axis=1) ) # concatenate rotation and translation matrix
        transformationMatrix =  np.array( np.concatenate((transformationMatrix, filerMat), axis=0) ) # add filer matrix, 4x4 matrix

        return dthetaZ, dthetaY, dthetaX, transformationMatrix

        pass


    # Find robot desired x,y coordinate for correct needle alignment
    def getDesiredArmCoordinate(self, costmapVector, transformationMatrix, robot_base_plane_unit_normal, point_on_robot_base_plane, nodule_coords):

        # Extrapolate costmapVector. Convert to vector form. REDUNDANT
        extrapolatedcostmapVector = np.array( [np.dot(costmapVector,self.extrapolationMagnitude)] ) # 1x3 vector

        
        # Find robot base plane equation        
        # Find c of robot base plane equation => point.normal = c
        # robot base plane equation: (Nx)X + (Ny)Y + (Nz)Z = c
        c = np.dot(robot_base_plane_unit_normal, point_on_robot_base_plane)
        

        # Find robot arm plane by translating in robot frame z axis by height of robot
        # Find d of robot arm plane equation => c + translationVector.normal = d'
        # robot arm plane equation: (Nx)X + (Ny)Y + (Nz)Z = d
        d = c + np.dot(robot_base_plane_unit_normal, (self.h1*robot_base_plane_unit_normal) ) # h1 is 80mm


        #extrapolatedcostmapVector = np.array( [ [1,3,1] ] ) # For test, delete
      

        # Find point of intersection of vector and robot arm plane
        # Parametrization of vector to a line requires: costmapVector and a point that the line passes through (i.e. nodule coordiate)
        # This takes the form: lineEqn = (noduleX, noduleY, noduleZ) + t(vectorX, vectorY, vectorZ). Note that the vector needs to be parallel to the line
        # Therefore, lineEqnX = noduleX + tvectorX, linelineEqnY = noduleY + tvectorY, linelineEqnZ = noduleZ + tvectorZ
        # Solve for t by subbing lineEqnX, lineEqnY and lineEqnZ into the robot arm plane equation
        coefflineX = nodule_coords[0] # line's x coefficient
        coefflineY = nodule_coords[1] # line's y coefficient
        coefflineZ = nodule_coords[2] # line's z coefficient

        tlineX = extrapolatedcostmapVector[0][0] # coefficient of t in x 
        tlineY = extrapolatedcostmapVector[0][1] # coefficient of t in y 
        tlineZ = extrapolatedcostmapVector[0][2] # coefficient of t in z 


        # Sub points into robot arm plane equation. Multiply these coefficients by Nx, Ny and Nz
        newcoefflineX = coefflineX * robot_base_plane_unit_normal[0] # line's x coefficient after multiplication
        newcoefflineY = coefflineY * robot_base_plane_unit_normal[1] # line's y coefficient after multiplication
        newcoefflineZ = coefflineZ * robot_base_plane_unit_normal[2] # line's z coefficient after multiplication

        newtlineX = extrapolatedcostmapVector[0][0] * robot_base_plane_unit_normal[0] # coefficient of t in x after multiplication
        newtlineY = extrapolatedcostmapVector[0][1] * robot_base_plane_unit_normal[1] # coefficient of t in y after multiplication
        newtlineZ = extrapolatedcostmapVector[0][2] * robot_base_plane_unit_normal[2] # coefficient of t in z after multiplication


        # Solve for T
        # t = ( 4 - (newcoefflineX + newcoefflineY + newcoefflineZ) ) / ( newtlineX + newtlineY + newtlineZ ) # For test, delete
        t = ( d - (newcoefflineX + newcoefflineY + newcoefflineZ) ) / ( newtlineX + newtlineY + newtlineZ ) 


        # Sub T into parametrized line equation to find point of intersection x, y, z. 
        ctf_desiredX = coefflineX + ( t * tlineX )
        ctf_desiredY = coefflineY + ( t * tlineY )
        ctf_desiredZ = coefflineZ + ( t * tlineZ )
        ctf_desiredPoint = np.array( [ [ctf_desiredX], [ctf_desiredY], [ctf_desiredZ] ] ) # 3x1, represented in vector form




        # Redundant vector to desired point
        filerVector = np.array( [ [1] ] ) # 1x1
        # Concatenate reundant point to ctf_desiredpoint
        ctf_desiredPoint = np.array( np.concatenate((ctf_desiredPoint, filerVector), axis=0) ) # 4X1 , adds a 1 below the vector
        # Transform desired point from CT frame to robot frame
        rf_desiredPoint = np.array( np.matmul(transformationMatrix, ctf_desiredPoint) ) # 4x1, maps desired coordinates from CT frame to robot frame 
        
        desiredX = rf_desiredPoint[0][0] # Desired x in robot frame
        desiredY = rf_desiredPoint[1][0] # Desired y in robot frame

        return c, d, t, ctf_desiredPoint, desiredX, desiredY



        pass


    # Find motor angle theta 1 and theta 2 to reach desired x,y coordinate i.e inverse kinematics
    def getMotorAngles(self, x, y):
        #x and y in mm
        F = 2*y*self.l12
        G = (2*x-self.l0)*self.l12
        H = x**2 + y**2 - x*self.l0 + 0.25*self.l0**2 + self.l12**2 - (self.l22 + self.l3)**2
        t2 = 2*np.arctan((G + H)/(F - (F**2 + G**2 - H**2)**0.5))
        M = self.l3/(self.l22 + self.l3)*(y - self.l12*np.sin(t2))
        N = self.l3/(self.l22 + self.l3)*(x - 0.5*self.l0 + self.l12*np.cos(t2))
        J = (2*y - 2*M)*self.l11
        K = (2*x + self.l0 - 2*N)*self.l11
        L = x**2 + y**2 + x*self.l0 + 0.25*self.l0**2 + self.l11**2 - self.l21**2 - (2*x + self.l0 - N)*N - (2*y - M)*M
        t1 = 2*np.arctan((J + (J**2 + K**2 - L**2)**0.5)/(K + L))
        t1 = t1%(2*np.pi)   #convert from [-pi, pi] to [0, 2pi]
        t2 = t2%(2*np.pi)   #convert from [-pi, pi] to [0, 2pi]

        
        real = not(np.iscomplex(t1) or np.iscomplex(t2)) #checks if t1 and t2 are real numbers, if not real, the motor movement is impossible
        
        return t1, t2, real #t1 and t2 in radians

        pass






# ############################################################################################################
# ############################################################################################################
# ########################################## TEST SCENARIOS HERE #############################################
# ############################################################################################################
# ############################################################################################################
