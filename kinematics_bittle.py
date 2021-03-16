#!/usr/bin/env python
# coding: utf-8

import ikpy
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np
from numpy import sin, cos, pi
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3


deg2rad = pi/180
rad2deg = 180/pi

# Dimensions Bittle (should not be changed - all values in cm)
upperArmLength = 4.5
lowerArmLength = 5.0
bodyLength     = 10.5
bodyWidth      = 9.7

# Variable parameters
distanceFloor  = 6.0    # Distance torso to floor
stepLength     = 3.5    # Longitudinal movement of each paw within one period
swingHeight    = 0.5    # Amplitude for swing
swingFactorLegs = 5     # Increase swing amplitude for legs
leanForward =  0.0      # Shifting torso in x-direction
stanceFront = 0.0       # Distance of front paws from shoulder in x-direction
stanceBack = 3          # Distance of back paws from hip in negative x-direction

# Number of Frames
frames = 57

# Kinematic chain of Bittle
left_arm = Chain(name='left_arm', links=[
    URDFLink(
      name="center",
      translation_vector=[leanForward, 0, distanceFloor],
      orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    ),
    URDFLink(
      name="shoulder",
      translation_vector=[0, bodyWidth/2, 0],
      orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(0.1*deg2rad, 179.9*deg2rad),
    ),
    URDFLink(
      name="upperArm",
      translation_vector=[upperArmLength, 0, 0],
      orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(-179.9*deg2rad, -0.1*deg2rad),
    ),
    URDFLink(
      name="lowerArm",
      translation_vector=[lowerArmLength, 0, 0],
      orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    )
])

right_arm = Chain(name='right_arm', links=[
    URDFLink(
      name="center",
      translation_vector=[leanForward, 0, distanceFloor],
      orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    ),
    URDFLink(
      name="shoulder",
      translation_vector=[0, -bodyWidth/2, 0],
      orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(0.1*deg2rad, 179.9*deg2rad),
    ),
    URDFLink(
      name="upperArm",
      translation_vector=[upperArmLength, 0, 0],
      orientation=[0,0,0],
      rotation=[0,1,0],
      bounds=(-179.9*deg2rad, -0.1*deg2rad),
    ),
    URDFLink(
      name="lowerArm",
      translation_vector=[lowerArmLength, 0, 0],
      orientation=[0,0,0],
      rotation=[0, 1, 0],
    )
])

left_leg = Chain(name='left_leg', links=[
    URDFLink(
      name="center",
      translation_vector=[leanForward, 0, distanceFloor],
      orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    ),
    URDFLink(
      name="butt",
      translation_vector=[-bodyLength, 0, 0],
      orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    ),
    URDFLink(
      name="hip",
      translation_vector=[0, bodyWidth/2, 0],
      orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(0.1*deg2rad, 200*deg2rad),
    ),
    URDFLink(
      name="upperLeg",
      translation_vector=[upperArmLength, 0, 0],
      orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(-200*deg2rad, -0.1*deg2rad),
    ),
    URDFLink(
      name="lowerLeg",
      translation_vector=[lowerArmLength, 0, 0],
      orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    )
])

right_leg = Chain(name='right_leg', links=[
    URDFLink(
      name="center",
      translation_vector=[leanForward, 0, distanceFloor],
      orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    ),
    URDFLink(
      name="butt",
      translation_vector=[-bodyLength, 0, 0],
      orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    ),
    URDFLink(
      name="hip",
      translation_vector=[ 0, -bodyWidth/2, 0],
      orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(0.1*deg2rad, 200*deg2rad),
    ),
    URDFLink(
      name="upperLeg",
      translation_vector=[upperArmLength, 0, 0],
      orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(-200*deg2rad, -0.1*deg2rad),
    ),
    URDFLink(
      name="lowerLeg",
      translation_vector=[lowerArmLength, 0, 0],
      orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    )
])

def buildGait(frames):
    frame = np.arange(0,frames)
    swingEnd = pi/2

    # longitudinal Movement
    swing = -cos(2*(frame*2*pi/frames))
    stance = +cos(2/3*(frame*2*pi/frames-swingEnd))
    swingSlice = np.less_equal(frame,swingEnd/(2*pi/frames))
    stanceSlice = np.invert(swingSlice)
    longitudinalMovement = np.concatenate((swing[swingSlice], stance[stanceSlice]))    
    longitudinalMovement = np.concatenate((longitudinalMovement,longitudinalMovement, longitudinalMovement, longitudinalMovement))
    
    # vertical Movement
    lift = sin(2*(frame*2*pi/frames)) #np.ones(frames)
    liftSlice = swingSlice
    verticalMovement = np.concatenate((lift[liftSlice], np.zeros(np.count_nonzero(stanceSlice))))
    verticalMovement = np.concatenate((verticalMovement, verticalMovement, verticalMovement, verticalMovement))
    return longitudinalMovement, verticalMovement


longitudinalMovement, verticalMovement = buildGait(frames)
plt.plot(longitudinalMovement)
plt.plot(verticalMovement)
plt.show()
plt.close()

shiftFrame = np.round(np.array([0, pi/2, pi, 3*pi/2])/2/pi*frames)
#shiftFrame = np.round(np.array([0, pi, pi, 0])/2/pi*frames)
#shiftFrame = np.round(np.array([0, 0, pi, pi])/2/pi*frames)
shiftFrame = shiftFrame.astype(int)

for frame in range(0, frames):
    right_arm_angles_raw = right_arm.inverse_kinematics(target_position = np.array([stanceFront+stepLength*longitudinalMovement[frame], -bodyWidth/2 ,swingHeight*verticalMovement[frame]]))
    right_arm_correction = np.array([0, -pi/2, pi/2, 0])
    right_arm_angles = np.round(rad2deg*(right_arm_angles_raw+right_arm_correction))
    right_arm_angles = np.delete(right_arm_angles, np.s_[0,3], axis=0)
    right_arm_angles = right_arm_angles.astype(int)

    right_leg_angles_raw = right_leg.inverse_kinematics(target_position = np.array([-stanceBack-bodyLength+stepLength*longitudinalMovement[frame+shiftFrame[1]], -bodyWidth/2 , swingFactorLegs*swingHeight*verticalMovement[frame+shiftFrame[1]]]))    
    right_leg_correction = np.array([0, 0, -pi/2, pi/2, 0])
    right_leg_angles = np.round(rad2deg*(right_leg_angles_raw+right_leg_correction))
    right_leg_angles = np.delete(right_leg_angles, np.s_[0,1,4], axis=0)
    right_leg_angles = right_leg_angles.astype(int)

    left_arm_angles_raw = left_arm.inverse_kinematics(target_position = np.array([stanceFront+stepLength*longitudinalMovement[frame+shiftFrame[2]], +bodyWidth/2 , swingHeight*verticalMovement[frame+shiftFrame[2]]]))
    left_arm_correction = np.array([0, -pi/2, pi/2, 0])
    left_arm_angles = np.round(rad2deg*(left_arm_angles_raw+left_arm_correction))
    left_arm_angles = np.delete(left_arm_angles, np.s_[0,3], axis=0)
    left_arm_angles = left_arm_angles.astype(int)

    left_leg_angles_raw = left_leg.inverse_kinematics(target_position = np.array([-stanceBack-bodyLength+stepLength*longitudinalMovement[frame+shiftFrame[3]], +bodyWidth/2 , swingFactorLegs*swingHeight*verticalMovement[frame+shiftFrame[3]]]))
    left_leg_correction = np.array([0, 0, -pi/2, pi/2, 0])
    left_leg_angles = np.round(rad2deg*(left_leg_angles_raw+left_leg_correction))
    left_leg_angles = np.delete(left_leg_angles, np.s_[0,1,4],axis=0)
    left_leg_angles = left_leg_angles.astype(int)
    
    
    # Writing sequence to file
    gait_sequence = np.concatenate((left_arm_angles, right_arm_angles, right_leg_angles, left_leg_angles))
    print(frame," ",gait_sequence)
    f = open("gait_sequence.csv", "a")
    #f.write("[")
    np.savetxt(f, gait_sequence[[0,2,4,6,1,3,5,7]],  fmt='%3.1i', delimiter=',', newline=', ')
    #f.write("],")
    f.write("\n") 
    f.close()
    
    # Create plot and image for each frame
    fig = plt.figure()
    ax = p3.Axes3D(fig)
    ax.set_box_aspect([3, 3/2,1])
    ax.set_xlim3d([-20, 10])
    ax.set_xlabel('X')

    ax.set_ylim3d([-10, 10])
    ax.set_ylabel('Y')

    ax.set_zlim3d([0.0, 10])
    ax.set_zlabel('Z')
    right_arm.plot(right_arm_angles_raw, ax)
    right_leg.plot(right_leg_angles_raw, ax)
    left_arm.plot(left_arm_angles_raw, ax)
    left_leg.plot(left_leg_angles_raw, ax)

    figureName = "Bittle_" + str(frame)
    plt.savefig(figureName)
    plt.close()
    #plt.show()

