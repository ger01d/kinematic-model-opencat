import ikpy
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np
from numpy import sin, cos, pi
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

deg2rad = pi/180
rad2deg = 180/pi

# Values in cm
armLength   =    5
bodyLength  =   12
bodyWidth   =   10     
distanceFloor = 7
stepLength = 5.0
swingHeight = 0.3

leanLeft = 0           # Not working, yet
leanRight = -leanLeft  # Not working, yet
leanForward =  0       # cm

left_arm = Chain(name='left_arm', links=[
    URDFLink(
      name="center",
      origin_translation=[leanForward, 0, distanceFloor],
      origin_orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    ),
    URDFLink(
      name="shoulder",
      origin_translation=[0, bodyWidth/2, leanLeft],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(0.1*deg2rad, 179.9*deg2rad),
    ),
    URDFLink(
      name="upperArm",
      origin_translation=[armLength, 0, 0   ],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(-179.9*deg2rad, -0.1*deg2rad),
    ),
    URDFLink(
      name="lowerArm",
      origin_translation=[armLength, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    )

])

right_arm = Chain(name='right_arm', links=[
    URDFLink(
      name="center",
      origin_translation=[leanForward, 0, distanceFloor],
      origin_orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    ),
    URDFLink(
      name="shoulder",
      origin_translation=[0, -bodyWidth/2, leanRight],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(0.1*deg2rad, 179.9*deg2rad),
    ),
    URDFLink(
      name="upperArm",
      origin_translation=[armLength, 0, 0],
      origin_orientation=[0,0,0],
      rotation=[0,1,0],
      bounds=(-179.9*deg2rad, -0.1*deg2rad),
    ),
    URDFLink(
      name="lowerArm",
      origin_translation=[armLength, 0, 0   ],
      origin_orientation=[0,0,0],
      rotation=[0, 1, 0],
    )

])

left_leg = Chain(name='left_leg', links=[
    URDFLink(
      name="center",
      origin_translation=[leanForward, 0, distanceFloor],
      origin_orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    ),
    URDFLink(
      name="butt",
      origin_translation=[-bodyLength, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    ),
    URDFLink(
      name="hip",
      origin_translation=[0, bodyWidth/2, leanLeft],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(0.1*deg2rad, 120*deg2rad),
    ),
    URDFLink(
      name="upperLeg",
      origin_translation=[armLength, 0, 0   ],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(0.1*deg2rad, 179.9*deg2rad),
    ),
    URDFLink(
      name="lowerLeg",
      origin_translation=[armLength, 0, 0   ],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    )

])

right_leg = Chain(name='right_leg', links=[
    URDFLink(
      name="center",
      origin_translation=[leanForward, 0, distanceFloor],
      origin_orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    ),
    URDFLink(
      name="butt",
      origin_translation=[-bodyLength, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    ),
    URDFLink(
      name="hip",
      origin_translation=[ 0, -bodyWidth/2, leanRight],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(0.1*deg2rad, 120*deg2rad),
    ),
    URDFLink(
      name="upperLeg",
      origin_translation=[armLength, 0, 0   ],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(0.1*deg2rad, 179*deg2rad),
    ),
    URDFLink(
      name="lowerLeg",
      origin_translation=[armLength, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    )

])


def buildGait(frames):
    frame = np.arange(0,frames)
    swingEnd = pi/2

    # longitudinal Movement
    swing = -cos(2*(frame*2*pi/frames))
    stance = cos(2/3*(frame*2*pi/frames-swingEnd))
    swingSlice = np.less_equal(frame,swingEnd/(2*pi/frames))
    stanceSlice = np.invert(swingSlice)
    longitudinalMovement = np.concatenate((swing[swingSlice], stance[stanceSlice]))    
    longitudinalMovement = np.concatenate((longitudinalMovement,longitudinalMovement, longitudinalMovement, longitudinalMovement))
    
    # vertical Movement
    lift = sin(2*(frame*2*pi/frames))
    liftSlice = swingSlice
    verticalMovement = np.concatenate((lift[liftSlice], np.zeros(np.count_nonzero(stanceSlice))))
    verticalMovement = np.concatenate((verticalMovement, verticalMovement, verticalMovement, verticalMovement))
    return longitudinalMovement, verticalMovement


frames = 43
longitudinalMovement, verticalMovement = buildGait(frames)
plt.plot(longitudinalMovement)
plt.plot(verticalMovement)
plt.show()

# Walking pattern
#    [0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 1,1,1,1,1], # LL
#    [0,0,0,0,0, 1,1,1,1,1, 0,0,0,0,0, 0,0,0,0,0], # RL
#    [0,0,0,0,0, 0,0,0,0,0, 1,1,1,1,1, 0,0,0,0,0], # LA
#    [1,1,1,1,1, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0], # RA

# Phase shift between arms/legs, [RA, RL, LA, LL]
shiftFrame = np.round(np.array([0, pi/2, pi, 3*pi/2])/2/pi*frames)
#shiftFrame = np.round(np.array([0, 0, pi, pi])/2/pi*frames)
#shiftFrame = np.round(np.array([0, pi, pi, 0])/2/pi*frames)

shiftFrame = shiftFrame.astype(int)

for frame in range(0, frames):
    right_arm_angles_raw = right_arm.inverse_kinematics(target_position = np.array([stepLength*longitudinalMovement[frame], -bodyWidth/2 ,swingHeight*verticalMovement[frame]]), optimization_method='SLSQP')
    right_arm_correction = np.array([0, -pi/2, pi/2, 0])
    right_arm_angles = np.round(rad2deg*(right_arm_angles_raw+right_arm_correction))
    right_arm_angles = np.delete(right_arm_angles, np.s_[0,3], axis=0)
    right_arm_angles = right_arm_angles.astype(int)

    right_leg_angles_raw = right_leg.inverse_kinematics(target_position = np.array([-bodyLength+stepLength*longitudinalMovement[frame+shiftFrame[1]], -bodyWidth/2 , swingHeight*verticalMovement[frame+shiftFrame[1]]]), optimization_method='SLSQP')    
    right_leg_correction = np.array([0, 0, -pi/2, -pi/2, 0])    
    right_leg_angles = np.round(rad2deg*(right_leg_angles_raw+right_leg_correction))
    right_leg_angles = np.delete(right_leg_angles, np.s_[0,1,4], axis=0)
    right_leg_angles = right_leg_angles.astype(int)

    left_arm_angles_raw = left_arm.inverse_kinematics(target_position = np.array([stepLength*longitudinalMovement[frame+shiftFrame[2]], +bodyWidth/2 , swingHeight*verticalMovement[frame+shiftFrame[2]]]), optimization_method='SLSQP')
    left_arm_correction = np.array([0, -pi/2, pi/2, 0])     
    left_arm_angles = np.round(rad2deg*(left_arm_angles_raw+left_arm_correction))
    left_arm_angles = np.delete(left_arm_angles, np.s_[0,3], axis=0)
    left_arm_angles = left_arm_angles.astype(int)

    left_leg_angles_raw = left_leg.inverse_kinematics(target_position = np.array([-bodyLength+stepLength*longitudinalMovement[frame+shiftFrame[3]], +bodyWidth/2 , swingHeight*verticalMovement[frame+shiftFrame[3]]]), optimization_method='SLSQP')
    left_leg_correction = np.array([0, 0, -pi/2, -pi/2, 0])
    left_leg_angles = np.round(rad2deg*(left_leg_angles_raw+left_leg_correction))
    left_leg_angles = np.delete(left_leg_angles, np.s_[0,1,4],axis=0)
    left_leg_angles = left_leg_angles.astype(int)
    
    
    # Writing sequence to file
    gait_sequence = np.concatenate((left_arm_angles, right_arm_angles, right_leg_angles, left_leg_angles))
    print(frame," ",gait_sequence)
    f = open("gait_sequence.csv", "a")
    #f.write("#")
    np.savetxt(f, gait_sequence[[0,2,4,6,1,3,5,7]],  fmt='%3.1i', delimiter=',', newline=", ")
    #f.write("+")
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

    figureName = "Nybble_" + str(frame)
    plt.savefig(figureName)
    plt.close()

    #plt.show()

