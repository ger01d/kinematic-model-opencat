# Inverse Kinematic Model OpenCat

This is an inverse kinematic model and gait generator for the OpenCat project based on the IKPY library (http://phylliade.github.io/ikpy).

## Nybble-Version
![Nybble](Nybble_moving.gif) 

## Bittle-Version
![Bittle](Bittle_moving.gif)

## Usage and Tweaks
For best results change the optimization algorithm in IKPY from _L-BFGS-B_ to _SLSQP_. 

_inverse_kinematics.py_ line 134 from _L-BFGS-B_ to _SLSQP_.

`res = scipy.optimize.minimize(optimize_total, chain.active_from_full(starting_nodes_angles), method='SLSQP', bounds=real_bounds, options=options)`



