# MPC_vehicleOvertaking
MPC-based vehicle overtaking system

## main file
nlmpc_main.m file runs the mpc computation.

set the obstacle (other cars)
obsState = [10, -2.5, 0, 0;
    30, -2.5, 0, 0;
    30, 2.5, pi, 0];

initial state of the vehicle
x = [0; -2.5; 0; 0];
y = x;

reference point
yref = [60 -2.5 0 0];

other vehicle's velocity
obsV = 10;

## simulating the output

plotting(yref, obsHistory, xHistory, time_obs, lane_length, nlobj);
