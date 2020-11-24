function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

r1_ddot_des = des_state.acc(1)+ 1.5*(des_state.vel(1)-state.vel(1)) + 10*(des_state.pos(1)-state.pos(1));
r2_ddot_des = des_state.acc(2)+ 1.5*(des_state.vel(2)-state.vel(2)) + 10*(des_state.pos(2)-state.pos(2));
r3_ddot_des = des_state.acc(3)+ 10*(des_state.vel(3)-state.vel(3)) + 800*(des_state.pos(3)-state.pos(3));
phi_des = (r1_ddot_des*des_state.yaw - r2_ddot_des)/params.gravity;
theta_des = (r1_ddot_des + r2_ddot_des*des_state.yaw)/params.gravity;
psi_des = des_state.yaw;
F = params.mass*params.gravity + params.mass*r3_ddot_des;

% Moment
G = [3000*(phi_des-state.rot(1))+2*(0-state.omega(1));
     3000*(theta_des-state.rot(2))+2*(0-state.omega(2));
     3000*(psi_des-state.rot(3))+10*(des_state.yawdot-state.omega(3))];
M = params.I * G;

% =================== Your code ends here ===================

end
