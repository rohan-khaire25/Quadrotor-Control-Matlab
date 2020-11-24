function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls
phic = -(des_state.acc(1)+0*(des_state.pos(1)-state.pos(1))+20.2*(des_state.vel(1)-state.vel(1)))/params.gravity;
u1 = params.mass*(params.gravity+des_state.acc(2)+20*(des_state.pos(2)-state.pos(2))+10*(des_state.vel(2)-state.vel(2)));
u2 = params.Ixx*(450*(phic-state.rot(1))+105*(0-state.omega(1)));



end

