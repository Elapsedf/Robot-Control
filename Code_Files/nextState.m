% Calculation the next state of a Robotic Arm
% Mengtang Li, Ph.D. 
% 2023 06 01

function [next_conf] = nextState(current_conf, control, dt)
% configuration of the robot arm: theta1 ~ theta6
next_conf = current_conf + control'*dt; % control is a column vector, 
%                               current_conf and next_conf are row vector.
end

