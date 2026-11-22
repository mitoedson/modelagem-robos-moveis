close all
clear all
clc

% modelo robô móvel diferencial
syms x y theta v omega xd xd_dot yd yd_dot thetad thetad_dot xo yo l rs ctheta

x_model = [x;y;theta]
f_model = [0;0;0]
g_model = [cos(theta) 0;sin(theta) 0;0 1]

% V para pose
%V = ((xd-x)^2 + (yd-y)^2 + (thetad-theta)^2)/2
%V = ((x-xd)^2 + (y-yd)^2 + (theta-thetad)^2)/2

ex = cos(theta)*(xd-x) + sin(theta)*(yd-y)
ey = -sin(theta)*(xd-x) + cos(theta)*(yd-y)
etheta = thetad-theta
% ex = cos(theta)*(x-xd) + sin(theta)*(y-yd)
% ey = -sin(theta)*(x-xd) + cos(theta)*(y-yd)
% etheta = theta-thetad

% V para trajetória
%V = ((ex)^2 + (ey)^2 + (etheta)^2)/2
V = ((ex)^2 + (ey)^2)/2 + ctheta*(1-cos(etheta))

% CBF
h = (x+l*cos(theta)-xo)^2 + (y+l*sin(theta)-yo)^2 - (rs)^2
%h = (x-xo)^2 + (y-yo)^2 - (rs)^2;

Lfh = transpose(gradient(h,[x;y;theta]))*f_model
LfLfh = transpose(gradient(Lfh,[x;y;theta]))*f_model
Lgh = transpose(gradient(h,[x;y;theta]))*g_model
LgLfh = transpose(gradient(Lfh,[x;y;theta]))*g_model

LfV = transpose(gradient(V,[x;y;theta]))*f_model
LgV1 = transpose(gradient(V,[x;y;theta]))*g_model
LgV = simplify(LgV1)

Vt1 = diff(V,xd)*xd_dot + diff(V,yd)*yd_dot + diff(V,thetad)*thetad_dot
Vt = simplify(Vt1)

% B = 1/h
% LfB = transpose(gradient(B,[x;y;theta]))*f_model
% LfLfB = transpose(gradient(LfB,[x;y;theta]))*f_model
% LgB = transpose(gradient(B,[x;y;theta]))*g_model
% LgLfB = transpose(gradient(LfB,[x;y;theta]))*g_model