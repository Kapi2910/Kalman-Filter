function [p,v] = IMuMeasurement(imu, del_t, z, zv)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


[acc,~] = imu([1, 1, 0], zeros(1, 3));

v(1) = zv(1) + del_t * acc(1);
v(2) = zv(2) + del_t * acc(2);

x = z(1) + del_t * v(1) + 0.5*del_t^2 * acc(1);
y = z(2) + del_t * v(2) + 0.5*del_t^2 * acc(2);
p = [x; y];
end

