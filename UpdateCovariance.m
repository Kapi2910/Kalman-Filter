function Pn = UpdateCovariance(K, H, P, R)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
X = eye(4, 4) - K*H;
Pn = X * P * X' + K*R*K';
end

