function K = KalmanGain(P, H, R)
%KALMANGAIN Summary of this function goes here
%   Detailed explanation goes here
K = P*H'/(H*P*H' + R);
end

