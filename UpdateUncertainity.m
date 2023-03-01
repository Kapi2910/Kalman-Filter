function Pn = UpdateUncertainity(P, F, Q)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
Pn = F * P * F' + Q;
end

