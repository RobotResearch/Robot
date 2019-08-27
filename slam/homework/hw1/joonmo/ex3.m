clc; clear; close all;

% (a)
fprintf("----- PROB. (a) -----\n");
p1 = [0; 0; 0]; % [x,y,theta]
p2 = [1; 1; pi/2];
p3 = [-1; 1; pi];
p4 = [2; -1; -pi/4];

t1 = v2t(p1)
t2 = v2t(p2)
t3 = v2t(p3)
t4 = v2t(p4)

p1_ = t2v(t1)
p2_ = t2v(t2)
p3_ = t2v(t3)
p4_ = t2v(t4)

% (b)
fprintf("----- PROB. (b) -----\n");
t12 = inv(t1)*t2

% (c)
fprintf("----- PROB. (c) -----\n");
xt = [1;1;pi/2];
zt = [2;0;1];

zo = v2t(xt)*zt



