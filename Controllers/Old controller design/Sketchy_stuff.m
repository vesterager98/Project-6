clear
close all
clc






%Derivative gain rlocus
GD = tf([92.26,0,0],[1.472, 2.3, 0, 92.26, 92.26]);
rlocus(GD)