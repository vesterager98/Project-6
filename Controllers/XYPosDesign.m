clear
close all
clc


% Requirements
    MP = 30; %Overshoot
    RT = 1; %Rise time

    df = -log(MP/100) * sqrt(1/(pi^2 + log(MP/100)^2)); %Find dampning factor
    
    
% Plant (Angle loop - Dynamics)
    
    G = tf([0.0001189,0.001309,0.0009945,0.0002427,0,0,0],[4.357e-06,8.892e-05,0.0005726,0.001309,0.0009945,0.0002427,0,0,0]) * tf(9.82,[1,0,0]);
    

% Controller
    
    K = 0.05;
    KD = 0.1;
    KI = 0.001;
    
    Deri = tf([1,0],1);
    Intg = tf(1,[1,0]);
    
    D = K + KD * Deri + KI * Intg;
    

    
% Plot root locus 
    figure(), rlocus(G*D);
    
% Draw our requirements on the graph
    line(tan(acos(df)+(pi/2)));
    circle(0,0,(1.8/RT));

% At last draw the closed loop step response
    CL = (G*D)/(1+G*D);
    
    figure(), step(CL);
    [y, t] = step(CL);
    sse = abs(1-y(length(y)));
    stepinfo(CL)
    
    
    
    
    
    
    
    
% For drawing circles
function h = circle(x,y,r)
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit);
hold off
end

% For drawing lines
function h = line(a)
hold on
xunit = -100:100;
yunit = a * xunit;
h = plot(xunit, yunit);
hold off
end


