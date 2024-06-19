clear
close all
clc


% Requirements
    MP = 30; %Overshoot
    RT = 0.1; %Rise time

    df = -log(MP/100) * sqrt(1/(pi^2 + log(MP/100)^2)); %Find dampning factor
    
    
% Plant (Motor - Dynamics)
    %G = tf(889.02,[0.09,1,0,0]);
    
    G = tf(70.55,[0.098,1]) * tf(0.1615,[0.0213,0]) * tf(1,[1,0]);
    

% Controller

    %K = 1*0.0044;
    %KD = 0.9*0.0044;
    %KI = 0.01*0.0044;
    
    %K = 0.005;
    %KD = 0.04;
    %KI = 0.001;
    
    %Accel = tf([1,10.2041],[1,10.2041*8]);
    
    %K = 1;
    %KD = 0.2;
    %KI = 2;
    
    %Deri = tf([1,0],1);
    %Intg = tf(1,[1,0]);
    
    %D = (K + KD * Deri + KI * Intg) * Accel;
    
    
    
    K = 0.004;
    KD = 0.005;
    KI = 0.001;
    
    Deri = tf([1,0],1);
    Intg = tf(1,[1,0]);
    
    D = (K + KD * Deri + KI * Intg);
    
    
    
    %zR = 0.5;
    %zI = 0.2;
    
    %K = 0.0060;
    %D = K * tf([1,2*zR,zR^2+zI^2],[0,1,0]);
    
    
    %Accel = tf([1,10.2041],[1,10.2041*8]);
    %z = 1.8/RT/2;
    %Lead = tf([1,z],[1,z*20]);
    %p = 50/100;
    %Lag = tf([1,p],[1,p/2]);
    %K = 3.1691;
    %D = K * Accel * Lag * Lead;
    
    
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


