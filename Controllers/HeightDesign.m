clear
close all
clc


% Investigate bode plot method and gain margin.


% Requirements
    MP = 5; %Overshoot
    RT = 5; %Rise time

    df = -log(MP/100) * sqrt(1/(pi^2 + log(MP/100)^2)); %Find dampning factor
    
    
% Plant (Motor - Dynamics)
    G = tf(70.55,[0.098,1]) * tf(1,[0.942,0,0]);
    

% Controller

    K = 0.008;
    KD = 0.06;
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
    %hold on;
    %step(1 / tf('s'));
    %hold off;
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


