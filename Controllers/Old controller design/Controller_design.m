clear
close all
clc


% Requirements
MP = 10; %Overshoot
RT = 0.5; %Rise time

df = -log(MP/100) * sqrt(1/(pi^2 + log(MP/100)^2)); %Find dampning factor


% Plant (Motor - Dynamics)
G = 4 * tf([0,23.06574206],[0.46,1]) * tf([0,0.1615],[0.02130,0]);

% Integrator
I = tf([1],[1,0]);


% Controller time

P = tf([1,2.1739],[1,2.1739*10]); %Poles replacement

%PID = 0.15 + 0.5*tf([1,0],[0,1]) + 1*tf([0,1],[1,0]);

%L = tf([1,4,1],[1,0]);

z = 1.8/RT*2;

L = tf([1,z],[1,20*z]);

K = 654;

D = P * L * L * K;


% Generate a root locus of the plant and controller
figure(), rlocus(G*D*I);


% Draw our requirements on the graph
line(tan(acos(df)+(pi/2)));
circle(0,0,(1.8/RT));


% At last draw the closed loop step response
CL = ((G*D)/(1+G*D*I))*I;

figure(), step(CL);
[y, t] = step(CL);
sse = abs(1-y(length(y)));


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