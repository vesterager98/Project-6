clear
close all
clc


% Requirements
MP = 0; %Overshoot
RT = 5; %Rise time

df = -log(MP/100) * sqrt(1/(pi^2 + log(MP/100)^2)); %Find dampning factor


% Plant (Motor - Dynamics)
G = tf([92.2628],[0.46,1]) * tf([1],[1.2,0,0]);


% Controller time

P = tf([1,2.1730],[1,2.1730*5]); %Pole replacement

%PID = 1 + 4*tf([1,0],[0,1]) + 1*tf([0,1],[1,0]); %PID contoller

%z = 5;

%N = tf([1,z],[1,z*20]); %Lead

z = (1.8/RT);

N = tf([1,z],[1,z*25]); %Lead

K = 25.33*0.3794;

D = K*P*N*N;


% Generate a root locus of the plant and controller
figure(), rlocus(G*D);


% Draw our requirements on the graph
line(tan(acos(df)+(pi/2)));
circle(0,0,(1.8/RT));


CL = ((G*D)/(1+G*D));


% At last draw the closed loop step response
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