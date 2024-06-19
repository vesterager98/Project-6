clear
close all
clc


% Requirements
    MP = 10; %Overshoot
    RT = 1; %Rise time

    df = -log(MP/100) * sqrt(1/(pi^2 + log(MP/100)^2)); %Find dampning factor


% Plant (Motor - Dynamics)
    G = tf(17.67,[0.09,1]) * tf(0.161,[0.0128,0,0]);


% Controllers


    % Fast edition 
    %{
        % We need many speed so a pole replacement is in order
        p = 2.1739;
        PR = tf([1,p],[1,p*10]);
    
        % With the PR in effect, we can add two lead at the speed of p*8/10
        z = p*10/5;
        L = tf([1,z],[1,z*25]);
    
        K = 200;
        
        D = PR * L * L * K;
    %}
    % Slowed down edition
    %{
        % We need many speed so a pole replacement is in order
        p = 2.1739;
        PR = tf([1,p],[1,p*5]);
    
        % With the PR in effect, we can add two lead at the speed of p*8/10
        z = 1.8/RT;
        L = tf([1,z],[1,z*15]);
    
        K = 0.129;
        
        D = K * PR * L;
    %}
    % New edition
    %%{

        %p = 2.1739;
        %PR = tf([1,p],[1,p*6]);

        %z = 1.8/RT/2;
        %L = tf([1,z],[1,z*20]);

        %K = 1;

        
        
        D = 0.006 + 0.01 * tf([1,0],[0,1]) + 0 * tf([0,1],[1,0]);

    %}


% Generate a root locus of the plant and controller
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
