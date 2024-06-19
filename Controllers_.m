clear all
close all
clc




% -------- Transfer function ----------

%The plant
G = 2 * tf([0,23.06574206],[0.46,1]) * tf([0,0.1615],[0.02130,0]) * tf([0,1],[1,0]);
%G = tf([0,0,760.3832],[1,0,0]);

%An integrator
%Intg = tf([0,1],[1,0]);

% -------- Controller design ----------


z1 = tf([1,2],[0,1]);
z2 = tf([1,2.1739],[0,1]);
p = tf([0,1],[1,10]);

%test1 = 0.05 + tf([0,0.002],[1,0]) + tf([100,0],[0,1]);

D = z1 * z2;

K = 0.021;

%K = 1;


%---------------- Test ----------------

c2d(K*D, 1/800)


%Closed-loop calculation
CL = (K*D*G) / (1 + K*D*G);


%Root locus generation
rlocus(D*G);

%[K, p] = rlocfind(D*G);

circle(0,0,1.8);

line(-1.952088342);


%Step generation
figure();
step(CL);
S = stepinfo(CL)



%Old stuff
%{ 


    
    %%{

    %For the X,Y control



    % System

    G = 2 * tf([0,23.06574206],[0.46,1]) * tf([0,0.1615],[0.02130,0]);

    Intg = tf([0,1],[1,0]);

    % Controller 1
    %%{

    z = 2;

    p = 25;

    %D = tf([1,z],[1,z*p]) * tf([1,2.1739],[1,8*2.1739]);

    D = tf([1,z],[1,z*p])

    K = 8.1352;

    %%}

    % Controller 2
    %%{

    z = 0.2;

    p = 25;

    D = tf([1,z],[1,z*p]) * tf([1,1.2098,0.38066626],[1,12.0980,36.60516325]);

    K = 45.10;

    %%}

    % Controller 3
    %%{

    D = tf([1,10],[0,1]) * tf([1,10],[0,1]) * tf([1,10],[0,1]);

    K = 0.0822;

    %%}

    %%{

    %Højdecontroller


    G = tf([0,92.26296824],[0.46,1]) * tf([0,0,1],[700,0,0]);

    %z = 0.2;

    %p = 25*z;

    %D = tf([1,z],[1,p]);

    D = tf([1,0.2],[0,1]);

    K = 5.1418;

    %%}


    % Test

    rlocus(D*G);

    %[K, p] = rlocfind(D*G);

    figure();

    %%{

    CL = Intg * ((K*D*G) / (1 + K*D*G));

    %%}

    %CL = (K*D*G) / (1 + K*D*G);

    step(CL);



%}



function h = circle(x,y,r)
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit);
hold off
end

function h = line(a)
hold on
xunit = [-10:10];
yunit = a * xunit;
h = plot(xunit, yunit);
hold off
end
