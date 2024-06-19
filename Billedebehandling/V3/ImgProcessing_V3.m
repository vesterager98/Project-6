close
clear all
clc


%Get the image
img = imread('2m x20.png');
%img = imrotate(img, 180);


%View the image
figure, image(img);

%Convert img to double
imgD = double(img);

%Get color channels
R = imgD(:,:,1);
G = imgD(:,:,2);
Blue = imgD(:,:,3);

%{
imgN = cat(1,uint8(R));
figure, image(imgN);
for i = 1:numel(R)
       
    R(i) = R(i)*1 - G(i)/5 - Blue(i)/5;
        
end
imgN = cat(1,uint8(R));
figure, image(imgN);
%}

%Filter for the blue values to make the circle stand out more
for i = 1:numel(Blue)  
    Blue(i) = Blue(i)*1 - G(i)/2 - R(i)/2;     
end
%Same filter applied to red
for i = 1:numel(R)
    %R(i) = R(i)*1 - G(i)/3 - Blue(i)/3;
end

%Filter
F = [0, 1, 2, 1, 0;
     1, 2, 3, 2, 1;
     2, 3, 5, 3, 2;
     1, 2, 3, 2, 1;
     0, 1, 2, 1, 0];

Border = (size(F, 1) - 1) / 2;

greenPoint = [0, 0, 0]';
%Image procssing (Finding the green dot and blue poitns of interest)
for i = 1:size(G,1)
   for k = 1:size(G,2)
       %Don't the borders
       if k > Border && i > Border && k < size(G,2) - Border && i < size(G,1) - Border
           %Do the math
           T = 0;
           for x = 1:size(F, 1)
               for y = 1:size(F, 2)
                   T = T + G(x + i - 2, y + k - 2)*F(x, y);
               end
           end
           if T > greenPoint(1)
               greenPoint = [T, i, k]';
           end
       end          
   end
end

%Removing the score
greenPoint = greenPoint(2:3);

blueThreshold = 2500;

blueTop = [0, 0]';
blueBottom = [0, 0]';
blueLeft = [0, 0]';
blueRight = [0, 0]';

blueFurthest = [0, 0, 0]';
blueClosest = [0, 0, 0]';

%Finding the bounds of the circle
for i = 1:size(Blue,1)
   for k = 1:size(Blue,2)
       %Don't the borders
       if k > Border && i > Border && k < size(Blue,2) - Border && i < size(Blue,1) - Border
           %Do the math
           T = 0;
           for x = 1:size(F, 1)
               for y = 1:size(F, 2)
                   T = T + Blue(x + i - 2, y + k - 2)*F(x, y);
               end
           end
           if T > blueThreshold
               if i < blueTop(1) || blueTop(1) == 0
                   blueTop = [i, k]';
               end
               if i > blueBottom(1) || blueBottom(1) == 0
                   blueBottom = [i, k]';
               end
               if k < blueLeft(2) || blueLeft(2) == 0
                   blueLeft = [i, k]';
               end
               if k > blueRight(2) || blueRight(2) == 0
                   blueRight = [i, k]';
               end
           end
       end
   end
end

center = [(blueTop(1) + blueBottom(1))/2, (blueLeft(2) + blueRight(2))/2]';

%Finding the spots on the blue circle closest to and from the center of it
for i = blueTop(1):blueBottom(1)
   for k = blueLeft(2):blueRight(2)
       %Don't the borders
       if k > Border && i > Border && k < size(Blue,2) - Border && i < size(Blue,1) - Border
           %Do the math
           T = 0;
           for x = 1:size(F, 1)
               for y = 1:size(F, 2)
                   T = T + Blue(x + i - 2, y + k - 2)*F(x, y);
               end
           end
           if T > blueThreshold
               if norm([i, k]' - center) > norm(blueFurthest(2:3) - center) || blueFurthest(1) == 0
                   blueFurthest = [T, i, k]';
               end
           else
               if norm([i, k]' - center) < norm(blueClosest(2:3) - center) || blueClosest(1) == 0
                   blueClosest = [T, i, k]';
               end
           end
       end
   end
end

%Removing the score
blueFurthest = blueFurthest(2:3);
blueClosest = blueClosest(2:3);

%Calculate approximate distance from the marker
%Angle per pixel
blueWidth = norm(blueFurthest - center) * 2;
anglePerPixel = 65 / 1600;
totalAngle = blueWidth * anglePerPixel;

actualSpacing = 0.2; %Actual diameter of the blue circle

distance = cos(deg2rad(totalAngle/2)) * (actualSpacing/2 / sin(deg2rad(totalAngle/2)))
%Calibration compensation
%testHeight = testHeight / 1.1073 - 0.012

totalAngle = rad2deg(acos(norm(blueClosest - center)*2 / blueWidth))

%Container for ranking the red points
P = [[0, 0, 0];
     [0, 0, 0];
     [0, 0, 0]];

%Thresholds for finding the red and how close they are allowed to be to
%each other
ThreshR = 150;
ThreshDist = 40;

for i = 1:size(R,1)
   for k = 1:size(R,2)
       %Don't the borders 
       %_____The -500 is a temporary fix_____
       if k > Border && i > Border && k < size(R,2) - Border - 500 && i < size(R,1) - Border
           %Do the math
           T = 0;
           for x = 1:size(F, 1)
               for y = 1:size(F, 2)
                   T = T + R(x + i - 2, y + k - 2)*F(x, y);
               end
           end
           %Check if the new point is too close to other existing points
           for j = 1:size(P, 1)
               if abs(P(j, 2) - i) < ThreshDist && abs(P(j, 3) - k) < ThreshDist
                   if P(j, 1) < T
                       if addPoint([T, i, k], j, P)
                           P(j, 1) = T;
                           P(j, 2) = i;
                           P(j, 3) = k;
                       end
                   end
                   break
               elseif P(j, 1) < T
                   if j < size(P, 1) && addPoint([T, i, k], j, P)
                       for l = size(P, 1) - 1:j
                           P(l + 1) = P(l);
                       end
                       P(j, 1) = T;
                       P(j, 2) = i;
                       P(j, 3) = k;

                       P(j, :) = [T, i, k];
                       break
                   elseif addPoint([T, i, k], j, P)
                       P(j, 1) = T;
                       P(j, 2) = i;
                       P(j, 3) = k;
                       break
                   end
                end
            end
       end          
   end
end

for j = 1:size(P, 1)
    %A = P(3,2:3)';
    vecs = [0, 0;
            0, 0];
    vecNum = 1;
    for k = 1:size(P, 1)
        if k ~= j
            vecs(vecNum:vecNum+1) = P(k, 2:3) - P(j, 2:3);
            vecNum = vecNum +2;
        end

    end
    P(j, 1) = rad2deg(subspace(vecs(1:2)', vecs(3:4)'));

end

%Determine which red dots are which
redBotRight = P(1, 1:3);
point = 1;
for j = 2:size(P, 1)
    if P(j, 1) > redBotRight(1)
        redBotRight = P(j,1:3);
        point = j;
    end
end

otherPoints = [0, 0, 0;
               0, 0, 0];
count = 1;
for j = 1:size(P, 1)
    if j ~= point
        otherPoints(count, :) = P(j, :);
        count = count +1;
    end
end
P1 = otherPoints(1, 2:3)';
P2 = otherPoints(2, 2:3)';

redBotRight = redBotRight(1, 2:3)';
redTopRight = P2;
redBotLeft = P1;
A = redTopRight - redBotRight;
A(3) = 0;
B = redBotLeft - redBotRight;
B(3) = 0;

C = cross(A, B);

if C(3) < 0
    redTopRight = P1;
    redBotLeft = P2;
end

A = center - (greenPoint + redTopRight)/2;
B = [1, 0]';

angle = rad2deg(subspace(A, B));

if A(1) < 0
    angle = 180 - angle;
end
if A(2) > 0
    angle = -angle;
end

%Determining the angles
angleVector = (center - blueClosest)/norm((center - blueClosest)) * totalAngle;
angleVector = [cos(angle), -sin(angle); sin(angle), cos(angle)] * angleVector;

angleVector = angleVector

lengthX = (norm(greenPoint - redTopRight) + norm(redBotRight - redBotLeft))/2;
lengthY = (norm(greenPoint - redBotLeft) + norm(redBotRight - redTopRight))/2;


testAngleX = rad2deg(acos(lengthX / blueWidth));
testAngleY = rad2deg(acos(lengthY / blueWidth));

totalAngleFromRed = norm([testAngleX, testAngleY]');


%Stitch the picture back together
imgN = cat(1,uint8(R));

hold on

%Plot all sorts of interesting points
plot(center(2), center(1), "wo");
plot(blueTop(2), blueTop(1), "xc", blueBottom(2), blueBottom(1), "oc", blueRight(2), blueRight(1), "oc", blueLeft(2), blueLeft(1), "oc");
plot(blueFurthest(2), blueFurthest(1), "wx", blueClosest(2), blueClosest(1), "w+");
plot(greenPoint(2), greenPoint(1), "bx");
plot(redBotRight(2), redBotRight(1), "-xk", redTopRight(2), redTopRight(1), "-^k", redBotLeft(2), redBotLeft(1), "<k");


img = imrotate(img, angle, "bilinear", "crop");

figure, image(img);

%View the image

%figure, image(imgN);


%Function for checking hte validity of a new point placements for a certain
%rank
function valid = addPoint(point, rank, P)
ThreshDist = 20;
    for i = 1:size(P, 1)
        if abs(P(i, 2) - point(2)) < ThreshDist && abs(P(i, 3) - point(3)) < ThreshDist
            if i ~= rank
                valid = false;
                return
            end
        end
    end
    valid = true;
    return
end
