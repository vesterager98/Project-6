close
clear all
clc


%Get the image
img = imread('2mZ-2mY.png');
%img = imrotate(img, 37.5);


%View the image
figure, image(img);

%Convert img to double
imgD = double(img);

%Get color channels
R = imgD(:,:,1);
G = imgD(:,:,2);
Blue = imgD(:,:,3);

%{
imgN = cat(1,uint8(Blue));
figure, image(imgN);
for i = 1:numel(Blue)
       
    Blue(i) = Blue(i)*1 - G(i)/2.5 - R(i)/2.5;
        
end
imgN = cat(1,uint8(Blue));
figure, image(imgN);
%}

%Filter for the blue values to make the circle stand out more
for i = 1:numel(Blue)  
    Blue(i) = Blue(i)*1 - G(i)/2.5 - R(i)/2.5;     
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

A = center - greenPoint;
B = [1, 0]';

angle = rad2deg(subspace(A, B));

if A(1) < 0
    angle = 180 - angle;
end
if A(2) > 0
    angle = -angle;
end

%Calculate approximate distance from the marker
%Angle per pixel
blueWidth = norm(blueFurthest - center) * 2;
anglePerPixel = 65 / 1600;
testAngle = blueWidth * anglePerPixel;

actualSpacing = 0.2; %Actual diameter of the blue circle

testHeight = cos(deg2rad(testAngle/2)) * (actualSpacing/2 / sin(deg2rad(testAngle/2)))
%Calibration compensation
%testHeight = testHeight / 1.1073 - 0.012

testAngle = rad2deg(acos(norm(blueClosest - center)*2 / blueWidth))



%Stitch the picture back together
imgN = cat(1,uint8(R));

hold on

%Plot all sorts of interesting points
plot(center(2), center(1), "wo");
plot(blueTop(2), blueTop(1), "xc", blueBottom(2), blueBottom(1), "oc", blueRight(2), blueRight(1), "oc", blueLeft(2), blueLeft(1), "oc");
plot(blueFurthest(2), blueFurthest(1), "wx", blueClosest(2), blueClosest(1), "w+");
plot(greenPoint(2), greenPoint(1), "bx");

img = imrotate(img, angle, "bilinear", "crop");

figure, image(img);

%View the image

%figure, image(imgN);


