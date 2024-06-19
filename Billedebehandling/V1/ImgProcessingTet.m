close
clear all
clc


%Get the image
img = imread('Straight 1.5m.png');
%img = imrotate(img, 180);


%View the image
figure, image(img);

%Convert img to double
imgD = double(img);

%Get color channels
R = imgD(:,:,1);
G = imgD(:,:,2);
Blue = imgD(:,:,3);

%Thresh
ThreshR = 190;
ThreshDist = 20;
%{
for i = 1:numel(R)
       
    if R(i) < ThreshR
        R(i) = 0;
    else
        R(i) = 255;
    end
        
end
%}

%{
imgN = cat(1,uint8(Blue));
figure, image(imgN);
for i = 1:numel(Blue)
       
    Blue(i) = Blue(i)*1 - G(i)/2.5 - R(i)/2.5;
        
end
imgN = cat(1,uint8(Blue));
figure, image(imgN);
%}

for i = 1:numel(Blue)
       
    Blue(i) = Blue(i)*1 - G(i)/2.5 - R(i)/2.5;
        
end

%Filter
%{
F = [1,2,1;
     2,4,2;
     1,2,1];

P1 = [0,0,0];
P2 = [0,0,0];
P3 = [0,0,0];
%}
F = [0, 1, 2, 1, 0;
     1, 2, 3, 2, 1;
     2, 3, 5, 3, 2;
     1, 2, 3, 2, 1;
     0, 1, 2, 1, 0];

P = [[0, 0, 0];
     [0, 0, 0];
     [0, 0, 0]];
Border = (size(F, 1) - 1) / 2;

greenPoint = [0, 0, 0];
%Image procssing (Finding the dots)
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
           if T > greenPoint(1, 1)
               greenPoint = [T, i, k];
           end
       end          
   end
end

for i = 1:size(R,1)
   for k = 1:size(R,2)
       %Don't the borders
       if k > Border && i > Border && k < size(R,2) - Border && i < size(R,1) - Border
           %Do the math
           T = 0;
           for x = 1:size(F, 1)
               for y = 1:size(F, 2)
                   T = T + R(x + i - 2, y + k - 2)*F(x, y);
               end
           end
           %T =  R(i-1,k-1)*F(1,1) +  R(i-1,k)*F(1,2) +  R(i-1,k+1)*F(1,3) + R(i,k-1)*F(2,1) +    R(i,k)*F(2,2) +    R(i,k+1)*F(2,3) + R(i+1,k-1)*F(3,1) +  R(i+1,k)*F(3,2) + R(i+1,k+1)*F(3,3);
           
           %Check if score is higher and placement isn't too close
           %{
           if P1(1) < T && (abs(P1(2) - i) > ThreshDist || abs(P1(3) - k) > ThreshDist)
               
               P3 = P2;
               
               P2 = P1;
               
               P1 = [T,i,k];
               
           end
           %}
           for j = 1:size(P, 1)
               if abs(P(j, 2) - i) < ThreshDist && abs(P(j, 3) - k) < ThreshDist
                   if P(j, 1) < T
                       if addPoint([T, i, k], j, P)
                           P(j, 1) = T;
                           P(j, 2) = i;
                           P(j, 3) = k;
                       else
                           break
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

center = (greenPoint(1, 2:3)' + P(1, 2:3)' + P(2, 2:3)' + P(3, 2:3)')/4;

bluePoints = [[0, 0, 0];
              [0, 0, 0]];
blueThreshold = 2500;
%{
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
               if norm([i, k]' - center) > norm(bluePoints(1, 2:3)' - center) || bluePoints(1, 1) == 0
                   if norm([i, k]' - bluePoints(2, 2:3)') > 50
                       bluePoints(1, :) = [T, i, k];
                   end
               elseif norm([i, k]' - center) > norm(bluePoints(2, 2:3)' - center) || bluePoints(2, 1) == 0
                   if norm([i, k]' - bluePoints(1, 2:3)') > 50
                       bluePoints(2, :) = [T, i, k];
                   end
               end

           end
       end
   end
end
%}

blueTop = [0, 0]';
blueBottom = [0, 0]';
blueLeft = [0, 0]';
blueRight = [0, 0]';


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
               if norm([i, k]' - center) > norm(bluePoints(1, 2:3)' - center) || bluePoints(1, 1) == 0
                   bluePoints(1, :) = [T, i, k];
               end
           end
       end
   end
end

bluePoints(2, :) = [0, center'] + [0, center'] - bluePoints(1, :);

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

%{
A = P(1,2:3)' - P(3,2:3)';
B = P(2, 2:3)' - P(3,2:3)';
rad2deg(subspace(B, A))
%}

%Find the cornerpoint
cornerPoint = P(1, 1:3);
point = 1;
for j = 2:size(P, 1)
    if P(j, 1) > cornerPoint(1)
        cornerPoint = P(j,1:3);
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
P1 = otherPoints(1, :) - cornerPoint;
P2 = otherPoints(2, :) - cornerPoint;

bottomPoint = P2;
lastPoint = P1;
A = P1(1, 2:3)';
A(3) = 0;
B = P2(1, 2:3)';
B(3) = 0;

C = cross(A, B);

if C(3) > 0
    bottomPoint = P1;
    lastPoint = P2;
end
bottomPoint = bottomPoint + cornerPoint;
lastPoint = lastPoint + cornerPoint;

A = (lastPoint(1, 2:3)' + cornerPoint(1, 2:3)')/2 - (greenPoint(1, 2:3)' + bottomPoint(1, 2:3)')/2;
B = [1, 0]';

angle = rad2deg(subspace(A, B));

if A(1) < 0
    angle = 180 - angle;
end
if A(2) > 0
    angle = -angle;
end

%Transform all points to vectors
cornerPoint = cornerPoint(1, 2:3)';
lastPoint = lastPoint(1, 2:3)';
greenPoint = greenPoint(1, 2:3)';
bottomPoint = bottomPoint(1, 2:3)';


%Calculate approximate distance from the marker
%Angle per pixel
longest = norm(bluePoints(1, 2:3)' - bluePoints(2, 2:3)');
anglePerPixel = 65 / 1600;
testAngle = longest * anglePerPixel;

actualSpacing = 0.2; %Smallest spacing on the marker

lengthX = (norm(greenPoint - lastPoint) + norm(cornerPoint - bottomPoint))/2;
lengthY = (norm(greenPoint - bottomPoint) + norm(cornerPoint - lastPoint))/2;

testSpacing = sin(deg2rad(testAngle)) * 2;

testLength = actualSpacing/2 / sin(deg2rad(testAngle/2));

testHeight = cos(deg2rad(testAngle/2)) * (actualSpacing/2 / sin(deg2rad(testAngle/2)))
%Calibration compensation
%testHeight = testHeight / 1.1073 - 0.012

testAngleX = rad2deg(acos(lengthX / longest));
testAngleY = rad2deg(acos(lengthY / longest));



%Stitch the picture back together
imgN = cat(1,uint8(R));

hold on

plot(center(2), center(1), "wo");
plot(blueTop(2), blueTop(1), "xc", blueBottom(2), blueBottom(1), "oc", blueRight(2), blueRight(1), "oc", blueLeft(2), blueLeft(1), "oc");
plot(bluePoints(1, 3), bluePoints(1, 2), "wx", bluePoints(2, 3), bluePoints(2, 2), "w+");
plot(cornerPoint(2), cornerPoint(1), "-xk", bottomPoint(2), bottomPoint(1), "-vk", lastPoint(2), lastPoint(1), ">k", greenPoint(2), greenPoint(1), "ok");
%plot(bottomPoint(3), bottomPoint(2), 'r+');

img = imrotate(img, angle, "bilinear", "crop");

figure, image(img);

%View the image

%figure, image(imgN);

%{
for j = 1:size(P, 1)
    plot(P(j, 3), P(j, 2), 'r*');

end
%}
%plot(P1(3), P1(2), 'r*');
%plot(P2(3), P2(2), 'r*');
%plot(P3(3), P3(2), 'r*');


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



