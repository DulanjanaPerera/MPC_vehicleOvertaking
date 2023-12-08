function [xVerticesRotated, yVerticesRotated] = vehicle_drawing(xCenter, yCenter, head)
% Define the rotated rectangle's position and size
W = 3.5;
H = 2;
xRect = xCenter - W/2;      % x-coordinate of the bottom-left corner
yRect = yCenter - H/2;   % y-coordinate of the bottom-left corner
widthRect = W;  % Width of the rectangle
heightRect = H; % Height of the rectangle
theta = head; % Rotation angle in degrees

% Calculate the vertices of the rotated rectangle

xVertices = [xRect, xRect + widthRect, xRect + widthRect, xRect];
yVertices = [yRect, yRect, yRect + heightRect, yRect + heightRect];
xCenter = mean(xVertices);
yCenter = mean(yVertices);
xVerticesRotated = (xVertices - xCenter) * cos(theta) - (yVertices - yCenter) * sin(theta) + xCenter;
yVerticesRotated = (xVertices - xCenter) * sin(theta) + (yVertices - yCenter) * cos(theta) + yCenter;

end