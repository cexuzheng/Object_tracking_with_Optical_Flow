function [objFrame, obj, props] = detectObject(OFvariable,Factor)
% DETECTOBJECT localizes the objects in the current video frame, detected
% through optical flow. Factor here is multiplied with the threshold 
% calculated using Otsu's method, to allow tuning the threshold.

% Velocities
u = OFvariable.Vx;
v = OFvariable.Vy;

% Compute absolute velocity
Vabs = sqrt(u.^2 + v.^2);

% Convert absolute velocity into binary image
Vabs_res = rescale(Vabs);
T = graythresh(Vabs_res)*Factor;
bw = imbinarize(Vabs_res,T);

% Perform morphological operations for image optimization
structElem = strel('disk',10);           % structuring element
bw_closed = imclose(bw,structElem);      % close objects
bw_filled = imfill(bw_closed,'holes');   % fill closed objects
bw_removed = bwareaopen(bw_filled, 100); % remove objects smaller 100px
bw_convhull = bwconvhull(bw_removed,'objects');
objFrame = bw_convhull;

% Information about detected objects
props = regionprops(objFrame,'ConvexHull');

% Polyshape objects for plotting
for iter = 1:numel(props)
    obj(iter) = polyshape(props(iter).ConvexHull);
end
if isempty(props)
    obj = [];
end

% Show Images
% figure('Name','Absolute Velocity')
% imshow(Vabs)
% figure('Name','Rescaled absolute Velocity')
% imshow(rescale(Vabs))
% figure('Name','BW Image')
% imshow(bw)
% figure('Name','Optimized Image')
% imshow(objFrame)
end