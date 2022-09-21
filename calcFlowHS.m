function [OpticFlow] = calcFlowHS(Frames,Smoothness)
% CALCFLOWHS computes the optical flow using the Horn-Schunck Method for a
% grayscale image. The equation to be solved is:
%
% Ix*u + Iy*v + It = 0
%
% Ix,Iy,It are the spatiotemporal image brightness derivatives

if nargin == 1
    alpha = 1;
elseif nargin == 2
    alpha = Smoothness;
else
    error('The amount of input variables is wrong.')
end

% Defining Frames
currentFrame = Frames(:,:,2);
previousFrame = Frames(:,:,1);

% Initialization
u = zeros(size(currentFrame,1),size(currentFrame,2));
v = zeros(size(currentFrame,1),size(currentFrame,2));

% Derivatives of Brightness
%mask
hx = -fspecial('sobel');
hy = hx';
%derivatives
Ix = double(imfilter(currentFrame,hx));
Iy = double(imfilter(currentFrame,hy));
It = double(currentFrame)-double(previousFrame);

% Compute Optical Flow
hvel = [0,1,0;1,0,1;0,1,0];
for iter = 1:1
    % Estimation for Velocities
    u_bar = imfilter(u,hvel);
    v_bar = imfilter(v,hvel);

    % Estimate Optical Flow
    u = u_bar - (Ix.*(Ix.*u_bar+Iy.*v_bar+It))./(alpha^2+Ix.^2+Iy.^2);
    v = v_bar - (Iy.*(Ix.*u_bar+Iy.*v_bar+It))./(alpha^2+Ix.^2+Iy.^2);

    %figure()
    %imshow(u)
end

% Compute Optical Flow Vectors
OpticFlow = opticalFlow(u,v);

% figure()   
% plot(OpticFlow,'DecimationFactor',[5 5],'ScaleFactor',10);
end 