function [OpticFlow] = calcFlowLK(Frames,Smoothness)
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

% Weights
W = 1;

% Derivatives of Brightness
%mask
hx = [-1,8,0,-8,1]/12;
hy = hx';
%derivatives
Ix_n = double(imfilter(currentFrame,hx));
Iy_n = double(imfilter(currentFrame,hy));
It_n = double(currentFrame)-double(previousFrame);

% Smoothening Derivatives
Ix = double(imgaussfilt(Ix_n));
Iy = double(imgaussfilt(Iy_n));
It = double(imgaussfilt(It_n));



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