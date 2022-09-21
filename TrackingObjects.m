%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Design and implementation of a technique to track moving objects based 
% on optical flow introduction.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Short Project by Ce Xu and Valentin Ott %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
% Location of video frames
folder_name = 'video_fot/';
files = dir([folder_name,'*.bmp']);

% Length of video
partOfVideo = [0,10]; % in percentage [startFrame,endFrame]
currentFrame = defineFrames(files,partOfVideo);

% Figures
Axes = prepareFigure('Optical Flow - Motion Detection','Lucas-Kanade','Gunner Farneback');
Axes_self = prepareFigure('Optical Flow - Motion Detection - Own Computation', ...
            'own Lucas-Kanade computation','own Horn&Schunck computaion');

% Optical flow Algorithms
opticFlow_LK = opticalFlowLK;
opticFlow_GF = opticalFlowFarneback;
flow_HS_self = opticalFlow;
flow_LK_self = opticalFlow;

% Loop for each frame of video
iter = 0;
for cf = currentFrame
    
    iter = iter + 1;
    
    % Get VideoFrame
    [cVideoFrame,frameRGB] = prepareFrame(folder_name,files,cf);
    
    % Define current and previous frame    
    if iter == 1
        Frames = cat(3,cVideoFrame,cVideoFrame);
    else
        Frames = cat(3,previousFrame,cVideoFrame);
    end
    previousFrame = cVideoFrame;
    
    % Compute Optical Flowclose 
    flow_LK = estimateFlow(opticFlow_LK,cVideoFrame);
    flow_GF = estimateFlow(opticFlow_GF,cVideoFrame);
    flow_LK_self = calcFlowLK(Frames);
    flow_HS_self = calcFlowHS(Frames);
    
    % Detect Objects
    if cf>1
        [img_obj_LK, obj_LK] = detectObject(flow_LK,1);
        [img_obj_GF, obj_GF] = detectObject(flow_GF,1.2);
        [img_obj_LK_self, obj_LK_self] = detectObject(flow_LK_self,1);
        [img_obj_HS_self, obj_HS_self] = detectObject(flow_HS_self,1.2);
    end
    
    % Show Original VideoFrame
    imshow(frameRGB,'Parent',Axes(1),'Border','tight')
    imshow(frameRGB,'Parent',Axes(2),'Border','tight')
    imshow(frameRGB,'Parent',Axes(3),'Border','tight')
    imshow(frameRGB,'Parent',Axes(4),'Border','tight')
    imshow(frameRGB,'Parent',Axes_self(1),'Border','tight')
    imshow(frameRGB,'Parent',Axes_self(2),'Border','tight')
    imshow(frameRGB,'Parent',Axes_self(3),'Border','tight')
    imshow(frameRGB,'Parent',Axes_self(4),'Border','tight')
%     
    % Show resulting Optical Flow Vectors
    hold([Axes,Axes_self],'on')
    plot(flow_LK,'DecimationFactor',[5 5],'ScaleFactor',10,'Parent',Axes(1));
    plot(flow_GF,'DecimationFactor',[5 5],'ScaleFactor',10,'Parent',Axes(2));
    plot(flow_LK_self,'DecimationFactor',[5 5],'ScaleFactor',10,'Parent',Axes_self(1));
    plot(flow_HS_self,'DecimationFactor',[5 5],'ScaleFactor',10,'Parent',Axes_self(2));
    if cf>1
        plot(obj_LK,'Parent',Axes(3));
        plot(obj_GF,'Parent',Axes(4));
        plot(obj_LK_self,'Parent',Axes_self(3));
        plot(obj_HS_self,'Parent',Axes_self(4));
    end
    hold([Axes,Axes_self],'off')
    
    % Pause
    pause(10^-3)
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function cf = defineFrames(files,partOfVideo)
% DEFINEFRAMES defines the VideoFrames taken into account for the
% operation. This is equal to defining the part of the Video to be
% considered.

if nargin == 1
    cf = 1:size(files);
elseif nargin == 2
    numFrames = numel(files);
    limits = partOfVideo/100*numFrames;
    lim_round = ceil(limits);
    cf = lim_round(1):1:lim_round(2);
    cf(cf==0|cf>numFrames) = [];
else 
    error('Number of Input Arguments must be smaller or equal 2.')
end
end

function [Axes] = prepareFigure(Name,NameSparseAlgorithm,NameDenseAlgorithm)
% PREPAREFIGURE defines the figure window for showing two different
% optical flow algorithms next to each other

h = figure('Name',Name,'Position',[250 25 800 600], ...
            'Color','white','NumberTitle','off');
movegui(h);

hViewPanel = uipanel(h,'Position',[.01 .01 .98 .98],'Title','Plot of Optical Flow Vectors', ...
                        'BackgroundColor','white','BorderType','none', ...
                        'TitlePosition','centertop','FontWeight','bold', ...
                        'FontSize',14);
hViewPanel_SA = uipanel('Parent',hViewPanel,'Position',[.01 .51 .48 .48], ...
                        'Title',[' Sparse Optical Flow with ',NameSparseAlgorithm], ...
                        'TitlePosition','centertop','FontSize',12);
hViewPanel_DA = uipanel('Parent',hViewPanel,'Position',[.51 .51 .48 .48], ...
                        'Title',[' Dense Optical Flow with ',NameDenseAlgorithm], ...
                        'TitlePosition','centertop','FontSize',12);
hViewPanel_SA_obj = uipanel('Parent',hViewPanel,'Position',[.01 .01 .48 .48], ...
                        'Title',['Detected Objects with ',NameSparseAlgorithm], ...
                        'TitlePosition','centertop','FontSize',12);
hViewPanel_DA_obj = uipanel('Parent',hViewPanel,'Position',[.51 .01 .48 .48], ...
                        'Title',[' Detected Objects with ',NameDenseAlgorithm], ...
                        'TitlePosition','centertop','FontSize',12);                    

% Define panels to axes
Axis_SA = axes(hViewPanel_SA);
Axis_DA = axes(hViewPanel_DA);
Axis_SA_obj = axes(hViewPanel_SA_obj);
Axis_DA_obj = axes(hViewPanel_DA_obj);

% Output Axes
Axes = [Axis_SA,Axis_DA,Axis_SA_obj,Axis_DA_obj];
end

function[frame,frameRGB] = prepareFrame(folder_name,files,cf)
% PREPAREFRAME loads VideoFrame and performs image optimization algorithms
% to prepare Frame for Optical Flow estimations
    
    % Load Image
    frameRGB = imread([folder_name, files(cf).name]);
    % Convert image to grayscale image
    frameGray = im2gray(frameRGB);
    % Noise reduction with median filter
    frameMedian = medfilt2(frameGray);
    % Define Output Image
    frame = frameMedian;
end