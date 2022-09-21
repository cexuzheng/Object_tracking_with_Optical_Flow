%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Design and implementation of a technique to track moving objects based 
% on optical flow introduction.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Short Project by Ce Xu and Valentin Ott %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%% computing time; make video; compare GF and LK; 


clear

V = VideoReader('trafficVid1.avi','CurrentTime',32)
% V = VideoReader('VID2.mp4','CurrentTime',0)

N = V.NumFrames;


% PREPAREFIGURE defines the figure window for showing two different
% optical flow algorithms next to each other

h = figure('Name','Optical flow','Position',[250 25 2400 1440], 'Color','white','NumberTitle','off');
movegui(h);

hViewPanel = uipanel(h,'Position',[.01 .01 .98 .98],'Title','Plot of Optical Flow Vectors', ...
                        'BackgroundColor','white','BorderType','none', ...
                        'TitlePosition','centertop','FontWeight','bold', ...
                        'FontSize',14);
hViewPanel_SA = uipanel('Parent',hViewPanel,'Position',[.01 .01 .48 .98], ...
                        'Title',[' Optical Flow with ','algorithm'], ...
                        'TitlePosition','centertop','FontSize',12);

hViewPanel_SA_obj = uipanel('Parent',hViewPanel,'Position',[.51 .01 .48 .98], ...
                        'Title',['Detected Objects with ','algorithm'], ...
                        'TitlePosition','centertop','FontSize',12);

% Define panels to axes
Axis_SA = axes(hViewPanel_SA);
Axis_SA_obj = axes(hViewPanel_SA_obj);


opticFlow_GF = opticalFlowFarneback;
opticFlow_HS = opticalFlowHS;
opticFlow = opticalFlowLKDoG;
opticFlow_LK = opticalFlowLK;

warning('off')
obj_counter = 0;
prev_obj_counter = 0;

prev_indexes = [];
curr_objs = [];
colors_vector = [];
curr_indexes = [];
points = [];

Frame_save = [];

for iter = 1:18
    frameRGB = readFrame(V);
    im_grey = rgb2gray(frameRGB);
    cVideoFrame = medfilt2(im_grey);
    cVideoFrame = imgaussfilt(cVideoFrame);
    flow_prev = estimateFlow(opticFlow_LK,cVideoFrame);
    %%% self HS
%     if iter > 1
%         Frames = cat(3,previousFrame,cVideoFrame);
%     else
%         Frames = cat(3,cVideoFrame,cVideoFrame);
%     end
% 
%     previousFrame = cVideoFrame;
% 
%     flow_prev = calcFlowHS(Frames);
    % around 30 milisec

    % stabilizer
    % flow_str = stabilize_flow_image(im_grey, flow_prev);
     flow_str = flow_prev;
    % flow_GF = calcFlowHS(cat(3,cVideoFrame,cVideoFrame), );
    % 
    % [img_obj_GF, obj_GF] = detectObject(flow_str,1.2);
    % 
    % object detection
    [img_obj_GF, curr_obj] = segment_objects(flow_str,0.5);
    % object tracking
    if iter > 1
        [curr_indexes, obj_counter, points] = tracking_function(curr_obj, prev_obj, prev_indexes, flow_str, obj_counter);
    else
        prev_indexes = zeros(1,numel(curr_obj));
    end
    prev_obj = curr_obj;
    prev_indexes = curr_indexes;
    
    % now for drawing representation
    curr_objs = curr_obj(curr_indexes > 0);
    curr_centroids = points(curr_indexes > 0,:);
    curr_names = curr_indexes(curr_indexes > 0);

    imshow(frameRGB,'Parent',Axis_SA,'Border','tight')
    % label the objects
    if(~isempty(curr_names))
        tracker_im = insertText(frameRGB, curr_centroids, curr_names);
    else
        tracker_im = frameRGB;
    end
    imshow(tracker_im,'Parent',Axis_SA_obj,'Border','tight')

    hold([Axis_SA,Axis_SA_obj],'on')
    % paint the convex hulls with a consistent color
    colors_vector = [colors_vector; rand(obj_counter-prev_obj_counter,3)];
    prev_obj_counter = obj_counter;
    for i_obj = 1:numel(curr_objs)
        plot(curr_objs(i_obj),'Parent',Axis_SA_obj, 'FaceColor',colors_vector(curr_names(i_obj),:));
    end
    % plot the optical flow
    plot(flow_prev,'DecimationFactor',[5 5],'ScaleFactor',10,'Parent',Axis_SA);
    hold([Axis_SA,Axis_SA_obj],'off')

    F(iter) = getframe(gcf) ;
    drawnow
    pause(10^-3)
end


% create the video writer with 1 fps
writerObj = VideoWriter('truck.avi');
writerObj.FrameRate = 30;
% set the seconds per image
% open the video writer
open(writerObj);
% write the frames to the video
for i=1:length(F)
    % convert the image to a frame
    frame = F(i) ;    
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);
%%
% LK
mean_time = 0.0365

std_time = 0.0069

% GF

