%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Design and implementation of a technique to track moving objects based 
% on optical flow introduction.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Short Project by Ce Xu and Valentin Ott %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear

V = VideoReader('trafficVid1.avi')

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
                        'Title',[' Sparse Optical Flow with ','algorithm'], ...
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
obj_names = [];
obj_counter = 0;

prev_indexes = [];
curr_objs = [];

for iter = 1:N
    frameRGB = readFrame(V);
    im_grey = rgb2gray(frameRGB);
    cVideoFrame = medfilt2(im_grey);
    cVideoFrame = imgaussfilt(cVideoFrame);
    flow_GF = estimateFlow(opticFlow_GF,cVideoFrame);
    % [img_obj_GF, obj_GF] = detectObject(flow_GF,1.2);
    [img_obj_GF, obj_GF] = segment_objects(flow_GF,0.5);
    curr_objs = [];
    curr_centroids = [];
    curr_names = [];
    curr_indexes = zeros(1,numel(obj_GF));
    if iter > 1
        % look for the searching points
        points = zeros(numel(obj_GF), 2);
        for i_obj = 1:numel(obj_GF)
            [cy, cx] = centroid(obj_GF(i_obj));
            cx = round(cx); cy = round(cy);
            Vx = flow_GF.Vx(cx,cy); Vy = flow_GF.Vy(cx,cy);
            points(i_obj,:) = [cy-Vy, cx-Vx];
        end
        % if cx-Vx, cy-Vy belongs to a previous object, then we consier
        % it a consistent detection
        for j_obj = 1:numel(prev_objs)
            aux = inhull(points, prev_objs(j_obj).Vertices);
            if(any(aux))
                aux = find(aux);
                curr_objs = [curr_objs,obj_GF(aux(1))];
                curr_centroids = 
                if prev_indexes(j_obj) ~= 0 
                    curr_indexes(aux(1)) = prev_indexes(j_obj);
                else
                    obj_counter = obj_counter+1;
                    curr_indexes(aux(1)) = obj_counter;
                end
                curr_names = [curr_names, curr_indexes(aux(1))];
            end
        end
    end
    prev_objs = obj_GF;
    prev_indexes = curr_indexes;
    
    
    imshow(frameRGB,'Parent',Axis_SA,'Border','tight')
    imshow(frameRGB,'Parent',Axis_SA_obj,'Border','tight')
    hold([Axis_SA,Axis_SA_obj],'on')
    plot(flow_GF,'DecimationFactor',[5 5],'ScaleFactor',10,'Parent',Axis_SA);
    plot(curr_objs,'Parent',Axis_SA_obj);
    hold([Axis_SA,Axis_SA_obj],'off')
    pause(10^-3)
end
