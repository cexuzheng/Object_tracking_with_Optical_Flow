figure(1)

subplot(1,2,1); imshow(frameRGB); title('original frame')
subplot(1,2,2); imshow(im_grey); title('processed')

%% Stabilizer

figure(2);
imshow(frameRGB); title('Previous OF')
hold on
plot(flow_prev,'DecimationFactor',[5 5],'ScaleFactor',10);
%%
figure(3); 
imshow(frameRGB); title('Corner detection')
hold on
C = detectHarrisFeatures(im_grey);
plot(C)
%%
figure(4); imshow(frameRGB); title('Stabilized OF')
hold on
plot(flow_str,'DecimationFactor',[5 5],'ScaleFactor',10);
%%

figure; imshow(flow_prev.Magnitude);
figure; imshow(flow_str.Magnitude);

figure; imshow(flow_prev.Magnitude > 0.7);
figure; imshow(flow_str.Magnitude>0.7);
%%
threshlim = 0.5;
OFvariable = flow_str;
Magnitude = OFvariable.Magnitude;
    im_area = size(Magnitude,1)*size(Magnitude,2);

    % Convert absolute velocity into binary image
    bw = Magnitude > threshlim;

    % Perform morphological operations for image optimization
    structElem = strel('disk',10);           % structuring element
    bw_closed = imclose(bw,structElem);      % close objects
    bw_filled = imfill(bw_closed,'holes');   % fill closed objects
    bw_removed = bwareaopen(bw_filled, 100); % remove objects smaller 100px
    bw_convhull = bwconvhull(bw_removed,'objects');
    objFrame = bw_convhull;

    %%
figure(1); imshow(Magnitude); title('Magnitude');
figure(2); imshow(bw); title('Binarized');
figure(3); imshow(bw_closed); title('Closing')
figure(4); imshow(bw_filled); title('Filling')
figure(5); imshow(bw_removed); title('Remove')
figure(6); imshow(bw_convhull); title('Convex hull')

%%
prev_frame = frameRGB;
figure(1); imshow(prev_frame); hold on
for i_obj = 1:numel(prev_obj)
    plot(prev_obj(i_obj), 'FaceColor',colors_vector(prev_indexes(i_obj),:));
end
[i,j] = centroid(curr_objs);
plot(points(:,1), points(:,2) ,'*g')

%%
frameRGB = readFrame(V);
im_grey = rgb2gray(frameRGB);
cVideoFrame = medfilt2(im_grey);
cVideoFrame = imgaussfilt(cVideoFrame);
flow_prev = estimateFlow(opticFlow_LK,cVideoFrame);
flow_str = stabilize_flow_image(im_grey, flow_prev);
[img_obj_GF, curr_obj] = segment_objects(flow_str,0.5);
[curr_indexes, obj_counter, points] = tracking_function(curr_obj, prev_obj, prev_indexes, flow_str, obj_counter);
colors_vector = [colors_vector; rand(obj_counter-prev_obj_counter,3)];

%%

figure(2); imshow(frameRGB); hold on
for i_obj = 1:numel(curr_obj)
    plot(curr_obj(i_obj), 'FaceColor',colors_vector(curr_names(i_obj),:));
end
[i,j] = centroid(curr_objs);
plot(i,j,'+b')
plot(points(:,1), points(:,2) ,'*g')


% frame 18