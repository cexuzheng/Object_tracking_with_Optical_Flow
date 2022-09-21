figure; imshow(im_grey)
hold('on')
plot(obj)
hold('off')

figure; imshow(rescale(Magnitude))
figure; imshow(rescale(Orientation))
figure; imshow(bw)

figure; imshow(im_grey)
hold on
tic
C = corner(im_grey);
toc
tic
C = detectHarrisFeatures(im_grey);
toc
%plot(C(:,1),C(:,2),'*', 'MarkerSize',5)
plot( 576  , 381,'o','MarkerSize', 5)
plot(round(C.Location(:,1)), round(C.Location(:,2)),'*', 'MarkerSize',5)
hold off

flow_GF.Vx(576*600+381)
flow_GF.Magnitude(576*600+381)
flow_GF.Vx(round(C.Location(:,1))*size(im_grey,1)+round(C.Location(:,2)))

figure; imshow(flow_GF.Vx)
figure; imshow(flow_GF.Vy)


tic
inhull([cy,cx;cx,cy;cx,cx;cy,cy;cy-Vy,cx-Vx], obj_GF(i_obj).Vertices)
toc


tic
inhull([cy,cx], obj_GF(i_obj).Vertices)
toc

tic
find(ans)
toc

tic
frameRGB = readFrame(V);
im_grey = rgb2gray(frameRGB);
cVideoFrame = medfilt2(im_grey);
cVideoFrame = imgaussfilt(cVideoFrame);
flow_GF = estimateFlow(opticFlow_LK,cVideoFrame);
toc


%%
syms t x
fig = figure;

syms t x
fanimator(@fplot,cos(x)+t,sin(x)+1,[-pi pi])
axis equal

writeAnimation(fig,'loop.gif','LoopCount',1)

%%


clear

V = VideoReader('trafficVid1.avi')

N = V.NumFrames;

opticFlow_GF = opticalFlowFarneback;
opticFlow_HS = opticalFlowHS;
opticFlow = opticalFlowLKDoG;
opticFlow_LK = opticalFlowLK;

frame_times_vector = zeros(1,N);
LK_times_vector = zeros(1,N);
GF_times_vector = zeros(1,N);
HS_times_vector = zeros(1,N);
stabilize_times_vector = zeros(1,N);
segment_times_vector = zeros(1,N);
tracking_times_vector = zeros(1,N);

warning('off')
obj_counter = 0;
prev_obj_counter = 0;

prev_indexes = [];
curr_objs = [];
times_vector = zeros(1,N);
colors_vector = [];
curr_indexes = [];
points = [];


for iter = 1:N
    tic
    frameRGB = readFrame(V);
    im_grey = rgb2gray(frameRGB);
    cVideoFrame = medfilt2(im_grey);
    cVideoFrame = imgaussfilt(cVideoFrame);
    frame_times_vector(iter) = toc;
    tic
    flow_str = estimateFlow(opticFlow_LK,cVideoFrame);
    LK_times_vector(iter) = toc;
    
    tic
    flow_GF = estimateFlow(opticFlow_GF,cVideoFrame);
    GF_times_vector(iter) = toc;
    tic
    if iter > 1
        Frames = cat(3,previousFrame,cVideoFrame);
    else
        Frames = cat(3,cVideoFrame,cVideoFrame);
    end

    previousFrame = cVideoFrame;
    flow_HS = calcFlowHS(Frames);
    HS_times_vector(iter) = toc;

    tic
    flow_str = stabilize_flow_image(im_grey,flow_str);
    stabilize_times_vector(iter) = toc;

    tic
    [img_obj_GF, curr_obj] = segment_objects(flow_str,0.5);
    segment_times_vector(iter) = toc;

    tic
    if iter > 1
        [curr_indexes, obj_counter, points] = tracking_function(curr_obj, prev_obj, prev_indexes, flow_str, obj_counter);
    else
        prev_indexes = zeros(1,numel(curr_obj));
    end
    prev_obj = curr_obj;
    prev_indexes = curr_indexes;
    tracking_times_vector(iter) = toc;
    
    
end

save('times.mat', "frame_times_vector","LK_times_vector","tracking_times_vector","GF_times_vector","HS_times_vector","stabilize_times_vector","segment_times_vector")
%%

V = VideoReader('trafficVid1.avi')

N = V.NumFrames;
frame_times_vector = zeros(1,N);
for iter = 1:N
    frameRGB = readFrame(V);
    tic
    im_grey = rgb2gray(frameRGB);
    cVideoFrame = medfilt2(im_grey);
    cVideoFrame = imgaussfilt(cVideoFrame);
    frame_times_vector(iter) = toc;
end
%%

total_time =HS_times_vector+frame_times_vector+tracking_times_vector+segment_times_vector;
total_stab_time = total_time+stabilize_times_vector;

fprintf('97.5% \n')
stab_fps = 1/(mean(total_stab_time) + 1.96*std(total_stab_time))
not_stab_fps = 1/(mean(total_time) + 1.96*std(total_time))


fprintf('mean \n')
stab_fps = 1/(mean(total_stab_time) )
not_stab_fps = 1/(mean(total_time) )

