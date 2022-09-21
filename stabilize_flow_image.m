function [OpticFlow] = stabilize_flow_image(im_grey,in_flow)
    C = detectHarrisFeatures(im_grey);
    % Obtain the velocities images
    im_Vx = in_flow.Vx;
    im_Vy = in_flow.Vy;
    % Make the corner points readable for indexing
    points = round((C.Location(:,1))-1)*size(im_grey,1)+round(C.Location(:,2));
    % Correct the velocity image with the median
    im_Vx = im_Vx-median(im_Vx(points));
    im_Vy = im_Vy-median(im_Vy(points));
    % Create the new optical flow structure
    OpticFlow = opticalFlow(im_Vx,im_Vy);
end
