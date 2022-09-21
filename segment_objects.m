function [objFrame, obj, props] = segment_objects(OFvariable,threshlim)
    

    if(~exist('threshlim')); threshlim = 0.5; end
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
    % figure()
    % imshow(bw)
    
    % Information about detected objectsx
    props = regionprops(objFrame,'ConvexHull');
    
    % Polyshape objects for plotting
    obj = [];
    for iter = 1:numel(props)
        aux_obj = polyshape(props(iter).ConvexHull);
        aux_area = area(aux_obj);
        if(aux_area> 0.001*im_area && aux_area < 0.8*im_area)
            obj = [obj, aux_obj];
        end
    end

end