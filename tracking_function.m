function [curr_indexes, obj_counter, points] = tracking_function(curr_obj, prev_objs, prev_indexes, flow_str, obj_counter)
    curr_indexes = zeros(1,numel(curr_obj));
    points = zeros(numel(curr_obj), 2);
    % look for the searching points
    for i_obj = 1:numel(curr_obj)
        [cy, cx] = centroid(curr_obj(i_obj));
        cx = round(cx); cy = round(cy);
        Vx = flow_str.Vx(cx,cy); Vy = flow_str.Vy(cx,cy);
        points(i_obj,:) = [cy-Vy, cx-Vx];
    end
    % if cx-Vx, cy-Vy belongs to a previous object, then we consier
    % it a consistent detection
    for j_obj = 1:numel(prev_objs) % look for all the previous objects
        aux = inhull(points, prev_objs(j_obj).Vertices);
        if(any(aux)) % some point belonged to this object
            aux = find(aux);
            if prev_indexes(j_obj) ~= 0 
                % it was aready named
                curr_indexes(aux(1)) = prev_indexes(j_obj);
            else
                % we will name it now
                obj_counter = obj_counter+1;
                curr_indexes(aux(1)) = obj_counter;
            end
        end
    end
