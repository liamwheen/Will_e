function [optim_path] = a_star(start, dest, rob, bot_rad, low_res)  

    %resolutions for point sweeps
    max_lim = max(rob.getMap());
    x_range = [1 max_lim(1)-1];
    y_range = [1 max_lim(2)-1];
    if nargin < 5
        low_res = 6;
    end
    high_res = 2*low_res-1;
    num_points = 6;
    step = 1; %only jump one node at a time

    [X, Y] = meshgrid(linspace(x_range(1),x_range(2),num_points),linspace(y_range(1),y_range(2),num_points));
    dest_point = round_to_point(dest, X, Y);

    %% reconfigure X, Y etc.
    [x, y] = meshgrid(1:num_points, 1:num_points);
    coord_list = [X(:) Y(:)];
    point_list = [x(:) y(:)];
    safe_list_bool = rob.pointInsideMap(coord_list);
    safe_list_coords = point_list(safe_list_bool==1,:);
    %removing dots too close to the edge
    run_count = 1;
    while run_count <= size(safe_list_coords,1)
        p_x = X(1,safe_list_coords(run_count,1)); p_y = Y(safe_list_coords(run_count,2),1);
        around_point = [p_x+bot_rad p_y; p_x+bot_rad p_y+bot_rad;p_x p_y+bot_rad; p_x-bot_rad p_y+bot_rad; p_x-bot_rad p_y; p_x-bot_rad p_y-bot_rad; p_x p_y-bot_rad; p_x+bot_rad p_y-bot_rad];
        if ismember(0, rob.pointInsideMap(around_point)); safe_list_coords(run_count,:) = []; run_count=run_count-1; end
        run_count=run_count+1;
    end
    %add goal back in if removed
    safe_list_coords(end,:) = dest_point;

%     scatter(X(1,safe_list_coords(:,1)), Y(safe_list_coords(:,2)),5,'filled')

    dest_point = round_to_point(dest, X, Y);
    start_point = round_to_point(start, X, Y);
    if sum(start_point == dest_point) == 2
        optim_path = [start; dest];
%         plot(optim_path(:,1), optim_path(:,2))
        return
    end
    
    path_list = {start_point};
    closed = cell(length(safe_list_coords),2); %collection of efficient paths and end point in each path
    optim_path = {};

    run_count = 0;
    while run_count<500
        run_count=run_count+1;     
        solved_points = reshape([closed{:,1}]',2,[])';  
        adj_paths = add_adj(step, path_list{1}, safe_list_coords, solved_points);
        path_list = [adj_paths, path_list(2:end)];  
        i = 1;
        while i <= length(path_list)
            [duplicate, indx] = ismember(path_list{i}(end,:),solved_points,'rows');
            %replace path with new shortest path to point
            if duplicate
                if length(path_list{i})<length(closed{indx,2})
                    closed{indx,2} = path_list{i};
                else
                    path_list(i)=[];
                end
            end
            i=i+1;
        end
        dest_cells = cell(1, length(path_list)); %necessary to run cellfun (must be equally sized cell array)
        dest_cells(:) = {dest_point};
        costs = cellfun(@f_n, path_list, dest_cells);
        [costs, new_order] = sort(costs); %sorting fn and open into fn order (smallest first)
        path_list = path_list(new_order);
        if isempty(path_list) %goal cannot be reached through given points (more res required)
            optim_path = a_star(start, dest, rob, bot_rad, round(num_points*1.2));break;end
        path = path_list{1};
        closed{run_count,1} = path(end,:);
        closed{run_count,2} = path;

        if costs(1) == -1 %optimum path found!
            optim_path_points = path_list{1}(2:end-1,:);
            optim_path = [start; X(1,optim_path_points(:,1))' Y(optim_path_points(:,2),1); dest];
%             plot(optim_path(:,1), optim_path(:,2))
            %check for any points crossing through walls
            
            
%             x_coords = optim_path(:,1);
%             y_coords = optim_path(:,2);
%             x_res = [];
%             y_res = [];
%             for i = 1:length(x_coords)-1
%                 x_res = [x_res(1:end-1) linspace(x_coords(i),x_coords(i+1), 6)];
%                 y_res = [y_res(1:end-1) linspace(y_coords(i),y_coords(i+1), 6)];
%             end
% 
%             inds = find(~rob.pointInsideMap([x_res' y_res'])); %in terms of high res
%             real_inds = unique(floor(inds/5)); %in terms of actual path indices
%             adj_inds = find(real_inds-circshift(real_inds,1)==1); %check if multiple points are of map in a row
% 
%             ind = 1;
            %if found, resample those unsafe areas with high res
            %this method will be too costly for cw2
%             while ind <= length(real_inds)
%                 interp_start = optim_path(max(real_inds(ind),1),:);
%                 end_ind = ind+sum(ismember(ind+1:ind+5,adj_inds));   
%                 interp_end = optim_path(real_inds(end_ind)+2,:);
%                 orig_path_length = length(optim_path);
%                 replace_path = a_star(interp_start, interp_end, rob, bot_rad, high_res);
%                 optim_path = [optim_path(1:real_inds(ind)-1,:); replace_path; optim_path(real_inds(end_ind)+3:end,:)];
%                 length_change = length(optim_path)-orig_path_length;
%                 real_inds = real_inds+length_change;
%                 ind=end_ind+1;
%             end
            



            i = 2;
            while i < length(optim_path) %pruning redundant nodes
                p0 = round(optim_path(i-1,:));
                p1 = round(optim_path(i,:));
                p2 = round(optim_path(i+1,:));
                %check the dy/dx of two edges, if same, remove middle node
                if (p2(2)-p1(2))/(p2(1)-p1(1)) == (p1(2)-p0(2))/(p1(1)-p0(1))
                    optim_path(i,:) = [];
                else
                    i = i+1;
                end
            end
            
%             plot(optim_path(:,1), optim_path(:,2))
            drawnow;
            break
        end
    end
    %if all fails, just return straight line to dest
    if isempty(optim_path)
        optim_path = [start; dest];
    end
end
