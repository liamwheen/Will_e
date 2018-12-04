function new_paths = add_adj(step, curr_path, safe_list_coords, solved_points)
    new_paths = {};   
    %perim = [step 0; step step;0 step; -step step; -step 0; -step -step; 0 -step; step -step];
    [x, y] = meshgrid([-step:-1 0 1:step]);
    perim_list = [x(:) y(:)];
    adj_points = curr_path(end,:)+perim_list;
    %adj_points = curr_path(end, :)+perim;
    % removing points outside map  
    adj_points = adj_points(ismember(adj_points, safe_list_coords,'rows'),:);
    adj_points = adj_points(~ismember(adj_points, curr_path, 'rows'), :);
    adj_points = adj_points(~ismember(adj_points, solved_points, 'rows'), :);
    adj_points = reshape(adj_points',1, []);
    for i = 1:2:length(adj_points)%copys of path with each new node added to end
        new_paths{end+1} = [curr_path;adj_points(i:i+1)];
    end
end

