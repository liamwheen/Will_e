function lost_navigate(bot, parts)

    num = length(parts);
    too_close = 10;
    dists = bot.dists;
    [dist, ind] = max(dists);
%     dist = -1;
%     sort_dist = sort(dists,'descend');
%     sort_dist(isnan(sort_dist))=[];
%     while dist == -1 && length(sort_dist)>2
%         ind = find(dists==sort_dist(1));
%         dist = dists(ind);
%         if ind == 1; dists = circshift(dists,1); ind = ind+1;end
%         if ind == length(dists); dists = circshift(dists,-1); ind = ind-1;end
%         if dists(ind-1)+20 < dist || isnan(dists(ind-1))|| dists(ind+1)+20 < dist || isnan(dists(ind+1))
%             sort_dist(1) = [];
%             dist = -1;
%         end
%     end
%     if dist == -1; dist = max(dists);end
    [min_dist, min_ind] = min(dists);

    dest_ang = 360*(ind-1)/abs(bot.num_of_scans);
    min_ang = 360*(min_ind-1)/abs(bot.num_of_scans);   % if we want to turn and reverse
    if dest_ang > 180
       dest_ang = dest_ang - 360;
    end

    dest_dist = min(randi([10,25]), dist-2);          %step at most 10 in that direction
    
    if min_ang > 180
       min_ang = min_ang - 360;
    end
    
    if min_dist < too_close
        % make robot reverse so it doesn't hit wall if it is too close
        dest_ang = min_ang;
        dest_dist = -10;
    end
    bot.turn_op(dest_ang);
    bot.move(dest_dist);
    dest_ang = -pi*dest_ang/180;
    for i = 1:num
        parts(i).turn(dest_ang); %turn the particle in the same way as the real robot
        parts(i).move(dest_dist); %move the particle in the same way as the real robot
        if ~parts(i).insideMap(); parts(i).randomPose(5);end
    end   
end