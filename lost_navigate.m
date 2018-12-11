function lost_navigate(bot, parts)
    toc
    num = length(parts);
    too_close = 15;
    dists = bot.dists;
    [dist, ind] = max(dists);
    [min_dist, min_ind] = min(dists);

    dest_ang = 360*(ind-1)/abs(bot.num_of_scans);
    min_ang = 360*(min_ind-1)/abs(bot.num_of_scans);   % if we want to turn and reverse
    if dest_ang > 180
       dest_ang = dest_ang - 360;
    end

    dest_dist = min(randi([40,80]), dist-10);          %step at most 10 in that direction
%     dest_dist = 40 + 10*(rand -0.5);
    
    if min_ang > 180
       min_ang = min_ang - 360;
    end
    
    if min_dist < too_close
        % make robot reverse so it doesn't hit wall if it is too close
        dest_ang = min_ang;
        dest_dist = -10;
    end
    bot.turn_op(dest_ang);
    [moved, turned] = bot.move(dest_dist);
    dest_ang = -pi*dest_ang/180;
    turned = -pi*turned/180;
    for i = 1:num
        parts(i).turn(dest_ang); %turn the particle in the same way as the real robot
        parts(i).move(moved); %move the particle in the same way as the real robot
        parts(i).turn(turned); % if the robot hit wall, reversed then turned
        if ~parts(i).insideMap(); parts(i).randomPose(5);end
    end   
    hold off
    parts(1).drawMap()
    for i =1:num
            parts(i).drawBot(3); %draw particle with line length 3 and default color
    end
    drawnow;
end