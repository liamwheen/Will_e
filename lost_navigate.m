function lost_navigate(bot, parts)

    num = length(parts);

    dists = bot.get_scans() %scan around
    [dist,ind] = max(dists);              %find max dist and ind of it
    [min_dist,min_ind] = min(dists);
    dest_ang = 2*pi*(ind-1)/bot.num_of_scans; %make the robot point in the angle of furthest wall
   
    dest_ang = -360*(ind-1)/abs(bot.num_of_scans);
    min_ang = -360*(min_ind-1)/abs(bot.num_of_scans);   % if we want to turn and reverse
    if dest_ang < -180
       dest_ang = 360 + dest_ang;
    end
    dest_dist = min(10, dist-2);          %step at most 10 in that direction
    if min_dist < 15
        % make robot reverse so it doesn't hit wall if it is too close
        dest_ang = min_ang + 180;
        dest_dist = -dest_dist
    end
    bot.turn_op(dest_ang);
    bot.move(dest_dist);

    for i = 1:num
        parts(i).turn(dest_ang - parts(i).getBotAng); %turn the particle in the same way as the real robot
        parts(i).move(dest_dist); %move the particle in the same way as the real robot
    end   
end