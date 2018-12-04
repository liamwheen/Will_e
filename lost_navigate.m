function lost_navigate(bot, parts)

    num = length(parts);

    dists = bot.scan_dists(bot.sens_num); %scan around
    [dist,ind] = max(dists);              %find max dist and ind of it
    dest_ang = 2*pi*(ind-1)/bot.sens_num; %make the robot point in the angle of furthest wall
    
    dest_dist = min(10, dist-2);          %step at most 10 in that direction

    bot.turn_op(dest_ang);
    bot.move(dest_dist);

    for i = 1:num
        parts(i).turn(turn); %turn the particle in the same way as the real robot
        parts(i).move(move); %move the particle in the same way as the real robot
    end   
end