function navigate(bot, est_bot, est_bot_pos_ang)

start = est_bot_pos_ang(1:2);
if est_bot.pointInsideMap(start)
    optim_path = a_star(start, target, bot, 0.8);
    turn = det_dest_ang(start, optim_path(2,:)) - est_bot_pos_ang(3);
    bot.turn(turn);
    move = max(0.2, sqrt(sum((optim_path(2,:)-optim_path(1,:)).^2)));
    bot_dists = bot.ultraScan();
    front_dists = bot_dists(1);

    max_dist = 0;
    if front_dists < move
        for i = 1:length(bot_dists)
            max_dist = max(bot_dists(i),max_dist);
        end
        safe_turn = (find(max_dist==bot_dists)-1)*pi/3;
        bot.turn(safe_turn)
        turn = turn+safe_turn;
        move = 5;
    end
end





end