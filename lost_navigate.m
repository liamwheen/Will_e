function lost_navigate(rob, parts)
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];

num = length(parts);

bot_pos = [50,50];
bot_ang = 0;

dest = randi([0, 100], 1, 2);

iter = 0;
converged = 0;

while converged == 0
    dists = rob.scan_dists(rob.sens_num);
    [dist,ind] = max(dists);
    dest_ang = 2*pi*(ind-1)/rob.sens_num;
    
    dest_dist = min(10, dist-2);
    fear = 15;

    rob.turn_op(dest_ang);
    rob.move(dest_dist);
    for i = 1:num
        parts(i).turn(turn); %turn the particle in the same way as the real robot
        parts(i).move(move); %move the particle in the same way as the real robot
        part_pos_ang(i,1:2) = particle(i).getBotPos();
        part_pos_ang(i,3) = mod(particle(i).getBotAng(), 2*pi);
    end
    
    if any(dists<fear) && dest_dist>fear %danger approaching
        if any(dists<buffer) %avoid hitting sides on walls despite clear path ahead
            disp(['step: ' int2str(iter) ' buf-sensors: ' mat2str(find(dists<buffer))])
            bump_sensor = find(dists<buffer, 1);
            if bump_sensor<sens_num/2; turn_ang = -0.7;
            else; turn_ang = 0.7; end
            rob.turn(turn_ang);
        else
            disp(['step: ' int2str(iter) ' sensors: ' mat2str(find(dists<fear))])
            priority_cross_points = []; %rearranging sensors into prioritised order (front first)
            priority_dists = [];
            for i = 1:round((sens_num-1)/2)
                priority_cross_points = vertcat(priority_cross_points,cross_points(i,:));
                priority_cross_points = vertcat(priority_cross_points,cross_points(end+1-i,:));
                priority_dists = vertcat(priority_dists,dists(i));
                priority_dists = vertcat(priority_dists,dists(end+1-i));
            end

    %         [val, arg] = max(priority_dists(1:3));
    %         %checking that new route is at least a 1.5x improvement in safeness
    %         if val<1.5*buffer; [val, arg] = max(priority_dists(3:end)); end

            %testing this way to potentially avoid zig zag as discussed below
            %by taking more direct route when available
            %%%%%%%%
            safe_arg = find(priority_dists>dest_dist);
            if ~any(safe_arg)
                [val, safe_arg] = max(priority_dists(1:3));
                %checking that new route is at least a 1.5x improvement in safeness
                if val<1.5*fear; [val, safe_arg] = max(priority_dists(3:end)); end
            end
            %%%%%%%%%

            dest_ang = det_dest_ang(bot_pos, priority_cross_points(safe_arg(1),:));
            disp(['new angle: ' num2str(dest_ang/pi) ' new dest: ' mat2str(priority_cross_points(safe_arg,:))])
            turn_ang = dest_ang - bot_ang;
            rob.turn(turn_ang);
        end
    end
        

    rob.move(jump_size);
    scatter(bot_pos(1), bot_pos(2),10,'b')
    text(bot_pos(1)+0.02, bot_pos(2)+0.02, int2str(iter))
    bot_pos = rob.getBotPos();
    bot_ang = rob.getBotAng();
    iter = iter+1;
end

function dest_ang = det_dest_ang(bot_pos, dest)
    if bot_pos(1)<=dest(1) %check dest is in the right half of bots grid
        dest_ang = mod(atan((dest(2)-bot_pos(2))/(dest(1)-bot_pos(1))), 2*pi);
    else %dest is to the left of bot
        dest_ang = pi - atan((dest(2)-bot_pos(2))/(bot_pos(1)-dest(1)));
    end
end
end