target = [50,10];
bot = BigWill_e;
est_bot_pos_ang = [44, 22, 0];
state = 'localised';
est_bot = BotSim(bot.getMap())
est_bot.setBotPos(est_bot_pos_ang(1:2))
est_bot.setBotAng(est_bot_pos_ang(3))
while strcmp(state, 'localised')
    
    start = est_bot_pos_ang(1:2);
    if est_bot.pointInsideMap(start)
        optim_path = a_star(start, target, est_bot, 5)
        turn = det_dest_ang(start, optim_path(2,:)) - est_bot_pos_ang(3);
        bot.turn_op(-360*turn/(2*pi));
        360*turn/(2*pi)
        est_bot.turn(turn);
        move = max(1, sqrt(sum((optim_path(2,:)-optim_path(1,:)).^2)));

        front_dist = bot.front_scan();
        if isnan(front_dist)
            if 0.2 > mod(est_bot.getBotAng(),pi/2) || 1.4 < mod(est_bot.getBotAng(),pi/2)
                est_dist = est_bot.ultraScan();
                if est_dist(1) < 50 && est_dist > 6
                    disp('ERROR IN NAVIGATION - WALL ISNT WHERE WE THOUGHT')
                end
            end
        end
        

        if ~isnan(front_dist) && front_dist < move
            if front_dist-move > -3
                move = move-3;
            else
                disp('PROBLEM IN REAL LOCALISE')
            end
%             safe_turn = (find(max_dist==bot_dists)-1)*pi/3;
%             bot.turn(safe_turn)
%             est_bot.turn(safe_turn)
%             turn = turn+safe_turn;
%             move = 5;
        end
% 
%     else
%         turn = 0;
%         move = 4;
    
        bot.move(move);
        est_bot.move(move);
        pos = est_bot.getBotPos();
        est_bot_pos_ang = [pos(1), pos(2), est_bot.getBotAng()];
%         for i = 1:num
%             particle(i).turn(turn); %turn the particle in the same way as the real robot
%             particle(i).move(move); %move the particle in the same way as the real robot
%             part_pos_ang(i,1:2) = particle(i).getBotPos();
%             part_pos_ang(i,3) = mod(particle(i).getBotAng(), 2*pi);
%         end
    
%     %estimate bot
%     est_bot_pos_ang = mean(part_pos_ang(1:30,:));
%     est_bot = BotSim(map);
%     est_bot.setScanConfig(bot.generateScanConfig(bot.num_of_scans))
%     est_bot.setBotPos(est_bot_pos_ang(1:2));
%     est_bot.setBotAng(est_bot_pos_ang(3));
%     similar_check = sum(abs(est_bot.ultraScan()-bot.ultraScan()));
   
%     %convergence test
%     convergence_score = sum(sqrt(sum((part_pos_ang(1:100,1:2)-circshift(part_pos_ang(1:100,1:2),1)).^2,2)));
%     if convergence_score < 900 || similar_check < 4
%         bot_dists = bot.ultraScan(); %get a scan from the real robot.
%         mu = bot_dists;
%         sigma = 8;
%         for i = 1:bot.num_of_scans; pd(i) = makedist('Normal','mu',mu(i),'sigma',sigma); end
%  
%         est_part_dists(:) = est_bot.ultraScan();
%         for j = 1:bot.num_of_scans 
%             sense_scores(j) = pdf(pd(j), est_part_dists(j));
%         end
%         sense_scores = sort(sense_scores);
%         est_weight = prod(sense_scores(2:end));
%  
%         if est_weight < 1e-8
%             for i = 1:100
%                 part = particle(i);
%                 part.randomPose(0);
%             end
%         else
%             if sqrt(sum((est_bot.getBotPos()-target).^2)) < 4
%                 converged = 1;
%             end
%         end
%     end
        if sum(abs(est_bot.getBotPos()-target))<2
            state = 'done';
            disp('Gucci')
        end
    end
    %% Drawing

%         hold off; %the drawMap() function will clear the drawing when hold is off
%         bot.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
%         bot.drawBot(10,'g'); %draw robot with line length 30 and green
%         est_bot.drawBot(5);
%         scatter(target(1), target(2),'filled')
%         for i =1:num
%             particle(i).drawBot(3); %draw particle with line length 3 and default color
%         end
%         drawnow;


end
function dest_ang = det_dest_ang(bot_pos, dest)
    dest_ang = atan2(dest(2) - bot_pos(2), dest(1) - bot_pos(1));
end
% function dest_ang = det_dest_ang(bot_pos, dest)
%     if bot_pos(1)<=dest(1) %check dest is in the right half of bots grid
%         dest_ang = mod(atan2((dest(2)-bot_pos(2))/(dest(1)-bot_pos(1))), 2*pi);
%     else %dest is to the left of bot
%         dest_ang = pi - atan((dest(2)-bot_pos(2))/(bot_pos(1)-dest(1)));
%     end
% end