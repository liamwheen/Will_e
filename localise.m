%%%%For cw2 implementation: create 3 state vars - [lost, solving, solved]
%%%%simplify A* to just cut corners to improve speed and to reduce turns, if
%%%%possible, just extend straight before turning so the turn avoids the
%%%%corners all together. I think minimum turning and straight lines is the
%%%%practical solution for A*. If A* doesnt cut out corners, then a
%%%%autonomous component for the robot to detect oncoming collisions
%%%%(similar to that in robot_lab1 (orig)) is a good idea. That would mean
%%%%A* can be simplified and robot can follow simpler paths.



function [botSim] = localise(botSim,map,target)
hold on

%% setup code
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);
sens_num = 4;
botSim.setScanConfig(botSim.generateScanConfig(sens_num))

%generate some random particles inside the map
num = 300; % number of particles
particle(num,1) = BotSim; %how to set up a vector of objects
part_pos_ang = zeros(num,3);
part_dists = zeros(num,sens_num);
sense_scores = zeros(sens_num,1);
weights = zeros(num,1);
for i = 1:num
    particle(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particle(i).randomPose(0); %spawn the particles in random locations
    particle(i).setScanConfig(botSim.generateScanConfig(sens_num))
    part_pos_ang(i,1:2) = particle(i).getBotPos();
    part_pos_ang(i,3) = particle(i).getBotAng();
    particle(i).setScanConfig(botSim.generateScanConfig(sens_num))
end

%% Localisation code
max_lim = max(map);
maxNumOfIterations = 15;
n = 0;
converged = 0; %The filter has not converged yet
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    est_bot_pos_ang = mean(part_pos_ang(1:20,:));
    bot_dists = botSim.ultraScan(); %get a scan from the real robot.
    if sum(bot_dists==inf) > 0
        break;
    end
    mu = bot_dists;
    sigma = max(max_lim)/10;
    for i = 1:sens_num; pd(i) = makedist('Normal','mu',mu(i),'sigma',sigma); end
    
    for i = 1:num
        part_dists(i,:) = particle(i).ultraScan();
        for j = 1:sens_num 
            sense_scores(j) = pdf(pd(j), part_dists(i,j));
        end
        weights(i) = prod(sense_scores);
    end
    
    [weights, order] = sort(weights,'descend');
    particle = particle(order);
    part_pos_ang = part_pos_ang(order,:);
    
    for i = 0:270
        part = particle(end-i);
        if mod(i, 10) == 0
            part.randomPose(0);
        else
            
            new_part = datasample(part_pos_ang(1:30,:), 1);
            pos = new_part(1:2) + ((15-n)/4)*[randn() randn()];
            ang = new_part(3) + 0.1*randn;
            part.setBotPos(pos);    
            part.setBotAng(ang);
            part_pos_ang(end-i,:) = [pos ang];
        end
    end
    
    start = est_bot_pos_ang(1:2);
    if n > 1 && botSim.pointInsideMap(start)
        optim_path = a_star(start, target, botSim, 4);
        turn = det_dest_ang(start, optim_path(2,:)) - est_bot_pos_ang(3);
        botSim.turn(turn);
        move = max(0.2, sqrt(sum((optim_path(2,:)-optim_path(1,:)).^2)));
        bot_dists = botSim.ultraScan();
        front_dists = bot_dists(1);
        
        max_dist = 0;
        if front_dists < move
            for i = 1:length(bot_dists)
                max_dist = max(bot_dists(i),max_dist);
            end
            safe_turn = (find(max_dist==bot_dists)-1)*pi/3;
            botSim.turn(safe_turn)
            turn = turn+safe_turn;
            move = 5;
        end

    else
        turn = 0;
        move = 4;
    end
    
    botSim.move(move);

    for i = 1:num
        particle(i).turn(turn); %turn the particle in the same way as the real robot
        particle(i).move(move); %move the particle in the same way as the real robot
        part_pos_ang(i,1:2) = particle(i).getBotPos();
        part_pos_ang(i,3) = mod(particle(i).getBotAng(), 2*pi);
    end
    
    %estimate bot
    est_bot_pos_ang = mean(part_pos_ang(1:30,:));
    est_bot = BotSim(modifiedMap);
    est_bot.setScanConfig(botSim.generateScanConfig(sens_num))
    est_bot.setBotPos(est_bot_pos_ang(1:2));
    est_bot.setBotAng(est_bot_pos_ang(3));
    similar_check = sum(abs(est_bot.ultraScan()-botSim.ultraScan()));
   
    %convergence test
    convergence_score = sum(sqrt(sum((part_pos_ang(1:100,1:2)-circshift(part_pos_ang(1:100,1:2),1)).^2,2)));
    if convergence_score < 900 || similar_check < 4
        bot_dists = botSim.ultraScan(); %get a scan from the real robot.
        mu = bot_dists;
        sigma = 8;
        for i = 1:sens_num; pd(i) = makedist('Normal','mu',mu(i),'sigma',sigma); end
 
        est_part_dists(:) = est_bot.ultraScan();
        for j = 1:sens_num 
            sense_scores(j) = pdf(pd(j), est_part_dists(j));
        end
        sense_scores = sort(sense_scores);
        est_weight = prod(sense_scores(2:end));
 
        if est_weight < 1e-8
            for i = 1:100
                part = particle(i);
                part.randomPose(0);
            end
        else
            if sqrt(sum((est_bot.getBotPos()-target).^2)) < 4
                converged = 1;
            end
        end
    end

    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(10,'g'); %draw robot with line length 30 and green
        est_bot.drawBot(5);
        scatter(target(1), target(2),'filled')
        for i =1:num
            particle(i).drawBot(3); %draw particle with line length 3 and default color
        end
        drawnow;
    end

end
end

function dest_ang = det_dest_ang(bot_pos, dest)
    if bot_pos(1)<=dest(1) %check dest is in the right half of bots grid
        dest_ang = mod(atan((dest(2)-bot_pos(2))/(dest(1)-bot_pos(1))), 2*pi);
    else %dest is to the left of bot
        dest_ang = pi - atan((dest(2)-bot_pos(2))/(bot_pos(1)-dest(1)));
    end
end
    
