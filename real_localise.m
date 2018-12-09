%%%%For cw2 implementation: create 3 state vars - [lost, solving, solved]
%%%%simplify A* to just cut corners to improve speed and to reduce turns, if
%%%%possible, just extend straight before turning so the turn avoids the
%%%%corners all together. I think minimum turning and straight lines is the
%%%%practical solution for A*. If A* doesnt cut out corners, then a
%%%%autonomous component for the robot to detect oncoming collisions
%%%%(similar to that in robot_lab1 (orig)) is a good idea. That would mean
%%%%A* can be simplified and robot can follow simpler paths.



function [bot] = real_localise(bot,target)
state = 'lost'; %can be lost, localised, or done
%generate some random particles inside the map
map = bot.getMap();
num = 300; % number of particles
particle(num,1) = BotSim; %set up a vector of objects
max_lim = max(map);
part_pos_ang = zeros(num,3);
part_dists = zeros(num,bot.num_of_scans);
sense_scores = zeros(bot.num_of_scans,1);
weights = zeros(num,1);
for i = 1:num
    particle(i) = BotSim(map);  %each particle should use the same map as the botSim object
    particle(i).randomPose(5); %spawn the particles in random locations
    particle(i).setScanConfig(particle(i).generateScanConfig(bot.num_of_scans))
    part_pos_ang(i,1:2) = particle(i).getBotPos();
    part_pos_ang(i,3) = particle(i).getBotAng();
end

iter = 1;
%%%%%LOST MODE%%%%%
% bot.move_to_good_scan()
while strcmp(state,'lost') && iter<15
    bot.get_scans(); %scan around
    num_nans = find(isnan(bot.dists));
    while length(num_nans) > bot.num_of_scans - 3
        bot.turn_op(15);
        for i = 1:num
            particle(i).turn(-0.2618); %turn the particle in the same way as the real robot
        end
        bot.get_scans();
        num_nans = find(isnan(bot.dists));
    end
    mu = bot.dists;
    ind = find(isnan(mu));
    mu(ind) = [];
    if isempty(mu)
        continue;
    end
    sigma = 6;
    for i = 1:bot.num_of_scans-length(ind); pd(i) = makedist('Normal','mu',mu(i),'sigma',sigma); end
    
    good_scans = 1:bot.num_of_scans;
    good_scans(ind) =[];
    for i = 1:num
        part_dists(i,:) = particle(i).ultraScan();
        k = 1;
        for j = good_scans
            sense_scores(k) = pdf(pd(k), part_dists(i,j));
            k=k+1;
        end
        weights(i) = prod(sense_scores(sense_scores~=0));
    end
    
    [weights, order] = sort(weights,'descend');
    particle = particle(order);
    part_pos_ang = part_pos_ang(order,:);
    
    for i = 0:250
        part = particle(end-i);
        if mod(i, 10) == 0
            part.randomPose(5);
        else          
            new_part = datasample(part_pos_ang(1:50,:), 1);
            pos = new_part(1:2) + ((20-iter)/4)*[randn() randn()];
            ang = new_part(3) + 0.2*randn;
            part.setBotPos(pos);    
            part.setBotAng(ang);
            part_pos_ang(end-i,:) = [pos ang];
        end
    end
    
    
%     est_bot_pos_ang = mean(part_pos_ang(1:50,:));
    est_bot_pos_ang = part_pos_ang(1,:);
    est_bot = BotSim(map);
%     thresh = 49;
%     while ~est_bot.pointInsideMap(est_bot_pos_ang(1:2)) && thresh > 1
%         est_bot_pos_ang = mean(part_pos_ang(1:thresh,:));
%         thresh = thresh-1;
%     end

    est_bot.setScanConfig(est_bot.generateScanConfig(bot.num_of_scans))
    est_bot.setBotPos(est_bot_pos_ang(1:2));
    est_bot.setBotAng(est_bot_pos_ang(3));
    
    %---IF USED ACCOUNT FOR NANS IN DISTS-----
%     similar_check = sum(abs(est_bot.ultraScan()- bot.dists))

    %convergence test
    convergence_score = sum(sqrt(sum((part_pos_ang(1:100,1:2)-circshift(part_pos_ang(1:100,1:2),1)).^2,2)))
    if convergence_score < 900 % || similar_check < 30
        
        mu = bot.dists;
        ind = find(isnan(mu));
        mu(ind) = [];
        if isempty(mu)
            continue;
        end
        sigma = max(max_lim)/10;
        for i = 1:bot.num_of_scans-length(ind); pd(i) = makedist('Normal','mu',mu(i),'sigma',sigma); end

        good_scans = 1:bot.num_of_scans;
        good_scans(ind) =[];
        est_part_dists(:) = est_bot.ultraScan();
        k = 1;
        for j = good_scans
            sense_scores(k) = pdf(pd(k), est_part_dists(j));
            k=k+1;
        end
        sense_scores = sort(sense_scores);
        est_score = prod(sense_scores(4:end));
        
        if est_score < 1e-10
            for i = 1:200
                part = particle(i);
                part.randomPose(0);
                iter  = 1;
            end
        else
            state = 'localised'
            continue;
        end
    end
    
    lost_navigate(bot,particle);
    for i = 1:num
        part_pos_ang(i,1:2) = particle(i).getBotPos();
        part_pos_ang(i,3) = particle(i).getBotAng();
    end
    iter=iter+1
end


while strcmp(state, 'localised')
    
    start = est_bot_pos_ang(1:2);
    if est_bot.pointInsideMap(start)
        optim_path = a_star(start, target, est_bot, 3);
        turn = det_dest_ang(start, optim_path(2,:)) - est_bot_pos_ang(3);
        bot.turn_op(turn);
        est_bot.(turn);
        move = max(0.2, sqrt(sum((optim_path(2,:)-optim_path(1,:)).^2)));

        front_dist = bot.front_scan();

        if front_dist < move

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

    else
        turn = 0;
        move = 4;
    end
    
    bot.move(move);

    for i = 1:num
        particle(i).turn(turn); %turn the particle in the same way as the real robot
        particle(i).move(move); %move the particle in the same way as the real robot
        part_pos_ang(i,1:2) = particle(i).getBotPos();
        part_pos_ang(i,3) = mod(particle(i).getBotAng(), 2*pi);
    end
    
    %estimate bot
    est_bot_pos_ang = mean(part_pos_ang(1:30,:));
    est_bot = BotSim(map);
    est_bot.setScanConfig(bot.generateScanConfig(bot.num_of_scans))
    est_bot.setBotPos(est_bot_pos_ang(1:2));
    est_bot.setBotAng(est_bot_pos_ang(3));
    similar_check = sum(abs(est_bot.ultraScan()-bot.ultraScan()));
   
    %convergence test
    convergence_score = sum(sqrt(sum((part_pos_ang(1:100,1:2)-circshift(part_pos_ang(1:100,1:2),1)).^2,2)));
    if convergence_score < 900 || similar_check < 4
        bot_dists = bot.ultraScan(); %get a scan from the real robot.
        mu = bot_dists;
        sigma = 8;
        for i = 1:bot.num_of_scans; pd(i) = makedist('Normal','mu',mu(i),'sigma',sigma); end
 
        est_part_dists(:) = est_bot.ultraScan();
        for j = 1:bot.num_of_scans 
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
    if bot.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        bot.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        bot.drawBot(10,'g'); %draw robot with line length 30 and green
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
    
