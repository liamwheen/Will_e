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
    particle(i).randomPose(10); %spawn the particles in random locations
    particle(i).setScanConfig(particle(i).generateScanConfig(bot.num_of_scans))
    part_pos_ang(i,1:2) = particle(i).getBotPos();
    part_pos_ang(i,3) = particle(i).getBotAng();
end

iter = 1;
%%%%%LOST MODE%%%%%

while strcmp(state,'lost') && toc<125
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
    sigma = 5;
    for i = 1:bot.num_of_scans-length(ind); pd(i) = makedist('Normal','mu',mu(i),'sigma',sigma); end
    
    good_scans = 1:bot.num_of_scans;
    good_scans(ind) =[];
    sense_scores = zeros(length(good_scans),1);
    for i = 1:num
        part_dists(i,:) = particle(i).ultraScan();
        part_dists(i,2:end) = part_dists(i,end:-1:2);
        k = 1;
        for j = good_scans
            sense_scores(k) = pdf(pd(k), part_dists(i,j)) + 0.03^bot.num_of_scans;
            k=k+1;
        end
        weights(i) = prod(sense_scores(sense_scores~=0));
        weights(i) = weights(i);
    end
    [weights, order] = sort(weights,'descend');
    particle = particle(order);
    part_pos_ang = part_pos_ang(order,:);
    
    for i = 0:260
        part = particle(end-i);
        if mod(i, 8) == 0
            part.randomPose(10);
        else          
            new_part = datasample(part_pos_ang(1:40,:), 1);
            pos = new_part(1:2) + max((15-0.3*iter^2),-5)*[randn() randn()];%((20-iter)/4)*[randn() randn()]; % change this
            ang = new_part(3) + (0.3/iter)*randn;
            part.setBotPos(pos);    
            part.setBotAng(ang);
            part_pos_ang(end-i,:) = [pos ang];
        end
    end
%      particle = resample_particles(particle, weights(order), iter);
    
    part_pos_ang(:,3) = mod(part_pos_ang(:,3), 2*pi);
    est_bot_pos_ang = mean(part_pos_ang(1:5,:));
%     est_bot_pos_ang = part_pos_ang(1,:);
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
    if convergence_score < 800 % || similar_check < 30
        %------PERHAPS DO SCANS AGAIN WITH MUCH MORE--------
        mu = bot.dists;
        ind = find(isnan(mu));
        mu(ind) = [];
        if isempty(mu)
            continue;
        end
        sigma = 9;
        for i = 1:bot.num_of_scans-length(ind); pd(i) = makedist('Normal','mu',mu(i),'sigma',sigma); end

        good_scans = 1:bot.num_of_scans;
        good_scans(ind) =[];
        good_count = length(good_scans);
        sense_scores = zeros(good_count,1);
        est_part_dists(:) = est_bot.ultraScan();
        k = 1;
        for j = good_scans
            sense_scores(k) = pdf(pd(k), est_part_dists(j));
            k=k+1;
        end
        sense_scores = sort(sense_scores);
        est_score = prod(sense_scores(end-min(3,good_count-1):end));
        
        if false %est_score^(-1/min(4,good_count)) > 20
            for i = 1:200
                part = particle(i);
                part.randomPose(10);
                iter  = 1;
            end
        else
            state = 'localised'
            toc
            break;
        end
    end
    
    lost_navigate(bot,particle);
    toc
    for i = 1:num
        part_pos_ang(i,1:2) = particle(i).getBotPos();
        part_pos_ang(i,3) = particle(i).getBotAng();
        if part_pos_ang(i,3) > pi
            part_pos_ang(i,3) = part_pos_ang(i,3)-2*pi;
        elseif part_pos_ang(i,3) < -pi
            part_pos_ang(i,3) = part_pos_ang(i,3)+2*pi;
        end
    end
    iter=iter+1
end



if toc > 124 && strcmp(state,'lost')
    [~, ~, ~, part_pos_ang] = rapid_part_filter(bot, particle, part_dists, weights, part_pos_ang, est_bot);
    est_bot.setBotPos(part_pos_ang(1,1:2));
    est_bot.setBotAng(part_pos_ang(1,3));
    state = 'localised';
end

%------FOUND MODE------
while strcmp(state, 'localised')
    
    start = est_bot_pos_ang(1:2)
    if est_bot.pointInsideMap(start)
        optim_path = a_star(start, target, est_bot, 3)
        turn = det_dest_ang(start, optim_path(2,:)) - est_bot_pos_ang(3);   
        dest_ang = -360*turn/(2*pi);
        if dest_ang < -180
            dest_ang = dest_ang + 360;
        elseif dest_ang > 180
            dest_ang = dest_ang - 360;
        end
        
        bot.turn_op(dest_ang);
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
    
        [dist_moved, ang_turned] = bot.move(move);
        est_bot.move(dist_moved);
        est_bot.turn(-ang_turned*180/pi);
        
        if ang_turned > 0 
            [particle, part_dists, weights, part_pos_ang] = rapid_part_filter(bot, particle, part_dists, weights, part_pos_ang, est_bot);
            est_bot.setBotPos(part_pos_ang(1,1:2));
            est_bot.setBotAng(part_pos_ang(1,3));
        end
        
        
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
            [particle, part_dists, weights, part_pos_ang] = rapid_part_filter(bot, particle, part_dists, weights, part_pos_ang, est_bot);
            
            est_bot.setBotPos(part_pos_ang(1,1:2));
            est_bot.setBotAng(part_pos_ang(1,3));
            dest_ang = det_dest_ang(est_bot.getBotPos(), target);
            dest_dist = sqrt(sum((est_bot.getBotPos()-target).^2));
            if dest_dist < 15
                bot.turn_op(-180*dest_ang/pi);
                bot.move(dest_dist);
            end
                          
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


end


function [particle, part_dists, weights, part_pos_ang] = rapid_part_filter(bot, particle, part_dists, weights, part_pos_ang, est_bot)
    bot.get_scans();
    
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
    sigma = 5;
    for i = 1:bot.num_of_scans-length(ind); pd(i) = makedist('Normal','mu',mu(i),'sigma',sigma); end
    
    good_scans = 1:bot.num_of_scans;
    good_scans(ind) =[];
    sense_scores = zeros(length(good_scans),1);
    for i = 1:300
        particle(i).setBotPos(est_bot.getBotPos() + 10*[randn randn]);
        particle(i).setBotAng(est_bot.getBotAng() + 0.2*randn);
        part_dists(i,:) = particle(i).ultraScan();
        part_dists(i,2:end) = part_dists(i,end:-1:2);
        part_pos_ang(i,1:2) = particle(i).getBotPos();
        part_pos_ang(i,3) = particle(i).getBotAng();
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
    
end


function dest_ang = det_dest_ang(bot_pos, dest)
    if bot_pos(1)<=dest(1) %check dest is in the right half of bots grid
        dest_ang = mod(atan((dest(2)-bot_pos(2))/(dest(1)-bot_pos(1))), 2*pi);
    else %dest is to the left of bot
        dest_ang = pi - atan((dest(2)-bot_pos(2))/(bot_pos(1)-dest(1)));
    end
end

function particles = resample_particles(particles, particle_scores, iter)
    rand_distribute = 50;
    sum_of_probs = sum(particle_scores);
    rand_wieght = max((8 - iter)*5, 4);
%     particle_scores = particle_scores/sum_of_probs;  % make sure it is a PDF
    % make into cumulative  -- doing this before to save computation
    for i = 2:length(particle_scores)
        particle_scores(i) = particle_scores(i) + particle_scores(i-1);
    end
    % resample
    for i = 1: length(particles) - rand_distribute
        chosen_index = roulette_sample(particle_scores, sum_of_probs);
        setBotPos(particles(i), getBotPos(particles(chosen_index)) + rand_wieght*([rand , rand]-0.5));
        setBotAng(particles(i), getBotAng(particles(chosen_index)) + rand_wieght*(rand - 0.5)*0.1 );
    end
    % randomly distribute
    for i = length(particles) - rand_distribute: length(particles)
       particles(i).randomPose(0); 
    end
end


function chosen_index = roulette_sample(cumulative_probs, sum_of_probs)
    rand_num = rand()*sum_of_probs;
    for i = 1:length(cumulative_probs)
        if cumulative_probs(i) > rand_num  % in the selected 'window'
            chosen_index = i;
            break
        end
    end  
end
