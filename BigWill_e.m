classdef BigWill_e < handle
     
    properties
        pos;    %position of the robot
        ang;    %angle of the robot (radians)
        dir;    %angle of the robot (stored as 2d vector)
        map;    %map coordinates with a copy of the first coordiantes at the end
        num_of_scans;
        dists
        step_size;
        rw;
        lw;
        head;
        chassis_radius;
        circum;
    end
    
    methods
        
    function bot = BigWill_e()
        
        COM_CloseNXT all
        h = COM_OpenNXT();
        COM_SetDefaultNXT(h);
        
        
        bot.rw = NXTMotor('A');
        bot.lw = NXTMotor('C');
        bot.head = NXTMotor('B');
        bot.head.ResetPosition();
        OpenUltrasonic(SENSOR_2)
        bot.pos = [0 0];
        bot.step_size = 5;
        diameter = 3.75;
        bot.circum = pi*diameter;
        bot.chassis_radius = 6.5; %cm
        bot.ang = 0;
        bot.dir = [cos(bot.ang) sin(bot.ang)];
        bot.map = [0,0;66,0;66,44;44,44;44,66;110,66;110,110;0,110];
        bot.num_of_scans = 12;
    end
    
    function move_to_good_scan(bot)
        num = bot.num_of_scans
        bot.num_of_scans = 2*num
        mode_counter = zeros(abs(bot.num_of_scans), 1);
        mode_counter(1) = bot.get_scan_variance();    
        for i = 2:abs(bot.num_of_scans)
            bot.sweep_head(385/bot.num_of_scans)
            mode_counter(i) = bot.get_scan_variance();
        end
        % reset back head postion
        position = getfield(bot.head.ReadFromNXT(), 'Position');
        bot.sweep_head(-position);
        % turn to the best scan
        [m, ind] = min(mode_counter);  % get the most accurate reading
        ang = 360*(ind-1)/abs(bot.num_of_scans)
        bot.turn_op(ang);
        bot.num_of_scans = num
    end
 
    function mode_counter = get_scan_variance(bot)
        scan_num = 40;
        scans = zeros(scan_num, 1);
        for i = 1: scan_num
            scans(i) = GetUltrasonic(SENSOR_2);
        end

        mode_counter = 0;
        my_mode = mode(scans);
        for i = 1: scan_num
            mode_counter = mode_counter + abs(scans(i) - my_mode);
        end
    end
        
    function dist = front_scan(bot)
%         make a more accurate way of returning a distance ??
        scan_num = 10;
        scans = zeros(scan_num, 1);
        for i = 1: scan_num
            scans(i) = GetUltrasonic(SENSOR_2);
        end
        if length(unique(scans)) == length(scans); dist = nan; return ;end
        my_mode = mode(scans);
        error = my_mode/10; 
        error = max([error, 2]);
        out_range = find(abs(scans-my_mode)>error);
        if length(out_range)>2; dist = nan; return; end
        dist = mean(scans);
    end
    
    function sweep_head(bot, angle)
        bot.head.TachoLimit = round(abs(angle));
        bot.head.Power = sign(angle) * 100;
        bot.head.SendToNXT();
        bot.head.WaitFor();
    end
    
    function scan_dists = get_scans(bot)
        scan_dists = nan*ones(abs(bot.num_of_scans), 1);
        scan_dists(1) = bot.front_scan();     
        for i = 2:abs(bot.num_of_scans) 
            bot.sweep_head(360/bot.num_of_scans)
            scan_dists(i) = bot.front_scan();
%             if scan_dists(i) < 15
%                 position = getfield(bot.head.ReadFromNXT(), 'Position');
%                 bot.sweep_head(-position);
%                 bot.dists = scan_dists;
%                 return
%             end
        end
        position = getfield(bot.head.ReadFromNXT(), 'Position');
        bot.sweep_head(-position);
%         scan_dists(scan_dists > 80) = nan;
        scan_dists(scan_dists == -1) = nan;
        
        scan_dists
        
        %prune potentially incorrect readings - comparing neighbours
%         temp_dists = scan_dists;
%         dist = temp_dists(1);
%         if temp_dists(bot.num_of_scans)+20 < dist || isnan(temp_dists(bot.num_of_scans))|| temp_dists(2)+20 < dist || isnan(temp_dists(2))
%             scan_dists(1) = nan;
%         end
%         for ind = 2:bot.num_of_scans-1
%             dist = temp_dists(ind);
%             if temp_dists(ind-1)+20 < dist || isnan(temp_dists(ind-1))|| temp_dists(ind+1)+20 < dist || isnan(temp_dists(ind+1))
%                 scan_dists(ind) = nan;
%             end
%         end
%             
%         dist = temp_dists(bot.num_of_scans);
%         if temp_dists(1)+20 < dist || isnan(temp_dists(1))|| temp_dists(bot.num_of_scans-1)+20 < dist || isnan(temp_dists(bot.num_of_scans-1))
%             scan_dists(bot.num_of_scans) = nan;
%         end
        
        
        bot.dists = scan_dists;
%         bot.sweep_head(360/bot.num_of_scans-370)
%         if sign(bot.num_of_scans) == -1   % flip if we turning other way
%            scan_dists = flip(scan_dists);
%         end
%     bot.num_of_scans =  bot.num_of_scans * -1;
    end  
   function scan_dists = test_scans(bot)
       scan_num = 10;
        scan_dists = zeros( scan_num, abs(bot.num_of_scans));
        scan_dists(:, 1) = bot.test_front_scan(scan_num)   
        for i = 2:abs(bot.num_of_scans) 
            bot.sweep_head(360/bot.num_of_scans)
            scan_dists(:, i) = bot.test_front_scan(scan_num)
        end
        position = getfield(bot.head.ReadFromNXT(), 'Position');
        bot.sweep_head(-position);
        bot.dists = scan_dists;
%         bot.sweep_head(360/bot.num_of_scans-370)
%         if sign(bot.num_of_scans) == -1   % flip if we turning other way
%            scan_dists = flip(scan_dists);
%         end
%     bot.num_of_scans =  bot.num_of_scans * -1;
   end  
    
   function scans = test_front_scan(bot, scan_num)
%         make a more accurate way of returning a distance ??
        scans = zeros(scan_num, 1);
        for i = 1: scan_num
            scans(i) = GetUltrasonic(SENSOR_2);
        end
    end
    
    function moved = move(bot, dist)
        tach_lim = round(360*abs(dist)/bot.circum);
        bot.lw.TachoLimit = tach_lim;
        bot.rw.TachoLimit = tach_lim;
        bot.lw.Power = sign(dist)*64;
        bot.rw.Power = sign(dist)*80;
        bot.lw.SendToNXT();
        bot.rw.SendToNXT();
        moved = bot.step_size;
        bot.rw.WaitFor();
        bot.lw.WaitFor();
    end
    
    function turned = turn_op(bot, degree) %takes CCW as negative 
        distance = bot.chassis_radius*degree*pi/180;
        tach_lim = abs(round(360*distance/bot.circum));
        direc = sign(degree);
        bot.lw.TachoLimit = tach_lim;
        bot.rw.TachoLimit = tach_lim;
        bot.lw.Power = 50*direc;
        bot.rw.Power = -50*direc;
        bot.lw.SendToNXT()
        bot.rw.SendToNXT()
        turned = degree;
        bot.rw.WaitFor();
        bot.lw.WaitFor();
    end
    
    function brake = brake(bot)
        bot.lw.Stop('brake')
        bot.rw.Stop('brake')
        bot.lw.SendToNXT()
        bot.rw.SendToNXT()
        brake = true;
    end

    function map = getMap(bot)
        map = bot.map;
    end
end
end