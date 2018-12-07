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
        OpenUltrasonic(SENSOR_2)
        bot.pos = [0 0];
        bot.step_size = 5;
        diameter = 3.75;
        bot.circum = pi*diameter;
        bot.chassis_radius = 6.5; %cm
        bot.ang = 0;
        bot.dir = [cos(bot.ang) sin(bot.ang)];
        bot.map = [0,0;66,0;66,44;44,44;44,66;110,66;110,110;0,110];
        bot.num_of_scans = 10;
    end
        
    function dist = front_scan(bot)
%         make a more accurate way of returning a distance ??
        scan_num = 10;
        scans = zeros(scan_num, 1);
        for i = 1: scan_num
            scans(i) = GetUltrasonic(SENSOR_2)
        end
        std = sqrt(var(scans));
        my_mode = mode(scans);
        if length(unique(scans)) == length(scans); my_mode=median(scans);end
        scans = scans(scans >= my_mode-std);
        scans = scans(scans <= my_mode+std);
        dist = mean(scans);
        if var(scans)>10; dist = nan;end

%         if min(size(scans)) > 0
%            s_d = var(scans);
%            if s_d < 1
%                s_d = 1;
%            end
%            mean_scans = mean(scans);
%            scans(scans > (mean_scans + s_d)) = [];
%            scans(scans < (mean_scans - s_d)) = [];
%         end
%         dist = mean(scans);
        
    end
    
    function sweep_head(bot, angle)
        bot.head.TachoLimit = round(abs(angle));
        bot.head.Power = sign(angle) * 100;
        bot.head.SendToNXT();
        bot.head.WaitFor();
    end
    
    function scan_dists = get_scans(bot)
        scan_dists = zeros(abs(bot.num_of_scans), 1);
        scan_dists(1) = bot.front_scan();     
        for i = 2:abs(bot.num_of_scans) 
            bot.sweep_head(375/bot.num_of_scans)
            scan_dists(i) = bot.front_scan();
        end
        position = getfield(bot.head.ReadFromNXT(), 'Position');
        bot.sweep_head(-position);
        if any(scan_dists == -1)
            pause(10);
            disp('got a -1 dist, not sure what it means')
        end
        scan_dists(scan_dists > 100) = nan;
        scan_dists(scan_dists == -1) = nan;
        bot.dists = scan_dists;
%         bot.sweep_head(360/bot.num_of_scans-370)
%         if sign(bot.num_of_scans) == -1   % flip if we turning other way
%            scan_dists = flip(scan_dists);
%         end
%     bot.num_of_scans =  bot.num_of_scans * -1;
    end  
    
    function moved = move(bot, dist)
        tach_lim = round(360*abs(dist)/bot.circum);
        bot.lw.TachoLimit = tach_lim;
        bot.rw.TachoLimit = tach_lim;
        bot.lw.Power = sign(dist)*50;
        bot.rw.Power = sign(dist)*50;
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