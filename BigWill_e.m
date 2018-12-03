classdef BigWill_e < handle
     
    properties
        pos;    %position of the robot
        ang;    %angle of the robot (radians)
        dir;    %angle of the robot (stored as 2d vector)
        map;    %map coordinates with a copy of the first coordiantes at the end
        mapLines;   %The map stored as a list of lines (for easy line interection)
        inpolygonMapformatX; %The map stored as a polygon for the insidepoly function
        inpolygonMapformatY; %The map stored as a polygon for the insidepoly function
        scanConfig;     %stores how the robot performs a scan (number of points, angle between points)
        scanLines;      %the scan configuration stored as 2d lines
        sl;
        step_size;
        rw;
        lw;
        head;
        chassis_radius;
        circum;
%         sensorNoise;     %how much noise to add to a scan. Constant noise model. Error standard deviation in cm
%         motionNoise;     %how much noise when moving. Proportional noise model. cm error stdDev per unit length in cm/cm
%         turningNoise;    %how much noise when turning. Porportional noise model. Radian stdDev error per radian rad/rad
        adminKey;       %the key to lock off certain features
        PosHistory;  %stores history of where the robot has been
        MoveCount;   %stored index for appending to the history
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
        bot.setMap([0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]);
%         bot.scanOffset = [0 0];
%         bot.scanConfig = generateScanConfig(bot,6);
%         bot.updateScanLines(0,1);
%         bot.sensorNoise = noiseLevel(1);  %constant noise model. Error standard deviation in cm
%         bot.motionNoise = noiseLevel(2);  %proportional noise model. cm error stdDev per unit length in cm/cm
%         bot.turningNoise = noiseLevel(3); %porportional noise model. Radian stdDev error per radian rad/rad
%         bot.PosHistory =[];
%         bot.MoveCount =1;
    end
        
    function dist = front_scan(bot)
%         make a more accurate way of returning a distance ??
        scan_num = 10;
        scans = zeros(scan_num, 1);
        for i = 1: scan_num
            scans(i) = GetUltrasonic(SENSOR_2);
        end
        scans(scans == 255) = []; % remove max scan values
        if min(size(scans)) > 0
           s_d = var(scans);
           mean_scans = mean(scans);
           scans(scans > (mean_scans + s_d)) = [];
           scans(scans < (mean_scans - s_d)) = [];
        end
        dist = mean(scans);
        
    end
    
    function sweep_head(bot, angle)
        bot.head.TachoLimit = round(abs(angle));
        bot.head.Power = sign(angle) * 50;
        bot.head.SendToNXT();
        bot.head.WaitFor();
    end
    
    function scan_dists = get_scans(bot, num_of_scans)
        scan_dists = zeros(abs(num_of_scans), 1);
        scan_dists(1) = bot.front_scan();
        for i = 2:abs(num_of_scans) 
            bot.sweep_head(370/num_of_scans)
            scan_dists(i) = bot.front_scan();
        end
        bot.sweep_head(360/num_of_scans-370)
        if sign(num_of_scans) == -1   % flip if we turning other way
           flip(scan_dists)
        end
    end
    
    function moved = move(bot)
        tach_lim = round(360*bot.step_size/bot.circum);
        bot.lw.TachoLimit = tach_lim;
        bot.rw.TachoLimit = tach_lim;
        bot.lw.Power = 50;
        bot.rw.Power = 50;
        bot.lw.SendToNXT();
        bot.rw.SendToNXT();
        moved = bot.step_size;
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
    end
    
    function brake = brake(bot)
        bot.lw.Stop('brake')
        bot.rw.Stop('brake')
        bot.lw.SendToNXT()
        bot.rw.SendToNXT()
        brake = true;
    end
    
    function setMap(bot,newMap)
            bot.map = newMap;
            bot.inpolygonMapformatX = cat(1,newMap(:,1), newMap(1,1));
            bot.inpolygonMapformatY = cat(1,newMap(:,2), newMap(1,2));
            
            newMap(length(newMap)+1,:)= newMap(1,:);
            bot.map = newMap;
            bot.mapLines = zeros(length(bot.map)-1,4);  %each row represents a border of the map
            for i =1:size(bot.mapLines,1)
                bot.mapLines(i,:) = [bot.map(i,:) bot.map(i+1,:)] ;
            end
    end
end
end