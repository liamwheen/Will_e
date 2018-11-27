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
        sensorNoise;     %how much noise to add to a scan. Constant noise model. Error standard deviation in cm
        motionNoise;     %how much noise when moving. Proportional noise model. cm error stdDev per unit length in cm/cm
        turningNoise;    %how much noise when turning. Porportional noise model. Radian stdDev error per radian rad/rad
        adminKey;       %the key to lock off certain features
        PosHistory;  %stores history of where the robot has been
        MoveCount;   %stored index for appending to the history
    end
    
    methods
        
    function bot = BigWill_e()
        
        COM_CloseNXT all
        h = COM_OpenNXT();
        COM_SetDefaultNXT(h);
        
        rW = NXTMotor('A');
        lW = NXTMotor('C');
        
        bot.pos = [0 0];
        bot.step_size = 5;
        diameter = 3.5;
        bot.circum = pi*diamter;
        bot.chassis_radius = 5; %cm
        bot.ang = 0;
        bot.dir = [cos(bot.ang) sin(bot.ang)];
        bot.setMap([0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]);
        bot.scanOffset = [0 0];
        bot.scanConfig = generateScanConfig(bot,6);
        bot.updateScanLines(0,1);
%         bot.sensorNoise = noiseLevel(1);  %constant noise model. Error standard deviation in cm
%         bot.motionNoise = noiseLevel(2);  %proportional noise model. cm error stdDev per unit length in cm/cm
%         bot.turningNoise = noiseLevel(3); %porportional noise model. Radian stdDev error per radian rad/rad
%         bot.PosHistory =[];
%         bot.MoveCount =1;
    end
        
    function dist = scan()
        OpenUltrasonic(SENSOR_2);
        dist = GetUltrasonic(SENSOR_2);
    end
    
    function moved = move()
        tach_lim = 360*bot.step_size/bot.circ;
        lw.TachoLimit = tach_lim;
        rw.TachoLimit = tach_lim;
        lw.sendToNXT()
        rw.sendToNXT()
        moved = bot.step_size;
    end
    
    function turned = turn_op(degree) %takes CW as negative 
        distance = bot.chassis_radius*degree*pi/180;
        tach_lim = 360*distance/bot.circ;
        lw.TachoLimit = -tach_lim;
        rw.TachoLimit = tach_lim;
        lw.sendToNXT()
        rw.sendToNXT()
        turned = degree;
    end
    
    function brake = brake()
        lw.Stop('brake')
        rw.Stop('brake')
        lw.sendToNXT()
        rw.sendToNXT()
        brake = true;
    end
end
end