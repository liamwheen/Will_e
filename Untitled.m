hold off
    particle(1).drawMap()
    for i =1:300
            particle(i).drawBot(3); %draw particle with line length 3 and default color
    end
    drawnow;
    
    
    NXT_PlayTone(800, 500, h)
    NXT_GetBatteryLevel(h)