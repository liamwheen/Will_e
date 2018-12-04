map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];
bot = BotSim(map);
target = bot.getRndPtInMap(10);
will_e = BigWill_e;
returnedBot = real_localise(will_e,target);