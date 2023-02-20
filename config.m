conf.filename='data/Output_500';
conf.start=[280 80 500 3.14 * 45/180 0 0];
conf.goal=[70 600 600 3.14 * 45/180 0 0];
conf.threshold= 1500;
conf.maxFailedAttempts = 10000;
conf.search=1.6;
conf.randnum=0.6;


conf.height_limit = 600;
conf.deltaT = 60;
%stepSize = 0.1;
conf.g = 9.8;
conf.v = 80;
conf.GammaMax = 30/180 * 3.1416;
conf.GammaMin = -30/180 * 3.1416;
conf.pitchMax = 30/180 * 3.1416;
conf.pitchMin = -30/180 * 3.1416;

conf.GammaStep = 15/180 * 3.1416; %滚转角最大步长
conf.pitchstep = 10/180 * 3.1416; %俯仰角最大步长



