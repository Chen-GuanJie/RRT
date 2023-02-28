clc; clear;
conf.filename = 'data/Output_500';
conf.start = [280 80 500 3.14 * 45/180 0 0];
conf.goal = [70 600 600 3.14 * 45/180 0 0];
conf.threshold = 3000;
conf.maxFailedAttempts = 10000;
conf.search = 1.6;
conf.randnum = 0.6;

conf.height_limit = 600;
conf.deltaT = 150;
%stepSize = 0.1;
conf.g = 9.8;
conf.v = 30;
conf.acc = 999999999;
conf.GammaMax = 30/180 * 3.1416;
conf.GammaMin = -30/180 * 3.1416;
conf.pitchMax = 30/180 * 3.1416;
conf.pitchMin = -30/180 * 3.1416;

conf.GammaStep = 15/180 * 3.1416; %滚转角最大步长
conf.pitchstep = 10/180 * 3.1416; %俯仰角最大步长
stable_distance = 'data/mini_stable_distance';
load(stable_distance, 'mini_stable_distance')
conf.mini_stable_distance = mini_stable_distance;
conf.speeds=[240 260 280 300 320 340 360 380 400 440 480 520 560 600 640];
r = rrt(conf);
%r.start_star(true);

% mini_stable_distance=[0.9 1 1.2 1.3 1.5 1.6 1.8 2 2.2 2.6 3 3.4 3.9 4.5 5;
%     1 1.1 1.3 1.4 1.6 1.8 2 2.2 2.4 2.8 3.3 3.8 4.3 4.9 5.5;
%     1.1 1.2 1.4 1.5 1.7 1.9 2.1 2.3 2.6 3 3.5 4.1 4.7 5.3 6;
%     1.1 1.3 1.5 1.7 1.9 2.1 2.3 2.5 2.8 3.3 3.8 4.4 5.1 5.8 6.5;
%     1.2 1.4 1.6 1.8 2.0 2.2 2.5 2.7 3 3.5 4.2 4.8 5.5 6.3 7.1;
%     1.3 1.5 1.7 1.9 2.1 2.4 2.6 2.9 3.2 3.8 4.5  5.2 6 6.8 7.7;
%     1.4 1.6 1.8 2 2.3 2.6 2.9 3.1 3.5 4.1 4.8 5.6 6.5 7.4 8.3;
%     1.5 1.7 1.9 2.2 2.5 2.8 3.1 3.4 3.7 4.4 5.2 6.1 7 8 9;
%     1.6 1.8 2.1 2.4 2.7 3 3.3 3.6 4 4.8 5.6 6.6 7.6 8.6 9.7;
%     1.7 2 2.2 2.5 2.9 3.2 3.6 3.9 4.3 5.2 6.1 7.1 8.2 9.3 10.6;
%     1.9 2.1 2.4 2.7 3.1 3.5 3.8 4.2 4.7 5.6 6.6 7.7 8.9 10.1 11.4;
%     2 2.3 2.6 3 3.3 3.7 4.2 4.6 5.1 6.1 7.2 8.3 9.6 11 12.4;
%     2.2 2.4 2.8 3.2 3.6 4 4.5 5 5.5 6.6 7.8 9.1 10.5 11.9 13.5;
%     2.3 2.7 3 3.5 3.9 4.4 4.9 5.4 6 7.2 8.5 9.9 11.4 13 14.8;
%     2.5 2.9 3.3 3.8 4.3 4.8 5.4 5.9 6.5 7.9 9.3 10.8 12.5 14.3 16.2];
% mesh(mini_stable_distance)