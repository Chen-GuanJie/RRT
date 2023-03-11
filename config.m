clc; clear;close all;
conf.filename = 'data/Output_500';
conf.start = [280 80 500 3.14 * 45/180 0 0];
conf.goal = [70 600 600 3.14 * 45/180 0 0];
conf.threshold_close = 300;
conf.threshold_goal = 6000;
conf.search = 1.6;
conf.randnum = 0.4;
conf.max_nodes = 3000;

conf.height_limit = 600;
conf.deltaT = 200; %s
conf.g = 9.8;
conf.v = 5; %m/s
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
conf.speeds = [240 260 280 300 320 340 360 380 400 440 480 520 560 600 640];
r = rrt(conf);
r.start_star(false, 7, 0);
