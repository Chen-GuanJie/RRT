function [r, output] = rrt_func(total_time)

    if nargin < 1
        total_time = 7;
    end

    conf.filename = 'data/dem';
    conf.start = [324 99 500 pi * 45/180 0 0];
    conf.goal = [58 376 500 pi * 45/180 0 0];
    conf.threshold_close = 300;
    conf.threshold_goal = 6000;
    conf.search = 1.6;
    conf.randnum = [0.6 2];
    conf.max_nodes = 3000;
    conf.height_cost_rate = 15;

    conf.height_limit = 500;
    conf.deltaT = 200; %s
    conf.g = 9.8;
    conf.v = 5; %m/s
    conf.acc = 999999999;
    conf.GammaMax = 30/180 * pi;
    conf.GammaMin = -30/180 * pi;
    conf.pitchMax = 30/180 * pi;
    conf.pitchMin = -30/180 * pi;

    conf.GammaStep = 15/180 * pi; %滚转角最大步长
    conf.pitchstep = 10/180 * pi; %俯仰角最大步长
    %stable_distance = 'data/mini_stable_distance.csv';
    mini_stable_distance=coder.load('data/mini_stable_distance.csv' );
    conf.mini_stable_distance = mini_stable_distance;
    conf.speeds = [240 260 280 300 320 340 360 380 400 440 480 520 560 600 640];
    r = rrt(conf);
    output = r.start_star(total_time);

end
