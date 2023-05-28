classdef uav < handle

    properties (SetAccess = public)
        dimension = 3
        transfer
    end

    properties (SetAccess = private)
        name = 'uav'
        config_manger
        msd_path = "./data/map/"
        msd_name = ""
        transfer_tactics = "direct"
        maps
        pitchMax = 0 %俯仰角最大
        pitchMin = 0 %俯仰角最小
        pitchstep = 0 %俯仰角最大步长
        direct_step = 1

        %动力学相关的参数
        GammaMax = 0 %滚转角最大
        GammaMin = 0 %滚转角最小
        GammaStep = 0 %滚转角最大步长
        v %恒速度
        deltaT = 0 %每步长飞行时间
        g = 9.8 %重力加速度

        %最短稳定距离相关的参数
        mini_stable_distance = zeros(15, 15) %最短稳定距离
        course_change = 0 %航向角变化
        speeds = zeros(2, 2) %速度们
        acc = 0 %加速度
        max_delta_h = 1
    end

    methods (Access = private)

        function [z, pitchangle] = climb(this, temp, closest_node)
            targetHeight = this.maps.Z(round(temp(1)), round(temp(2))); %目标点的目标高度
            distanceee = norm(temp - closest_node(1:2)); %距离
            pitchangle = atan((targetHeight - closest_node(3)) / distanceee); %应该的爬升角
            pitchangle = this.limiter(pitchangle, this.pitchMax, this.pitchMin);
            pitchangle = this.limiter(pitchangle, closest_node(6) + this.pitchstep, closest_node(6) - this.pitchstep);
            z = pitchangle * distanceee + closest_node(3);
        end

    end

    methods (Access = public)

        function this = uav()
            this.config_manger = configs.get_config(this.name);
            this.maps = map.get_instance();
        end

        function output = get_threshold(this, rate)

            switch this.transfer_tactics
                case 'direct'
                    output = rate * this.direct_step;
                case 'kinetic'
                    output = rate * this.v * this.deltaT / this.maps.map_scale;
            end

        end

        function get_new_config(this, config_dir)
            this.config_manger = configs.get_config([this.name, '_', config_dir]);
        end

        function init(this)
            conf = this.config_manger.load();
            this.transfer_tactics = conf.transfer_tactics;
            this.pitchMax = conf.kinetic.pitchMax;

            switch this.transfer_tactics
                case 'direct'
                    conf = conf.direct;
                    this.direct_step = conf.direct_step;
                    this.transfer = @this.transfer_direct;
                    this.dimension = 3;
                case 'kinetic'
                    conf = conf.kinetic;
                    this.transfer = @this.transfer_kinetic;
                    this.dimension = 6;
                    this.deltaT = conf.deltaT;
                    this.g = conf.g;
                    this.v = conf.v;
                    this.acc = conf.acc;
                    this.GammaMax = conf.GammaMax;
                    this.GammaMin = conf.GammaMin;
                    this.pitchMax = conf.pitchMax;
                    this.pitchMin = conf.pitchMin;
                    this.GammaStep = conf.GammaStep;
                    this.pitchstep = conf.pitchstep;
                case 'stable'
                    conf = conf.stable;
                    this.transfer = @this.transfer_stable;
                    this.dimension = 4;
                    this.msd_path = conf.mini_stable_distance_path;
                    this.msd_name = conf.mini_stable_distance_name;

                    if ~strcmp(this.msd_name, conf.mini_stable_distance_name)
                        this.msd_name = conf.msd_name;
                        this.mini_stable_distance = 1000 * utils.load_file(this.msd_path, this.mini_stable_distance);
                        this.speeds = conf.speeds;
                        this.course_change = (0:5:120) * pi / 180;
                    end

            end

            this.max_delta_h = tan(this.pitchMax);
        end

        function newNode = transfer_stable(this, sample, closest_node)
            %根据稳定距离转移
            %计算航向角变化
            phi1 = atan2(sample(2) - closest_node(2), sample(1) - closest_node(1));
            phi = closest_node(4);
            deltaPhi = this.limit2pi(phi1 - phi);
            distance = norm(sample(1:2) - closest_node(1:2));
            d = distance * sin(this.course_change - deltaPhi) ./ sin(this.course_change); %正弦
            speed_index = this.find_closest(520, this.speeds);
            sd = this.mini_stable_distance(:, speed_index)';
            sd = [ones(1, 10) * this.mini_stable_distance(1, speed_index) sd];
            ind = this.find_closest(sd, d);

            if ind <= 10
                mini_distance = this.mini_stable_distance(1, speed_index);
            else
                mini_distance = this.mini_stable_distance(ind - 10, speed_index);
            end

            if deltaPhi > 0
                newPhi = phi + this.course_change(ind);
            else
                newPhi = phi - this.course_change(ind);
            end

            %两次转移，每次均移动最短稳定距离
            rotation1 = [cos(phi) -sin(phi); sin(phi) cos(phi); ];
            rotation2 = [cos(newPhi) -sin(newPhi); sin(newPhi) cos(newPhi); ];
            temp = closest_node(1:2) + (rotation1 * [mini_distance; 0])' + (rotation2 * [mini_distance; 0])';
            [z, pitchangle] = this.climb(temp, closest_node);
            newNode = [temp(1), temp(2), z, newPhi, 0, pitchangle];
        end

        function step = get_step(this)
            step = this.direct_step;
        end

        function update_step(this, direct_step)

            if direct_step < this.direct_step
                this.direct_step = direct_step;
            end

        end

        function flag = transferable_stable(this, from, to)
            %判断是否可以从 from 到 to
            flag = false;
            movingVec = [to(1) - from(1), to(2) - from(2)];
            phi1 = atan2(movingVec(2), movingVec(1));
            phi = from(4);
            deltaPhi = 2 * this.limit2pi(phi1 - phi);
        end

        function newNode = transfer_kinetic(this, sample, closest_node)
            %根据动力学约束的状态转移
            phi1 = atan2(sample(2) - closest_node(2), sample(1) - closest_node(1));
            phi = closest_node(4);
            deltaPhi = this.limit2pi(phi1 - phi); %航向角变化
            newGamma = atan(deltaPhi * this.v / this.g / this.deltaT); %新的滚转角
            newGamma = this.limiter(newGamma, closest_node(5) + this.GammaStep, closest_node(5) - this.GammaStep);
            newGamma = this.limiter(newGamma, this.GammaMax, this.GammaMin);
            rotation = [cos(phi) -sin(phi); sin(phi) cos(phi); ];

            if newGamma ~= 0 %转弯
                Rou = tan(newGamma) * this.g / (this.v ^ 2); %计算水平转弯曲率
                deltaPhi = Rou * this.v * this.deltaT;
                newPhi = deltaPhi + phi;
                a = [sin(deltaPhi) / Rou; ((1 - cos(deltaPhi)) / Rou)] ./ this.maps.map_scale;
            else %直飞
                newPhi = phi;
                a = [this.v * this.deltaT; 0] ./ this.maps.map_scale;
            end

            temp = (rotation * a)' + closest_node(1:2);
            [z, pitchangle] = this.climb(temp, closest_node);
            newNode = [temp(1), temp(2), z, newPhi, newGamma, pitchangle];
        end

        function consumption = calc_consumption(~)
            %飞机动力学cost
            consumption = 0;
        end

        function target_h = just_follow(this, target_h, delta_dist)
            max_delta = this.max_delta_h * delta_dist / length(target_h);
            tmp = diff(target_h);
            index = find(abs(tmp) > max_delta);
            n = 0;

            while (~isempty(index))
                target_h = smooth(target_h)';
                n = n + 1;
                tmp = diff(target_h);
                index = find(abs(tmp) > max_delta);

                if n > 3
                    break
                end

            end

        end

        function [target_h, flag] = follow(this, from, to, target_h, delta_dist)
            % todo:平滑轨迹
            flag = false;
            max_delta = this.max_delta_h * delta_dist;
            [n, ~] = size(target_h);
            %             if this.transferable(from, to)
            tmp = diff(target_h);
            tmp(tmp > max_delta) = max_delta;
            tmp(tmp <- max_delta) = -max_delta;

            for i = 2:n
                target_h(i) = target_h(i - 1) + tmp(i - 1);
            end

            flag = true;
            %             end
        end

        function flag = transferable(this, from, to)
            %判断是否可以从 from 直线到 to
            flag = false;
            movingVec = [to(1) - from(1), to(2) - from(2)];
            phi1 = atan2(movingVec(2), movingVec(1));
            phi = from(4);
            deltaPhi = this.limit2pi(phi1 - phi);
            distanceee = norm(movingVec);
            newGamma = atan(deltaPhi * this.v ^ 2 / this.g / (distanceee * this.maps.map_scale));

            if (abs(newGamma - from(5)) < this.GammaStep) && (newGamma < this.GammaMax) && (newGamma > this.GammaMin)
                pitchangle = atan((to(3) - from(3)) / distanceee);

                if (abs(pitchangle - from(6)) < this.pitchstep * this.maps.map_scale * distanceee / this.v / this.deltaT) && (pitchangle < this.pitchMax) && (pitchangle > this.pitchMin)
                    flag = true;
                end

            end

        end

        function newNode = transfer_direct(this, sample, closest_node)
            movingVec = [sample(1) - closest_node(1), sample(2) - closest_node(2)];
            movingVec = movingVec / sqrt(sum(movingVec .^ 2)); %单位化
            newNode = zeros(1, 3);
            newNode(1:2) = closest_node(1:2) + this.direct_step * movingVec;
            x = uav.limiter(round(newNode(1)), this.maps.X_num, 1);
            y = uav.limiter(round(newNode(2)), this.maps.Y_num, 1);
            newNode(3) = this.maps.Z(x, y); %目标点的目标高度
        end

    end

    methods (Static)

        function index = find_closest(x, list)
            a = abs(list - x);
            [~, index] = min(a);
        end

        function x = limiter(x, xmax, xmin)

            if x > xmax
                x = xmax;
            elseif x < xmin
                x = xmin;
            end

        end

        function x = limit2pi(x)

            while x > pi
                x = x - 6.2832;

            end

            while x <- pi
                x = x + 6.2832;

            end

        end

    end

end
