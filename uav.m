classdef uav < handle

    properties (SetAccess = private)
        height_limit = 1.2 %贴地高度限制
        pitchMax = 0 %俯仰角最大
        pitchMin = 0 %俯仰角最小
        pitchstep = 0 %俯仰角最大步长

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

        map_scale = 0
        max_delta_h = 1

    end

    methods (Access = private)

        function [z, pitchangle] = climb(this, temp, closestNode, map)
            targetHeight = map(round(temp(1)), round(temp(2))) + this.height_limit; %目标点的目标高度
            distanceee = norm(temp - closestNode(1:2)); %距离
            pitchangle = atan((targetHeight - closestNode(3)) / distanceee); %应该的爬升角
            pitchangle = this.limiter(pitchangle, this.pitchMax, this.pitchMin);
            pitchangle = this.limiter(pitchangle, closestNode(6) + this.pitchstep, closestNode(6) - this.pitchstep);
            z = pitchangle * distanceee + closestNode(3);
        end

    end

    methods (Access = public)

        function this = uav(conf)
            this.map_scale = conf.map_scale;
            this.height_limit = conf.height_limit / this.map_scale;
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
            this.mini_stable_distance = 1000 * conf.mini_stable_distance; %单位米
            this.course_change = (0:5:120) * pi / 180;
            this.speeds = conf.speeds;
            this.max_delta_h = tan(this.pitchMax);
        end

        function newNode = transfer_stable(this, sample, closestNode, map)
            %根据稳定距离转移
            %计算航向角变化
            phi1 = atan2(sample(2) - closestNode(2), sample(1) - closestNode(1));
            phi = closestNode(4);
            deltaPhi = this.limit2pi(phi1 - phi);
            distance = norm(sample(1:2) - closestNode(1:2));
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
            temp = closestNode(1:2) + (rotation1 * [mini_distance; 0])' + (rotation2 * [mini_distance; 0])';
            [z, pitchangle] = this.climb(temp, closestNode, map);
            newNode = [temp(1), temp(2), z, newPhi, 0, pitchangle];

        end

        function flag = transferable_stable(this, from, to)
            %判断是否可以从 from 到 to
            flag = false;
            movingVec = [to(1) - from(1), to(2) - from(2)];
            phi1 = atan2(movingVec(2), movingVec(1));
            phi = from(4);
            deltaPhi = 2 * this.limit2pi(phi1 - phi);
        end

        function newNode = transfer(this, sample, closestNode, map)
            %根据动力学约束的状态转移
            phi1 = atan2(sample(2) - closestNode(2), sample(1) - closestNode(1));
            phi = closestNode(4);
            deltaPhi = this.limit2pi(phi1 - phi); %航向角变化
            newGamma = atan(deltaPhi * this.v / this.g / this.deltaT); %新的滚转角
            newGamma = this.limiter(newGamma, closestNode(5) + this.GammaStep, closestNode(5) - this.GammaStep);
            newGamma = this.limiter(newGamma, this.GammaMax, this.GammaMin);
            rotation = [cos(phi) -sin(phi); sin(phi) cos(phi); ];

            if newGamma ~= 0 %转弯
                Rou = tan(newGamma) * this.g / (this.v ^ 2); %计算水平转弯曲率
                deltaPhi = Rou * this.v * this.deltaT;
                newPhi = deltaPhi + phi;
                a = [sin(deltaPhi) / Rou; ((1 - cos(deltaPhi)) / Rou)] ./ this.map_scale;
            else %直飞
                newPhi = phi;
                a = [this.v * this.deltaT; 0] ./ this.map_scale;
            end

            temp = (rotation * a)' + [closestNode(1), closestNode(2)];
            [z, pitchangle] = this.climb(temp, closestNode, map);
            newNode = [temp(1), temp(2), z, newPhi, newGamma, pitchangle];

        end

        function consumption = calc_consumption(~)
            %飞机动力学cost
            consumption = 0;
        end

        function target_h = just_follow(this, target_h, delta_dist)
            max_delta = this.max_delta_h * delta_dist;
            % [n, ~] = size(target_h);
            %             tmp = diff(target_h);
            %             index = find(abs(tmp) > max_delta);
            %
            %             while (~isempty(index))
            %                 target_h = smooth(target_h);
            %                 tmp = diff(target_h);
            %                 index = find(abs(tmp) > max_delta);
            %             end

            % tmp(tmp > max_delta) = max_delta;
            % tmp(tmp < -max_delta) = -max_delta;

            % for i = 2:n
            %     target_h(i) = target_h(i - 1) + tmp(i - 1);
            % end

        end

        function [target_h, flag] = follow(this, from, to, target_h, delta_dist)
            % todo:平滑轨迹
            flag = false;
            max_delta = this.max_delta_h * delta_dist;
            [n, ~] = size(target_h);

            if this.transferable(from, to)
                tmp = diff(target_h);
                tmp(tmp > max_delta) = max_delta;
                tmp(tmp <- max_delta) = -max_delta;

                for i = 2:n
                    target_h(i) = target_h(i - 1) + tmp(i - 1);
                end

                flag = true;
            end

        end

        function flag = transferable(this, from, to)
            %判断是否可以从 from 直线到 to
            flag = false;
            movingVec = [to(1) - from(1), to(2) - from(2)];
            phi1 = atan2(movingVec(2), movingVec(1));
            phi = from(4);
            deltaPhi = this.limit2pi(phi1 - phi);
            distanceee = norm(movingVec);
            newGamma = atan(deltaPhi * this.v ^ 2 / this.g / (distanceee * this.map_scale));

            if (abs(newGamma - from(5)) < this.GammaStep) && (newGamma < this.GammaMax) && (newGamma > this.GammaMin)
                pitchangle = atan((to(3) - from(3)) / distanceee);

                if (abs(pitchangle - from(6)) < this.pitchstep * this.map_scale * distanceee / this.v / this.deltaT) && (pitchangle < this.pitchMax) && (pitchangle > this.pitchMin)
                    flag = true;
                end

            end

        end

    end

    methods (Static)

        function newNode = transfer_directly(sample, closestNode, map)
            movingVec = [sample(1) - closestNode(1), sample(2) - closestNode(2), sample(3) - closestNode(3)];
            movingVec = movingVec / sqrt(sum(movingVec .^ 2)); %单位化
            newNode = zeros(1, 6);
            newNode(1:3) = closestNode(1:3) + 4 * movingVec;
            newNode(4:6) = [0 0 0];
            x = uav.limiter(round(newNode(1)), 372, 1);
            y = uav.limiter(round(newNode(2)), 673, 1);
            newNode(3) = map(x, y) + 1.2; %目标点的目标高度

        end

        function index = find_closest(x, list)
            a = abs(list - x);
            [~, index] = min(a);
        end

        function y = limiter(x, xmax, xmin)

            if x > xmax
                x = xmax;
            elseif x < xmin
                x = xmin;
            end

            y = x;
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
