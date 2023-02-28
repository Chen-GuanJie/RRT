classdef uav < handle

    properties (SetAccess = private)
        height_limit %贴地高度限制
        deltaT %每步长飞行时间
        g %重力加速度
        v %恒速度
        acc %加速度
        GammaMax %滚转角最大
        GammaMin %滚转角最小
        pitchMax %俯仰角最大
        pitchMin %俯仰角最小
        mini_stable_distance %最短稳定距离

        GammaStep %滚转角最大步长
        pitchstep %俯仰角最大步长
        %最短稳定距离
        course_change %航向角变化
        speeds %速度们
    end

    methods (Access = private)
        % function

        % end
    end

    methods (Access = public)

        function this = uav(conf)
            this.height_limit = conf.height_limit;
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
            this.mini_stable_distance = conf.mini_stable_distance;
            this.course_change = (0:5:120) * pi / 180;
            this.speeds = conf.speeds;
        end

        function [z, pitchangle] = climb(this, temp, closestNode, map)
            targetHeight = map.Z(map.find_closest(temp(1), 0), map.find_closest(temp(2), 1)) + this.height_limit;
            distanceee = norm(temp - closestNode(1:2));
            pitchangle = atan((targetHeight - closestNode(3)) / distanceee);
            pitchangle = this.limiter(pitchangle, this.pitchMax, this.pitchMin);
            pitchangle = this.limiter(pitchangle, closestNode(6) + this.pitchstep, closestNode(6) - this.pitchstep);
            z = pitchangle * distanceee + closestNode(3);
        end

        function newNode = transfer_stable(this, sample, closestNode, map)
            %根据稳定距离转移
            phi1 = atan2(sample(2) - closestNode(2), sample(1) - closestNode(1));
            phi = closestNode(4);
            deltaPhi = this.limit2pi(phi1 - phi);
            distance = norm(sample(1:2) - closestNode(1:2));
            d = distance * sin(this.course_change - deltaPhi) ./ sin(this.course_change);
            speed_index = this.find_closest(520, this.speeds);
            sd = this.mini_stable_distance(:, speed_index)';
            sd = [ones(1, 10) * this.mini_stable_distance(1, speed_index) sd];
            ind = this.find_closest(sd, d);

            if ind <= 10
                mini_distance = 1000 * this.mini_stable_distance(1, speed_index);
            else
                mini_distance = 1000 * this.mini_stable_distance(ind - 10, speed_index);
            end

            if deltaPhi > 0
                newPhi = phi + this.course_change(ind);
            else
                newPhi = phi - this.course_change(ind);
            end

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
            deltaPhi = this.limit2pi(phi1 - phi);
            newGamma = atan(deltaPhi * this.v / this.g / this.deltaT);
            newGamma = this.limiter(newGamma, closestNode(5) + this.GammaStep, closestNode(5) - this.GammaStep);
            newGamma = this.limiter(newGamma, this.GammaMax, this.GammaMin);
            rotation = [cos(phi) -sin(phi); sin(phi) cos(phi); ];

            if newGamma ~= 0
                Rou = tan(newGamma) * this.g / (this.v ^ 2); %计算水平转弯曲率
                deltaPhi = Rou * this.v * this.deltaT;
                newPhi = deltaPhi + phi;
                a = [sin(deltaPhi) / Rou; ((1 - cos(deltaPhi)) / Rou)];
            else
                newPhi = phi;
                a = [this.v * this.deltaT; 0];
            end

            temp = (rotation * a)' + [closestNode(1), closestNode(2)];
            [z, pitchangle] = this.climb(temp, closestNode, map);
            newNode = [temp(1), temp(2), z, newPhi, newGamma, pitchangle];

        end

        function consumption = calc_consumption(~)
            %飞机动力学cost
            consumption = 0;
        end

        function flag = transferable(this, from, to)
            %判断是否可以从 from 到 to
            flag = false;
            movingVec = [to(1) - from(1), to(2) - from(2)];
            phi1 = atan2(movingVec(2), movingVec(1));
            phi = from(4);
            deltaPhi = this.limit2pi(phi1 - phi);
            distanceee = norm(movingVec);
            newGamma = atan(deltaPhi * this.v ^ 2 / this.g / distanceee);

            if (abs(newGamma - from(5)) < this.GammaStep) && (newGamma < this.GammaMax) && (newGamma > this.GammaMin)
                pitchangle = atan((to(3) - from(3)) / distanceee);

                if (abs(pitchangle - from(6)) < this.pitchstep * distanceee / this.v / this.deltaT) && (pitchangle < this.pitchMax) && (pitchangle > this.pitchMin)
                    flag = true;
                end

            end

        end

    end

    methods (Static)

        function index = find_closest(x, list)
            a = abs(list - x);
            [~, index] = min(a);
            % index = find(a == mini);
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

            while x > 3.1416
                x = x - 6.2832;

            end

            while x <- 3.1416
                x = x + 6.2832;

            end

        end

    end

end
