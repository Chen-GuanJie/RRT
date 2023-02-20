classdef uav < handle

    properties (SetAccess = private)
        height_limit
        deltaT
        g
        v
        GammaMax
        GammaMin
        pitchMax
        pitchMin

        GammaStep %滚转角最大步长
        pitchstep %俯仰角最大步长

    end

    methods (public)

        function this = methodName(conf)
            this.height_limit = conf.height_limit;
            this.deltaT = conf.deltaT;
            this.g = conf.g;
            this.v = conf.v;
            this.GammaMax = conf.GammaMax;
            this.GammaMin = conf.GammaMin;
            this.pitchMax = conf.pitchMax;
            this.pitchMin = conf.pitchMin;

            this.GammaStep = conf.GammaStep;
            this.pitchstep = conf.pitchstep;

        end

        function newNode = transfer(this, sample, closestNode)
            movingVec = [sample(1) - closestNode(1), sample(2) - closestNode(2), ]; %sample(3) - closestNode(3)];
            phi1 = atan(movingVec(2) / movingVec(1));

            if movingVec(2) > 0 && movingVec(1) < 0
                phi1 = phi1 + 3.1416;
            elseif movingVec(2) < 0 && movingVec(1) < 0
                phi1 = phi1 - 3.1416;
            end

            phi = closestNode(4);
            deltaPhi = (-phi + phi1);

            while deltaPhi > 3.1416
                deltaPhi = deltaPhi - 6.2832;

            end

            while deltaPhi <- 3.1416
                deltaPhi = deltaPhi + 6.2832;

            end

            newGamma = atan(deltaPhi * v / g / deltaT);

            if (newGamma - closestNode(5)) > this.GammaStep
                newGamma = closestNode(5) + this.GammaStep;

            elseif (newGamma - closestNode(5)) <- this.GammaStep
                newGamma = closestNode(5) - this.GammaStep;

            end

            if newGamma > this.GammaMax
                newGamma = this.GammaMax;
            elseif newGamma < this.GammaMin
                newGamma = this.GammaMin;
            end

            rotation = [cos(phi) -sin(phi);
                        sin(phi) cos(phi); ];

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

            targetHeight = Height(find_closest(temp(1), X), find_closest(temp(2), Y)) + this.height_limit;
            distanceee = distanceCost([temp(1), temp(2)], [closestNode(1), closestNode(2)]);
            pitchangle = atan((targetHeight - closestNode(3)) / distanceee);

            if pitchangle > this.pitchMax
                pitchangle = this.pitchMax;
            elseif pitchangle < this.pitchMin
                pitchangle = this.pitchMin;
            end

            if pitchangle - closestNode(6) > this.pitchstep
                pitchangle = closestNode(5) + this.pitchstep;

            elseif pitchangle - closestNode(6) <- this.pitchstep
                pitchangle = closestNode(5) - this.pitchstep;

            end

            z = pitchangle * distanceee + closestNode(3);

            newNode = [temp(1), temp(2), z, newPhi, newGamma, pitchangle];

        end

    end

end
