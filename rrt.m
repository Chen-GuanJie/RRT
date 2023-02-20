classdef rrt < handle

    properties (SetAccess = private)
        tree
        maps
        plane
        start
        goal
        threshold
        maxFailedAttempts
        searchSize
        searchbBase
        randnum

        nearNode
    end

    methods (private)

        function this = rrt(conf)
            maps = map(conf.filename);
            X = maps.X;
            Y = maps.Y;
            Height = maps.Z;
            this.start = [X(conf.start(1)), Y(conf.start(2)), Height(conf.start(1), conf.start(2)) + conf.start(3), conf.start(4), conf.start(5), conf.start(6)];
            this.goal = [X(conf.goal(1)), Y(conf.goal(2)), Height(conf.goal(1), conf.goal(2)) + conf.goal(3), conf.goal(4), conf.goal(5), conf.goal(6)];
            this.threshold = conf.threshold;
            this.maxFailedAttempts = conf.maxFailedAttempts;
            %this.searchSize = conf.search * [(goal(1) - start(1)), (goal(2) - start(2)), goal(3) - start(3), 0, 0, 0];
            this.randnum = conf.randnum;
            this.searchbBase = [min(X), min(Y), min(Height), -3.1416, -3.1416, -3.1416];

            this.searchSize = [max(X) - min(X), max(Y) - min(Y), max(Height) - min(Height), 2 * 3.1416, 2 * 3.1416, 2 * 3.1416];
        end

        function sample = get_sample(this)

            if rand < this.randnum
                sample = rand(1, 6) .* this.searchSize + this.searchbBase;
            else
                sample = this.goal;
            end

        end

        function get_nearest(this, sample, dis)
            distaces = norm(this.tree(:, 1:6), sample);
            this.nearNode = find(distaces < dis);
        end

        function node=get_closest()
            node=min()
        end

        function newPoint = extends(sample, closestNode)
            newPoint = this.robot.transfer(sample, closestNode);
        end

    end

end
