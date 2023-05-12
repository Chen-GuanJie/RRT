classdef benchmark < handle

    properties (SetAccess = private)
        interest
        is_benchmark = false
        states
        data = struct
        step_record
        time_stamp
        iter_stamp
        last_benchmark = 0
        ind_benchmark = 1
        is_time_step = true
        num_record = 1;
    end

    methods (Access = private)

        function new_obj = copy(this)
            new_obj = struct;

            for i = 1:length(this.interest)
                new_obj.(this.interest{i}) = this.(this.interest{i});
            end

        end

    end

    methods (Access = public)

        function start_benchmark(this, conf)
            this.is_benchmark = conf.is_benchmark;

            if this.is_benchmark
                this.interest = conf.interest;
                this.is_time_step = conf.is_time_step;
                this.assert_interest();
                this.step_record = conf.benchmark_record_step;
                this.num_record = conf.max_iter / this.step_record;
                this.states = cell(this.num_record, 1);
                this.time_stamp = zeros(this.num_record, 1);
                this.iter_stamp = zeros(this.num_record, 1);
                this.last_benchmark = 0;
                this.ind_benchmark = 1;
            end

        end

        function is_finished = record(this, time, num)
            t = tic;
            is_finished = true;

            if this.is_time_step
                ind = time;
            else
                ind = num;
            end

            if this.is_benchmark && ind - this.last_benchmark > this.step_record
                this.last_benchmark = ind;
                this.states{this.ind_benchmark} = {this.copy() this.record_fun()};
                this.time_stamp(this.ind_benchmark) = time;
                this.iter_stamp(this.ind_benchmark) = num;
                this.ind_benchmark = this.ind_benchmark + 1;

                if this.ind_benchmark > this.num_record
                    is_finished = false;
                end

            end

            record_time = toc(t);

            if this.is_time_step
                this.last_benchmark = this.last_benchmark + record_time;
            end

        end

        function assert_interest(this)
            props = properties(this);
            no_prop = false(length(this.interest), 1);

            for i = 1:length(this.interest)

                if ~contains(props, this.interest{i})
                    no_prop(i) = true;
                    disp(strcat('no interest propertie ', this.interest{i}))
                end

            end

            this.interest(no_prop) = [];
        end

        result = record_fun(obj)

    end

end
