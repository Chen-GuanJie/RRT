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
        shaped_states
    end

    properties (SetAccess = public)
        rand_id
    end

    methods (Access = private)

        function new_obj = copy(this)
            new_obj = struct;

            for i = 1:length(this.interest)
                new_obj.(this.interest{i}) = this.(this.interest{i});
            end

        end

        function save_shaped_states(this, path)
            %save shaped_states in one file
            sample_table = table();
            col = fieldnames(this.shaped_states);

            for i = 1:length(col)
                sample_table.(col{i}) = this.shaped_states.(col{i});
            end

            writetable(sample_table, [path, 'sample_result.csv']);
        end

        function save_other_states(this, path, only_last)
            %save other states
            if nargin == 2
                only_last = true;
            end

            fn2 = fieldnames(this.states{1, 2});

            for i = 1:length(fn2)

                if length(this.states{1, 2}.(fn2{i})) > 1
                    this.states{1, 1}.(fn2{i}) = this.states{1, 2}.(fn2{i});
                end

            end

            fn1 = fieldnames(this.states{1, 1});
            data_num = zeros(length(fn1), 1);

            for i = 1:length(fn1)
                data_num(i, 1) = length(this.states{end, 1}.(fn1{i}));
            end

            unique_len = unique(data_num);
            unique_len(unique_len == 1) = [];

            for k = 1:this.ind_benchmark - 1

                if only_last && k ~= this.ind_benchmark - 1
                    continue
                end

                s = this.states{k, 1};

                for i = 1:length(unique_len)
                    gather_index = find(data_num == unique_len(i));
                    table_name = '';
                    sample_table = table();

                    for j = 1:length(gather_index)
                        sample_table.(fn1{j}) = s.(fn1{j});
                        table_name = [table_name, '-', fn1{j}];
                    end

                    table_name(1) = [];
                    writetable(sample_table, [path, table_name, '-', num2str(k), '.csv']);
                end

            end

        end

    end

    methods (Access = public)

        function show_result(this)
            display = this.config_manger.load(this.rand_id).display;
            fn = fieldnames(display);

            for i = 1:length(fn)
                plot_name = fn{i};
                plot_info = display.(plot_name);

                if ~utils.is_drawable(plot_name, plot_info)
                    continue
                end

                t = title(strrep(plot_name, '_', ' '));
                xl = xlabel(plot_info.x_lable.txt);
                yl = ylabel(plot_info.y_lable.txt);
                utils.assign_value(t, plot_info, 'title_property');
                utils.assign_value(xl, plot_info.x_lable, 'property');
                utils.assign_value(yl, plot_info.y_lable, 'property');
                utils.draw(this.shaped_states, plot_info);

                if isfield(plot_info, 'eval')
                    funcs = fieldnames(plot_info.eval);

                    for index = 1:length(funcs)
                        utils.eval_execute(funcs{index}, plot_info.eval.(funcs{index}));
                    end

                end

            end

        end

        function reshape_states(this)
            fn = fieldnames(this.states{1, 2});
            in1 = {};
            in2 = {};

            for i = 1:length(this.interest)

                if length(this.states{1, 1}.(this.interest{i})) == 1
                    in1{end + 1} = this.interest{i};
                end

            end

            for i = 1:length(fn)

                if length(this.states{1, 2}.(fn{i})) == 1
                    in2{end + 1} = fn{i};
                end

            end

            for j = 1:this.ind_benchmark - 1

                for i = 1:length(in1)

                    new_obj.(in1{i})(j, 1) = this.states{j, 1}.(in1{i});

                end

                for i = 1:length(in2)

                    new_obj.(in2{i})(j, 1) = this.states{j, 2}.(in2{i});

                end

            end

            this.shaped_states = new_obj;
            this.shaped_states.time_stamp = this.time_stamp;
            this.shaped_states.iter_stamp = this.iter_stamp;

        end

        function start_benchmark(this, conf)
            this.is_benchmark = conf.is_benchmark;

            if this.is_benchmark
                this.interest = conf.interest;
                this.is_time_step = conf.is_time_step;
                this.assert_interest();
                this.step_record = conf.benchmark_record_step;
                this.num_record = conf.max_iter / this.step_record;
                this.states = cell(this.num_record, 2);
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
                this.states{this.ind_benchmark, 1} = this.copy();
                this.states{this.ind_benchmark, 2} = this.record_fun();
                this.time_stamp(this.ind_benchmark) = time;
                this.iter_stamp(this.ind_benchmark) = num;
                this.ind_benchmark = this.ind_benchmark + 1;
                fprintf('search %d times in %f seconds\n', num, time)

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

        function path = save_all(this, annotation)
            conf = this.config_manger.load(this.rand_id);

            if conf.is_save

                if nargin == 1
                    path = [this.config_manger.save_path, datestr(now, 'mmmm-dd-yyyy HH-MM-SS AM'), '/'];
                elseif nargin == 2 && ~isempty(annotation)
                    path = [this.config_manger.save_path, datestr(now, 'mmmm-dd-yyyy HH-MM-SS AM '), annotation, '/'];
                end

                utils.checkdir(path, true);
                disp(['files saved at ', path]);
                utils.get_instance().save_figures(path);
                this.config_manger.save(path);
                this.save_shaped_states(path);
                this.save_other_states(path, conf.only_save_last);
            end

        end

        result = record_fun(obj)
    end

end
