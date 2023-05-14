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

    end

    methods (Access = public)

        function show_result(this)
            display = this.config_manger.load(this.rand_id).display;
            fn = fieldnames(display);

            for i = 1:length(fn)
                plot_name = fn{i};
                plot_info = display.(plot_name);

                if ~isfield(plot_info, 'x_lable') || yaml.isNull(plot_info.x_lable) ...
                        || ~isfield(plot_info, 'y_lable') || yaml.isNull(plot_info.y_lable) ...
                        || ~isfield(plot_info, 'y_data') || yaml.isNull(plot_info.y_data)
                    continue
                end

                y_data = plot_info.y_data;

                if yaml.isNull(y_data)
                    continue
                else
                    to_plot = fieldnames(y_data);
                end

                legend_str = {};
                utils.get_instance().locate_figure(plot_name, plot_info.save_format);
                title(strrep(plot_name, '_', ' '));
                xlabel(plot_info.x_lable);
                ylabel(plot_info.y_lable);

                for j = 1:length(to_plot)

                    if isfield(plot_info, 'x_data') && ~yaml.isNull(plot_info.x_data)
                        x = this.(plot_info.x_data);

                    else
                        x = 1:length(this.shaped_states.(to_plot{j}));
                    end

                    if isfield(this.shaped_states, to_plot{j})

                        p = plot(x, this.shaped_states.(to_plot{j}));

                        if ~yaml.isNull(y_data.(to_plot{j}))
                            line_properties = y_data.(to_plot{j});
                            line_property = fieldnames(line_properties);

                            for k = 1:length(line_property)
                                p.(line_property{k}) = line_properties.(line_property{k});
                            end

                        end

                        legend_str{end + 1} = strrep(to_plot{j}, '_', ' ');
                        hold on
                    end

                end

                legend(legend_str);
                hold off
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

        function path = save_all(this)
            conf = this.config_manger.load(this.rand_id);

            if conf.is_save
                path = [this.config_manger.save_path, datestr(now, 'mmmm-dd-yyyy HH-MM-SS AM'), '/'];
                disp(['files saved at ', path]);
                utils.checkdir(path, true);
                utils.get_instance().save_figures(path);
                this.config_manger.save(path);
            end

        end

        result = record_fun(obj)
    end

end
