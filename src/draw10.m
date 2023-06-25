ax = utils.get_instance().locate_figure('path_ten', 'png');
if isa(ax, 'logical')
    ax = gca;
end
rrt_func('debug',"show_map()")
for i=1:10
    bp = map.get_instance().to_normal_size(p.(['path_',num2str(i)]).prop.best_path);
    plot3(ax, bp(:, 1), bp(:, 2), bp(:, 3), 'LineWidth', 1, 'Color', 'red');
end      
y_lim = [map.get_instance().Y(1) map.get_instance().Y(end)];
x_lim = [map.get_instance().X(1) map.get_instance().X(end)];
view(ax, 2); axis(ax, 'equal');
ylim(ax, y_lim); xlim(ax, x_lim);
if map.get_instance().Y(end) > map.get_instance().X(end)
    view(ax, -90, 90);
end 
hold(ax, 'off');
