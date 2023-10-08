function fig = animate_system_seq(Yhist, Thist, tar0, uavs, obs, collision_over_time, ...
    VIDEO, video_title, store, picture_title, animate, ...
    t_inc, c, t_snaps, e, scenario_num, focus_uav)

if VIDEO, video = video_writer(video_title, t_inc); end
env_pre_req;

fontsize = 35;
fig = plot_system_seq(Yhist, Thist, tar0, collision_over_time, ...
        uavs, obs, 0, c, animate);
n_uavs = size(Yhist, 1);
uavs = 1:n_uavs;
t_range = 1:10*round(log(n_uavs)):size(Thist, 2);

for i = uavs
    mav_view(i) = uav_viewer(focus_uav);
    h(i) = animatedline('Color', c(i, :), 'LineWidth', 1.5);
end

coll_point_run = true(1, size(Yhist, 1));
for j = t_range
    for i = uavs
        path_index = ~isnan(Yhist(i, 7, :));
        if path_index(j)
            mav_view(i).update(Thist(j), Yhist(i, :, j), i, e, scenario_num);
            hold on
            addpoints(h(i), Yhist(i, 2, j), Yhist(i, 1, j), -Yhist(i, 3, j))
        end
        collision_index = find(collision_over_time(:, i)==1);
        if ~isempty(collision_index)
            if j >= collision_index && coll_point_run(i)
                coll_point_run(i) = false;
                scatter3(Yhist(i, 2, collision_index), Yhist(i, 1, collision_index), ...
                        -Yhist(i, 3, collision_index), 150, 'r', 'p', 'filled');
            end
        end
    end

    if any(Thist(j) == t_snaps)
        fig_name = "preview_time_" + Thist(j) + ".png";
        if store
            exportgraphics(fig, picture_title + fig_name, 'Resolution', 300); 
        end
        if VIDEO
            for loop = 1:30
             video.update(Thist(j));
            end
        end
    end
    if VIDEO, video.update(Thist(j));  end
end

if VIDEO
    video.close();
end
