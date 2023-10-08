function fig = animate_system(uav,uavs,t_span,VIDEO,video_title,t_inc)

if VIDEO, video=video_writer(video_title, t_inc); end

[fig,c] = plot_system(uav,uavs,0);
set(gcf,'Visible','on')

t_range = 1:size(t_span,2);

for i = uavs
    mav_view(i) = uav_viewer();
    c(i,:) = rand(1,3);
    h(i) = animatedline('Color',c(uavs==i,:),'LineWidth',1.2);
end

for j = t_range
    for i = uavs
        path_index = ~isnan(uav(i).states(:,4));
        if path_index(j)
            mav_view(i).update(uav(i).states(j,:),fig);
            hold on
            addpoints(h(i),uav(i).states(j,2),uav(i).states(j,1),-uav(i).states(j,3))
        end
        collision_index = uav(i).collision.status == 1;
        if collision_index(j)
            scatter3(uav(i).states(j,2),uav(i).states(j,1),...
                -uav(i).states(j,3),20,'r','p');
        end
%         pause(0.01)
    end
    if VIDEO, video.update(t_span(j));  end
end

if VIDEO, video.close(); end
