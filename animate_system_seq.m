function fig = animate_system_seq(Yhist,Thist,tar0,collision_over_time,VIDEO,video_title,t_inc)

if VIDEO, video=video_writer(video_title, t_inc); end

fig = plot_system_seq(Yhist,tar0,collision_over_time,0);
set(gcf,'Visible','on')

uavs = 1:size(Yhist,1);
t_range = 1:5:size(Thist,2);

for i = uavs
    mav_view(i) = uav_viewer();
    c(i,:) = rand(1,3);
    h(i) = animatedline('Color',c(uavs==i,:),'LineWidth',1.2);
end

coll_point_run = false(1,size(Yhist,1));
for j = t_range
    for i = uavs
        if ~coll_point_run(i)
            path_index = ~isnan(Yhist(i,4,:));
            if path_index(j)
                mav_view(i).update(Yhist(i,:,j),fig);
                hold on
                addpoints(h(i),Yhist(i,2,j),Yhist(i,1,j),-Yhist(i,3,j))
            end
            collision_index = collision_over_time(:,i);
            if collision_index(j)
                coll_point_run(i) = true;
                scatter3(Yhist(i,2,j),Yhist(i,1,j),-Yhist(i,3,j),20,c(i,:),'p');
            end
        end
    end
    if VIDEO, video.update(Thist(i,j));  end
end

if VIDEO, video.close(); end
