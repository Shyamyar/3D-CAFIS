%% Plot Given UAV States

function fig = plot_system_seq(Yhist,tar0,collision_over_time,path)
plot_rot = [0 1 0;
            1 0 0;
            0 0 -1];

fig = figure();
uavs = 1:size(Yhist,1);
states0_plot = mtimesx(Yhist(uavs,1:3,1),plot_rot);
states_plot = mtimesx(Yhist(:,1:3,:), plot_rot);
target_states0_plot = tar0(uavs,:) * plot_rot;
c = rand(size(uavs,2),3);
scatter3(states0_plot(:,1),states0_plot(:,2),states0_plot(:,3),20,c,'.')
text(states0_plot(:,1),states0_plot(:,2),states0_plot(:,3),"\leftarrow"+string(uavs))
grid on
hold on
axis equal
scatter3(target_states0_plot(:,1),target_states0_plot(:,2),target_states0_plot(:,3),20,c,'*')
coll_point_run = false;

for i = uavs
    index = uavs==i;
    collision_index = collision_over_time(:,i);
    temp = num2cell(squeeze(Yhist(i,:,1)),1);
    [pn, pe, pd, psi, Va, gamma, phi, phi_dot] = deal(temp{:});
    V_ned = Vel_vec(Va,psi,gamma,phi);
    V_ned_plot(index,:) = knots2nms(plot_rot * V_ned);

    line = [states0_plot(index,:);target_states0_plot(index,:)];
%     plot3(line(:,1),line(:,2),line(:,3),':','LineWidth',0.1,'Color',c(index,:))

    quiver3(states0_plot(index,1),states0_plot(index,2),states0_plot(index,3),V_ned_plot(index,1),...
        V_ned_plot(index,2),V_ned_plot(index,3),10,'Color',c(index,:),'LineStyle','-','LineWidth',1)

    if path
        path = squeeze(states_plot(index,:,:))';
        plot3(path(:,1),path(:,2),path(:,3),'-','LineWidth',1.2,'Color',c(index,:))
        if any(collision_index)
            coll_points = reshape(states_plot(index,:,collision_index),[3,sum(collision_index)])';
            scatter3(coll_points(end,1),coll_points(end,2),coll_points(end,3),50,c(index,:),'p')
        end
    end
    
end

xlabel('pe (nm)')
ylabel('pn (nm)')
zlabel('h (nm)')
axis(0.1*[-15,15,-15,15,-5,15]);