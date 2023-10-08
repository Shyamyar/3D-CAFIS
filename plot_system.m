%% Plot Given UAV States
% plot_system(uav,uav_range_plot,path)
% plot_system(states0,target_states0)

function [fig,c] = plot_system(uav,uavs,path)
if nargin < 2 && isa(uav,"struct")
    uavs = 1:size(uav,2);
    states0 = NaN(size(uav,2),8);
    target_states0 = NaN(size(uav,2),3);
    for i = uavs
        target_states0(i,:) = uav(i).target_states0;
        states0(i,:) = uav(i).states0;
    end
    path = 0;
elseif nargin < 3
    path = 0;
    if isa(uav,"struct")
        states0 = NaN(size(uav,2),8);
        target_states0 = NaN(size(uav,2),3);
        for i = uavs
            target_states0(i,:) = uav(i).target_states0;
            states0(i,:) = uav(i).states0;
        end
    elseif isa(uav,"double")
        states0 = uav;
        target_states0 = uavs;
        uavs = 1:size(states0,1);
    end
elseif nargin == 3
    states0 = NaN(size(uav,2),8);
    target_states0 = NaN(size(uav,2),3);
    for i = uavs
        target_states0(i,:) = uav(i).target_states0;
        states0(i,:) = uav(i).states0;
    end
end

plot_rot = [0 1 0;
            1 0 0;
            0 0 -1];

fig = figure();

states0_plot = states0(uavs,1:3) * plot_rot;
target_states0_plot = target_states0(uavs,:) * plot_rot;
c = rand(size(uavs,2),3);
scatter3(states0_plot(:,1),states0_plot(:,2),states0_plot(:,3),20,c,'*')
text(states0_plot(:,1),states0_plot(:,2),states0_plot(:,3),"\leftarrow"+string(uavs))
grid on
hold on
axis equal
scatter3(target_states0_plot(:,1),target_states0_plot(:,2),target_states0_plot(:,3),20,c,'o')

for i = uavs
    index = uavs==i;
    temp = num2cell(states0(i,:));
    [pn, pe, pd, psi, Va, gamma, phi, phi_dot] = deal(temp{:});
    V_ned = Vel_vec(Va,psi,gamma,phi);
    V_ned_plot(index,:) = knots2nms(plot_rot * V_ned);

    line = [states0_plot(index,:);target_states0_plot(index,:)];
    plot3(line(:,1),line(:,2),line(:,3),':','LineWidth',1,'Color',c(index,:))
    
    if path == 1
        path_index = ~isnan(uav(i).states(:,4));
        collision_index = uav(i).collision.status == 1;
        uav(i).states_plot = uav(i).states(path_index,1:3) * plot_rot;
        plot3(uav(i).states_plot(path_index,1),uav(i).states_plot(path_index,2),uav(i).states_plot(path_index,3),'LineWidth',1,'Color',c(index,:));
        scatter3(uav(i).states_plot(collision_index,1),uav(i).states_plot(collision_index,2),...
            uav(i).states_plot(collision_index,3),20,c(index,:),'p');
    end
    quiver3(states0_plot(index,1),states0_plot(index,2),states0_plot(index,3),V_ned_plot(index,1),...
        V_ned_plot(index,2),V_ned_plot(index,3),10,'Color',c(index,:),'LineStyle','-','LineWidth',1)
end

xlabel('pe (nm)')
ylabel('pn (nm)')
zlabel('h (nm)')
axis(0.1*[-15,15,-15,15,-5,15]);