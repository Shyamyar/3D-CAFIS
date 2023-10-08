%% Plot Given UAV States

function fig = plot_system_seq(Yhist, Thist, tar0, collision_over_time, ...
                uavs, obs, path0, c, animate)
plot_rot = [0 1 0;
            1 0 0;
            0 0 -1];

% uavs = 1:size(Yhist, 1);
states0_plot = pagemtimes(Yhist(:, 1:3, 1), plot_rot);
states_plot = pagemtimes(Yhist(:, 1:3, :), plot_rot);
target_states0_plot = pagemtimes(tar0(:, :), plot_rot);
% c = rand(size(uavs, 2), 3);

fig = figure();
% ch  = 400;
% fig.Position = [680-ch   558-ch   560+ch   420+ch];
fontsize = 18;
set(fig, 'DefaultAxesFontSize', fontsize)
set(fig, 'DefaultLineLineWidth', 1.1);

s1 = scatter3(states0_plot(:, 1), states0_plot(:, 2), ...
        states0_plot(:, 3), 40, c, '^', 'filled', 'DisplayName', 'Start Positions');
% text(states0_plot(:, 1), states0_plot(:, 2), states0_plot(:, 3), "   "+string(uavs))
grid on
hold on

s2 = scatter3(target_states0_plot(:, 1), target_states0_plot(:, 2), ...
      target_states0_plot(:, 3), 40, c, 'diamond', 'DisplayName', 'Target Positions');

empty_struct = struct();
p = [];
if ~isequal(obs, empty_struct)
    scatter3(obs.pe(:), obs.pn(:), -obs.pd(:), 20, 'o', ...
        'MarkerEdgeColor', 'b', ...
        'MarkerEdgeAlpha', 0.2, ...
        'MarkerFaceColor', [0 .75 .75], ...
        'MarkerFaceAlpha', 0.2)
end

for i = 1:size(uavs,2)
    collision_index = find(collision_over_time(:, i)==1);
    temp = num2cell(squeeze(Yhist(i, :, 1)), 1);
    [pn, pe, pd, Va, phi, gamma, psi, phi_dot] = deal(temp{:});
    V_ned = Vel_vec(Va, psi, gamma, phi);
    V_ned_plot(i, :) = knots2nms(plot_rot * V_ned);

    line = [states0_plot(i, :);target_states0_plot(i, :)];
%     plot3(line(:, 1), line(:, 2), line(:, 3), ':', 'Color', c(index, :))

    quiv = quiver3(states0_plot(i, 1), states0_plot(i, 2), ...
                    states0_plot(i, 3), V_ned_plot(i, 1), ...
                    V_ned_plot(i, 2), V_ned_plot(i, 3), 10, ...
                    'MaxHeadSize', 1, 'Color', c(i, :), 'LineStyle', '-');
    quiv.HandleVisibility = "off";

    if path0
        path = squeeze(states_plot(i, :, :))';
        uav_name = sprintf("UAV %d path", i);
        p(i) = plot3(path(:, 1), path(:, 2), path(:, 3), '-', ...
                'Color', c(i, :), 'DisplayName', uav_name);
        if ~isempty(collision_index)
            coll_points = states_plot(i, :, collision_index);
            scat = scatter3(coll_points(1), coll_points(2), coll_points(3), ...
                150, 'r', 'p', 'filled', 'DisplayName', 'Collisions');
            scat.HandleVisibility = "off";
        end
    end

end

zoomin = 1.2;
az = -50;
el = 30;

axis equal
axis([-zoomin, zoomin, ...
      -zoomin, zoomin, ...
      0, zoomin]);
view(az, el)
if length(p) <= 2 && path0
    leg = legend([s1 s2 p(1:2)], 'FontSize', 16);
elseif ~animate
    leg = legend([s1 s2]);
end
title(gca, sprintf("Time: %.2f sec", floor(Thist(end))));
leg.Location = "best";
xlabel('$\rho_e$ (nm)', 'Interpreter', 'latex')
ylabel('$\rho_n$ (nm)', 'Interpreter', 'latex')
zlabel('$h$ (nm)', 'Interpreter', 'latex')
set(gca, 'FontName', 'times')

