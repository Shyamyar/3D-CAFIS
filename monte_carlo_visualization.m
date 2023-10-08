%% Monte Carlo Simulation Results Visualization
load("results\monte_coll_20.mat")

%% Bar Diagram Viusalization
b = bar3(Num_UAV_iter,monte_coll_nca);
xlabel("Monte Carlo Simulation No.")
ylabel("Number of UAVs")
zlabel("Number of collisions")
title("Monte Carlo Simulations with CA Off")
colorbar
for k = 1:length(b)
zdata = b(k).ZData;
b(k).CData = zdata;
b(k).FaceColor = 'interp';
end

%% Average Plot Visualizaiton for CA and NCA
avg_mont_coll_nca = mean(monte_coll_nca,2);
avg_mont_coll_ca = mean(monte_coll_ca,2);
avg_diff_ca_nca = (avg_mont_coll_nca - avg_mont_coll_ca);
avg_diff_ca_nca_percent = mean((avg_mont_coll_nca - avg_mont_coll_ca)./ avg_mont_coll_nca);

plot(avg_mont_coll_ca)
hold on
plot(avg_mont_coll_nca)
legend("ca","nca")
title("Average Collisions over time for CA on and off ")

%% Plot Comparing Collisions over time for CA and NCA
plot(sum_coll_over_time_ca) 
hold on
plot(sum_coll_over_time_nca)
legend("ca","nca")
title("Comparing Collisions over time for CA on and off (for 100 UAVs)")
