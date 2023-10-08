function [total_cost, total_collisions_post, over_time] = ...
    cost_calc(collision_over_time_log, dist2target_time, d_slant_time, time, th)
%% Collision and Cost Stats Calculation
post_time               = time.post_time; %sec
collision_over_time_log_post = collision_over_time_log;
collision_over_time_log_post(time.t_span<=post_time,:) = 0;

over_time.sum_coll      = cumsum(sum(collision_over_time_log, 2));
over_time.sum_coll_post = cumsum(sum(collision_over_time_log_post, 2));
total_collisions        = over_time.sum_coll(end);
total_collisions_post   = over_time.sum_coll_post(end);

time_of_collision       = time.t_span';
time_of_collision(~any(collision_over_time_log,2)) = time.tmax;
time_of_collision_post  = time.t_span';
time_of_collision_post(~any(collision_over_time_log_post,2)) = time.tmax;

d_slant_th = norm(th.DMOD, convlength(th.ZTHR, 'ft','naut mi'));
d_slant_time(isnan(d_slant_time)) = d_slant_th; % d_slant is NAN

% Weights for Penalties
w_coll          = 300000;
w_route         = 10;
w_separation    = 1000;
w_toc           = 500;

% Penalty over time
over_time.penalty_coll   = w_coll .* over_time.sum_coll_post; % for safety, no collision
over_time.penalty_route  = w_route .* cumsum(sum(dist2target_time, 1)'); % for shortest route
over_time.penalty_sep    = w_separation .* cumsum(sum(d_slant_th - d_slant_time, 1)'); % penalizing distances lower than threshold
over_time.penalty_toc    = w_toc * cumsum(time.tmax - time_of_collision_post); % penalizing quicker collisions
over_time.cost           = over_time.penalty_coll + over_time.penalty_route + ...
                                    over_time.penalty_sep + over_time.penalty_toc;
% Penalty in the end
penalty_coll    = over_time.penalty_coll(end);
penalty_route   = over_time.penalty_route(end);
penalty_sep     = over_time.penalty_sep(end);
penalty_toc     = over_time.penalty_toc(end);
total_cost      = over_time.cost(end);

% Display Costs
disp_coll       = "Total Colliding UAVs in the end: " + string(total_collisions);
disp_coll_post  = "Total Colliding UAVs post "+ string(post_time) +" sec till the end: " + string(total_collisions_post);
disp_pen_coll   = "Total Collision Cost at the end: " + string(penalty_coll);
disp_pen_route  = "Total Route Cost at the end: " + string(penalty_route);
disp_pen_sep    = "Total Separation Cost at the end: " + string(penalty_sep);
disp_pen_toc    = "Total Time of Collision Cost at the end: " + string(penalty_toc);
disp_cost       = "Total Cost (including route) at the end: " + string(total_cost);
disp(disp_coll)
disp(disp_coll_post)
disp(disp_pen_coll)
disp(disp_pen_route)
disp(disp_pen_sep)
disp(disp_pen_toc)
disp(disp_cost)
