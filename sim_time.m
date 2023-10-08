function time = sim_time(tau)
%% Simulation Time
time.t0         = 0;        % Start time of simulation
time.t_inc      = 0.1;      % Sampling time (second)
time.dt         = 0.1;      % Step size for ode (assumed to be same as sampling time)
time.tmax       = 130;      % Total SIM time (seconds)
time.t_span     = time.t0:time.t_inc:time.tmax;
time.post_time  = tau;
