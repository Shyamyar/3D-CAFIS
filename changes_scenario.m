%% Load Scenario
clear;
clc;
addpath("scenarios\")
scenarios = dir("scenarios/*.mat");
scenario_str = string({scenarios.name});

s = size(scenario_str,2);
for scenario_num = 1:s
    scenario  = scenario_str(scenario_num);
    load(scenario,'states0','target_states0')
    % states0 = states0(:,[1,2,3,4,7,6,5,8]); % Change positions of states in state vector
    states0(:,7) = wrapTo2Pi(states0(:,7));
    filename = 'scenarios\'+ scenario_str(scenario_num);
    save(filename,'states0','target_states0')
end
