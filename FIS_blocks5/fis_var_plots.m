clc;
close all;
uav_plot = input("UAV: ");
n_fis = input("FIS: ");
n_in = size(fis.FIS(1,n_fis).Inputs,2);
n_out = size(fis.FIS(1,n_fis).Outputs,2);
% rel = del_psi_oi(:), d_h(:), d_v(:), d_v_mag(:), V_x_seq, V_z_seq, d_slant(:)
% des = chi_d, h_d, Va_d, del_chi_d, del_h_d
% out = W_h, W_v, del_chi_ca, del_V_ca_h, del_h_ca, del_V_ca_z
switch n_fis
    case 1
        fis_in.rel = [2 1]; % d_h, del_psi_oi
        fis_in.des = 4; % del_chi_d
        fis_in.out = [];
        fis_out = 3; % del_chi_ca
    case 2
        fis_in.rel = [5 2]; % V_x, d_h
        fis_in.des = [];
        fis_in.out = [];
        fis_out = 4; % del_V_ca_h
    case 3
        fis_in.rel = [];
        fis_in.des = [];
        fis_in.out = [3 4]; % del_chi_ca, del_V_ca_h
        fis_out = 1; % W_h
    case 4
        fis_in.rel = [4 3]; % d_v_mag, d_v
        fis_in.des = 5; % del_h_d
        fis_in.out = [];
        fis_out = 5; % del_h_ca
    case 5
        fis_in.rel = [6 3]; % V_z, d_v
        fis_in.des = [];
        fis_in.out = [];
        fis_out = 6; % del_V_ca_z
    case 6
        fis_in.rel = [];
        fis_in.des = [];
        fis_in.out = [5 6]; % del_h_ca, del_V_ca_z
        fis_out = 2; % W_v
end

% Relative Inputs in FIS
if ~isempty(fis_in.rel)
figure()
for i = 1:size(fis_in.rel,2)
    subplot(size(fis_in.rel,2),2,i)
    plot(t_span,squeeze(nearest_coll_det_states_time(uav_plot,fis_in.rel(i),:)))
    grid on

    subplot(size(fis_in.rel,2),2,i+2)
    plotmf(fis.FIS(1,n_fis),'input',i)
    grid on
end
end

% Desired Inputs in FIS
if ~isempty(fis_in.des)
figure()
for j = 1:size(fis_in.des)
    subplot(size(fis_in.des,2),2,j)
    plot(t_span,squeeze(deshist(uav_plot,fis_in.des(j),:)))
    grid on

    subplot(size(fis_in.des,2),2,j+1)
    plotmf(fis.FIS(1,n_fis),'input',j+i)
    grid on
end
end

% Output Inputs in FIS
if ~isempty(fis_in.out)
figure()
for k = 1:size(fis_in.out)
    subplot(size(fis_in.out,2),2,k)
    plot(t_span,squeeze(fis_outputs(uav_plot,fis_in.out(k),:)))
    grid on

    subplot(size(fis_in.out,2),2,k+1)
    plotmf(fis.FIS(1,n_fis),'input',k+j)
    grid on
end
end

% Outputs in FIS
if ~isempty(fis_out)
figure()
for i = 1:size(fis_out)
    subplot(size(fis_out,2),2,i)
    plot(t_span,squeeze(fis_outputs(uav_plot,fis_out(i),:)))
    grid on

    subplot(size(fis_out,2),2,i+1)
    plotmf(fis.FIS(1,n_fis),'output',i)
    grid on
end
end