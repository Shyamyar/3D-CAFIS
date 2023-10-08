function fis_var_plots(uavs, uav2plot, n_fis_plot, fis_norm, fis_actual, th, check_in_ca, time_in_ca, deshist, ...
                nearest_coll_det_states_time, fis_outputs, store, picture_title)

uav_plot = find(uavs==uav2plot);
for n_fis = n_fis_plot
fisx = fis_norm.FIS(1,n_fis_plot);
n_in = size(fis_norm.FIS(1,n_fis).Inputs,2);
n_out = size(fis_norm.FIS(1,n_fis).Outputs,2);
% rel = del_psi_M(:), x(:), z(:), z_mag(:), V_x_seq, V_z_seq, d_slant(:), chi_rel, del_chi_rel
% des = chi_d, h_d, Va_d, del_chi_d, del_z_d
% out = del_V_ca_x, del_V_ca_z, W_x, W_z, del_chi_ca, del_z_ca
switch n_fis
    case 1
        fis_in.rel = [2 1]; % x, del_psi_M
        fis_in.des = 4; % del_chi_d
        fis_in.out = [];
        fis_out = 5; % del_chi_ca
    case 2
        fis_in.rel = [5 2]; % V_x, x
        fis_in.des = [];
        fis_in.out = [];
        fis_out = 1; % del_V_ca_x
    case 3
        fis_in.rel = [2 3]; % x, z
        fis_in.des = 5; % del_z_d
        fis_in.out = [];
        fis_out = 6; % del_z_ca
    case 4
        fis_in.rel = [6 3]; % V_z, z
        fis_in.des = [];
        fis_in.out = [];
        fis_out = 2; % del_V_ca_z
    case 5
        fis_in.rel = [];
        fis_in.des = [];
        fis_in.out = [5 6]; % del_chi_ca, del_z_ca
        fis_out = [3 4]; % W_x W_z
end

del_width = 200;
del_length = 150;
fontsize = 12;

% Relative Inputs in FIS
if ~isempty(fis_in.rel)
    fig = figure('Position',[3   51   560+del_width   420+del_length]);
    fig_name = fisx.Name + "_inputs_rel" + string(uav2plot);
    set(gcf,'DefaultLineLineWidth',1.5)
    set(gcf,'DefaultAxesFontSize',fontsize)
    plotmf(fis_norm.FIS(1,5),'input',1)
    grid on
    for i = 1:size(fis_in.rel,2)
        var_name = fis_actual.FIS(1,n_fis).Inputs(i).Name;
        subplot(2,size(fis_in.rel,2),i)
        plot(time_in_ca, squeeze(nearest_coll_det_states_time(uav_plot,...
                        fis_in.rel(i),check_in_ca)))
        grid on
        if fis_in.rel(i) == 2 % x
            yline(th.coll_nm)
        elseif fis_in.rel(i) == 4 % x
            yline(th.coll_ft)
        end
        xlabel("$t$ (sec)")
        ylabel(var_name,'Interpreter','none')
        set(gca,'FontName', 'times', 'FontSize', fontsize)
        set(gca().XLabel,'Interpreter','latex')
        set(gca().YLabel,'Interpreter','latex')
        set(findall(gcf,'type','text'),'FontSize', fontsize)
    
        subplot(2,size(fis_in.rel,2),size(fis_in.rel,2)+i)
        plotmf(fis_norm.FIS(1,n_fis),'input',i)
        grid on
        set(gca,'FontName', 'times', 'FontSize', fontsize)
        set(gca,'FontName', 'times')
        set(gca().XLabel,'Interpreter','latex')
        set(gca().YLabel,'Interpreter','latex')
        set(findall(gcf,'type','text'),'FontSize', fontsize)
    end
    if store
        exportgraphics(fig, picture_title + fig_name + ".png", 'Resolution', 300)
    end
else
    i=0;
end

% Desired Inputs in FIS
if ~isempty(fis_in.des)
    fig = figure('Position',[3   51   560+del_width   420+del_length]);
    fig_name = fisx.Name + "_inputs_des" + string(uav2plot);
    set(gcf,'DefaultLineLineWidth',1.5)
    set(gcf,'DefaultAxesFontSize',fontsize)
    plotmf(fis_norm.FIS(1,5),'input',1)
    grid on
    for j = 1:size(fis_in.des,2)
        var_name = fis_actual.FIS(1,n_fis).Inputs(j+i).Name;
        subplot(2,size(fis_in.des,2),j)
        plot(time_in_ca,squeeze(deshist(uav_plot,...
                            fis_in.des(j),check_in_ca)))
        grid on
        xlabel("$t$ (sec)")
        ylabel(var_name,'Interpreter','latex')
        set(gca,'FontName', 'times', 'FontSize', fontsize)
        set(gca().XLabel,'Interpreter','latex')
        set(gca().YLabel,'Interpreter','latex')
        set(findall(gcf,'type','text'),'FontSize', fontsize)
    
        subplot(2,size(fis_in.des,2),size(fis_in.des,2)+j)
        plotmf(fis_norm.FIS(1,n_fis),'input',j+i)
        grid on
        set(gca,'FontName', 'times', 'FontSize', fontsize)
        set(gca().XLabel,'Interpreter','latex')
        set(gca().YLabel,'Interpreter','latex')
        set(findall(gcf,'type','text'),'FontSize', fontsize)
    end
    if store
        exportgraphics(fig, picture_title + fig_name + ".png", 'Resolution', 300)
    end
else
    j=0;
end

% Output Inputs in FIS
if ~isempty(fis_in.out)
    fig = figure('Position',[3   51   560+del_width   420+del_length]);
    fig_name = fisx.Name + "_inputs_out" + string(uav2plot);
    set(gcf,'DefaultLineLineWidth',1.5)
    set(gcf,'DefaultAxesFontSize',fontsize)
    plotmf(fis_norm.FIS(1,5),'input',1)
    grid on
    for k = 1:size(fis_in.out,2)
        var_name = fis_actual.FIS(1,n_fis).Inputs(k+j+i).Name;
        subplot(2,size(fis_in.out,2),k)
        plot(time_in_ca,squeeze(fis_outputs(uav_plot,fis_in.out(k),check_in_ca)))
        grid on
        xlabel("$t$ (sec)")
        ylabel(var_name,'Interpreter','latex')
        set(gca,'FontName', 'times', 'FontSize', fontsize)
        set(gca().XLabel,'Interpreter','latex')
        set(gca().YLabel,'Interpreter','latex')
        set(findall(gcf,'type','text'),'FontSize', fontsize)
        
        subplot(2,size(fis_in.out,2),size(fis_in.out,2)+k)
        plotmf(fis_norm.FIS(1,n_fis),'input',k+j+i)
        grid on
        set(gca,'FontName', 'times', 'FontSize', fontsize)
        set(gca().XLabel,'Interpreter','latex')
        set(gca().YLabel,'Interpreter','latex')
        set(findall(gcf,'type','text'),'FontSize', fontsize)
    end
    if store
        exportgraphics(fig, picture_title + fig_name + ".png", 'Resolution', 300)
    end
end

% Outputs in FIS
if ~isempty(fis_out)
    fig = figure('Position',[3   51   560+del_width   420+del_length]);
    fig_name = fisx.Name + "_outputs" + string(uav2plot);
    set(gcf,'DefaultLineLineWidth',1.5)
    set(gcf,'DefaultAxesFontSize',fontsize)
    plotmf(fis_norm.FIS(1,5),'input',1)
    grid on
    for i = 1:size(fis_out,2)
        var_name = fis_actual.FIS(1,n_fis).Outputs(i).Name;
        subplot(2,size(fis_out,2),i)
        plot(time_in_ca,squeeze(fis_outputs(uav_plot,fis_out(i),check_in_ca)))
        grid on
        xlabel("$t$ (sec)")
        ylabel(var_name,'Interpreter','latex')
        set(gca,'FontName', 'times', 'FontSize', fontsize)
        set(gca().XLabel,'Interpreter','latex')
        set(gca().YLabel,'Interpreter','latex')
        set(findall(gcf,'type','text'),'FontSize', fontsize)
        
        subplot(2,size(fis_out,2),size(fis_out,2)+i)
        plotmf(fis_norm.FIS(1,n_fis),'output',i)
        grid on
        set(gca,'FontName', 'times', 'FontSize', fontsize)
        set(gca().XLabel,'Interpreter','latex')
        set(gca().YLabel,'Interpreter','latex')
        set(findall(gcf,'type','text'),'FontSize', fontsize)
    end
    if store
        exportgraphics(fig, picture_title + fig_name + ".png", 'Resolution', 300)
    end
end

end