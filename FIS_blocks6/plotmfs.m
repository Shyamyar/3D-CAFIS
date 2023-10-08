function fig = plotmfs(fis, n_fis_plot, store, fis_type)

if ~isempty(fis)
fisx = fis.FIS(1,n_fis_plot);
n_in = size(fisx.Inputs, 2);
n_out = size(fisx.Outputs, 2);
n_tot = n_in + n_out;

fontsize = 12;
del_width = 150;
del_length = 150;

fig = figure('Position',[3   51   560+del_width   420+del_length]);
fig_name = fisx.Name + "_MFs";
set(gcf,'DefaultLineLineWidth',1.1)
set(gcf,'DefaultAxesFontSize',fontsize)
for i = 1:n_tot
    subplot(2,2,i)
    if i<=n_in
        plotmf(fisx, 'input', i);
    else
        plotmf(fisx, 'output', i-n_in);
    end
    grid on
    set(gca,'FontName', 'times', 'FontSize', fontsize)
    set(gca().XLabel,'Interpreter','latex')
    set(gca().YLabel,'Interpreter','latex')
    set(findall(gcf,'type','text'),'FontSize', fontsize)
end
if store
    exportgraphics(fig, "pictures\scenario" + fis_type + "_" + ...
        fig_name + ".png", 'Resolution', 300)
end
end

% % For single MF
% set(gcf,'DefaultLineLineWidth',1.1)
% plotmf(fis.FIS(1,1), 'input', 1)
% grid on
% set(gca().XLabel,'Interpreter','latex')
% set(gca().YLabel,'Interpreter','latex')
% set(findall(gcf,'type','text'),'FontSize', 18)
% set(gca,'FontName', 'times', 'FontSize', 16)