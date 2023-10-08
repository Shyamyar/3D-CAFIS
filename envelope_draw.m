%%
all_pn = squeeze(Yhist(:,1,:));
all_pe = squeeze(Yhist(:,2,:));
all_pd = squeeze(Yhist(:,3,:));
[pe_sorted,sort_id] = sort(all_pe,2);
% [pe_sorted, Tfrm] = rmoutliers(pe_sorted);
for i = 1:size(sort_id,1)
    pn_sorted(i,:) = all_pn(i,sort_id(i,:));
    pd_sorted(i,:) = all_pd(i,sort_id(i,:));
end
% pn_sorted = pn_sorted(~Tfrm,:);
% pd_sorted = pd_sorted(~Tfrm,:);
min_pn = min(pn_sorted);
max_pn = max(pn_sorted);
rad_pn = abs((max_pn - min_pn)/2);
cen_pn = min_pn + rad_pn;
min_pe = min(pe_sorted);
max_pe = max(pe_sorted);
rad_pe = abs((max_pe - min_pe)/2);
cen_pe = linspace(-1,1,size(rad_pe,2));
min_pd = min(pd_sorted);
max_pd = max(pd_sorted);
rad_pd = abs((max_pd - min_pd)/2);
cen_pd = min_pd + rad_pd;
cen_cyl = [cen_pn',cen_pe',cen_pd']';
plot_rot = [0 1 0;
            1 0 0;
            0 0 -1];
cen_cyl_plot = plot_rot * cen_cyl;
% rad_cyl = max([rad_pn',rad_pd'],[],2); 
rad_cyl = 1.5 * circle_range(2) * ones(size(all_pe,2),1);
plot3(cen_cyl_plot(1,:),cen_cyl_plot(2,:),cen_cyl_plot(3,:))
axis('equal')
hold on
var_tube(rad_cyl,cen_cyl_plot,50);
% alpha 0.1
% colormap summer
shading interp