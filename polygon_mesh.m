%%
function [poly_pn,poly_pe,poly_pd] = polygon_mesh(points_xy,height,n_points,n_h)
n_sides = size(points_xy,1);
for i = 1:n_sides
    if i<n_sides
        next_pn = points_xy(i+1,1)-points_xy(i,1);
        next_pe = points_xy(i+1,2)-points_xy(i,2);
    else
        next_pn = points_xy(1,1)-points_xy(i,1);
        next_pe = points_xy(1,2)-points_xy(i,2);
    end
    points_along_side_pn(:,i) = points_xy(i,1) + ((0:n_points)./n_points) * next_pn;
    points_along_side_pe(:,i) = points_xy(i,2) + ((0:n_points)./n_points) * next_pe;
%     side_log(i,:) = points_along_perimeter>=start & points_along_perimeter<start+sides(i);
%     start = start + sides(i);
end
% points_along_side_pn = [points_along_side_pn;points_along_side_pn(1,:)];
% points_along_side_pe = [points_along_side_pe;points_along_side_pe(1,:)];
poly_pn = repmat(points_along_side_pn(:)',[n_h,1]);
poly_pe = repmat(points_along_side_pe(:)',[n_h,1]);
poly_pd = -repmat(linspace(0,height,n_h)',[1,(n_points+1)*n_sides]);