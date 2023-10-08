function [obs_pts_states0,n_obs_pts,obs] = obstacles(obs_scenario,n_states)
n_obs_pts = 0;
obs_pts_states0 = double.empty(0,n_states);
obs = struct();
if ~isempty(obs_scenario)
    if any(obs_scenario==1)
        % Vertical Cylinder
        radius = convlength(500,'ft','naut mi');
        circum_res_cyl = 0.0165;
        n_radius = round((2*pi*radius)/circum_res_cyl);
        height_cyl = 0.6; %0.7
        height_res_cyl = 0.02;
        n_h_cyl = round(height_cyl/height_res_cyl);
        [cyl_pn,cyl_pe,cyl_pd] = cylinder_mesh(radius,height_cyl,n_radius,n_h_cyl);
        offset_cyl = [0,0,0];
        cyl_pn = cyl_pn + offset_cyl(1);
        cyl_pe = cyl_pe + offset_cyl(2);
        cyl_pd = cyl_pd + offset_cyl(3);    
        cyl_pts0 = [cyl_pn(:),cyl_pe(:),cyl_pd(:)];
        n_cyl_pts = size(cyl_pts0,1);
        cyl_pts_states0 = [cyl_pts0,zeros(n_cyl_pts,5)];
        obs_pts_states0 = [obs_pts_states0;cyl_pts_states0];
    end
    if any(obs_scenario==2)
        % Horizontal Cylinder
        radius = convlength(220,'ft','naut mi');
        circum_res_cyl = 0.0165;
        n_radius = round((2*pi*radius)/circum_res_cyl);
        height_cyl = 0.6;
        height_res_cyl = 0.02;
        n_h_cyl = round(height_cyl/height_res_cyl);
        [cyl_pn,cyl_pe,cyl_pd] = cylinder_mesh(radius,height_cyl,n_radius,n_h_cyl);
        offset_cyl = [0,0,height_cyl/2]; %[0.1,-0.1,0.3];
        cyl_pn = cyl_pn + offset_cyl(1);
        cyl_pe = cyl_pe + offset_cyl(2);
        cyl_pd = cyl_pd + offset_cyl(3);    
        cyl_pts0 = [cyl_pn(:),cyl_pe(:),cyl_pd(:)];
        cyl_pts0 = transpose((C(2,pi/2)*C(3,-pi/4))' * cyl_pts0');
        offset_cyl = [0,0,-1];
        cyl_pts0 = cyl_pts0 + offset_cyl;
        n_cyl_pts = size(cyl_pts0,1);
        cyl_pts_states0 = [cyl_pts0,zeros(n_cyl_pts,5)];
        obs_pts_states0 = [obs_pts_states0;cyl_pts_states0];
    end
    if any(obs_scenario==3)        
        % Vertical Cuboid
        len = convlength(900,'ft','naut mi');
        breadth = convlength(500,'ft','naut mi');
        height_cub = 0.6;
        height_res_cub = 0.02;
        n_h_cub = round(height_cub/height_res_cub);
        bbox = [0,0,len,breadth];
        points_xy = bbox2points(bbox);
        circum_res_rec = 0.0165;
        n_points = round(2*(len+breadth)/circum_res_rec);
        [rec_pn,rec_pe,rec_pd] = polygon_mesh(points_xy,height_cub,n_points,n_h_cub);
        offset_rec = [-len/2,-breadth/2,0];
        rec_pn = rec_pn + offset_rec(1);
        rec_pe = rec_pe + offset_rec(2);
        rec_pd = rec_pd + offset_rec(3);    
        rec_pts0 = [rec_pn(:),rec_pe(:),rec_pd(:)];
        n_rec_pts = size(rec_pts0,1);
        rec_pts_states0 = [rec_pts0,zeros(n_rec_pts,5)];
        obs_pts_states0 = [obs_pts_states0;rec_pts_states0];
    end
    if any(obs_scenario==4)        
        % Horizontal Cuboid
        len = convlength(230,'ft','naut mi');
        breadth = convlength(500,'ft','naut mi');
        height_cub = 0.6;
        height_res_cub = 0.02;
        n_h_cub = round(height_cub/height_res_cub);
        bbox = [0,0,len,breadth];
        points_xy = bbox2points(bbox);
        circum_res_rec = 0.0165;
        n_points = round(2*(len+breadth)/circum_res_rec);
        [rec_pn,rec_pe,rec_pd] = polygon_mesh(points_xy,height_cub,n_points,n_h_cub);
        offset_cyl = [-len/2,-breadth/2,height_cub/2]; %[0.1,-0.1,0.3];
        rec_pn = rec_pn + offset_cyl(1);
        rec_pe = rec_pe + offset_cyl(2);
        rec_pd = rec_pd + offset_cyl(3);    
        rec_pts0 = [rec_pn(:),rec_pe(:),rec_pd(:)];
        rec_pts0 = transpose((C(2,pi/2)*C(3,-pi/4))' * rec_pts0');
        offset_cyl = [0,0,-1];
        rec_pts0 = rec_pts0 + offset_cyl;
        n_rec_pts = size(rec_pts0,1);
        rec_pts_states0 = [rec_pts0,zeros(n_rec_pts,5)];
        obs_pts_states0 = [obs_pts_states0;rec_pts_states0];
    end

    % Combine
    n_obs_pts = size(obs_pts_states0,1);
    obs.pn = obs_pts_states0(:,1);
    obs.pe = obs_pts_states0(:,2);
    obs.pd = obs_pts_states0(:,3);
end
