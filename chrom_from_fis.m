%% Creating chromosome using manual FIS
run fis_ranges.m
load("rulelist.mat")
chrom_psi_oi = chrom_range(angle_pi);
chrom_psi_d = chrom_range(angle_pi);
chrom_d_h = chrom_range(dist_nm_mag);
chrom_psi_io = chrom_range(angle_pi);
chrom_interm_A1 = chrom_range(angle_pi);
chrom_interm_A2 = chrom_range(unit_mag);
chrom_V_x = chrom_range(vel_kts);
chrom_d_v = chrom_range(dist_ft);
chrom_del_h_t = chrom_range(dist_ft);
chrom_d_v_mag = chrom_range(dist_ft_mag);
chrom_V_z = chrom_range(vel_ftm);
chrom_interm_B1 = chrom_range(unit_range);
chrom_interm_C1 = chrom_range(unit_mag);
chrom_interm_C2 = chrom_range(unit_mag);
chrom_del_psi_ca = chrom_range(del_angle_pi);
chrom_del_V_x_ca = chrom_range(del_vel_kts);
chrom_del_V_z_ca = chrom_range(del_vel_ftm);
chrom_del_h_ca = chrom_range(del_dist_ft);
chrom_W_h = chrom_range(unit_mag);
chrom_rule = [rulelist.FIS_A1(:,3);
              rulelist.FIS_A2(:,3);
              rulelist.FIS_A3(:,3);
              rulelist.FIS_A4(:,3);
              rulelist.FIS_B1(:,3);
              rulelist.FIS_B2(:,3);
              rulelist.FIS_B3(:,3);
              rulelist.FIS_C1(:,3);
              rulelist.FIS_C2(:,3);
              rulelist.FIS_C3(:,3)]';

% [psi_oi psi_d d_h psi_i/o interm_A1 interm_A2 V_x d_v del_h_t d_v_mag V_z 
% interm_B1 interm_C1 interm_C2 del_psi_ca del_V_x_ca del_V_z_ca
% del_h_ca W_h] for Chromosome of GA

Field_DR_in_out = [chrom_psi_oi, chrom_psi_d, chrom_d_h, chrom_psi_io, chrom_interm_A1, chrom_interm_A2, chrom_V_x, chrom_d_v, chrom_del_h_t, ...
             chrom_d_v_mag, chrom_V_z, chrom_interm_B1, chrom_interm_C1, chrom_interm_C2, chrom_del_psi_ca, chrom_del_V_x_ca, ...
             chrom_del_V_z_ca, chrom_del_h_ca, chrom_W_h];

chrom_fis_in_out = mean(Field_DR_in_out,1);

chrom_fis_initial = [chrom_fis_in_out,chrom_rule];