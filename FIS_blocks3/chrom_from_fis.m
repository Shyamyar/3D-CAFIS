%% Creating chromosome using manual FIS
run fis_ranges.m
load("rulelist.mat")
chrom_del_chi_d = chrom_range(angle_pi);
chrom_del_psi_oi= chrom_range(angle_pi);
chrom_V_x = chrom_range(vel_kts_mag);
chrom_d_h = chrom_range(dist_nm_mag);
chrom_V_z = chrom_range(vel_ftm);
chrom_d_v = chrom_range(dist_ft);
chrom_del_h_d = chrom_range(dist_ft);
chrom_d_v_mag = chrom_range(dist_ft_mag);
chrom_del_chi_ca = chrom_range(del_angle_pi);
chrom_del_chi_ca_ = chrom_range(del_angle_pi);
chrom_del_V_ca_h = chrom_range(del_vel_kts);
chrom_del_V_ca_z = chrom_range(del_vel_ftm);
chrom_del_h_ca = chrom_range(del_dist_ft);
chrom_del_h_ca_ = chrom_range(del_dist_ft);
chrom_W_h = chrom_range(unit_mag);
chrom_W_v = chrom_range(unit_mag);
chrom_rule = [rulelist.FIS_A1(:,3);
              rulelist.FIS_A2(:,3);
              rulelist.FIS_A3(:,3);
              rulelist.FIS_A4(:,3);  
              rulelist.FIS_B1(:,3);
              rulelist.FIS_B2(:,3);
              rulelist.FIS_B3(:,3);
              rulelist.FIS_B4(:,3);]';

% [d_h, del_psi_oi, V_x, del_chi_ca, del_V_ca_h, del_chi_d,
% del_chi_ca_, d_v_mag, d_v, V_z, del_h_ca, del_V_ca_z, del_h_d,
% del_h_ca_, W_h, W_v] for Chromosome of GA

Field_DR_in_out = [chrom_d_h, chrom_del_psi_oi, chrom_V_x, chrom_del_chi_ca, chrom_del_V_ca_h, chrom_del_chi_d,...
                chrom_del_chi_ca_, chrom_d_v_mag, chrom_d_v, chrom_V_z, chrom_del_h_ca, chrom_del_V_ca_z, chrom_del_h_d,...
                chrom_del_h_ca_, chrom_W_h, chrom_W_v];
n_rules = size(chrom_rule,2);
Field_DR_rule = [ones(1,n_rules); 3*ones(1,n_rules)];
FieldDR = Field_DR_in_out; %[Field_DR_in_out, Field_DR_rule]; % from chrom FIS

% Mean Chromosome for inclusion in random chromosome lists
chrom_fis_in_out = mean(Field_DR_in_out,1);
chrom_fis_initial = [chrom_fis_in_out,chrom_rule];