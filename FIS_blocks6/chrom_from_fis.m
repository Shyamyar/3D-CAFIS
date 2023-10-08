%% Creating chromosome using manual FIS
function FieldDR = chrom_from_fis(r)
load("rulelist.mat","rulelist")
chrom_del_chi_d = chrom_range(r.unit_range);
chrom_del_psi_oi= chrom_range(r.unit_range);
chrom_V_x = chrom_range(r.unit_mag);
chrom_x = chrom_range(r.unit_mag);
chrom_V_z = chrom_range(r.unit_range);
chrom_z = chrom_range(r.unit_range);
chrom_del_z_d = chrom_range(r.unit_range);
chrom_d_v_mag = chrom_range(r.unit_mag);
chrom_del_chi_ca = chrom_range(r.unit_range);
chrom_del_V_ca_x = chrom_range(r.unit_range);
chrom_del_V_ca_z = chrom_range(r.unit_range);
chrom_del_h_ca = chrom_range(r.unit_range);
chrom_W_h = chrom_range(r.unit_mag);
chrom_W_v = chrom_range(r.unit_mag);
chrom_rule = [rulelist.FIS_H1(:,3);
              rulelist.FIS_H2(:,3);
              rulelist.FIS_V1(:,3);
              rulelist.FIS_V2(:,3);
              rulelist.FIS_W(:,3)]';

% [d_h, del_psi_oi, del_chi_d, V_x, d_v, del_z_d, V_z, del_chi_ca, 
%  del_h_ca, del_V_ca_h, del_V_ca_z, W_h, W_v] 
% for Chromosome of GA

Field_DR_in_out = [chrom_x, chrom_del_psi_oi, chrom_del_chi_d, chrom_V_x,...
                    chrom_z, chrom_del_z_d, chrom_V_z, chrom_del_chi_ca,...
                    chrom_del_h_ca, chrom_del_V_ca_x, chrom_del_V_ca_z, ...
                    chrom_W_h, chrom_W_v];

n_rules = size(chrom_rule,2);
Field_DR_rule = [ones(1,n_rules); 3*ones(1,n_rules)];
FieldDR = Field_DR_in_out; %[Field_DR_in_out, Field_DR_rule]; % from chrom FIS

% Mean Chromosome for inclusion in random chromosome lists
chrom_fis_in_out = mean(Field_DR_in_out,1);
chrom_fis_initial = [chrom_fis_in_out,chrom_rule];