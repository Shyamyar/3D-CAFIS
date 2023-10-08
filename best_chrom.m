function best = best_chrom(best_fis)
c = 1; % count mf param
rc = 1; % count rule param
nfis = 6;
nin = 2;
nout = 1;
nmf_param = 4;
nparam = nfis * (nin + nout) * nmf_param;
nrule = nfis*9;
fis_param = NaN(1,nparam);
fis_rule = NaN(1,nrule);
for fis = 1:nfis
    for i = 1:2
        for mf = 1:3
            switch mf
                case 1
                    fis_param(c) = best_fis.FIS(1, fis).Inputs(1, i).MembershipFunctions(1, mf).Parameters(3); c = c+1;
                case 2
                    fis_param(c) = best_fis.FIS(1, fis).Inputs(1, i).MembershipFunctions(1, mf).Parameters(1); c = c+1;
                    fis_param(c) = best_fis.FIS(1, fis).Inputs(1, i).MembershipFunctions(1, mf).Parameters(3); c = c+1;
                case 3
                    fis_param(c) = best_fis.FIS(1, fis).Inputs(1, i).MembershipFunctions(1, mf).Parameters(2); c = c+1;
            end
        end
    end
end
for fis = 1:nfis
    for o = 1
        for mf = 1:3
            switch mf
                case 1
                    fis_param(c) = best_fis.FIS(1, fis).Outputs(1, o).MembershipFunctions(1, mf).Parameters(3); c = c+1;
                case 2
                    fis_param(c) = best_fis.FIS(1, fis).Outputs(1, o).MembershipFunctions(1, mf).Parameters(1); c = c+1;
                    fis_param(c) = best_fis.FIS(1, fis).Outputs(1, o).MembershipFunctions(1, mf).Parameters(3); c = c+1;
                case 3
                    fis_param(c) = best_fis.FIS(1, fis).Outputs(1, o).MembershipFunctions(1, mf).Parameters(2); c = c+1;
            end
        end
    end
end

for fis = 1:nfis
    for r = 1:9
        fis_rule(rc) = best_fis.FIS(1, fis).Rules(1,r).Consequent; rc = rc+1;
    end
end

best_mf_param = fis_param([1:16,21:40,45:48,53:60,65:72]);
best_rule_param = fis_rule;
best = [best_mf_param, best_rule_param];

