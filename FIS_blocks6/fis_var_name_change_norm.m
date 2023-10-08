function fis = fis_var_name_change_norm(fis)
%% Mathematical Expressions
var_name = ["$\hat{x}$"                  %1
            "$\hat{z}$"                  %2
            "$\Delta\hat{\psi}_M$"       %3
            "$\Delta\hat{\chi}^d$"       %4
            "$\Delta \hat{z}^d$"         %5
            "$\hat{\nu}_x$"              %6
            "$\hat{\nu}_z$"              %7
            "$\Delta\hat{\nu}^{\rm ca}_x$"  %8
            "$\Delta\hat{\nu}^{\rm ca}_z$"  %9
            "$\Delta\hat{\chi}^{\rm ca}$"    %10
            "$\Delta \hat{h}^{\rm ca}$"      %11
            "$\Upsilon_x$"         %12
            "$\Upsilon_z$"];       %13

k = [1
3
4
6
1
1
2
5
7
2
10
11
10
8
11
9
12
13];

c = 0;

if ~isempty(fis)
for i = 1:5
    n_in = length(fis.FIS(1,i).Inputs);
    for j = 1:n_in
        c = c + 1;
%         disp(fis.FIS(1,i).Name)
%         disp(fis.FIS(1,i).Inputs(j).Name)
%         k = input("Var number: ");
        fis.FIS(1,i).Inputs(j).Name = var_name(k(c));
    end
end

for i = 1:5
    n_out = length(fis.FIS(1,i).Outputs);
    for j = 1:n_out
        c = c + 1;
%         disp(fis.FIS(1,i).Name)
%         disp(fis.FIS(1,i).Outputs(j).Name)
%         k = input("Var number: ");
        fis.FIS(1,i).Outputs(j).Name = var_name(k(c));
    end
end

%% Creating new FIS
conh1 = [fis.FIS(1,1).Name + "/$\hat{x}$" fis.FIS(1,2).Name + "/$\hat{x}$"];
conhv1 = [fis.FIS(1,1).Name + "/$\hat{x}$" fis.FIS(1,3).Name + "/$\hat{x}$"];

conv1 = [fis.FIS(1,3).Name + "/$\hat{z}$" fis.FIS(1,4).Name + "/$\hat{z}$"];

conw1 = [fis.FIS(1,1).Name + "/$\Delta\hat{\chi}^{\rm ca}$" fis.FIS(1,5).Name ...
            + "/$\Delta\hat{\chi}^{\rm ca}$"];
conw2 = [fis.FIS(1,3).Name + "/$\Delta \hat{h}^{\rm ca}$" fis.FIS(1,5).Name ...
            + "/$\Delta \hat{h}^{\rm ca}$"];

fis = fistree([fis.FIS(1,1) fis.FIS(1,2) fis.FIS(1,3) fis.FIS(1,4) fis.FIS(1,5)],...
    [conh1;conhv1;conv1;conw1;conw2]);

%% More Outputs
fis.Outputs(end+1) = fis.FIS(1,1).Name + "/$\Delta\hat{\chi}^{\rm ca}$";
fis.Outputs(end+1) = fis.FIS(1,3).Name + "/$\Delta \hat{h}^{\rm ca}$";

% best_fis = fis;
% save('bestFIS\bestfis54_2','best_fis','best_chrom');
end