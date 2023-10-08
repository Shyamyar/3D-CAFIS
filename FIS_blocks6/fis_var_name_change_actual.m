function fis = fis_var_name_change_actual(fis)

%% Mathematical Expressions
var_name = ["$x$ (nm)"                  %1
            "$z$ (ft)"                  %2
            "$\Delta\psi_M$ (rad)"      %3
            "$\Delta\chi^d$ (rad)"      %4
            "$\Delta z^d$ (ft)"         %5
            "$\nu_x$ (kts)"             %6
            "$\nu_z$ (ft)"              %7
            "$\Delta\nu^{\rm ca}_x$ (kts)"  %8
            "$\Delta\nu^{\rm ca}_z$ (ftmin$^{-1}$)"   %9
            "$\Delta\chi^{\rm ca}$ (rad)"    %10
            "$\Delta h^{\rm ca}$ (ft)"      %11
            "$\Upsilon_x$"              %12
            "$\Upsilon_z$"];            %13

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
conh1 = [fis.FIS(1,1).Name + "/$x$ (nm)" fis.FIS(1,2).Name + "/$x$ (nm)"];
conhv1 = [fis.FIS(1,1).Name + "/$x$ (nm)" fis.FIS(1,3).Name + "/$x$ (nm)"];

conv1 = [fis.FIS(1,3).Name + "/$z$ (ft)" fis.FIS(1,4).Name + "/$z$ (ft)"];

conw1 = [fis.FIS(1,1).Name + "/$\Delta\chi^{\rm ca}$ (rad)" fis.FIS(1,5).Name + "/$\Delta\chi^{\rm ca}$ (rad)"];
conw2 = [fis.FIS(1,3).Name + "/$\Delta h^{\rm ca}$ (ft)" fis.FIS(1,5).Name + "/$\Delta h^{\rm ca}$ (ft)"];

fis = fistree([fis.FIS(1,1) fis.FIS(1,2) fis.FIS(1,3) fis.FIS(1,4) fis.FIS(1,5)],...
    [conh1;conhv1;conv1;conw1;conw2]);

%% More Outputs
fis.Outputs(end+1) = fis.FIS(1,1).Name + "/$\Delta\chi^{\rm ca}$ (rad)";
fis.Outputs(end+1) = fis.FIS(1,3).Name + "/$\Delta h^{\rm ca}$ (ft)";

% best_fis = fis;
% save('bestFIS\bestfis54_2','best_fis','best_chrom');
end