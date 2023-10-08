% XOVDP.M        (CROSSOVer Double Point)
%
% This function performs double point crossover between pairs of
% individuals and returns the current generation after mating.
%
% Syntax:  NewChrom = xovdp(OldChrom, XOVR)
%
% Input parameters:
%    OldChrom  - Matrix containing the chromosomes of the old
%                population. Each line corresponds to one individual
%                (in any form, not necessarily real values).
%    XOVR      - Probability of recombination occurring between pairs
%                of individuals.
%
% Output parameter:
%    NewChrom  - Matrix containing the chromosomes of the population
%                after mating, ready to be mutated and/or evaluated,
%                in the same format as OldChrom.

%  Author:    Hartmut Pohlheim
%  History:   28.03.94     file created
%             22.01.03     tested under MATLAB v6 by Alex Shenfield

function NewChrom = xovdp(OldChrom, XOVR);

if nargin < 2, XOVR = NaN; end

NewChrom = OldChrom;
for i = 1:size(OldChrom,1)
    if rand() < XOVR
        dp = sort(randi(size(OldChrom,2),1,2));
        xid = randi(size(OldChrom,1),1);
        NewChrom(i,dp(1):dp(2)) = OldChrom(xid,dp(1):dp(2));
    end
end
% End of function