% MUT.m
%
% This function takes the representation of the current population,
% mutates each element with given probability and returns the resulting
% population.
%
% Syntax:	NewChrom = mut(OldChrom,Pm,BaseV)
%
% Input parameters:
%
%		OldChrom - A matrix containing the chromosomes of the
%			   current population. Each row corresponds to
%			   an individuals string representation.
%
%		Pm	 - Mutation probability (scalar). Default value
%			   of Pm = 0.7/Lind, where Lind is the chromosome
%			   length is assumed if omitted.
%
% Output parameter:
%
%		NewChrom - A Matrix containing a mutated version of
%			   OldChrom.
%

function NewChrom = mutUNI(OldChrom,ranges, Pm)

% get population size (Nind) and chromosome length (Lind)
[Nind, Lind] = size(OldChrom) ;

% check input parameters
if nargin < 3, Pm = 0.7/Lind ; end
if isnan(Pm), Pm = 0.7/Lind; end
NewChrom = OldChrom;

for i = 1:Nind
    for j = 1:Lind
        if rand()<Pm
            NewChrom(i,j) = ranges(1,j) + rand()*(ranges(2,j)-ranges(1,j));
        end
    end
end