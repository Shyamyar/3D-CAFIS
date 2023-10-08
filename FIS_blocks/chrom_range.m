function chrom_var = chrom_range(range)

el1 = range(1);
el2 = range(2);

chrom_var = [el1, el1, (el1+el2)/2, (el1+el2)/2;(el1+el2)/2, (el1+el2)/2, el2, el2];
