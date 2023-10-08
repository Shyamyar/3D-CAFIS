function params = two2four_param(vec,range)

el1 = range(1);
el2 = range(2);

mid_point = (el1+el2)/2;
diff = mid_point - vec;
other_params = flip(vec + 2 * diff);
params = [vec, other_params];
