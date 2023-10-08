% Check is based on TA and RA defined by either Tau, DMOD, ZTHR for Small
% Fixed wing UAV flying at altitude less than 1000ft. The TCAS thresholds
% have been scaled down by 2/3 to map performance of Small UAV compared to
% large A/C it was prescribed for.

function [collision_check, coll_check_id_return] = coll_check(own_states,other_states,th)

n = size(other_states,1);   % Number of Intruding UAVs around
int_ids_temp = 1:n;

temp = num2cell(own_states,1);
[pn, pe, pd, ~, ~, ~, ~, ~] = deal(temp{:});

temp2 = num2cell(other_states,1);
[pni, pei, pdi, ~, ~, ~, ~, ~] = deal(temp2{:});

d_h = sqrt((pni - pn).^2 + (pei - pe).^2); % in nmi
d_v = distdim((pdi - pd),'nm','ft'); % in ft
% d_slant = sqrt(d_h.^2 + (pdi - pd).^2); % in nmi
d_v_mag = abs(d_v); % Absolute Value of d_v_mag (ft)

collision_check = d_h <= th.coll * th.DMOD & d_v_mag <= th.coll * th.ZTHR;

% temp = num2cell(own_states);
% [pn, pe, pd, ~, ~, ~, ~, ~] = deal(temp{:});
% collision_check = NaN(n,1);
% 
% for i = int_ids_temp
%     temp2 = num2cell(other_states(i,:));
%     [pni, pei, pdi, ~, ~, ~, ~, ~] = deal(temp2{:});
%     
%     d_h = sqrt((pni-pn)^2 + (pei - pe)^2); % in nmi
%     d_v = distdim((pdi - pd),'nm','ft'); % in ft
% %     d_slant = sqrt(d_h^2 + (pdi - pd)^2); % in nmi
%     d_v_mag = abs(d_v); % Absolute Value of d_v_mag (ft)
% 
%     if d_h <= th.coll * th.DMOD && d_v_mag <= th.coll * th.ZTHR
%         collision_check(i) = 1;
%     end
% end

coll_check_id_return = int_ids_temp(collision_check);

