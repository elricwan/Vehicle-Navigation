function [time] = H_check_time(node,left,right,W_flex)

DG = sparse(left,right,W_flex(:,1));
time = DG(node(1),node(2));
for i = 2:length(node)-1
    tt = 1 + floor(time/10);
    DG = sparse(left,right,W_flex(:,tt));
    time = time + DG(node(i),node(i+1));
end

