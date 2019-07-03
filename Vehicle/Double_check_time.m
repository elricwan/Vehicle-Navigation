function [time] = Double_check_time(xop,f)

time = sum(sum(sum(f(:,:,1).*xop(:,:,1))));
for i = 2:size(xop,3)
    tt = 1 + floor(time/10);
    time = time + sum(sum(sum(f(:,:,tt).*xop(:,:,i))));
    %time = time + sum(sum(sum(f(:,:,1).*xop(:,:,i))));
end
