%function [t,xop,street,segments] = Shortest_path_flexible(max_step,Dest,Dest_seg,Start,Start_seg,T)

%tic
%[~,Segment,Intersection,f,~,~,distance,Start,Start_seg,Dest,Dest_seg,Dest_intercept,Longest_subsegment] = Logic_data(Dest,Dest_seg,Start,Start_seg,T);
%toc

%Block Street
%Block 8359
% store_8359 = f_normal(8359,1:5,:);
% f_normal(8359,1:5,:) = ones(1,5,200); 
% 
% %Block 6147
% store_6147 = f_normal(6147,1:5,:);
% f_normal(6147,1:5,:) = ones(1,5,200);
tic
%clc
%clear all
%close all
%load 'Data01.mat'
%Block 3110
%f_normal(3110,1:6,:) = ones(1,6,200);
%f_normal(3110,1:6,:) = f(3110,1:6,:)/Upperbound_f;

%Block 24
%f_normal(24,1:6,:) = ones(1,6,200); 
%f_normal(6147,1:6,:) = f(6147,1:6,:)/Upperbound_f;
max_step = 80;
N = length(Segment);
%assume the vehicle could reach the destination within 50 time steps.
total_step = max_step;
[xx,emptyVRoad] = Box_map_variable(N,Longest_subsegment,total_step,Segment,Dest,Dest_seg,Start,Start_seg); 


xop= zeros(N,Longest_subsegment,total_step);
[Constraint]=Full_constrains(total_step,Dest_intercept,Segment,Intersection,xx,Start,Start_seg,Dest,Dest_seg,emptyVRoad,xop);
step = 1;
i = 1;
% calculate the time passing the start segments
t = sum(sum(f(:,:,i).*xop(:,:,i)));
while sum(xop(Dest,Dest_seg,:)) == 0 
        
    obj=sum(sum(sum(distance(:,:,1:size(xx,3)).*xx)));
    ops = sdpsettings('solver','gurobi');
    optimize(Constraint,obj,ops);
    xop(:,:,step:end)=value(xx);
    [street,segments,time]=ind2sub(size(xop),find(xop==1));
    if length(street) < 5
        break
    end
    while i <= length(time)
        i = i + 1;
        tt = 1 + floor(t/10);
        if sum(sum(f_normal(:,:,tt).*xop(:,:,i))) == 1 % there are road blocked during the path
        %if sum(sum(f(:,:,tt).*xop(:,:,i))) > 1000 % there are road blocked during the path
            total_step = max_step - i + 1;
            [xx,emptyVRoad] = Box_map_variable(N,Longest_subsegment,total_step,Segment,Dest,Dest_seg,Start,Start_seg); 
            Start = street(i-1);
            Start_seg = segments(i-1);
            [Constraint]=Full_constrains(total_step,Dest_intercept,Segment,Intersection,xx,Start,Start_seg,Dest,Dest_seg,emptyVRoad,xop);
             Constraint=[Constraint, xx(street(i),segments(i),:) == 0];
             xop(:,:,i:end) = 0;
             step = i;
             i = i - 1; % the new start point is i - 1
             break
        else    
            t = t + sum(sum(f(:,:,tt).*xop(:,:,i)));
        end
    end
end

[street,segments,~]=ind2sub(size(xop),find(xop==1));
Path=[street segments];

% plot the map and the route
for ii=1:length(Segment)
plot(Segment{ii}.X,Segment{ii}.Y,'b')
hold on

%str1 = num2str(ii);
%text((Segment{ii}.X(1)+Segment{ii}.X(2))/2+0.00005,(Segment{ii}.Y(1)+Segment{ii}.Y(2))/2-0.00005,str1);
end



for ii=1:length(street)
   
        plot(Segment{street(ii)}.subs{segments(ii)}.x,Segment{street(ii)}.subs{segments(ii)}.y,'r','LineWidth',2);
        hold on

    str1 = num2str(street(ii));
    text((Segment{street(ii)}.X(1)+Segment{street(ii)}.X(2))/2+0.00005,(Segment{street(ii)}.Y(1)+Segment{street(ii)}.Y(2))/2-0.00005,str1);
end
toc
[model,recoverymodel] = export(Constraint,obj,sdpsettings('solver','gurobi'));
[xx,emptyVRoad] = Box_map_variable(N,Longest_subsegment,total_step,Segment,Dest,Dest_seg,Start,Start_seg); 
xxnew = reshape(xx,9683*26*80,1);
idx = find(value(xxnew) ~= 0);
xxtry = xxnew(idx);
BB= model.A*xxtry;
CC=model.rhs;
[xequal,~]=find((model.sense == '=')==1);
constraintnew=[];
constraintnew=[constraintnew;BB(xequal)==CC(xequal)];
[xless,~]=find((model.sense == '<')==1);
constraintnew=[constraintnew;BB(xless)<=CC(xless)];
ops = sdpsettings('solver','gurobi');
dd = reshape(distance(:,:,1:80),9683*26*80,1);
dd = dd(idx);
objnew=sum(sum(dd.*xxtry));
tic
optimize(constraintnew,objnew,ops)
toc
xop = zeros(9683*26*80,1);
xop(idx) = value(xxtry);
xop = reshape(xop,9683,26,80);
% change start position
aaa = find(idx==9278);
model.A(1,aaa) = 1;
model.A(1,1216) = 0;