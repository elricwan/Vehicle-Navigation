clc
clear all
close all
load 'DTest.mat';
%assume the vehicle could reach the destination within 50 time steps.
max_step = 15;
% Dest = 2076;
% Dest_seg = 1;
% Start = 393;
% Start_seg = 1;


total_step = max_step;
[xx,emptyVRoad] = Box_map_variable(N,Longest_subsegment,total_step,Segment,Dest,Dest_seg,Start,Start_seg); 

tic
[Constraint]=Test_constraints(total_step,Segment,Intersection,xx,emptyVRoad);
toc
%save('Constraint.mat','Constraint')
%load('Constraint.mat')
% Add extra constraints
tic
Constraint = [Constraint, xx(Start,Start_seg,1)==1];
Constraint=[Constraint, sum(sum(xx(:,:,1)))==1];

for time=2:total_step
    Constraint=[Constraint, sum(sum(xx(:,:,time)))<=1+20000*sum(xx(Dest,Dest_seg,1:time-1))];
    Constraint=[Constraint, sum(sum(xx(:,:,time)))>=1-20000*sum(xx(Dest,Dest_seg,1:time-1))];
end

Constraint = [Constraint, sum(xx(Dest,Dest_seg,:))==1];
toc

xop(:,:,:) = zeros(N,Longest_subsegment,total_step);

obj=sum(sum(sum(distance(:,:,1:size(xx,3)).*xx)));
ops = sdpsettings('solver','gurobi');
optimize(Constraint,obj,ops);


xop=value(xx);
[street,segments,time]=ind2sub(size(xop),find(xop==1));

% calculate fitness value for the shortest value
i = 1;
t = sum(sum(f(:,:,i).*xop(:,:,i)));
while i <= length(time)
    i = i + 1;
    tt = floor(t/10);
    t = t + sum(sum(f(:,:,tt).*xop(:,:,i)));
end

% plot the map and the route
for ii=1:length(Segment)
plot(Segment{ii}.X,Segment{ii}.Y,'b')
hold on

% str1 = num2str(ii);
% text((Segment{ii}.X(1)+Segment{ii}.X(2))/2+0.00005,(Segment{ii}.Y(1)+Segment{ii}.Y(2))/2-0.00005,str1);
end



for ii=1:length(street)
   
        plot(Segment{street(ii)}.subs{segments(ii)}.x,Segment{street(ii)}.subs{segments(ii)}.y,'r','LineWidth',2);
        hold on

    str1 = num2str(street(ii));
    text((Segment{street(ii)}.X(1)+Segment{street(ii)}.X(2))/2+0.00005,(Segment{street(ii)}.Y(1)+Segment{street(ii)}.Y(2))/2-0.00005,str1);
end

plot(Segment{Start}.subs{Start_seg}.x(1),Segment{Start}.subs{Start_seg}.y(1),'ks','MarkerSize',12,'LineWidth',4)
plot(Segment{Dest}.subs{Dest_seg}.x(1),Segment{Dest}.subs{Dest_seg}.y(1),'ko','MarkerSize',12,'LineWidth',4)