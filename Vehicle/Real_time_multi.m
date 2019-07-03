clc
clear all
close all
rng(403)


load 'Data02.mat'
% Use 3 in Box_map
model = load('model_multi.mat');
N = length(Segment);
model = model.model;
ops = sdpsettings('solver','gurobi');
%expand f
f(:,:,201:400) = f(:,:,1:200);
f_normal(:,:,201:400) = f_normal(:,:,1:200);
% random generate 50 different Dest position
% Note that we generate the intersection instead of segments
[inte] = generate_dest(Number_intersection,Segment,Intersection,Dest,Dest_seg,Start,Start_seg); 
msize = numel(inte);
Set_ = inte(randperm(msize, 50));
window_size = [1200];
total_step = 200;

% assign the Destination, which represent the id of Intersection
id_1 = Set_(1+3);
id_2 = Set_(1+4);
id_3 = Set_(1+5);

% initialize xop
xop = zeros(N,Longest_subsegment,total_step);
%Define the variables, with r=*3 
[xx,emptyVRoad] = Box_map_variable(N,Longest_subsegment,total_step,Segment,Dest,Dest_seg,Start,Start_seg);
%obtain last update metric
ff = repmat(f_normal(:,:,time),1,1,total_step);
%objective function
obj = sum(sum(sum(ff.*xx)));
%load constraints
% initialize variable and load constraints
xxnew = reshape(xx,N*Longest_subsegment*total_step,1);
idx = find(value(xxnew) ~= 0);
xxtry = xxnew(idx);
A = model.A;
BB= A*xxtry;
CC=model.rhs;
[xequal,~]=find((model.sense == '=')==1);
Constraint=[];
Constraint=[Constraint;BB(xequal)==CC(xequal)];
[xless,~]=find((model.sense == '<')==1);
Constraint=[Constraint;BB(xless)<=CC(xless)];

% add extra constraint
% Start position initialization
tic
Constraint = [Constraint, xx(Start,Start_seg,1)==1];
%Constraint = [Constraint, sum(sum(xx(:,:,1)))==1];

% Vehicle must reach the destination
Constraint = [Constraint, sum(sum(xx(Intersection{id_1}.Out,1,:),3))>=1];
Constraint = [Constraint, sum(sum(xx(Intersection{id_2}.Out,1,:),3))>=1];
Constraint = [Constraint, sum(sum(xx(Intersection{id_3}.Out,1,:),3))>=1];

% Vehicle must back to the start position after reach all destinations
Constraint = [Constraint, sum(xx(Start,Start_seg,1:end))>=1];
for time=2:total_step
    Constraint = [Constraint, xx(Start,Start_seg,time)<=sum(sum(xx(Intersection{id_1}.Out,1,1:time),3))];
    Constraint = [Constraint, xx(Start,Start_seg,time)<=sum(sum(xx(Intersection{id_2}.Out,1,1:time),3))];
    Constraint = [Constraint, xx(Start,Start_seg,time)<=sum(sum(xx(Intersection{id_3}.Out,1,1:time),3))];
end
    
% Find a route as long as the vehicle does not reach destination (big M)
for time=2:total_step
    Constraint=[Constraint, sum(sum(xx(:,:,time)))<=1+20000*(sum(xx(Start,Start_seg,1:time-1))-1)];
    Constraint=[Constraint, sum(sum(xx(:,:,time)))>=1-20000*(sum(xx(Start,Start_seg,1:time-1))-1)];
end
toc

optimize(Constraint,obj,ops);
xop=value(xx);

[street,segments,~]=ind2sub(size(xop),find(xop==1));
t1 = Double_check_time(xop,f);

% plot the map and the route
for ii=1:length(Segment)
plot(Segment{ii}.X,Segment{ii}.Y,'b')
hold on

%str1 = num2str(ii);
%text((Segment{ii}.X(1)+Segment{ii}.X(2))/2,(Segment{ii}.Y(1)+Segment{ii}.Y(2))/2,str1);
end
% 
for ii=1:length(street)
   
        plot(Segment{street(ii)}.subs{segments(ii)}.x,Segment{street(ii)}.subs{segments(ii)}.y,'r','LineWidth',2);
        hold on

    str1 = num2str(street(ii));
    text((Segment{street(ii)}.X(1)+Segment{street(ii)}.X(2))/2,(Segment{street(ii)}.Y(1)+Segment{street(ii)}.Y(2))/2,str1);
end













