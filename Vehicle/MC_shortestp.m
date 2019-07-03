clc
clear all
close all
rng(403) 

model = load('model2.mat');
%load 'Data01.mat'
load 'D04.mat'
N = length(Segment);
model = model.model;
%Block 8359
f_normal(8359,1:5,:) =10 * ones(1,5,200);
% Block the road taht close to destination
f_normal(851,1:5,:) =10 * ones(1,5,200);
% Max steps to reach destination
total_step = 50;
ops = sdpsettings('solver','gurobi');

% random generate 50 different start position
[xx,emptyVRoad] = Box_map_variable(N,Longest_subsegment,total_step,Segment,Dest,Dest_seg,Start,Start_seg);
[road,b] = generate_start(N,Longest_subsegment,Segment,Dest,Dest_seg,Start,Start_seg);
Pair = [road,b];
msize = numel(road);
Set_ = road(randperm(msize, 100));

for ii=1:20
A = model.A;
A(1,82) = 0;
xx=[];
step=1;
time=1;
t = 0;
% set start position
matrix = reshape(emptyVRoad,9683*26,1);
index = sum(matrix(1:Set_(ii)*1));
A(1,index) = 1;
% initialize xop
xop = zeros(N,Longest_subsegment,100);
% initial Constraints
Constraint=[];
% Define the variables. 
[xx,emptyVRoad] = Box_map_variable(N,Longest_subsegment,total_step,Segment,Dest,Dest_seg,Start,Start_seg);
while sum(xop(Dest,Dest_seg,:)) == 0 
    % objective function
    obj=sum(sum(sum(distance(:,:,1:size(xx,3)).*xx)));
    % initialize variable and load constraints
    xxnew = reshape(xx,N*Longest_subsegment*total_step,1);
    idx = find(value(xxnew) ~= 0);
    xxtry = xxnew(idx);
    BB= A*xxtry;
    CC=model.rhs;
    [xequal,~]=find((model.sense == '=')==1);
    Constraint=[Constraint;BB(xequal)==CC(xequal)];
    [xless,~]=find((model.sense == '<')==1);
    Constraint=[Constraint;BB(xless)<=CC(xless)];
    Constraint=[Constraint, xx(8359,1,:) == 0];
    Constraint=[Constraint, xx(851,1,:) == 0];
    % add constraints that block 8359
    %Constraint=[Constraint, xx(8359,1:5,:) == zeros(1,5,total_step)];
    % optimized and store the route within window size
    optimize(Constraint,obj,ops);
    xop(:,:,step:step+total_step-1) = value(xx);
    [street,segments,time]=ind2sub(size(xop),find(xop==1));
    i = 1;
    t = sum(sum(f(:,:,i).*xop(:,:,i)));
    while i <= length(time)
        i = i + 1;
        tt = 1 + floor(t/10);
        if sum(sum(f_normal(:,:,tt).*xop(:,:,i))) > 1 % there are road blocked during the path
            
            [xx,emptyVRoad] = Box_map_variable(N,Longest_subsegment,total_step,Segment,Dest,Dest_seg,Start,Start_seg); 
            S = street(i-1);
            Sg = segments(i-1);
            index = sum(matrix(1:N*Sg+S-N));
            A = model.A;
            A(1,82) = 0;
            A(1,index) = 1;
            Constraint=[Constraint, xx(street(i),segments(i),:) == 0];
            xop(:,:,i:end) = 0;
            step = i-1;
            i = i - 1; % the new start point is i - 1
            break
        else    
            t = t + sum(sum(f(:,:,tt).*xop(:,:,i)));
        end
    end
end
t1 =  Double_check_time(xop,f);
mc_record(ii) = t1; 
end

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