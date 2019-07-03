clc
clear all
close all
rng(403) 

model = load('model2.mat');
%load 'Data01.mat'
load 'D05.mat'
N = length(Segment);
model = model.model;
%Block 8359
f_normal(8359,1:5,1:200) =10 * ones(1,5,200);
% Block the road taht close to destination
f_normal(851,1:5,1:200) =10 * ones(1,5,200);
% initialize window size (Monta Carlo with window size)
window_size = [30];
% Max steps to reach destination
total_step = 50;
ops = sdpsettings('solver','gurobi');

% random generate 50 different start position
[xx,emptyVRoad] = Box_map_variable(N,Longest_subsegment,total_step,Segment,Dest,Dest_seg,Start,Start_seg);
[road,b] = generate_start(N,Longest_subsegment,Segment,Dest,Dest_seg,Start,Start_seg);
Pair = [road,b];
msize = numel(road);
Set_ = road(randperm(msize, 50));
Niter = 1;
for wstep=1:length(window_size)
ws = window_size(wstep); 
for ii=1:Niter
A = model.A;
A(1,82) = 0;
xop_update=zeros(N,Longest_subsegment,total_step);
xx=[];
step=1;
time=1;
t = 0;
% set start position
matrix = reshape(emptyVRoad,9683*26,1);
index = sum(matrix(1:Set_(ii)*1));
A(1,index) = 1;
while sum(xop_update(Dest,Dest_seg,:)) == 0
    % initialize xop
    xop = zeros(N*Longest_subsegment*total_step,1);
    % Define the variables. 
    [xx,emptyVRoad] = Box_map_variable(N,Longest_subsegment,total_step,Segment,Dest,Dest_seg,Start,Start_seg); 
    % obtain last update metric
    ff = repmat(f_normal(:,:,time),1,1,total_step);
    % objective function
    obj = sum(sum(sum(ff.*xx)));
    % initialize variable and load constraints
    xxnew = reshape(xx,N*Longest_subsegment*total_step,1);
    idx = find(value(xxnew) ~= 0);
    xxtry = xxnew(idx);
    BB= A*xxtry;
    CC=model.rhs;
    [xequal,~]=find((model.sense == '=')==1);
    Constraint=[];
    Constraint=[Constraint;BB(xequal)==CC(xequal)];
    [xless,~]=find((model.sense == '<')==1);
    Constraint=[Constraint;BB(xless)<=CC(xless)];
    % optimized and store the route within window size
    optimize(Constraint,obj,ops);
    xop(idx) = value(xxtry);
    xop = reshape(xop,N,Longest_subsegment,total_step);
    [street,~,~]=ind2sub(size(xop),find(xop==1));
    i=length(street);
    while Double_check_time(xop(:,:,1:i),f(:,:,time:end)) >= ws
        i = i-1;
        if i == 1
            break
        end
    end
    if i > 1
    xop_update(:,:,(step:step+i-1)) = xop(:,:,1:i);
    t1 = Double_check_time(xop_update,f);
    t2 = Double_check_time(xop_update(:,:,1:step+i-2),f);
    step=step+i-1;
    % new start position is xop(:,:,i)
    matrix = reshape(emptyVRoad,9683*26,1);
    [S,Sg] = find(xop(:,:,i)==1);
    % at least forward one step
    else
    xop_update(:,:,(step:step+1)) = xop(:,:,1:2);
    t1 = Double_check_time(xop_update,f);
    t2 = Double_check_time(xop_update(:,:,1:step),f);
    step=step+1;
    % new start position is xop(:,:,2)
    matrix = reshape(emptyVRoad,9683*26,1);
    [S,Sg] = find(xop(:,:,2)==1);    
    end    
        
    if isempty(S) == 0
        index = sum(matrix(1:N*Sg+S-N));
        % Set it to constraints
        A = model.A;
        A(1,82) = 0;
        A(1,index) = 1;
    end
    time = 1 + floor(t2/10); % one time step is 10s.
    disp(step);
    disp(time);
    disp(i)
end

mc_record(ii) = t1; 
end
MC(wstep) = mean(mc_record);
end

[street,segments,~]=ind2sub(size(xop_update),find(xop_update==1));
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