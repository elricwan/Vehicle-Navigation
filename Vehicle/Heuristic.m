clc
clear all
close all
rng(403)


load 'Data02.mat'
% random generate 50 different start position
[road,b] = generate_start(N,Longest_subsegment,Segment,Dest,Dest_seg,Start,Start_seg);
Pair = [road,b];
msize = numel(road);
% Note, we find out that Set_(50) = 1911 and this is dead end.
Set_ = road(randperm(msize, 50));
window_size = [600,450,300,180,150,120,100];


total_step = 200;

% Create the harsh map for the index of different nodes of segments.
Key_set = {};
for ii=1:length(Segment)
    for j=1:Segment{ii}.num_sub
        x = Segment{ii}.subs{j}.x;
        y = Segment{ii}.subs{j}.y;
        Key_set{end+1} = num2str([x(1),y(1)],10);
        Key_set{end+1} = num2str([x(2),y(2)],10);
    end
end
Key_set = unique(Key_set);
Value_set = [1:length(Key_set)];

M = containers.Map(Key_set,Value_set);

% Create the harsh map that find cooresponding edge for two node index
new_key = {};
new_value = {};
for ii=1:length(Segment)
    for j=1:Segment{ii}.num_sub
        x = Segment{ii}.subs{j}.x;
        y = Segment{ii}.subs{j}.y;
        d1 = M(num2str([x(1),y(1)],10));
        d2 = M(num2str([x(2),y(2)],10));
        new_key{end+1} = num2str([d1,d2]);
        new_value{end+1} = num2str([ii,j]);
    end
end
M_new = containers.Map(new_key,new_value);

% Create a single graph
left = [];
right = [];
W = [];
for ii=1:length(Segment)
    for j=1:Segment{ii}.num_sub
        x = Segment{ii}.subs{j}.x;
        y = Segment{ii}.subs{j}.y;
        left(end+1) = M(num2str([x(1),y(1)],10));
        right(end+1) = M(num2str([x(2),y(2)],10));
        W(end+1) = f(ii,j,1);
    end
end
% get the attributes of W for different time slots
W_flex = zeros(length(W),total_step);
for time=1:total_step
    idx = 1;
    for ii=1:length(Segment)
        for j=1:Segment{ii}.num_sub
            % Block a certain road, for example: 3110
            if ii==3110
                W_flex(idx,time) = 10000;
                idx = idx + 1;
            %Block a certain road, for example: 152
            elseif ii==152
                W_flex(idx,time) = 10000;
                idx = idx + 1;
%             elseif ii==2701 && j==1
%                 W_flex(idx,time) = 10000;
%                 idx = idx + 1;
%             elseif ii==3110 && j==1
%                 W_flex(idx,time) = 10000;
%                 idx = idx + 1;
            else
                W_flex(idx,time) = f(ii,j,time);
                idx = idx + 1;
            end
        end
    end
end

% initialize the start and end position
% find the shortest path from (194,1) to (3565,1)
% In our algorithm, we need to reach the start node and end node for the
% destination edge
% x1 = Segment{194}.subs{1}.x;
% y1 = Segment{194}.subs{1}.y;
% Start_1 = M(num2str([x1(1),y1(1)],10));
% Start_2 = M(num2str([x1(2),y1(2)],10));
% 
x2 = Segment{2076}.subs{1}.x;
y2 = Segment{2076}.subs{1}.y;
Dest_1 = M(num2str([x2(1),y2(1)],10));
Dest_2 = M(num2str([x2(2),y2(2)],10));

Niter = 29;
% Algorithm with WS iteration
for wstep=1:length(window_size)
ws = window_size(wstep);
for ii=1:Niter
x1 = Segment{Set_(ii+20)}.subs{1}.x;
y1 = Segment{Set_(ii+20)}.subs{1}.y;
Start_1 = M(num2str([x1(1),y1(1)],10));
Start_2 = M(num2str([x1(2),y1(2)],10));
path = [];
step=1;
time=1;
t = 0;
% the first segment must be taken
DG = sparse(left,right,W_flex(:,time));
[dist,node,pred] = graphshortestpath(DG,Start_1,Start_2,'Method','Bellman-Ford');
i=length(node);
path(1,step:step+i-1) = node(1,1:i);
t1 = H_check_time(path,left,right,W_flex);
step=step+i-1;
% new start posiiton
Start_1 = node(1,i);
time = 1 + floor(t1/10);
while isempty(intersect(path,Dest_1))==1 % not reach the destination
    DG = sparse(left,right,W_flex(:,time));
    [dist,node,pred] = graphshortestpath(DG,Start_1,Dest_1);
    i=length(node);
    while H_check_time(node(1,1:i),left,right,W_flex(:,time:end)) >= ws
        i = i-1;
        if i == 1
            break
        end
    end
    if i > 1
        path(1,step:step+i-1) = node(1,1:i);
        t1 = H_check_time(path,left,right,W_flex);
        step=step+i-1;
        % new start posiiton
        Start_1 = node(1,i);
    else
        path(1,step:step+1) = node(1,1:2);
        t1 = H_check_time(path,left,right,W_flex);
        step=step+1;
        % new start position
        Start_1 = node(1,2);
    end
    
    time = 1 + floor(t1/10);
    disp(step)
    disp(i)
end
path(step+1) = Dest_2;
tt = H_check_time(path,left,right,W_flex);
mc_record(ii) = tt; 
end
MC(wstep) = mean(mc_record);
end
% Transform nodes to segments
street = [];
segments = [];
for ii=1:length(path)-1
    key = num2str([path(ii),path(ii+1)]);
    a = str2num(M_new(key));
    street(end+1) = a(1);
    segments(end+1) = a(2);
end

% plot
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

plot(Segment{Start}.subs{Start_seg}.x(1),Segment{Start}.subs{Start_seg}.y(1),'ks','MarkerSize',12,'LineWidth',4)
plot(Segment{Dest}.subs{Dest_seg}.x(1),Segment{Dest}.subs{Dest_seg}.y(1),'ko','MarkerSize',12,'LineWidth',4)
    
    
