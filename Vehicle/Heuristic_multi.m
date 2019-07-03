clc
clear all
close all
rng(403)


load 'Data02.mat'
%expand f
f(:,:,201:400) = f(:,:,1:200);
% random generate 50 different Dest position
% Note that we generate the intersection instead of segments
[inte] = generate_dest(Number_intersection,Segment,Intersection,Dest,Dest_seg,Start,Start_seg); 
msize = numel(inte);
Set_ = inte(randperm(msize, 50));
window_size = [1200];
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
            % Block a certain road, for example: 8359
            if ii==8359 
                W_flex(idx,time) = 10000;
                idx = idx + 1;
            %Block a certain road, for example: 851
            elseif ii==851
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
x1 = Segment{2702}.subs{1}.x;
y1 = Segment{2702}.subs{1}.y;
Start_1 = M(num2str([x1(1),y1(1)],10));
Start_2 = M(num2str([x1(2),y1(2)],10));
Start_3 = Start_1;

Niter = 1;

% Algorithm with WS iteration
for wstep=1:length(window_size)
ws = window_size(wstep);
for ii=1:Niter
% assume there are three destinations
x2 = Intersection{Set_(ii+3)}.x;
y2 = Intersection{Set_(ii+3)}.y;

x3 = Intersection{Set_(ii+4)}.x;
y3 = Intersection{Set_(ii+4)}.y;

x4 = Intersection{Set_(ii+5)}.x;
y4 = Intersection{Set_(ii+5)}.y;
% Our algorithm is go for the closest first, then the closest to the
% closest and then...
[Dest_1,Dest_2,Dest_3] = Close_algorithm(x1,y1,x2,y2,x3,y3,x4,y4,M);

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
Dest_now = Dest_1;
while isempty(intersect(path,Dest_1))+isempty(intersect(path,Dest_2))+isempty(intersect(path,Dest_3)) > 0 % not reach three destinations
    DG = sparse(left,right,W_flex(:,time));
    [dist,node,pred] = graphshortestpath(DG,Start_1,Dest_now,'Method','Bellman-Ford');
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
    if isempty(intersect(path,Dest_1)) == 0 && isempty(intersect(path,Dest_2)) == 1
        Dest_now = Dest_2;
    end
    if isempty(intersect(path,Dest_1)) == 0 && isempty(intersect(path,Dest_2)) == 0 && isempty(intersect(path,Dest_3)) == 1
        Dest_now = Dest_3;
    end
    time = 1 + floor(t1/10);
    disp(step)
    disp(i)
end
% now the vehicle need to return to the start position
while Start_1 ~= Start_3
DG = sparse(left,right,W_flex(:,time));
[dist,node,pred] = graphshortestpath(DG,Start_1,Start_3,'Method','Bellman-Ford');
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
end
path(step+1) = Start_2;
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
