%function [t,xop,street,segments] = Batch_optimize(batch_size,Dest,Dest_seg,Start,Start_seg,T)

% tic
% [~,Segment,Intersection,f,f_normal,g,~,Start,Start_seg,Dest,Dest_seg,Dest_intercept,Longest_subsegment] = Logic_data(Dest,Dest_seg,Start,Start_seg,T);
% toc
clc
clear all
close all
load 'Data01.mat'
N = length(Segment);

%Block 8359
f_normal(8359,1:5,:) =10 * ones(1,5,200);
%Block 6147
f_normal(6147,1:5,:) =10 * ones(1,5,200);

% initialize how batch size: how long will the information remain the same
ws = 300; %averaging time for passing one segment
%batch_size = 13;

% initialize the start point
ops = sdpsettings('solver','gurobi');

w_V=[0.6,0.5,0.4,0.3,0.2,0.1];

for wstep=1:length(w_V)
batch_size = 15;
w = w_V(wstep);
xop=[];
xop(:,:,1) = zeros(N,Longest_subsegment);
xop(Start,Start_seg,1) = 1;
x_batch=[];
step=1;
time=1;
t = 0;
while sum(xop(Dest,Dest_seg,:)) == 0 
    % Define the variables. 
    xop_batch = xop(:,:,step);
    [x_batch,emptyVRoad] = Shrink_map_variable(N,Longest_subsegment,batch_size,xop_batch,Segment,Dest,Dest_seg); 
    %obj = 0.1*sum(sum(sum(f(:,:,(step:step+batch_size-1)).*x_batch))) + 0.9*sum(sum(g(:,:,step+batch_size-1).*x_batch(:,:,batch_size)));
    % Assume the fitness value will valid for batch_size time, we use
    % normalized value to calculate objective function
    ff = repmat(f_normal(:,:,time),1,1,batch_size);
    % Also, we need the real passing time to update the data.
    f_pass =  repmat(f(:,:,time),1,1,batch_size);
    %obj = w*sum(sum(sum(ff.*x_batch))) + (1-w)*sum(sum(g(:,:,step+batch_size-1).*x_batch(:,:,batch_size)));
    obj = w*sum(sum(sum(ff.*x_batch)))/batch_size + (1-w)*sum(sum(g(:,:,batch_size).*x_batch(:,:,batch_size))) ...
    -0.07*sum(x_batch(Dest,Dest_seg,:));
    [Constraint]=Batch_constrains(batch_size,Dest_intercept,Segment,Intersection,x_batch,Dest,Dest_seg,xop,step,emptyVRoad);
    optimize(Constraint,obj,ops);
    % Assume the data is valid for 10s * batch_size time, if above the
    % time, the data will update during the batch_size
    
    i=batch_size;
    while sum(sum(sum(f_pass(:,:,2:i).*value(x_batch(:,:,2:i))))) >= ws
        i = i-1;
    end
    
    %t = t + sum(sum(sum(f(:,:,2:i).*value(x_batch(:,:,2:i)))));
    xop(:,:,(step:step+i-1)) = value(x_batch(:,:,1:i));
    t = Double_check_time(xop,f);
    step=step+i-1;
    time = 1 + floor(t/10); % one time step is 10s.
    % Let's make batch_size dynamic
    %Away_dest = sum(sum(g(:,:,batch_size).*value(x_batch(:,:,batch_size))));
    %batch_size = min(batch_size,round(Away_dest/0.01)+6);
    disp(step);
    disp(time);
    disp(batch_size);
end

objective(wstep) = t; 
end


[street,segments,~]=ind2sub(size(xop),find(xop==1));
Path=[street segments];


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

plot(Segment{Start}.subs{Start_seg}.x(1),Segment{Start}.subs{Start_seg}.y(1),'ks','MarkerSize',12,'LineWidth',4)
plot(Segment{Dest}.subs{Dest_seg}.x(1),Segment{Dest}.subs{Dest_seg}.y(1),'ko','MarkerSize',12,'LineWidth',4)

