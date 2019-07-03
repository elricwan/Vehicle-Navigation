clc
clear all
close all
rng(403)

tic
[T,tt,Segment,Intersection,f,g,x,Start,Start_seg,Dest,Dest_seg,Dest_intercept,Longest_subsegment] = Logic_data;
toc
Number_intersection = length(Intersection);
N = length(Segment);
% initialize the start point
ops = sdpsettings('solver','gurobi');
xop(:,:,1) = zeros(N,Longest_subsegment);
xop(Start,Start_seg,1) = 1;
step=2;

Delta = zeros(N,Longest_subsegment);
for ii=1:N
    for j=1:Segment{ii}.num_sub
        Delta(ii,j) = 1;
    end
end
[iis,jjs] = find(Delta==1);

while xop(Dest,Dest_seg,step-1) == 0
    % Define the variables.
    x_step = sparse(iis,jjs,binvar(length(iis),1));
    xop_step = xop(:,:,step-1);
    obj = 0.5*sum(sum(f(:,:,step).*x_step)) + 5*sum(sum(g(:,:,step).*x_step));
    [Constraint]=Step_constrains(Dest_intercept,Segment,Intersection,x_step,xop_step,Dest,Dest_seg);
    optimize(Constraint,obj,ops);
    xop(:,:,step) = value(x_step);
    step=step+1;
    disp(step);
end
       

[street,segments,time]=ind2sub(size(xop),find(xop==1));

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

plot(Segment{Start}.subs{Start_seg}.x(1),Segment{Start}.subs{Start_seg}.y(1),'ks','MarkerSize',12,'LineWidth',4)
plot(Segment{Dest}.subs{Dest_seg}.x(1),Segment{Dest}.subs{Dest_seg}.y(1),'ko','MarkerSize',12,'LineWidth',4)