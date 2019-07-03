clc
clear all
close all
rng(403)

tic
[T,Segment,Intersection,f,f_normal,g,x,Start,Start_seg,Dest,Dest_seg,Dest_intercept,Longest_subsegment] = Logic_data;
toc

%assume the vehicle could reach the destination within 50 time steps.
total_step = 50;



tic
[Constraint]=Full_constrains(total_step,Dest_intercept,Segment,Intersection,x,Start,Start_seg,Dest,Dest_seg);
toc


obj=sum(sum(sum(g.*x)));
ops = sdpsettings('solver','gurobi');
optimize(Constraint,obj,ops);



xop=value(x);
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