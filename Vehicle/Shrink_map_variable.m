function [x_batch,emptyVRoad] = Shrink_map_variable(N,Longest_subsegment,batch_size,xop_batch,Segment,Dest,Dest_seg) 

% find veicle location
[a,b] = find(xop_batch==1);
% plot the circle with appropriate radius
distance = batch_size * 150/1000;
% calculate the distance from other road to this road
g=100*zeros(N,Longest_subsegment);

Long_d = (Segment{a}.subs{b}.x(1)+Segment{a}.subs{b}.x(2))/2;
Lat_d = (Segment{a}.subs{b}.y(1)+Segment{a}.subs{b}.y(2))/2;
R=6371; %KM

for ii=1:N
    for j=1:Segment{ii}.num_sub
        Long1 = (Segment{ii}.subs{j}.x(1)+Segment{ii}.subs{j}.x(2))/2;
        Lat1 = (Segment{ii}.subs{j}.y(1)+Segment{ii}.subs{j}.y(2))/2;
        a=(sin((Lat1*pi/180-Lat_d*pi/180)./2)).^2+cos(Lat1*pi./180).*cos(Lat_d*pi./180).*(sin((Long1*pi/180-Long_d*pi/180)./2)).^2;
        c=2.*atan2(sqrt(a),sqrt(1-a));
        g(ii,j) = R.*c;
        if g(ii,j) <= distance
            g(ii,j)=1;
        else
            g(ii,j)=0;
        end
    end
end
g(Dest,Dest_seg)=1; %So that we make destination varialbe can be controled in yalmip

emptyVRoad = g;
%initialize the variable
gM=repmat(g,1,1,batch_size);
[iis,jjs] = find(gM==1);

sre = sparse(iis,jjs,binvar(length(iis),1),N,Longest_subsegment*batch_size);

x_batch = reshape(sre,N,Longest_subsegment,batch_size);