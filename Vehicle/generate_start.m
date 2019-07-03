function [road,b] = generate_start(N,Longest_subsegment,Segment,Dest,Dest_seg,Start,Start_seg) 


% calculate the distance from other road to this road
g=100*zeros(N,Longest_subsegment);

Long_d = (Segment{Dest}.subs{Dest_seg}.x(1)+Segment{Dest}.subs{Dest_seg}.x(2))/2;
Lat_d = (Segment{Dest}.subs{Dest_seg}.y(1)+Segment{Dest}.subs{Dest_seg}.y(2))/2;
Long_s = (Segment{Start}.subs{Start_seg}.x(1)+Segment{Start}.subs{Start_seg}.x(2))/2;
Lat_s = (Segment{Start}.subs{Start_seg}.y(1)+Segment{Start}.subs{Start_seg}.y(2))/2;
R=6371; %KM
a=(sin((Lat_s*pi/180-Lat_d*pi/180)./2)).^2+cos(Lat_s*pi./180).*cos(Lat_d*pi./180).*(sin((Long_s*pi/180-Long_d*pi/180)./2)).^2;
c=2.*atan2(sqrt(a),sqrt(1-a));
% find the distance between start point and end point
distance = R.*c;
% we draw a circle with the radius = 0.6*distance, and only considered the
% road within the circle

Long_c = (Long_d + Long_s)/2;
Lat_c = (Lat_d+Lat_s)/2;
for ii=1:N
    for j=1:Segment{ii}.num_sub
        Long1 = (Segment{ii}.subs{j}.x(1)+Segment{ii}.subs{j}.x(2))/2;
        Lat1 = (Segment{ii}.subs{j}.y(1)+Segment{ii}.subs{j}.y(2))/2;
        a=(sin((Lat1*pi/180-Lat_c*pi/180)./2)).^2+cos(Lat1*pi./180).*cos(Lat_c*pi./180).*(sin((Long1*pi/180-Long_c*pi/180)./2)).^2;
        c=2.*atan2(sqrt(a),sqrt(1-a));
        g(ii,j) = R.*c;
        if g(ii,j) <= 0.75*distance
            g(ii,j)=1;
        else
            g(ii,j)=0;
        end
    end
end
g(Dest,Dest_seg)=1; %So that we make destination variable can be controled in yalmip

emptyVRoad = g;
[road,b] = ind2sub(size(emptyVRoad),find(emptyVRoad==1));