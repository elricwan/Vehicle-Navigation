function [inte] = generate_dest(Number_intersection,Segment,Intersection,Dest,Dest_seg,Start,Start_seg) 


% calculate the distance from other road to this road
g=100*zeros(Number_intersection);

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
for ii=1:Number_intersection
    Long1 = Intersection{ii}.x;
    Lat1 = Intersection{ii}.y;
    a=(sin((Lat1*pi/180-Lat_c*pi/180)./2)).^2+cos(Lat1*pi./180).*cos(Lat_c*pi./180).*(sin((Long1*pi/180-Long_c*pi/180)./2)).^2;
    c=2.*atan2(sqrt(a),sqrt(1-a));
    g(ii) = R.*c;
    if g(ii) <= 1*distance
        g(ii)=1;
    else
        g(ii)=0;
    end
end

emptyVRoad = g;
[inte] = find(emptyVRoad==1);