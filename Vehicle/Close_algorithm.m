function [Dest_1,Dest_2,Dest_3] = Close_algorithm(x1,y1,x2,y2,x3,y3,x4,y4,M)

x = [x2,x3,x4];
y = [y2,y3,y4];

Long_1 = (x1(1)+x1(2))/2;
Lat_1 = (y1(1)+y1(2))/2;
De = [];
R=6371;
% find the closest destination first
while isempty(x) == 0
    Long = [];
    Lat = [];
    a = [];
    c = [];
    d = [];
    for ii=1:length(x)
        Long(ii) = x(ii);
        Lat(ii) = y(ii);
        a(ii) = (sin((Lat_1*pi/180-Lat(ii)*pi/180)./2)).^2+cos(Lat_1*pi./180).*cos(Lat(ii)*pi./180).*(sin((Long_1*pi/180-Long(ii)*pi/180)./2)).^2;
        c(ii) = 2.*atan2(sqrt(a(ii)),sqrt(1-a(ii)));
        d(ii) = R.*c(ii);
    end
    min_d = min(d);
    ind = find(d==min_d);
    De = [De ; M(num2str([x(ind),y(ind)],10))];
    Long_1 = x(ind);
    Lat_1 = y(ind);
    x = x(x~=x(ind));
    y = y(y~=y(ind));
end
    
Dest_1 = De(1); Dest_2=De(2); Dest_3 = De(3);