%function [T,Segment,Intersection,f,f_normal,g,distance,Start,Start_seg,Dest,Dest_seg,Dest_intercept,Longest_subsegment] = Logic_data(Dest,Dest_seg,Start,Start_seg,T)
clc
clear all
close all
rng(403)
tic
T = 200;

Dest = 158;
Dest_seg = 1;
Start = 26;
Start_seg = 1;
Segment_raw = loadjson('Segment.json');
Intersection_raw = loadjson('Intercept.json');

Intersection = struct2cell(Intersection_raw);
Segment = struct2cell(Segment_raw);

N = length(Segment); 
Number_intersection = length(Intersection);
lst_initial = 1; % initial longest_subsegment
for ii=1:N
    if Segment{ii}.num_sub > lst_initial
        lst_initial = Segment{ii}.num_sub;
    end
end
Longest_subsegment = lst_initial;

% Find out the desination belong to which intersection
if Dest_seg == Segment{Dest}.num_sub
    for ii=1:Number_intersection
        if isempty(Intersection{ii}.In) == 0
            if ismember(Dest,Intersection{ii}.In)
                Dest_intercept = ii;
            end
        end
    end
else
   Dest_intercept = 0;
end

% Every subsegments has its V,E,NL and some has expected Stop time.
% We have two fitness value, one for partial information, the other for
% full information.
f=100*zeros(N,Longest_subsegment,T);
distance = 100*zeros(N,Longest_subsegment,T);


% Assign speed
% to generate speed with time correlation, I use sin function and assume
% the circle is 20 time steps.
for ii=1:N
    v1(ii) = normrnd(0.67*Segment{ii}.maxspeed,3);
    v2(ii,1:Segment{ii}.num_sub) = v1(ii) + normrnd(0,2,[1,Segment{ii}.num_sub]);
    rr = randi(1200);
    for time = 1:T
        v3(ii,time,1:Segment{ii}.num_sub) = v2(ii,1:Segment{ii}.num_sub)*(0.4*sin(0.005*(time+rr))+0.8) ...
            + normrnd(0,2,[1,Segment{ii}.num_sub]);
        v3(ii,time,1:Segment{ii}.num_sub) = min(1.3*Segment{ii}.maxspeed,v3(ii,time,1:Segment{ii}.num_sub));
        v(ii,time,1:Segment{ii}.num_sub) = max(1,v3(ii,time,1:Segment{ii}.num_sub));
    end
end
% for ii=1:N
%     for time=1:T
%         % speed is the normal distribution between (0.67maxspeed,5)
%         v(ii,time,1:Segment{ii}.num_sub) = normrnd(0.67*Segment{ii}.maxspeed*(0.4*sin(0.005*time)+0.8),2,[1,Segment{ii}.num_sub]) ...
%             + normrnd(0,2,[1,Segment{ii}.num_sub]);
%         for j=1:Segment{ii}.num_sub
%             v(ii,time,j)=min(1.2*Segment{ii}.maxspeed,v(ii,time,j));
%             v(ii,time,j)=max(1,v(ii,time,j)); 
%         end
%     end
% end
% for ii=1:N
%     v1(ii) = normrnd(0.67*Segment{ii}.maxspeed,3);
%     v2(ii,1:Segment{ii}.num_sub) = v1(ii) + normrnd(0,2,[1,Segment{ii}.num_sub]);
%     for j = 1:Segment{ii}.num_sub
%         v3(ii,1:T,j) = v2(ii,j) + normrnd(0,2,[1,T]);
%         v3(ii,1:T,j) = min(1.2*Segment{ii}.maxspeed,v3(ii,1:T,j));
%         v(ii,1:T,j) = max(1,v3(ii,1:T,j));
%     end
% end
% speed reducing
% for ii=1:N
%     v1(ii) = normrnd(0.5*Segment{ii}.maxspeed,3);
%     v2(ii,1:Segment{ii}.num_sub) = v1(ii) + normrnd(0,2,[1,Segment{ii}.num_sub]);
%     for time = 1:T
%         if time <= 30
%             v3(ii,time,1:Segment{ii}.num_sub) = v2(ii,1:Segment{ii}.num_sub) + normrnd(0,2);
%             v3(ii,time,1:Segment{ii}.num_sub) = min(1.3*Segment{ii}.maxspeed,v3(ii,time,1:Segment{ii}.num_sub));
%             v(ii,time,1:Segment{ii}.num_sub) = max(1,v3(ii,time,1:Segment{ii}.num_sub));
%         elseif (30 < time) && (time < 60)
%             v3(ii,time,1:Segment{ii}.num_sub) = v2(ii,1:Segment{ii}.num_sub)/cos(6.283*(time-30)/180) + normrnd(0,2);
%             v3(ii,time,1:Segment{ii}.num_sub) = min(1.3*Segment{ii}.maxspeed,v3(ii,time,1:Segment{ii}.num_sub));
%             v(ii,time,1:Segment{ii}.num_sub) = max(1,v3(ii,time,1:Segment{ii}.num_sub));
%         else
%             v3(ii,time,1:Segment{ii}.num_sub) = v2(ii,1:Segment{ii}.num_sub)*2 + normrnd(0,2);
%             v3(ii,time,1:Segment{ii}.num_sub) = min(1.3*Segment{ii}.maxspeed,v3(ii,time,1:Segment{ii}.num_sub));
%             v(ii,time,1:Segment{ii}.num_sub) = max(1,v3(ii,time,1:Segment{ii}.num_sub));
%         end         
%     end
% end
 
% Assign speed,lanes,event,stop time.
% Calculate fitness value
% In this example, we assume it takes approximately the same time to pass
% every segments, and the fitness value is satisfaction of people
% for passing this segment. Our goal to compare full knowledge and partial
% knowledge results.

%Block Street 8993
%Segment{8993}.Event = zeros(1,500);
%Block Street 404
%Segment{404}.Event = ones(1,500);
for ii=1:N
    for j=1:Segment{ii}.num_sub
        for time=1:T
            % for the same road, the speed won't change a lot, normalize
            % it.
            %Segment{ii}.subs{j}.V(time) = v(ii,time,j)/Upperbound_v;
            Segment{ii}.subs{j}.V(time) = v(ii,time,j);
            % event 
            Segment{ii}.subs{j}.E(time) = Segment{ii}.Event(time);
            distance(ii,j,time) = Segment{ii}.subs{j}.distance;
            if j == Segment{ii}.num_sub
                f(ii,j,time) = (3600*Segment{ii}.subs{j}.distance)/(1610*Segment{ii}.subs{j}.V(time)) + Segment{ii}.probability * 10 + 1000 * Segment{ii}.Event(time);
            else
                f(ii,j,time) = (3600*Segment{ii}.subs{j}.distance)/(1610*Segment{ii}.subs{j}.V(time)) + 1000 * Segment{ii}.Event(time);
            end
        end
    end
end


Upperbound_f = max(f(:));
f_normal = f/Upperbound_f;

% Calculate the distance from the destination
g=100*zeros(N,Longest_subsegment,T);
Long_d = (Segment{Dest}.subs{Dest_seg}.x(1)+Segment{Dest}.subs{Dest_seg}.x(2))/2;
Lat_d = (Segment{Dest}.subs{Dest_seg}.y(1)+Segment{Dest}.subs{Dest_seg}.y(2))/2;
R=6371; %KM
for ii=1:N
    for j=1:Segment{ii}.num_sub
        Long1 = (Segment{ii}.subs{j}.x(1)+Segment{ii}.subs{j}.x(2))/2;
        Lat1 = (Segment{ii}.subs{j}.y(1)+Segment{ii}.subs{j}.y(2))/2;
        a=(sin((Lat1*pi/180-Lat_d*pi/180)./2)).^2+cos(Lat1*pi./180).*cos(Lat_d*pi./180).*(sin((Long1*pi/180-Long_d*pi/180)./2)).^2;
        c=2.*atan2(sqrt(a),sqrt(1-a));
        g(ii,j,1:T) = R.*c;
    end
end

Upperbound_g = max(g(:));
g = g/Upperbound_g;
save 'DTest.mat'
toc