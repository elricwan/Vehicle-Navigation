Segment_raw = loadjson('Segment.json');
Intersection_raw = loadjson('Intercept.json');

Intersection = struct2cell(Intersection_raw);
Segment = struct2cell(Segment_raw);
% Block street 8359. Compare two algorithmn
% go to segment 6308 then turn back.Go straight 7 step then turn back.
a = load('shortest_b2');
b = load('xop_1');
c = load('x1');

Dest = 3565;
Dest_seg = 1;
Start = 194;
Start_seg = 1;

xop1 = a.shortest_b;
xop2 = b.xop;
xop3 = c.xop_update;

[street1,segments1,~]=ind2sub(size(xop1),find(xop1==1));
Path1=[street1 segments1];
[street2,segments2,~]=ind2sub(size(xop2),find(xop2==1));
Path2=[street2 segments2];
[street3,segments3,~]=ind2sub(size(xop3),find(xop3==1));
Path2=[street3 segments3];

figure(1)
for ii=1:length(Segment)
plot(Segment{ii}.X,Segment{ii}.Y,'b')
hold on
end

for ii=1:7
     plot(Segment{street1(ii)}.subs{segments1(ii)}.x+0.0002,Segment{street1(ii)}.subs{segments1(ii)}.y-0.0002,'r','LineWidth',2);
     hold on
end

plot([Segment{street1(7)}.subs{segments1(7)}.x(2)+0.0002,Segment{street1(8)}.subs{segments1(8)}.x(1)],[Segment{street1(7)}.subs{segments1(7)}.y(2)-0.0002,
    Segment{street1(8)}.subs{segments1(8)}.y(1)],'r','LineWidth',2);
hold on

for ii=8:length(street1)
   
        plot(Segment{street1(ii)}.subs{segments1(ii)}.x,Segment{street1(ii)}.subs{segments1(ii)}.y,'r','LineWidth',2);
        hold on

    %str1 = num2str(street1(ii));
    %text((Segment{street1(ii)}.X(1)+Segment{street1(ii)}.X(2))/2,(Segment{street1(ii)}.Y(1)+Segment{street1(ii)}.Y(2))/2,str1);
end

for ii=1:length(street2)
   
        plot(Segment{street2(ii)}.subs{segments2(ii)}.x,Segment{street2(ii)}.subs{segments2(ii)}.y,'y','Linestyle','--','LineWidth',2);
        hold on
end

for ii=1:length(street3)
   
        plot(Segment{street3(ii)}.subs{segments3(ii)}.x,Segment{street3(ii)}.subs{segments3(ii)}.y,':b+','MarkerSize',4,'LineWidth',1.5);
        hold on
end

plot(Segment{Start}.subs{Start_seg}.x(1)+0.0002,Segment{Start}.subs{Start_seg}.y(1)-0.0002,'ks','MarkerSize',12,'LineWidth',4)
plot(Segment{Dest}.subs{Dest_seg}.x(1),Segment{Dest}.subs{Dest_seg}.y(1),'ko','MarkerSize',12,'LineWidth',4)

% No block, the same start and dest.
a = load('xop_5');
b = load('xop_2');
c = load('x2');
xop1 = a.xop;
xop2 = b.xop;
xop3 = c.xop_update;
[street1,segments1,~]=ind2sub(size(xop1),find(xop1==1));
Path1=[street1 segments1];
[street2,segments2,~]=ind2sub(size(xop2),find(xop2==1));
Path2=[street2 segments2];
[street3,segments3,~]=ind2sub(size(xop3),find(xop3==1));
Path2=[street3 segments3];
figure(2)
for ii=1:length(Segment)
plot(Segment{ii}.X,Segment{ii}.Y,'b')
hold on

%str1 = num2str(ii);
%text((Segment{ii}.X(1)+Segment{ii}.X(2))/2,(Segment{ii}.Y(1)+Segment{ii}.Y(2))/2,str1);
end
% 

for ii=1:length(street1)
   
        plot(Segment{street1(ii)}.subs{segments1(ii)}.x,Segment{street1(ii)}.subs{segments1(ii)}.y,'r','LineWidth',2);
        hold on

    %str1 = num2str(street(ii));
    %text((Segment{street(ii)}.X(1)+Segment{street(ii)}.X(2))/2,(Segment{street(ii)}.Y(1)+Segment{street(ii)}.Y(2))/2,str1);
end
for ii=1:length(street2)
   
        plot(Segment{street2(ii)}.subs{segments2(ii)}.x,Segment{street2(ii)}.subs{segments2(ii)}.y,'y','Linestyle','--','LineWidth',2);
        hold on

    %str1 = num2str(street(ii));
    %text((Segment{street(ii)}.X(1)+Segment{street(ii)}.X(2))/2,(Segment{street(ii)}.Y(1)+Segment{street(ii)}.Y(2))/2,str1);
end
for ii=1:length(street3)
   
        plot(Segment{street3(ii)}.subs{segments3(ii)}.x,Segment{street3(ii)}.subs{segments3(ii)}.y,':b+','MarkerSize',4,'LineWidth',1.5);
        hold on
end
plot(Segment{Start}.subs{Start_seg}.x(1),Segment{Start}.subs{Start_seg}.y(1),'ks','MarkerSize',12,'LineWidth',4)
plot(Segment{Dest}.subs{Dest_seg}.x(1),Segment{Dest}.subs{Dest_seg}.y(1),'ko','MarkerSize',12,'LineWidth',4)

% New start and Dest with Blocked 3110 and 851
a = load('shortest_b3');
b = load('xopb1_d2');
c = load('x4');
Dest = 2076;
Dest_seg = 1;
Start = 2702;
Start_seg = 1;

xop1 = a.xop;
xop2 = b.xop;
xop3 = c.xop_update;
[street1,segments1,~]=ind2sub(size(xop1),find(xop1==1));
Path1=[street1 segments1];
[street2,segments2,~]=ind2sub(size(xop2),find(xop2==1));
Path2=[street2 segments2];
[street3,segments3,~]=ind2sub(size(xop3),find(xop3==1));
Path2=[street3 segments3];
figure(3)
for ii=1:length(Segment)
plot(Segment{ii}.X,Segment{ii}.Y,'b')
hold on

%str1 = num2str(ii);
%text((Segment{ii}.X(1)+Segment{ii}.X(2))/2,(Segment{ii}.Y(1)+Segment{ii}.Y(2))/2,str1);
end
% 

for ii=1:length(street1)
   
        plot(Segment{street1(ii)}.subs{segments1(ii)}.x,Segment{street1(ii)}.subs{segments1(ii)}.y,'r','LineWidth',2);
        hold on
end
for ii=1:length(street2)
   
        plot(Segment{street2(ii)}.subs{segments2(ii)}.x,Segment{street2(ii)}.subs{segments2(ii)}.y,'y','Linestyle','--','LineWidth',2);
        hold on
end
for ii=1:length(street3)
   
        plot(Segment{street3(ii)}.subs{segments3(ii)}.x,Segment{street3(ii)}.subs{segments3(ii)}.y,':b+','MarkerSize',4,'LineWidth',1.5);
        hold on
end
plot(Segment{Start}.subs{Start_seg}.x(1),Segment{Start}.subs{Start_seg}.y(1),'ks','MarkerSize',12,'LineWidth',4)
plot(Segment{Dest}.subs{Dest_seg}.x(1),Segment{Dest}.subs{Dest_seg}.y(1),'ko','MarkerSize',12,'LineWidth',4)

% No block
a = load('xop_s2');
b = load('test01');
c = load('test02');
xop1 = a.xop;
xop2 = c.xop;
xop3 = b.xop_update;
[street1,segments1,~]=ind2sub(size(xop1),find(xop1==1));
Path1=[street1 segments1];
[street2,segments2,~]=ind2sub(size(xop2),find(xop2==1));
Path2=[street2 segments2];
[street3,segments3,~]=ind2sub(size(xop3),find(xop3==1));
Path2=[street3 segments3];
figure(4)
for ii=1:length(Segment)
plot(Segment{ii}.X,Segment{ii}.Y,'b')
hold on

%str1 = num2str(ii);
%text((Segment{ii}.X(1)+Segment{ii}.X(2))/2,(Segment{ii}.Y(1)+Segment{ii}.Y(2))/2,str1);
end
% 

for ii=1:length(street1)
   
        plot(Segment{street1(ii)}.subs{segments1(ii)}.x,Segment{street1(ii)}.subs{segments1(ii)}.y,'r','LineWidth',2);
        hold on
end
for ii=1:length(street2)
   
        plot(Segment{street2(ii)}.subs{segments2(ii)}.x,Segment{street2(ii)}.subs{segments2(ii)}.y,'y','Linestyle','--','LineWidth',2);
        hold on
end
for ii=1:length(street3)
   
        plot(Segment{street3(ii)}.subs{segments3(ii)}.x,Segment{street3(ii)}.subs{segments3(ii)}.y,':b+','MarkerSize',4,'LineWidth',1.5);
        hold on
end

plot(Segment{Start}.subs{Start_seg}.x(1),Segment{Start}.subs{Start_seg}.y(1),'ks','MarkerSize',12,'LineWidth',4)
plot(Segment{Dest}.subs{Dest_seg}.x(1),Segment{Dest}.subs{Dest_seg}.y(1),'ko','MarkerSize',12,'LineWidth',4)

% plot result 2
figure 5
ax1 = subplot(2,2,1); 
time = [600,450,300,150,140,120,100,90,60];
WSA = [496.396,497.3777,492.5331,486.2712,485.9996,482.4984,483.0207,488.0553,488.0861];
SPA = [527.543,527.543,527.543,527.543,527.543,527.543,527.543,527.543,527.543];
Advanced_SPA = [486.4749,486.4749,486.4749,486.4749,486.4749,486.4749,486.4749,486.4749,486.4749];
plot(time,WSA,'r',time,SPA,'b--o',time,Advanced_SPA,'g:*')
xlim([0 650])
ylim([480 540])
lgd = legend(ax1,{'WSA','SPA','Advanced SPA'});
lgd.FontSize = 6;
title('(a)')
ax2 = subplot(2,2,2);
time = [600,450,300,150,120,90];
WSA = [445.565,447.0974,448.442,447.4604,444.6961,445.8032];
SPA = [507.9609,507.9609,507.9609,507.9609,507.9609,507.9609];
Advanced_SPA = [464.6648,464.6648,464.6648,464.6648,464.6648,464.6648];
xlim([0 650])
ylim([440 520])
plot(time,WSA,'r',time,SPA,'b--o',time,Advanced_SPA,'g:*')
lgd = legend(ax2,{'WSA','SPA','Advanced SPA'});
lgd.FontSize = 6;
title('(b)')
ax3 = subplot(2,2,3);
time = [600,450,300,150,120,90,60];
WSA = [518.5905,518.6168,524.2356,521.6343,530.6919,532.3537,530.3917];
SPA = [618,618,618,618,618,618,618];
Advanced_SPA = [549.0488,549.0488,549.0488,549.0488,549.0488,549.0488,549.0488];
xlim([0 650])
ylim([500 650])
plot(time,WSA,'r',time,SPA,'b--o',time,Advanced_SPA,'g:*')
lgd = legend(ax3,{'WSA','SPA','Advanced SPA'});
lgd.FontSize = 6;
title('(c)')
ax4 = subplot(2,2,4);
time = [600,450,300,150,120,90,60];
WSA = [636.206,630.7933,611.9286,610.4339,620.5315,620.96,611.912];
SPA = [688.1601,688.1601,688.1601,688.1601,688.1601,688.1601,688.1601];
Advanced_SPA = [661.1263,661.1263,661.1263,661.1263,661.1263,661.1263,661.1263];
plot(time,WSA,'r',time,SPA,'b--o',time,Advanced_SPA,'g:*')
xlim([0 650])
ylim([600 700])
lgd = legend(ax4,{'WSA','SPA','Advanced SPA'});
lgd.FontSize = 6;
title('(d)')