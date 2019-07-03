clc
clear all
close all
rng(403)
tic

Dest = 375;
Dest_seg = 1;
%Dest_seg = randi(Segment{Dest}.num_sub); % So that the segment would exist
Start = 575;
Start_seg = 1;
%Start_seg =randi(Segment{Start}.num_sub);

T = 1000; % generate 100 timesteps data in cloud server, 1000 seconds total.


% Batch optimization
batch_size = 4;
[t,xop,street,segments] = Batch_optimize(batch_size,Dest,Dest_seg,Start,Start_seg,T);



% Shortest_path_algorithm
% max_step = 50;
% [t,xop,street,segments] = Shortest_path_flexible(max_step,Dest,Dest_seg,Start,Start_seg,T);
toc