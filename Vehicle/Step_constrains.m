function [Constraint]=Step_constrains(Dest_intercept,Segment,Intersection,x_step,xop_step,Dest,Dest_seg)
Number_intersection = length(Intersection);
N = length(Segment);
Constraint=[];
% Initialization constraint: unique start location of the vehicle
Constraint=[Constraint, sum(sum(x_step(:,:)))<=1+20000*sum(xop_step(Dest,Dest_seg))];
Constraint=[Constraint, sum(sum(x_step(:,:)))>=1-20000*sum(xop_step(Dest,Dest_seg))];
% flow conservation
for ii=1:Number_intersection
    road_in = Intersection{ii}.In;
    road_out = Intersection{ii}.Out;
    in_seg = [Segment{road_in}]; % find the corresponding subsement for each road
    in_s = [in_seg.num_sub];
    x_input= sum(diag(xop_step(road_in,in_s)));
    x_output = sum(x_step(road_out,1));
    if ii == Dest_intercept % in the intersection that contains destination
        Constraint=[Constraint,x_input == x_output + xop_step(Dest,Dest_seg)];
    else
        Constraint=[Constraint,x_input == x_output];
    end
end
% Make U turn
for ii=1:N
    for j=1:Segment{ii}.num_sub-1
        if ~(ii==Dest && j==Dest_seg)
            a = Segment{ii}.subs{j}.uturn(1);
            if a == 0
                Constraint = [Constraint, xop_step(ii,j)==x_step(ii,j+1)];
            else
                b =  Segment{ii}.subs{j}.uturn(2);
                if a==Dest && b==Dest_seg+1
                    Constraint = [Constraint, xop_step(ii,j)==x_step(ii,j+1) + x_step(a,b)];
                else
                    Constraint = [Constraint, xop_step(ii,j)+xop_step(a,b-1)==x_step(ii,j+1) + x_step(a,b)];
                end
            end
        end
    end
end


%Constraint = [Constraint, x(Start,Start_seg,1)==1];
%Constraint = [Constraint, x(Dest,Dest_seg,1)==0]; % The car cannot arrive dest in the first time step

% Find a route as long as the vehicle does not reach destination (big M)
% Use time-1 incase the car reach one place and dest at the same time



% for time=2:T
%     Constraint=[Constraint, sum(sum(x(:,:,time)))<=1+20000*sum(x(Dest,Dest_seg,1:time-1))];
%     Constraint=[Constraint, sum(sum(x(:,:,time)))>=1-20000*sum(x(Dest,Dest_seg,1:time-1))];
% end


% Flow conservation constraint: input = output for all intersections
% for ii=1:Number_intersection
%     road_in = Intersection{ii}.In;
%     road_out = Intersection{ii}.Out;
%     in_seg = [Segment{road_in}]; % find the corresponding subsement for each road
%     in_s = [in_seg.num_sub];
%     for time=1:T-1
%         x_input(time)= sum(diag(x(road_in,in_s,time)));
%         x_output(time) = sum(x(road_out,1,time+1));
%     end
%     if ii == Dest_intercept % in the intersection that contains destination
%         Constraint=[Constraint,x_input == x_output + reshape(x(Dest,Dest_seg,1:T-1),1,T-1)];
%     else
%         Constraint=[Constraint,x_input == x_output];
%     end   
% end



% % Vehicle must reach the destination
% Constraint = [Constraint, sum(x(Dest,Dest_seg,:))==1];

% Make uturn everywhere
% for ii=1:N
%     for j=1:Segment{ii}.num_sub-1
%         if ~(ii==Dest && j==Dest_seg)
%             a = Segment{ii}.subs{j}.uturn(1);
%             if a == 0
%                 Constraint = [Constraint, x(ii,j,1:T-1)==x(ii,j+1,2:T)];
%             else
%                 b =  Segment{ii}.subs{j}.uturn(2);
%                 if a==Dest && b==Dest_seg+1
%                     Constraint = [Constraint, x(ii,j,1:T-1)==x(ii,j+1,2:T) + x(a,b,2:T)];
%                 else
%                     Constraint = [Constraint, x(ii,j,1:T-1)+x(a,b-1,1:T-1)==x(ii,j+1,2:T) + x(a,b,2:T)];
%                 end
%             end
%         end
%     end
% end