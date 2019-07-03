function [Constraint]=Full_constrains(total_step,Dest_intercept,Segment,Intersection,xx,Start,Start_seg,Dest,Dest_seg,emptyVRoad,xop)
Number_intersection = length(Intersection);
N = length(Segment);
Constraint=[];
% Initialization constraint: unique start location of the vehicle
Constraint = [Constraint, xx(Start,Start_seg,1)==1];
Constraint = [Constraint, xx(Dest,Dest_seg,1)==0]; % The car cannot arrive dest in the first time step

% Find a route as long as the vehicle does not reach destination (big M)
% Use time-1 incase the car reach one place and dest at the same time
for time=2:total_step
    Constraint=[Constraint, sum(sum(xx(:,:,time)))<=1+20000*sum(xx(Dest,Dest_seg,1:time-1))];
    Constraint=[Constraint, sum(sum(xx(:,:,time)))>=1-20000*sum(xx(Dest,Dest_seg,1:time-1))];
end


% Flow conservation constraint: input = output for all intersections
% for ii=1:Number_intersection
%     road_in = Intersection{ii}.In;
%     road_out = Intersection{ii}.Out;
%     in_seg = [Segment{road_in}]; % find the corresponding subsement for each road
%     in_s = [in_seg.num_sub];
%     if nnz(diag(emptyVRoad(road_in,in_s))) + nnz(emptyVRoad(road_out,1))==length([road_in road_out]) % we only manage the road that within the range
%         for time=1:total_step-1
%             x_input(time)= sum(diag(xx(road_in,in_s,time)));
%             x_output(time) = sum(xx(road_out,1,time+1));
%         end
%         if ii == Dest_intercept % in the intersection that contains destination
%             Constraint=[Constraint,x_input == x_output + reshape(xx(Dest,Dest_seg,1:total_step-1),1,total_step-1)];
%         else
%             Constraint=[Constraint,x_input == x_output];
%         end
%     end
% end

for ii=1:Number_intersection
    road_in = Intersection{ii}.In;
    road_out = Intersection{ii}.Out;
    if isempty(road_in) == 0 && isempty(road_out) == 0
        in_seg = [Segment{(road_in)}]; % find the corresponding subsement for each road
        in_s = [in_seg.num_sub];
        if nnz(diag(emptyVRoad(road_in,in_s))) + nnz(emptyVRoad(road_out,1))==length([road_in road_out]) % we only manage the road that within the range
            for time=1:total_step-1
                x_input(time)= sum(diag(xx(road_in,in_s,time)));
                x_output(time) = sum(xx(road_out,1,time+1));
            end
            if ii == Dest_intercept % in the intersection that contains destination
                Constraint=[Constraint,x_input == x_output + reshape(xx(Dest,Dest_seg,1:total_step-1),1,total_step-1)];
            else
                Constraint=[Constraint,x_input == x_output];
            end
        end
    end
end

% Vehicle must reach the destination
Constraint = [Constraint, sum(xx(Dest,Dest_seg,:))==1];

% Make uturn everywhere
for ii=1:N
    for j=1:Segment{ii}.num_sub-1
        if ~(ii==Dest && j==Dest_seg)
            a = Segment{ii}.subs{j}.uturn(1);
            if a == 0
                if emptyVRoad(ii,j)>0 && emptyVRoad(ii,j+1)>0 % we only manage the road that within the range
                    Constraint = [Constraint, xx(ii,j,1:total_step-1)==xx(ii,j+1,2:total_step)];
                end
            else
                b =  Segment{ii}.subs{j}.uturn(2);
                if a==Dest && b==Dest_seg+1
                    if emptyVRoad(ii,j)>0 && emptyVRoad(ii,j+1)>0 && emptyVRoad(a,b)>0 % we only manage the road that within the range
                        Constraint = [Constraint, xx(ii,j,1:total_step-1)==xx(ii,j+1,2:total_step) + xx(a,b,2:total_step)];
                    end
                else
                    if emptyVRoad(ii,j)>0 && emptyVRoad(ii,j+1)>0 && emptyVRoad(a,b-1)>0 && emptyVRoad(a,b)>0
                        Constraint = [Constraint, xx(ii,j,1:total_step-1)+xx(a,b-1,1:total_step-1)==xx(ii,j+1,2:total_step) + xx(a,b,2:total_step)];
                    end
                end
            end
        end
    end
end