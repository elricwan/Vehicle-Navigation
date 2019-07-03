function [Constraint]=Batch_constrains(batch_size,Dest_intercept,Segment,Intersection,x_batch,Dest,Dest_seg,xop,step,emptyVRoad)

Number_intersection = length(Intersection);
N = length(Segment);
T = batch_size;
xop_batch = xop(:,:,step);
Constraint=[];
% Initialization constraint: unique start location of the vehicle
Constraint = [Constraint, x_batch(:,:,1)==xop_batch];
% Initialization constraint: unique start location of the vehicle
for time=2:T
    Constraint=[Constraint, sum(sum(x_batch(:,:,time)))<=1+20000*sum(x_batch(Dest,Dest_seg,1:time-1))];
    Constraint=[Constraint, sum(sum(x_batch(:,:,time)))>=1-20000*sum(x_batch(Dest,Dest_seg,1:time-1))];
end

% Flow conservation constraint: input = output for all intersections
for ii=1:Number_intersection
    road_in = Intersection{ii}.In;
    road_out = Intersection{ii}.Out;
    if isempty(road_in) == 0 && isempty(road_out) == 0
    in_seg = [Segment{(road_in)}]; % find the corresponding subsement for each road
    in_s = [in_seg.num_sub];
    if nnz(diag(emptyVRoad(road_in,in_s))) + nnz(emptyVRoad(road_out,1))==length([road_in road_out]) % we only manage the road that within the range
        for time=1:T-1
            x_input(time)= sum(diag(x_batch(road_in,in_s,time)));
            x_output(time) = sum(x_batch(road_out,1,time+1));
        end
        if ii == Dest_intercept % in the intersection that contains destination
            Constraint=[Constraint,x_input == x_output + reshape(x_batch(Dest,Dest_seg,1:T-1),1,T-1)];
        else
            Constraint=[Constraint,x_input == x_output];
        end
    end
    end
end

% Make uturn everywhere
for ii=1:N
    for j=1:Segment{ii}.num_sub-1
        if ~(ii==Dest && j==Dest_seg)
            a = Segment{ii}.subs{j}.uturn(1);
            if a == 0
                if emptyVRoad(ii,j)>0 && emptyVRoad(ii,j+1)>0 % we only manage the road that within the range
                    Constraint = [Constraint, x_batch(ii,j,1:T-1)==x_batch(ii,j+1,2:T)];
                end
            else
                b =  Segment{ii}.subs{j}.uturn(2);
                if a==Dest && b==Dest_seg+1
                    if emptyVRoad(ii,j)>0 && emptyVRoad(ii,j+1)>0 && emptyVRoad(a,b)>0 % we only manage the road that within the range
                        Constraint = [Constraint, x_batch(ii,j,1:T-1)==x_batch(ii,j+1,2:T) + x_batch(a,b,2:T)];
                    end
                else
                    if emptyVRoad(ii,j)>0 && emptyVRoad(ii,j+1)>0 && emptyVRoad(a,b-1)>0 && emptyVRoad(a,b)>0
                        Constraint = [Constraint, x_batch(ii,j,1:T-1)+x_batch(a,b-1,1:T-1)==x_batch(ii,j+1,2:T) + x_batch(a,b,2:T)];
                    end
                end
            end
        end
    end
end

% Cannot pass one road twice or three times
for ii=1:N
    for j=1:1:Segment{ii}.num_sub
        if emptyVRoad(ii,j) > 0 % we only manage the road that within the range
            Constraint = [Constraint, sum(x_batch(ii,j,2:end))+sum(xop(ii,j,:))<=2];
        end
    end
end
