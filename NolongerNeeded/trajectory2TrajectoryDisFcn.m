% return the distance function between agents
function [inspection_dis_fcn, inspection_dis_grad_fcn] = trajectory2TrajectoryDisFcn(inspection_constraints,traj_length)
    function proj_p = inspection_dis_fcn_gen(x)
        f = @(x) trajectory2TrajectoryDis(x,inspection_constraints,traj_length);
        [proj_p,grad] = f(x);
    end
    function grad = inspection_dis_grad_fcn_gen(x)
        f = @(x) trajectory2TrajectoryDis(x,inspection_constraints,traj_length);
        [proj_p,grad] = f(x);
    end
inspection_dis_fcn = @inspection_dis_fcn_gen;
inspection_dis_grad_fcn = @inspection_dis_grad_fcn_gen;
end

function [TrajDis,TrajDis_grad]=trajectory2TrajectoryDis(x,inspection_constraints,traj_length)
        inspection_pair_index = findInspectionVerticesPair(x,traj_length,inspection_constraints);
        % compute inspection pair distance
        inspection_pair_size = size(inspection_pair_index);
        inspection_pair_size = inspection_pair_size(2);
        TrajDis = zeros(2,inspection_pair_size);
        for i = 1:inspection_pair_size
            TrajDis(:,i) = (x(:,inspection_pair_index(1,i))-x(:,inspection_pair_index(2,i)));
        end
        TrajDis = vec(TrajDis);
        % compute grad of distance function
        inspect_pair_size = size(inspection_pair_index);
        TrajDis_grad = zeros(2*inspect_pair_size(2),2*length(x));
        for i = 1:inspect_pair_size(2)
            row = 2*i-1;
            column_1 = 2*inspection_pair_index(1,i)-1;
            column_2 = 2*inspection_pair_index(2,i)-1;
            TrajDis_grad(row,column_1) = 1;
            TrajDis_grad(row+1,column_1+1) = 1;
            TrajDis_grad(row,column_2) = -1;
            TrajDis_grad(row+1,column_2+1) = -1;
        end
end
        