%% inequality constraint, required distance between sway points
function constraint = constraintInspection(inspection_constraints,traj_length,n_agent)
[D,dD] = inspectionVector(inspection_constraints,traj_length,n_agent);
inspection_vector_number = sum([inspection_constraints.duration]);
constraint.intermediateVariableDim = [2,inspection_vector_number]';  %dimension of z
constraint.equalityConstrainDx = D;  % equality constrain D(x) = z;
constraint.equalityConstrainDer = dD;
constraint.intermediateVariableUpdateFcn =@inspection_vector_undate;
constraint.dov = @(z,x) inspection_constraints(1).inspectDis-cnorm(reshape(z,2,[]));
constraint.type =@(x) 'ineq';

    function zOut = inspection_vector_undate(z,~)
        n_pair = size(inspection_constraints,2);
        start_id = 1;
        for i = 1:n_pair
            end_id = start_id+inspection_constraints(i).duration*2-1;
            z(start_id:end_id)=sephereProject(z(start_id:end_id),inspection_constraints(i).inspectDistance);
            start_id = end_id+1;
        end
        zOut = vec(z);
    end
end


