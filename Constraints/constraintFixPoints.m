function constraint = constraintFixPoints(fixPoint,n_agent)
nFixPoints = size(fixPoint,2);
constraint.intermediateVariableDim = [2,1]';  %dimension of z (before vectorization)
[projected_point_fcn, projected_point_fcn_grad] = fixPoint2TrajectoryProject(fixPoint(:,1),n_agent);
constraint.equalityConstrainDx =@(x) projected_point_fcn(x)-fixPoint(:,1);  % equality constrain D(x) = z;
constraint.equalityConstrainDer =projected_point_fcn_grad;
constraint.intermediateVariableUpdateFcn =@(z,x) zeros(size(z));
if nFixPoints >1
    for i = 2:nFixPoints
        constraint_i.intermediateVariableDim = [2,1]';  %dimension of z (before vectorization)
        [projected_point_fcn, projected_point_fcn_grad] = fixPoint2TrajectoryProject(fixPoint(:,i),n_agent);
        constraint_i.equalityConstrainDx =@(x) projected_point_fcn(x)-fixPoint(:,i);  % equality constrain D(x) = z;
        constraint_i.equalityConstrainDer =projected_point_fcn_grad;
        constraint_i.intermediateVariableUpdateFcn =@(z,x) zeros(size(z));
        constraint = combineConstraints(constraint,constraint_i);
    end
end
constraint.dov = @(z,x) -cnorm(reshape(z,2,[]));
constraint.type =@(x) 'eq';   % need to define the type of constraint 'eq' for equality constraint, 'ineq' for inequality constraint
end
