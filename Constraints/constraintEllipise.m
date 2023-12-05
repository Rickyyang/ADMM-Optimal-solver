function constraint = constraintEllipise(timePeriod,vMax,avoid_point,nAgent)
a = vMax*diff(timePeriod)/2;
constraint.intermediateVariableDim = [2;nAgent];  %dimension of z (before vectorization)
constraint.equalityConstrainDx =@(x) fixPoint2EllipiseProject(x);  % equality constrain D(x) = z;
constraint.equalityConstrainDer =@(x) fixPoint2EllipiseProject_grad(x);
constraint.intermediateVariableUpdateFcn =@(z,x) zeros(2,nAgent);
constraint.dov = @(z,x) fixPoint2EllipiseProject(x);
constraint.type =@(x) 'ineq';   % need to define the type of constraint 'eq' for equality constraint, 'ineq' for inequality constraint
    function out = fixPoint2EllipiseProject(x)
        x = reshape(x,2,[],nAgent);
        out = zeros(2,nAgent);
        for i = 1:nAgent
            focus = x(:,timePeriod,i);
            avoid_projection = ellipseProjection(focus,a,avoid_point);%projection for x_avoid
            out(:,i) = avoid_projection-avoid_point;
        end
        out = vec(out);
    end
    function out = fixPoint2EllipiseProject_grad(x)
        x = reshape(x,2,[],nAgent);
        [w,l,~] = size(x);
        traj_size = w*l;
        der = cell(1,nAgent);
        for i = 1:nAgent
            focus = x(:,timePeriod,i);
            der_focus = ellipseProjection(focus,a,avoid_point,'der');
            der_agent_i = zeros(2,traj_size);
            focus_loc = [2*timePeriod(1)-1,2*timePeriod(1),2*timePeriod(2)-1,2*timePeriod(2)];
            der_agent_i(:,focus_loc) = der_focus;
            der{i} = der_agent_i;
        end
        out = blkdiag(der{:});
    end
end
