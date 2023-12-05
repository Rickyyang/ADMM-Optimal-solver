% zone constraint still need work
function constraint = constraintEllipse2Halfplane(timePeriod,vMax,n,d,nAgent)
a = vMax*diff(timePeriod)/2;
constraint.intermediateVariableDim = [nAgent;1];  %dimension of z (before vectorization)
constraint.equalityConstrainDx =@(x) focus2planeProject(x,n,d);  % equality constrain D(x) = z;
constraint.equalityConstrainDer =@(x) focus2planeProject_grad(x,n,d);
constraint.intermediateVariableUpdateFcn =@(z,x) max(z,0);
constraint.dov = @(z,x) -min(z,0);
constraint.type =@(x) 'ineq';   % need to define the type of constraint 'eq' for equality constraint, 'ineq' for inequality constraint
    function out = focus2planeProject(x,n,d)
        x = reshape(x,2,[],nAgent);
        out = zeros(nAgent,1);
        for i = 1:nAgent
            focus = x(:,timePeriod,i);
            sumDis2Focus = focus2halfplaneProjection(focus,a,n,d);%projection for x_avoid
            out(i) = sumDis2Focus;
        end
    end
    function out = focus2planeProject_grad(x,n,d)
        x = reshape(x,2,[],nAgent);
        [w,l,~] = size(x);
        traj_size = w*l;
        der = cell(1,nAgent);
        for i = 1:nAgent
            focus = x(:,timePeriod,i);
            der_focus = focus2halfplaneProjection(focus,a,n,d,'der');
            der_agent_i = zeros(1,traj_size);
            focus_loc = [2*timePeriod(1)-1,2*timePeriod(1),2*timePeriod(2)-1,2*timePeriod(2)];
            der_agent_i(:,focus_loc) = der_focus;
            der{i} = der_agent_i;
        end
        out = blkdiag(der{:});
    end
end
