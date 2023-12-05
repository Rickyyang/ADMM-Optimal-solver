% zone constraint still need work
function constraint = constraintEllipse2HalfplaneSumDis(timePeriod,vMax,n,d,nAgent,varargin)
% check if it is the distance to a segment or not
ivarargin = 1;
seg_mode = false;
agents = 1:nAgent; % Agents involved in this time period
while ivarargin <= length(varargin)
    switch lower(varargin{ivarargin})
        case 'segment'                % check the need for derivative output
            ivarargin = ivarargin+1;
            range=varargin{ivarargin};
            seg_mode = true;
        case 'agents_involved'
            ivarargin = ivarargin+1;
            agents = varargin{ivarargin};
        otherwise
            disp(varargin{:})
            error('Argument not valid!')
    end
    ivarargin = ivarargin+1;
end
a = vMax*diff(timePeriod)/2;
constraint.intermediateVariableDim = [2*nAgent;size(n,2)];  %dimension of z (before vectorization)
constraint.equalityConstrainDx =@(x) focus2planeProject(x);  % equality constrain D(x) = z;
constraint.equalityConstrainDer =@(x) focus2planeProject_grad(x);
constraint.intermediateVariableUpdateFcn =@(z,x) zeros(size(z));  % if z<0 then line intersect with ellipse
constraint.dov = @(z,x) vecnorm(reshape(z,2,[]));
constraint.type =@(x) 'ineq';   % need to define the type of constraint 'eq' for equality constraint, 'ineq' for inequality constraint
    function out = focus2planeProject(x)
        x = reshape(x,2,[],nAgent);
        out = zeros(2*size(n,2),nAgent);
        for i = 1:nAgent
            focus = x(:,timePeriod,i);
            for j = 1:size(n,2)
                if seg_mode
                    sumDis2Focus = focus2halfplaneProjection(focus,a,n(:,j),d(j),'segment',range{j});
                else
                    sumDis2Focus = focus2halfplaneProjection(focus,a,n(:,j),d(j));%projection for x_avoid
                end
                out((2*j-1):2*j,i) = sumDis2Focus; %out = [n to agent1;n to agent 2;...]
            end
        end
        out = vec(out);
    end

    function out = focus2planeProject_grad(x)
        x = reshape(x,2,[],nAgent);
        [w,l,~] = size(x);
        traj_size = w*l;
        der = cell(1,nAgent);
        for i = 1:nAgent
            focus = x(:,timePeriod,i);
            der_agent_i = zeros(2*size(n,2),traj_size);
            if any(agents==i)%if agent i is considered
                for j = 1:size(n,2)
                    if seg_mode
                        der_dis = focus2halfplaneProjection(focus,a,n(:,j),d(j),'segment',range{j},'der');
                    else
                        der_dis = focus2halfplaneProjection(focus,a,n(:,j),d(j),'der');
                    end
                    focus_loc = [2*timePeriod(1)-1,2*timePeriod(1),2*timePeriod(2)-1,2*timePeriod(2)];
                    der_agent_i((2*j-1):2*j,focus_loc) = der_dis;
                end
            end
            der{i} = der_agent_i;
        end
        out = blkdiag(der{:});
    end
end
