%Constraint the agents' reachability at a given time period to be within or
%keep out of the safe zone. 
%zoneType = 'in' for keep in and 'out' for keep out
function constraint = constraintSafeZoneTimeSpecified(timePeriod,vMax,n,d,nAgent,zoneType,varargin)
% check for agents specified
ivarargin = 1;
agents = 1:nAgent;
while ivarargin <= length(varargin)
    switch lower(varargin{ivarargin})
        case 'agents_involved'
            ivarargin = ivarargin+1;
            agents = varargin{ivarargin};
        otherwise
            disp(varargin{:})
            error('Argument not valid!')
    end
    ivarargin = ivarargin+1;
end
% calculating the corners
nAlt = [n(:,end),n,n(:,1)];
dAlt = [d(end);d;d(1)];
corner = zeros(2,2);
region = cell(length(d),1);
for i = 2:length(d)+1
    corner(:,1)=nAlt(:,(i-1):i)'\dAlt((i-1):i);
    corner(:,2)=nAlt(:,i:(i+1))'\dAlt(i:(i+1));
    region{i-1}=corner;
end
Ellipse2Segments = constraintEllipse2HalfplaneSumDis(timePeriod,vMax,n,d,nAgent,'segment',region,'agents_involved',agents);
center2zone = constraintEllipseCenter2SafeZone(n,d,zoneType);
constraint = combineConstraints(Ellipse2Segments,center2zone);

%% tempory creating constraint for position of center with respect to the zone
function constraint = constraintEllipseCenter2SafeZone(n,d,zoneType)
F_in = cell(size(n,2),1);
dF_in = cell(size(n,2),1);
for iFcn = 1:size(n,2)
    f =@(x) center2planeProject(x,n(:,iFcn),d(iFcn));
    df =@(x) center2planeProjectDer(x,n(:,iFcn));
    if  strcmp(zoneType,'out')  % zoneType 1 : forbidded zone
        F_in{iFcn} = f;    %input structure to logic or combination function
        dF_in{iFcn} = df;
    elseif  strcmp(zoneType,'in')  % zoneType 1 : keep in zone
        F_in{iFcn} =@(x) -f(x);    %input structure to logic or combination function
        dF_in{iFcn} =@(x) -df(x);
    else
        disp(zoneType)
        error('Zone Type not Valid!')
    end
end
if  strcmp(zoneType,'out')  % zoneType 1 : forbidded zone
    [f_out,df_out] = combineDxFcnWithLogicOr(F_in,dF_in);
elseif  strcmp(zoneType,'in')  % zoneType 1 : keep in zone
    [f_out,df_out] = combineDxFcnWithLogicAnd(F_in,dF_in);
else
    disp(zoneType)
    error('Zone Type not Valid!')
end
constraint.intermediateVariableDim = [1,1]';  %dimension of z (after vectorization)
constraint.equalityConstrainDx =f_out;  % equality constrain D(x) = z;
constraint.equalityConstrainDer =df_out;
constraint.intermediateVariableUpdateFcn =@(z,x) max(0,z);
constraint.dov = @(z,x) z;  % degree of violation
constraint.type =@(x) 'ineq';

    function out = center2planeProject(x,n,d)
        x = reshape(x,2,[],nAgent);
        nAgentInvolved = length(agents);
        out = zeros(nAgentInvolved,1);
        for iagent = agents
            focus = x(:,timePeriod,iagent);
            center = sum(focus,2)/2;
            out(iagent) = n'*center-d;
        end
    end
    function out = center2planeProjectDer(x,n)
        x = reshape(x,2,[],nAgent);
        [w,l,~] = size(x);
        traj_size = w*l;
%         nAgentInvolved = length(agents);
        der = cell(1,nAgent);
        for iagent = 1:nAgent
            der_agent_i = zeros(1,traj_size);
            if any(agents==iagent)%if agent i is considered
                der_dis = vec(n.*ones(2,2)/2)';
                focus_loc = [2*timePeriod(1)-1,2*timePeriod(1),2*timePeriod(2)-1,2*timePeriod(2)];
                der_agent_i(focus_loc) = der_dis;
            end
            der{iagent} = der_agent_i;
        end
        out = blkdiag(der{:});
        out = out(agents,:);
    end
end

end
