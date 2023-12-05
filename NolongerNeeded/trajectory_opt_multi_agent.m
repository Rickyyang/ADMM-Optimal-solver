function [traj_with_inspection,debug]=trajectory_opt_multi_agent(traj_no_inspection,problem,inspection_pair,min_inspection_length,varargin)
%problem is a structual contain start&end point, maximum distance, maximum
%time and field properities
%generate initial trajectory based on the start, end point and maximum time

%% setting up other default parameters

maxIt=100;                      %max iteration
n_agent = length(problem);
p = 10;                          %step size
flagDebug=false;                %Debug mode (save solutions for all iterations)
methodPenaltyUpdate = 'constant'; % penalty update type 
traj_length=problem(1).t_max-1;
fix_points = [];
r_max = problem(1).r_max;  %max distance between each wp
for i = 1:n_agent      
    xStart{i} = problem(i).x_start;
    xEnd{i} = problem(i).x_end;
    field_prop{i} = problem(i).field_prop;
    % change the problem to :
    % min f(x)+ g(z)
    % s.t. D(x)-z=0
    [Di{i},dDi{i}] = trajectoryEdge(xStart{i},xEnd{i});
%     dDi{i} = trajectoryEdgeDer();         % generate gredients for future use
%     z = Di(x);                   % intermediate vatiable
%     u = zeros(size(z));         % dual variable
    % cost function for trajectory and its gredients
    gi{i} = trajectory_cost(field_prop{i});
    dgi{i} = trajectory_cost_der(field_prop{i});
end

[g,dg]= combineTrajCost(gi,dgi,traj_length,n_agent);

%% optional parameters
ivarargin=1;
while ivarargin<=length(varargin)
    switch lower(varargin{ivarargin})
        case 'maxit'                    %change maximum iteration value
            ivarargin=ivarargin+1;
            maxIt=varargin{ivarargin};
        case 'orderlagrangian'          %change order of lag
            error('order setting function is not allowed in this mode');
        case 'debug'                    %debug mode
            flagDebug=true;
        case 'penalty'
            ivarargin=ivarargin+1;
            p = varargin{ivarargin};
        case 'penalty update'
            ivarargin=ivarargin+1;
            methodPenaltyUpdate = varargin{ivarargin};
        case 'fix points'
            ivarargin=ivarargin+1;
            fix_points = varargin{ivarargin};
        otherwise
            disp(varargin{ivarargin})
            error('Argument not valid!')
    end
    ivarargin=ivarargin+1;
end


%x: way point update initialize 
x = traj_no_inspection;
% generate constrain update function
constrainUpdate = @(x) constrainFcnUpdate(Di,dDi,x,fix_points,r_max,traj_length,inspection_pair,min_inspection_length,n_agent);
[D,dD,r]=constrainUpdate(x);
admm_dual_update_fcn = @(z) sephereProject(z,r);
p = p*ones(1,size(D(x),2));

%% admm function call
[traj_with_inspection,debug] = admm_solver(x,g,dg,D,dD,p,maxIt,admm_dual_update_fcn,'debug',flagDebug,'constrain update',constrainUpdate,'penalty update',methodPenaltyUpdate);

end

