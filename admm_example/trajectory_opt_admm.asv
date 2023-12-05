function [x,output]=trajectory_opt_admm(x_initial,problem,varargin)
%problem is a structual contain start&end point, maximum distance, maximum
%time and field properities
%generate initial trajectory based on the start, end point and maximum time

%% setting up other default parameters

maxIt=100;                      %max iteration
r_max = problem.r_max;          %max distance between each wp
xStart = problem.x_start;
xEnd = problem.x_end;
field_prop = problem.field_prop;
p = 1;                          %step size
flagDebug=false;                %Debug mode (save solutions for all iterations)
methodPenaltyUpdate = 'singleAdaptive'; % penalty update type 
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
        otherwise
            disp(varargin{ivarargin})
            error('Argument not valid!')
    end
    ivarargin=ivarargin+1;
end

%% variable setup
%x: way point update initialize 
x = x_initial;
% change the problem to :
% min f(x)+ g(z)
% s.t. D(x)-z=0
D = trajectory_diff(xStart,xEnd);
dD = trajectory_diff_der();         % generate gredients for future use
% cost function for trajectory and its gredients
g = trajectory_cost(field_prop);
dg = trajectory_cost_der(field_prop);
p = p*ones(1,length(D(x)));
admm_primal_update_fcn = @admm_primal_update;
admm_dual_update_fcn = @admm_dual_update;
%% admm function call
[x,output] = admm_solver(x_initial,g,dg,D,dD,p,r_max,maxIt,admm_primal_update_fcn,admm_dual_update_fcn,flagDebug,methodPenaltyUpdate);


end


