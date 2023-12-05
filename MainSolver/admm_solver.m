function [x,output] = admm_solver(problem,varargin)
% unzip problem info
x = problem.x_initial;
flagCustomSolver = problem.flagCustomSolver;
g = problem.costFcn;
dg = problem.costFcnDer;
if ~isempty(problem.constraint)
    constraint = problem.constraint;
    D = constraint.equalityConstrainDx;
    dD = constraint.equalityConstrainDer;
    intermediateVariableUpdateFcn = constraint.intermediateVariableUpdateFcn;
    z = D(x);
    u = zeros(size(z));
else
    z = problem.z_initial;
    u = zeros(size(z));
end
methodPenaltyUpdate = problem.penaltyUpdate;
% z = D(x);
% u = zeros(size(z));
% vectorizedZDim = sum(prod([constraint.intermediateVariableDim]));
p = problem.penalty*ones(size(z));
p_max = 100*p;
flagRunning=true; % algorithm termination flag
flagDebug = true;
maxIt = 100;
maxFminIt = 400;
Algorithm = 'trust-region';
ivarargin=1;
while ivarargin<=length(varargin)
    %%
    % $x^2+e^{\pi i}$
    switch lower(varargin{ivarargin})
        case 'maxfminit'                % maximum iteration for fmincon
            ivarargin=ivarargin+1;
            maxFminIt=varargin{ivarargin};
        case 'algorithm'
            ivarargin=ivarargin+1;
            Algorithm=varargin{ivarargin};
        case 'maxit'                    %change maximum iteration value
            ivarargin=ivarargin+1;
            maxIt=varargin{ivarargin};
        case 'debug'                    %debug mode
            flagDebug = true;
        otherwise
            disp(varargin{ivarargin})
            error('Argument not valid!')
    end
    ivarargin=ivarargin+1;
end
output.min_edge_index = [];
if flagDebug
    % initialize variable for all parameters
    output.primal=zeros(1,maxIt+1);
    output.xAll=zeros([size(x) maxIt+1]);
    output.zAll=zeros([size(z) maxIt+1]);
    output.gap =zeros(1,maxIt+1); % duality gap
    output.uAll=output.zAll;
    %     output.Lag=zeros(1,maxIt+1);
    output.primal_residual=zeros(1,maxIt+1);
    output.dual_residual=zeros(1,maxIt+1);
    output.penalty=zeros([length(p),maxIt+1]);
    output.uNorm = zeros(1,maxIt+1);
    % cost for initial trajectory
    %     output.Lag(1)=g(x);
    output.primal(1)=g(x);
    output.xAll(:,:,1)=x;
    output.primal_residual(1)=0;
    output.dual_residual(1)=0;
    output.penalty(:,1)=p;
    output.lag(:,1)=0;
    output.uNorm(1) = norm(u);
end

%% main algorithm iteration
for it=1:maxIt
    
    % stop the loop if result meet the requirement
    if ~flagRunning
        break
    end
    %% variable setup
    %store current values at the beginning of the iteration
    xPrev=x;
    uPrev=u;
    zPrev=z;
    %% update dual variables
    %
    %   xNew : argmin(f(x)+(p/2)*norm(D(x)-zPrev+uPrev)^2)
    %   zNew : project (D(xNew)+uPrev) on to r_max
    %   uNew : uPrev + D(xNew) - zNew
    if ~flagCustomSolver
        % set up primal update function with gredient included
        options = optimoptions('fminunc','Algorithm',Algorithm,'SpecifyObjectiveGradient',true,'FunctionTolerance',0,'MaxIterations',maxFminIt); % could try "trust-region" algorithm
        x_update_fcn = admm_primal_update(g,dg,z,u,p,D,dD);
        x=fminunc(x_update_fcn,x,options);
        z=intermediateVariableUpdateFcn(D(x)+uPrev,xPrev);
        u=uPrev+D(x)-z;
        L = g(x)+ (p/2)'*((D(x)-z+u).^2);
    else
        [x,z,u,L]=problem.customSolver(xPrev,zPrev,uPrev,p);
    end
    uNorm = norm(uPrev);
    %% Penalty Parameter update
    % primal residual
    r_k = D(x) - z;
    % dual residual
    s_k = -p(1)'*(z-zPrev);
    r_k_norm = r_k'*r_k;
    s_k_norm = s_k'*s_k;
    % penalty update
    switch methodPenaltyUpdate
        case 'constant'
            %nothing to do
        case 'singleAdaptive'
            Mu = 10;
            Tau_inc = 2;
            Tau_decr = 2;
            if r_k_norm > Mu*s_k_norm
                u = u.*p;
                if p<p_max
                    p = Tau_inc * p;
                end
                u = u./p;
            elseif s_k_norm > Mu*r_k_norm
                u = u.*p;
                p = p / Tau_decr;
                u = u./p;
            end
    end
    %     p = p+ 0.1;
    %     disp('Penalty parameter')
    %     disp(p);
    %% save internal value for debugging
    if flagDebug
        output.primal(it+1)=g(x);
        %         output.Lag(it+1)=L;
        output.xAll(:,:,it+1)=x;
        output.zAll(:,:,it+1)=z;
        output.uAll(:,:,it+1)=u;
        output.gap(it+1)=norm(D(x)-z)^2;
        output.primal_residual(it+1)=r_k_norm;
        output.dual_residual(it+1)=s_k_norm;
        output.penalty(:,it+1)=p;
        output.uNorm(it+1) = uNorm;
        output.lag(:,it+1)=L;
    end
    
    %% optimality condition
    if s_k_norm<1e-6 && r_k_norm<1e-7
        flagRunning=false;
    end   
end
% output.min_edge_index = min_edge_index;
if flagDebug
    output.it = it;
    output.primal=output.primal(1:it);
    %     output.Lag=output.Lag(1:(it+1));
    output.xAll=output.xAll(:,:,1:it);
    output.zAll=output.zAll(:,:,1:it);
    output.uAll=output.uAll(:,:,1:it);
    output.gap=output.gap(1:it);
    output.primal_residual=output.primal_residual(1:it);
    output.dual_residual=output.dual_residual(1:it);
    output.penalty=output.penalty(:,1:it);
    output.uNorm= output.uNorm(1:it);
end
end

