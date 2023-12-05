function [traj_with_inspection,debug]=trajectory_opt_multi_agent(traj_no_inspection,problem,inspection_pair,min_inspection_length,varargin)
%problem is a structual contain start&end point, maximum distance, maximum
%time and field properities
%generate initial trajectory based on the start, end point and maximum time

%% setting up other default parameters

maxIt=100;                      %max iteration
n_agent = length(problem);
p = 10;                          %step size
flagDebug=false;                %Debug mode (save solutions for all iterations)
flagRunning=true;               % algorithm termination flag
methodPenaltyUpdate = 'constant'; % penalty update type 
traj_length=problem(1).t_max-1;
r_max = problem(1).r_max;  %max distance between each wp
for i = 1:n_agent      
    xStart{i} = problem(i).x_start;
    xEnd{i} = problem(i).x_end;
    field_prop{i} = problem(i).field_prop;
    % change the problem to :
    % min f(x)+ g(z)
    % s.t. D(x)-z=0
    Di{i} = trajectory_diff(xStart{i},xEnd{i});
    dDi{i} = trajectory_diff_der();         % generate gredients for future use
%     z = Di(x);                   % intermediate vatiable
%     u = zeros(size(z));         % dual variable
    % cost function for trajectory and its gredients
    gi{i} = trajectory_cost(field_prop{i});
    dgi{i} = trajectory_cost_der(field_prop{i});
end
g = multi_agent_cost(gi,traj_length,n_agent);
dg = multi_agent_cost_der(dgi,traj_length,n_agent);

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


%x: way point update initialize 
x = traj_no_inspection;
min_edge_index = find_inspection_vertices_pair(x,traj_length,inspection_pair);
% generate distance calculation function
D = trajectory_diff_multi_agent(Di,min_edge_index,traj_length,n_agent);
dD = trajectory_diff_der_multi_agent(min_edge_index,traj_length,n_agent);
% set up dual and internal variable
z = D(x);                   % intermediate variable
u = zeros(size(z));         % dual variable
l_z = length(z);
n_insp_pair = size(min_edge_index);
n_insp_pair = n_insp_pair(2);
l_v = l_z-n_insp_pair;
p_d = ones(1,n_insp_pair);
p_single_agent = p;
penalty = p;
d = D(x);
for i = 1:n_insp_pair
    p_d(i)= penalty * (1+norm(d(:,l_v+i))/min_inspection_length);
end
p = [penalty*ones(1,l_v),p_d];


if flagDebug
    % initialize variable for all parameters
    output.primal=zeros(1,maxIt+1);
    output.xAll=zeros([size(x) maxIt+1]);
    output.zAll=zeros([size(z) maxIt+1]);
    output.gap =zeros(1,maxIt+1); % duality gap
    output.uAll=output.zAll;
    output.Lag=zeros(1,maxIt+1);
    output.primal_residual=zeros(1,maxIt+1);
    output.dual_residual=zeros(1,maxIt+1);
    output.penalty=zeros([length(p),maxIt+1]); 
    % cost for initial trajectory
    output.Lag(1)=g(x);
    output.primal(1)=g(x);
    output.xAll(:,:,1)=x;
    output.primal_residual(1)=0;
    output.dual_residual(1)=0;
    output.penalty(:,1)=p;
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
    % set up primal update function with gredient included
    options = optimoptions('fminunc','Algorithm','trust-region','SpecifyObjectiveGradient',true); % could try "trust-region" algorithm
    x_update_fcn = admm_primal_update_inspection(g,dg,z,u,p,D,dD);
    
    %% update dual variables
    % 
    %   xNew : argmin(f(x)+(p/2)*norm(D(x)-zPrev+uPrev)^2)
    %   zNew : project (D(xNew)+uPrev) on to r_max
    %   uNew : uPrev + D(xNew) - zNew
    x=fminunc(x_update_fcn,x,options);
    if flagDebug
        L = x_update_fcn(x);
    end
    
    z=admm_dual_update_multi_agent(z,r_max,min_inspection_length,min_edge_index);
    u=D(x)+uPrev-z;
%     plotPoints(x_initial,'bo');hold on;plotPoints(x,'rx');plotPoints(xPrev,'kx');hold off
   
    %% Penalty Parameter update
    % primal residual
    r_k = D(x) - z;
    % dual residual
    s_k = - penalty*(z-zPrev);
    
%     r_k_norm = trace(r_k'*r_k);
%     s_k_norm = trace(s_k'*s_k);
    Mu = 2;
    Tau_inc = 1.5;
    Tau_decr = 1.5;
    r_k_norm = sum(r_k.^2);
    s_k_norm = sum(s_k.^2);
%     p_inc_count = r_k_norm-Mu*s_k_norm;
%     p_dec_count = s_k_norm-Mu*r_k_norm;
%     p_inc_id = p_inc_count>0;
%     p_dec_id = p_dec_count>0;


    switch methodPenaltyUpdate
        case 'constant'
            %nothing to do
        case 'singleAdaptive'            
            % penalty update traj
            if sum(r_k_norm(1:l_v)) > Mu*sum(s_k_norm(1:l_v))
                penalty = Tau_inc * penalty;
            elseif sum(s_k_norm(1:l_v)) > Mu*sum(r_k_norm(1:l_v))
                penalty = penalty / Tau_decr;
            end
            % penalty update insp
            for i = 1:n_insp_pair
                if r_k_norm(l_v+i) > Mu*s_k_norm(l_v+i)
                    p_d(i) = Tau_inc * p_d(i);
                elseif s_k_norm(l_v+i) > Mu*r_k_norm(l_v+i)
                    p_d(i) = p_d(i) / Tau_decr;
                end
            end
            d = D(x);
           
            u = u.*p;
            p = [penalty*ones(1,l_v),p_d];
%             p(p_inc_id) = p(p_inc_id)* Tau_inc;
%             p(p_dec_id) = p(p_dec_id)/ Tau_decr;
            u = u./p;
    end
    %%
    %save internal value for debugging
    if flagDebug
        output.primal(it+1)=g(x);
        output.Lag(it+1)=L;
        output.xAll(:,:,it+1)=x;
        output.zAll(:,:,it+1)=z;
        output.uAll(:,:,it+1)=u;
        output.gap(it+1)=norm(D(x)-z)^2;
        output.primal_residual(it+1)=sum(r_k_norm);
        output.dual_residual(it+1)=sum(s_k_norm);
        output.penalty(:,it+1)=p;
    end
    
    %% optimality condition
    if sum(s_k_norm)<1e-3 && sum(r_k_norm)<0.01
        flagRunning=false;
    end
% 
    disp('Penalty parameter')
    disp(penalty);
    disp('traj residual')
    disp(sum(r_k_norm(1:l_v)));
    disp('inspection penalty')
    disp(p_d);
    disp('d inspection pair');
    disp(cnorm(d(:,l_v+1:l_v+3)));
    disp('r_k_norm');
    disp(sum(r_k_norm));
    disp('insp residual')
    disp(sum(r_k_norm(l_v+1:l_v+3)));
    disp('traj residual')
    disp(sum(r_k_norm(1:l_v)));
    disp('s_k_norm');
    disp(sum(s_k_norm));
    disp('r_k')
    disp(cnorm(r_k(:,l_v+1:l_v+3)));
    disp('iteration');
    disp(it);
%     scatter(x(1,1:39),x(2,1:39),50,'r','*');
%     hold on
%     scatter(x(1,40:78),x(2,40:78),50,'k','.');
%     scatter(xPrev(1,1:39),xPrev(2,1:39),50,'r','+');
%     scatter(xPrev(1,40:78),xPrev(2,40:78),50,'k','x');
%     hold off

end
output.min_edge_index= min_edge_index;
output.D = D;
traj_with_inspection = x;
if flagDebug
    output.it = it;
end
if nargout >1 
    debug = output;
end
% %check for free time 
% final_x=xStart;
% y = [xStart,x,xEnd];
% for i = 2:(nbWaypoints+2)
%     if norm(y(:,i-1)-y(:,i))>0.01
%         final_x=[final_x,y(:,i)];
%     end
% end
% %extre free time after finishing job
% Tfree = length(x)-length(final_x);
% 
% x = final_x;
Tfree = 0;
 %% additional function 


