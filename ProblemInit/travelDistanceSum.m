%output the cost for the whold trajectory
function [f,df] = travelDistanceSum(n_agent)
% need trajectory length for multi agent case

if n_agent == 1
    % single agent case
    df = @(x) diff([xStart,x,xEnd],[],2);
    f = @(x) full_cost_traj(x,field_prop);
else
    % multiple agent case
    for k = 1:n_agent
        field_prop_agent_i=field_prop([field_prop.agent_id]==k);
        df_single{k} = @(x) der_full_cost(x,field_prop_agent_i);
        f_single{k} = @(x) full_cost_traj(x,field_prop_agent_i);
    end
%     [f,df] = combineTrajCost(f_single,df_single,traj_length,n_agent);
    f = multiAgentFunctionIntegrate(f_single,traj_length,'add');
    df = multiAgentFunctionIntegrate(df_single,traj_length,'horizontal');
end
end

% trajectory cost function and its derivatie for single agent trajectory
function cost = full_cost_traj(x,field_prop)
n_gau = length(field_prop);
n_wp = length(x);
loc_value = 0;
for i = 1:n_gau
    prop = field_prop(i);
    for j = 1:n_wp
        loc_value = loc_value + prop.A/sqrt(det(prop.A))*exp(-(x(:,j)-prop.x)'/prop.Sigma*(x(:,j)-prop.x)/2);
    end
end
cost = -loc_value;
end
function der_cost = der_full_cost(x,field_prop)
n_gau = length(field_prop);
n_wp = length(x);
der_cost = zeros(size(x));
for i = 1:n_gau
    prop = field_prop(i);
    for j = 1:n_wp
        der_cost(:,j)=der_cost(:,j)-prop.A/sqrt(det(prop.A))*exp(-(x(:,j)-prop.x)'/(prop.Sigma)*(x(:,j)-prop.x)/2)*((prop.Sigma)\(x(:,j)-prop.x));
    end
end
der_cost = - der_cost;
end