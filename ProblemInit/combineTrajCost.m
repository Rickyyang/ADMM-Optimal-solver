%Generate function handle for multiple agent trajectory
%Could be applied for both traj_cost 
function [f,df] = combineTrajCost(g,dg,traj_length,n_agent)
    function cost = system_cost(x)
        sys_cost = 0;
        for i = 1:n_agent
            start_id = (i-1)*traj_length+1;
            end_id = i*traj_length;
            x_i = x(:,start_id:end_id);
            sys_cost = sys_cost + g{i}(x_i);
        end
        cost = sys_cost;
    end
    function cost = system_cost_der(x)
        sys_cost_der = [];
        for i = 1:n_agent
            start_id = (i-1)*traj_length+1;
            end_id = i*traj_length;
            x_i = x(:,start_id:end_id);
            sys_cost_der = [sys_cost_der, dg{i}(x_i)];
        end
        cost = sys_cost_der;
    end
df = @system_cost_der;
f = @system_cost;
end