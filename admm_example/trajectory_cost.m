%output the cost for the whold trajectory
function f = trajectory_cost(field_prop)
    function cost = full_cost_traj(x)
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
f = @full_cost_traj;
end