%% calculation vector from the fixed point to the closest point on the trajectory 
%   output vectorized for later use
function [f,df] = traj2FixPointDistance(fix_point,n_vertics,projection_vertex_id,projection_percentage)
    function dis_fix_point = traj_diff_pass_fixed_loc_function(x)
        dis_fix_point = fix_point-(projection_percentage*x(:,projection_vertex_id)+(1-projection_percentage)*x(:,projection_vertex_id+1));
        dis_fix_point = vec(dis_fix_point);
    end
    function dif_fix_point_vec_der = traj_diff_pass_fixed_loc_function_der(~)
        dif_fix_point_vec_der = zeros(2,2*n_vertics);
        dif_fix_point_vec_der(1,2*projection_vertex_id-1)=-projection_percentage;
        dif_fix_point_vec_der(2,2*projection_vertex_id) = -projection_percentage;
        dif_fix_point_vec_der(1,2*projection_vertex_id+1)= projection_percentage-1;
        dif_fix_point_vec_der(2,2*projection_vertex_id+2)= projection_percentage-1;
    end
f=@traj_diff_pass_fixed_loc_function;
df=@traj_diff_pass_fixed_loc_function_der;
end