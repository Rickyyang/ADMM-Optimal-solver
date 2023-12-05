function [f,df] = traj2TrajDistance(min_dis_wp_index)
%return the function& its derivative about distance between trajectory for
%each agent (output vectorized for derivative calculation)
    function t_diff = traj_distance(x)
        t_diff = [];
        inspection_pair_size = size(min_dis_wp_index);
        inspection_pair_size = inspection_pair_size(2);
        for i = 1:inspection_pair_size
            t_diff = [t_diff,(x(:,min_dis_wp_index(1,i))-x(:,min_dis_wp_index(2,i)))];
        end
        t_diff = vec(t_diff);
    end
    function der = traj_distance_der(x)
        inspect_pair_size = size(min_dis_wp_index);
        inspect_edge_der = zeros(2*inspect_pair_size(2),2*length(x));
        for i = 1:inspect_pair_size(2)
            row = 2*i-1;
            column_1 = 2*min_dis_wp_index(1,i)-1;
            column_2 = 2*min_dis_wp_index(2,i)-1;
            inspect_edge_der(row,column_1) = 1;
            inspect_edge_der(row+1,column_1+1) = 1;
            inspect_edge_der(row,column_2) = -1;
            inspect_edge_der(row+1,column_2+1) = -1;
        end
        der = inspect_edge_der;
    end
f = @traj_distance;
df = @traj_distance_der;
end