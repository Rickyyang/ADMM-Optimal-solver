function f = trajectory_diff_der_multi_agent(min_dis_wp_index)
    function der = traj_der(x)
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
f = @traj_der;
end