% generating the initial trajectory based on the start point, end point and
% the maximum time required

function x_initial = initTrajGen(Pstart,Pend,max_time,r_max)
% the direction and min speed
n_agent = size(Pstart,2);
x_initial = [];
for i = 1:n_agent
    dir = Pend(:,i) - Pstart(:,i);
    v_min = dir./max_time;
    if (norm(v_min)>r_max)
        disp('Destination too far away from starting point!')
        error('Argument not valid!');
    end
    % initial way point (dont include start and end point)
    x_initial_agent_i = zeros(2,max_time-1);
    for j  = 1 : (max_time - 1)
        x_initial_agent_i(:,j) = Pstart(:,i) + j*v_min;
    end
    x_initial = [x_initial,x_initial_agent_i];
end
end