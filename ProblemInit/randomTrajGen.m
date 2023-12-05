% generating the initial trajectory based on the start point, end point and
% the maximum time required

function x_initial = randomTrajGen(Pstart,Pend,max_time,r_max)
% the direction and min speed
n_agent = size(Pstart,2);
x_initial = [];
for i = 1:n_agent
    x_prev = Pstart(:,i);
    dir = Pend(:,i) - Pstart(:,i);
    v_min = norm(dir./max_time);
    if (v_min>r_max)
        disp('Destination too far away from starting point!')
        error('Argument not valid!');
    end
    for j = 1:(max_time - 1)
        random_speed = randn(2,1);
        v = v_min*random_speed./norm(random_speed);
        x_next = x_prev+v;
        x_prev = x_next;
        x_initial = [x_initial,x_next];
    end
end
end