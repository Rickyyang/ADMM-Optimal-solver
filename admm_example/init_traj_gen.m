% generating the initial trajectory based on the start point, end point and
% the maximum time required

function x_initial = init_traj_gen(Pstart,Pend,max_time,r_max)
% the direction and min speed
dir = Pend - Pstart;
v_min = dir./max_time;
if (norm(v_min)>r_max)
    disp('Destination too far away from starting point!')
    error('Argument not valid!');
end
% initial way point (dont include start and end point)
x_initial = zeros(2,max_time-1);
for i  = 1 : (max_time - 1)
    x_initial(:,i) = Pstart + i*v_min;
end
end