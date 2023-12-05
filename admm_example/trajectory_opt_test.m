close all
clc
clear

methodSolver='admm';

load("test_field.mat");
% load("init_wp.mat");

problem.field_prop=field_prop;
problem.x_start=[0;0];
problem.x_end=[10;10];
problem.r_max=1;
problem.t_max=40;

x_initial = init_traj_gen(problem.x_start,problem.x_end,problem.t_max,problem.r_max);
n = length(x_initial);
switch methodSolver
    case 'alm'
        [x,output]=trajectory_opt(x_initial,problem,'debug','orderlagrangian','2','maxit',200);
    case 'admm'
        [x,output]=trajectory_opt_admm(x_initial,problem,'debug','maxit',200,'penalty',0.1);
    otherwise
        error('methodSolver not recognized')
end

%visualization of the field
xx=linspace(0,10,100);
[X,Y]=meshgrid(xx,xx);
f=gen_field_test(problem.field_prop);
XY=cat(3,X,Y);
surf(X,Y,funEvalVec(f,XY,'indexDimensionVector',3))
x_init_cost = zeros(1,n);
x_final_cost = zeros(1,n);
for i = 1:n
    x_init_cost(i) = f(x_initial(:,i));
end

hold on
scatter3(x(1,:),x(2,:),x_final_cost,50,'r','*');
scatter3(x_initial(1,:),x_initial(2,:),x_init_cost,50,'y','s');


figure(2)

%visualization of the solution
plotPoints(x,'rx')
hold on
plotPoints(x_initial,'bo')

figure(3)
plot(output.primal)
title('primal')

figure(4)
plot(output.gap)
title('gap')

figure(5)
plot(output.primal_residual)
hold on
plot(output.dual_residual)
title('primal and dual residual')
legend('primal', 'dual')

figure(6)
plot(output.penalty(output.penalty~=0))
title('penalty parameter')
