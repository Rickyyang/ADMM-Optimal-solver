% function plotDerivative(x)
% load('agentProperities.mat')
function plotDerivative(x,problem,output,varargin)
der_type = 'primal';
ivarargin=1;
while ivarargin<=length(varargin)
    switch lower(varargin{ivarargin})
        case 'der_type'
            ivarargin=ivarargin+1;
            der_type=varargin{ivarargin};
        otherwise
            disp(varargin{ivarargin})
            disp('Option not recognized')
    end
end

g = problem.costFcn;
dg = problem.costFcnDer;
constraint = problem.constraint;
D = constraint.equalityConstrainDx;
dD = constraint.equalityConstrainDer;
z = output.zAll(:,:,output.it);
u = output.uAll(:,:,output.it);
p = output.penalty(output.it);
admm_primal_fcn = admm_primal_update(g,dg,z,u,p,D,dD);
switch der_type
    case 'primal'
        [~,df] = admm_primal_fcn(x);
    case 'cost'
        df = dg(x);
end
fprintf('Norm of derivative: %d\n',norm(df))
df = reshape(df,2,[]);
u = df(1,:);
v = df(2,:);
x = reshape(x,2,[]);
quiver(x(1,:),x(2,:),u,v)
