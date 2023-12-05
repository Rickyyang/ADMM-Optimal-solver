%linear constraints for A*x=b
function constraint = constraintLinearEq(A,b,xDim)
b = b/norm(A);
A = A/norm(A);
zDim = size(b);
constraint.intermediateVariableDim = zDim';  %dimension of z (after vectorization)
constraint.equalityConstrainDx =@(x) A*x;  % equality constrain A*x = z; z = b
constraint.equalityConstrainDer =@(x) A;
constraint.intermediateVariableUpdateFcn =@(z,x) b;
constraint.dov = @(z,x) z-b;  % degree of violation
constraint.type =@(x) 'eq';
end
