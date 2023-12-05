%linear constraints for A*x>=b
function constraint = constraintLinearInEq(A,b,xDim)
b = b/norm(A);
A = A/norm(A);
zDim = size(b);
constraint.intermediateVariableDim = zDim';  %dimension of z (after vectorization)
constraint.equalityConstrainDx =@(x) A*x;  % equality constrain A*x = z; z >= b
constraint.equalityConstrainDer =@(x) A;
constraint.intermediateVariableUpdateFcn =@(z,x) max(z,b);
constraint.dov = @(z,x) max(b-z,0);  % degree of violation
constraint.type =@(x) 'ineq';
end
