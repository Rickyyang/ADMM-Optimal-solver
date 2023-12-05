function constraintFixPoints_test
r=2*(rand(2,1)-0.5);
xDim=[2 10];
t=(1:10)*0.4;
x=[cos(t);sin(t)];
constraint = constraintFixPoints(r,xDim);

z=constraint.IntermediateVariableUpdateFcn(r,x);
plotPoints(x,'-')
hold on
plotPoints(r,'.')
plotPoints(z,'d')
plotPoints([r,z],':')
hold off
axis equal
