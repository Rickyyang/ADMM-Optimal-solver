function ellipse_standardRotation_test
figure()
resetRands()
t=linspace(0,pi,161);
x=[2*cos(t);sin(t)];
plotPoints(x,'r')
axis equal
x1=randn(2,1);
x2=randn(2,1);
hold on
plotLines(x1,x2)

u1=x1-x2;
u2=[1;0];
H = householderRotation(u1,u2);
% H=diag([-1,1])*householderRotation(u1,u2);
plotPoints(H'*x,'b') 
plotArrows(zeros(2,1),u2,'r')
plotArrows(zeros(2,1),H'*u2,'b')
