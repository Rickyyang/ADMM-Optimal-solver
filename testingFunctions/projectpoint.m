a = 2;
b = 1;
c =  sqrt(a^2-b^2);
focus = [c,0;-c,0]';
ellipseDraw(focus,a);
hold on
grid on
n = [1;1];
n1 = n/norm(n);
G = diag([a^2;b^2]);
d1 = sqrt(n1'*G*n1);
p1 = d1*G*n1/(n1'*G*n1);
p2 = -p1;

n2 = [-n1(2);n1(1)];
d2 = n2'*p1;
d22 = n2'*p2;

x = linspace(-5,5,100);
y1 = (-n1(1)*x+d1)/n1(2);
y4 = (-n1(1)*x-d1)/n1(2);
y2 = (-n2(1)*x+d2)/n2(2);
y3 = (-n2(1)*x+d22)/n2(2);
line(x,y1,'Color','r');
plotPoints([p1,p2,[0;0]],'o-')
line(x,y2,'Color','r');
line(x,y3);
line(x,y4);
hold off

