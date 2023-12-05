f = [1,0;-1,0]';
a = 4;
n = [1;2];
d = 1;
center = (f(:,1)+f(:,2))/2;
vf12 = diff(f,[],2);
H=householderRotation(vf12,[1;0]);
n_new = diag([-1,1])*H*n; %n_new
d_new = n'*center+d; %d_new
c = norm(vf12)/2;
b = sqrt(a^2-c^2);
out = [a,b]*diag(n_new.^2)*[a;b]-d_new^2;
g = n_new(2)*2*a*b*sqrt(out)/([a,b]*diag(n_new.^2)*[a;b]);
% figure()
% ellipseDraw(f,a)
% hold on
% x_o = linspace(-4,4,50);
% y_o = n(1)/n(2).*x_o+d/n(2);
% plot(x_o,y_o,'r')
% hold on
% y_n = n_new(1)/n_new(2)*x_o+d_new/n(2);
% plot(x_o,y_n,'b')
% grid on
syms x y
eqns = [n'*[x;y]+d == 0 , (x-center(1))^2/a^2 + (y-center(2))^2/b^2 == 1  ];
S = solve(eqns,[x y]);
sol = [double(S.x),double(S.y)];
diff_x = abs(diff(double(S.x)));
diff_x
g

f = 1;
