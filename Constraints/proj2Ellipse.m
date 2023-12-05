% project the vector z
function z_p = proj2Ellipse(focus,z,a)
%rotation matrix
% center = (focus(:,1)+focus(:,2))/2;
vf12 = diff(focus,[],2);
c = norm(vf12)/2;
if c>a
    z_p = z;
    return
end
b = sqrt(a^2-c^2);
theta = -atan(vf12(2)/vf12(1));
R = [cos(theta), -sin(theta);
     sin(theta), cos(theta)];
v_n = R*z; % v transformed to a standard ellipise
v = abs(v_n);  % v flip into first quadrant
v_p = v;
if v(1)>0 && v(2)>0
    y = v./[a;b];
    g = sum(y.^2)-1;
    if g<0
        r0 = (a/b)^2;
        s = GetRoot(r0,y,g);
        v_p = [r0*v(1)/(s+r0);v(2)/(s+1)]; % projected vector v
    else
        v_p = v; 
    end
    v_p = v_p.*sign(v_n);
elseif v(2)==0 && v(1)~=0
    if a*v(1)<c^2
        v_p = [a^2*v(1)/c^2;b*sqrt(1-(a*v(1)/c^2)^2)];
    else
        v_p(1) = max(v(1),a);
    end
    v_p(1) = v_p(1)*sign(v_n(1));
elseif v(1)==0 
    v_p(2) = max(v(2),b);
    v_p = v_p.*sign(v_n);
end
% v_p = v_p.*sign(v_n);
z_p = R\v_p;
end

function s = GetRoot(r0,y,g)
n0 = r0*y(1);
s0 = y(2)-1;
s1 = (g>=0)*(sqrt(n0^2+y(2)^2)-1);
s = 0;
for i = 1:100
    s = (s0 + s1)/2;
    if s==s0 || s==s1
        break
    end
%     ratio0 = n0/(s+r0);
%     ratio1 = z(2)/(s+1);
    g = (n0/(s+r0))^2 + (y(2)/(s+1))^2-1;
    if g>0
        s0 = s;
    elseif g<0
        s1 = s;
    else 
        break
    end
end
end