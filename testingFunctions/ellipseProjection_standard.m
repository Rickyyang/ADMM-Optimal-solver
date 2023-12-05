%function P = ellipseProjection(focus,a,point)
%Return the closest point on a given ellipse (defined by focus and major radius 'a')
%with respect to a given point
function out = ellipseProjection_standard(focus,a,point,varargin)
% check if derivative is needed
der_mode = false;
if length(varargin) >=1
    switch lower(varargin{1})
        case 'der'                % check the need for derivative output
            der_mode=true;
        otherwise
            disp(varargin{ivarargin})
            error('Argument not valid!')
    end
end
% check if the point is outside
dis2focus = sum(vecnorm(focus-point))/2;
if dis2focus > a % if point outside ellipse
    out = point;
    if der_mode
        out = zeros(2,4);
    end
    return
end
% transform ellipse into a standard one
center = (focus(:,1)+focus(:,2))/2;
z = point - center;
vf12 = diff(focus,[],2);
c = norm(vf12)/2;
if c>a
    z_p = z;
    out = z_p+center;
    return
end
b = sqrt(a^2-c^2);
% theta = -atan(vf12(2)/vf12(1));
% R = [cos(theta), -sin(theta);   %rotation matrix
%      sin(theta), cos(theta)];
H=householderRotation(vf12,[1;0]);
v_n = diag([-1,1])*H*z; % v transformed to a standard ellipise
v = abs(v_n);  % v flip into first quadrant
v_p = v;
if v(1)>0 && v(2)>0
    y = v./[a;b];
    g = sum(y.^2)-1;
    if g~=0
        r0 = (a/b)^2;
        s = GetRoot(r0,y,g);
        v_p = [r0*v(1)/(s+r0);v(2)/(s+1)]; % projected vector v
        if der_mode
            t= s*b^2;
            dv_pdx = [Dp_0Dx(a,b,v,t,c,focus,H,z,v_n);Dp_1Dx(a,b,v,t,c,focus,H,z,v_n)];
            dv_pdx = dv_pdx.*sign(v_n);
        end
    else
        v_p = v;
        if der_mode
            dv_pdx = zeros(2,4);
        end
    end
    v_p = v_p.*sign(v_n);
elseif v(2)==0 && v(1)~=0
    if a*v(1)<c^2
        v_p = [a^2*v(1)/c^2;b*sqrt(1-(a*v(1)/c^2)^2)];
        if der_mode
            %need to change here
            dcdx = [(focus(1,1)-focus(1,2))/c,(focus(2,1)-focus(2,2))/c,(focus(1,2)-focus(1,1))/c,(focus(2,2)-focus(2,1))/c]/4;
            dv_pdc = [-2*a^2*v(1)/c^3; (2*a^4*v(1)^2/c^5-c-a^2*v(1)^2/c^3)/(b*sqrt(1-(a*v(1)/c^2)^2))];
            dv_pdx = dv_pdc*dcdx;
            dv_pdx(1) = dv_pdx(1,:)*sign(v_n(1));
        end
    else
        v_p(1) = a;
        if der_mode
            dv_pdx = zeros(2,4);
        end
    end
    v_p(1) = v_p(1)*sign(v_n(1));
elseif v(1)==0 
    v_p(2) = b;
    v_p = v_p.*sign(v_n);
    if der_mode
        dv_pdx = zeros(2,4);
    end
end
z_p = H*diag([-1,1])*v_p;
out = z_p+center;
% out = v_p;
if der_mode
    dHdx = DHDu(vf12,[1;0])*[-1,0,1,0;0,-1,0,1];
    dPdx = kron(v_p'*diag([-1,1]),eye(2))*dHdx+H*diag([-1,1])*dv_pdx + [1/2,0,1/2,0;0,1/2,0,1/2];
    out = dPdx;
%     out = dv_pdx;
end
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

function out = Dv_nDx(H,z,focus,v_n,i)
    %return the ith row of dvpdx
    vf12 = diff(focus,[],2);
    dHdx = DHDu(vf12,[1;0])*[-1,0,1,0;0,-1,0,1];
    dzdx = -[1/2,0,1/2,0;0,1/2,0,1/2];
    dv_pdx = sign(v_n).*(kron(z',diag([-1,1]))*dHdx+diag([-1,1])*H*dzdx);
    out = dv_pdx(i,:);
end

function dp_0dx = Dp_0Dx(a,b,v,t,c,focus,H,z,v_n)
    dv_p0dx = Dv_nDx(H,z,focus,v_n,1);
    dp_0dx = a^2*dv_p0dx/(t+a^2)-a^2*v(1)*Dtdx(a,b,v,t,c,focus,H,z,v_n)/(t+a^2)^2;
end

function dp_1dx = Dp_1Dx(a,b,v,t,c,focus,H,z,v_n)

    dv_p1dx = Dv_nDx(H,z,focus,v_n,2);
    dbdx = -c/b*[(focus(1,1)-focus(1,2))/c,(focus(2,1)-focus(2,2))/c,(focus(1,2)-focus(1,1))/c,(focus(2,2)-focus(2,1))/c]/4;
    dp_1dx = (2*b*dbdx*v(2)+b^2*dv_p1dx)/(t+b^2)-(b^2*v(2)/(t+b^2)^2)*(2*b*dbdx+Dtdx(a,b,v,t,c,focus,H,z,v_n));
end

function dtdx = Dtdx(a,b,v,t,c,focus,H,z,v_n)
    %get this through dGdx=0
    dv_p0dx = Dv_nDx(H,z,focus,v_n,1);
    dv_p1dx = Dv_nDx(H,z,focus,v_n,2);
    dbdx = -c/b*[(focus(1,1)-focus(1,2))/c,(focus(2,1)-focus(2,2))/c,(focus(1,2)-focus(1,1))/c,(focus(2,2)-focus(2,1))/c]/4;
    top = a^2*v(1)*dv_p0dx/(t+a^2)^2 + (b*v(2)^2*dbdx + b^2*v(2)*dv_p1dx)/(t+b^2)^2 - 2*b^3*v(2)^2*dbdx/(t+b^2)^3;
    buttom = a^2*v(1)^2/(t+a^2)^3+b^2*v(2)^2/(t+b^2)^3;
    dtdx = top/buttom;
end

% householder rotation derivative
function dHdu = DHDu(u1,u2)
n_u1 = u1/norm(u1);
n_u2 = u2/norm(u2);
v_l = n_u1+n_u2;
v = v_l/norm(v_l);
dn_u1du1 = (eye(2)-n_u1*n_u1')/norm(u1);
dvdu = (eye(2)-v*v')/norm(v_l);
dHdv = 2*[2*v(1),v(2),v(2),0;0,v(1),v(1),2*v(2)]';
dHdu = dHdv*dvdu*dn_u1du1;
end
