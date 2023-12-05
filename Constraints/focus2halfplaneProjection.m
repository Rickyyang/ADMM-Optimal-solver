% Return the -x_diff for the two intersection between the line and ellipse
% D(x) = -abs(int_x_1)-int_x_2) >=0 
function out = focus2halfplaneProjection(focus,a,n,d,varargin)
% check if derivative is needed
der_mode = false;
segment_mode = false;
ivarargin = 1;
while ivarargin<=length(varargin)
    switch lower(varargin{ivarargin})
        case 'der'                % check the need for derivative output
            der_mode=true;
        case 'segment'
            ivarargin=ivarargin+1;
            segment_mode = true;
            range = varargin{ivarargin};
        otherwise
            disp(varargin{ivarargin})
            error('Argument not valid!')
    end
    ivarargin=ivarargin+1;
end
n = n/norm(n);
d = d/norm(n);
% for normal case
% transfer n'x=d  standard ellipse coordinate
% get the point with the smallest sum of distance to two focus
% x_new = R*(x-center);   x = R^(-1)*x_new + c
% R = [-1 0;0,1]*H
center = (focus(:,1)+focus(:,2))/2;
if a == 0
    out = center;
    if der_mode
        out = [1/2,0,1/2,0;0,1/2,0,1/2];
    end
    return
end
vf12 = diff(focus,[],2);
% H1=householderRotation(vf12,[1;0]);
H=hhR(vf12);
if segment_mode
    range_new =  diag([-1,1])*H*(range-center);
end
n_new = diag([-1,1])*H*n; %n_new
d_new = -n'*center+d; %d_new
c = norm(vf12)/2;
if c>a
    out = 0;
    if der_mode
        out = zeros(2,4);
    end
    return
else
    b = sqrt(a^2-c^2);
end
G_inv = diag([a^2;b^2]);
if der_mode
    DbDf = -1*[(focus(1,1)-focus(1,2)),(focus(2,1)-focus(2,2)),(focus(1,2)-focus(1,1)),(focus(2,2)-focus(2,1))]/(4*b);
    DGDf = [zeros(3,4);2*b*DbDf];
    Dn_newDf = [-n',[0,0];[0,0],n']*DHDx(vf12,[1;0]);
    Dd_newdf = - n'*[1/2,0,1/2,0;0,1/2,0,1/2];
end
if d_new>=0
    d_cut = sqrt(n_new'*G_inv*n_new);
else
    d_cut = -sqrt(n_new'*G_inv*n_new);
end
p_cut = d_cut*G_inv*n_new/(n_new'*G_inv*n_new);
% n2 = [-n_new(2);n_new(1)];
% d2 = n2'*p_cut;
relative_position = abs(d_cut)-abs(d_new);

if relative_position>=0 
    % When the line intersect with the ellipse
    p_on_line = d_new*G_inv*n_new/(n_new'*G_inv*n_new);
    if segment_mode
        % check if projection on the line is in range
        if (range_new(:,1)-range_new(:,2))'*(p_on_line-range_new(:,2))>=0 && (range_new(:,2)-range_new(:,1))'*(p_on_line-range_new(:,1))>=0
            % if projection on line is within range remains the same
            proj2Ellipse = p_cut-p_on_line;
            out = H*diag([-1,1])*proj2Ellipse + center;
            if der_mode
                Dp_on_lineDf = kron(Dd_newdf,G_inv*n_new/(n_new'*G_inv*n_new))+...
                           d_new*( blkdiag(n_new',n_new')*DGDf/(n_new'*G_inv*n_new)+...
                           G_inv*Dn_newDf/(n_new'*G_inv*n_new)- G_inv*n_new/(n_new'*G_inv*n_new)^2 *...
                           (2*n_new'*G_inv*Dn_newDf + vec(n_new*n_new')'*DGDf));
                Dd_cutDf = sign(d_cut)*(2*n_new'*G_inv*Dn_newDf + vec(n_new*n_new')'*DGDf)/(2*sqrt(n_new'*G_inv*n_new));
                Dp_cutDf = kron(Dd_cutDf,G_inv*n_new/(n_new'*G_inv*n_new))+...
                           d_cut*( blkdiag(n_new',n_new')*DGDf/(n_new'*G_inv*n_new)+...
                           G_inv*Dn_newDf/(n_new'*G_inv*n_new)- G_inv*n_new/(n_new'*G_inv*n_new)^2 *...
                           (2*n_new'*G_inv*Dn_newDf + vec(n_new*n_new')'*DGDf));
                Dproj2EllipseDf = Dp_cutDf-Dp_on_lineDf;
                der = kron(proj2Ellipse'*diag([-1,1]),eye(2))*DHDx(vf12,[1;0])+H*diag([-1,1])*Dproj2EllipseDf+[1/2,0,1/2,0;0,1/2,0,1/2];
            end
        elseif (range_new(:,1)-range_new(:,2))'*(p_on_line-range_new(:,2))<0
            p_on_line = range(:,2);
            p_on_ellipse = ellipseProjection(focus,a,range(:,2));
            out = p_on_ellipse-p_on_line;
            if der_mode
                der = ellipseProjection(focus,a,range(:,2),'der');
            end
        elseif (range_new(:,2)-range_new(:,1))'*(p_on_line-range_new(:,1))<0
            p_on_line = range(:,1);
            p_on_ellipse = ellipseProjection(focus,a,range(:,1));
            out = p_on_ellipse-p_on_line;
            if der_mode
                der = ellipseProjection(focus,a,range(:,1),'der');
            end
        end
    else    % not in segment mode
        proj2Ellipse = p_cut-p_on_line;
        out = H*diag([-1,1])*proj2Ellipse + center;
        if der_mode
            Dp_on_lineDf = kron(Dd_newdf,G_inv*n_new/(n_new'*G_inv*n_new))+...
                d_new*(blkdiag(n_new',n_new')*DGDf/(n_new'*G_inv*n_new)+...
                G_inv*Dn_newDf/(n_new'*G_inv*n_new) - G_inv*n_new/(n_new'*G_inv*n_new)^2 *...
                (2*n_new'*G_inv*Dn_newDf + vec(n_new*n_new')'*DGDf));
            Dd_cutDf = sign(d_cut)*(2*n_new'*G_inv*Dn_newDf + vec(n_new*n_new')'*DGDf)/(2*sqrt(n_new'*G_inv*n_new));
            Dp_cutDf = kron(Dd_cutDf,G_inv*n_new/(n_new'*G_inv*n_new))+...
                d_cut*( blkdiag(n_new',n_new')*DGDf/(n_new'*G_inv*n_new)+...
                G_inv*Dn_newDf/(n_new'*G_inv*n_new)- G_inv*n_new/(n_new'*G_inv*n_new)^2 *...
                (2*n_new'*G_inv*Dn_newDf + vec(n_new*n_new')'*DGDf));
            Dproj2EllipseDf = Dp_cutDf-Dp_on_lineDf;
            der = kron(proj2Ellipse'*diag([-1,1]),eye(2))*DHDx(vf12,[1;0])+H*diag([-1,1])*Dproj2EllipseDf+[1/2,0,1/2,0;0,1/2,0,1/2];
        end
    end   

else    % If line does not intersect with ellipse
    out = zeros(2,1);
    if der_mode
        der = zeros(2,4);
    end
end
if der_mode
    out = der;
end
end
function H = hhR(u1)
    u1 = u1/norm(u1);
    u2 = [1;0];
    u = (u1+u2)/norm(u1+u2);
    H = 2*(u*u')-eye(2);
end
% householder rotation derivative
function dHdx = DHDx(u1,u2)
n_u1 = u1/norm(u1);
n_u2 = u2/norm(u2);
v_l = n_u1+n_u2;
v = v_l/norm(v_l);
dn_u1du1 = (eye(2)-n_u1*n_u1')/norm(u1);
dvdu = (eye(2)-v*v')/norm(v_l);
dHdv = 2*[2*v(1),v(2),v(2),0;0,v(1),v(1),2*v(2)]';
dHdx = dHdv*dvdu*dn_u1du1*[-1,0,1,0;0,-1,0,1];
end

