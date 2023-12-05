%function P = ellipseProjection(focus,a,point)
%Return the closest point on a given ellipse (defined by focus and major radius 'a')
%with respect to a given point
function out = focus2LineDis(focus,n,d,varargin)
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
%% cvx compute output
if segment_mode % if give a range limit to the line
    % check if range limit points are one the line
    if n'*range(:,1)-d~=0 || n'*range(:,2)-d ~= 0
        disp(range)
        error('range limit points not on the line')
    end
    G = [range(:,1)-range(:,2),range(:,2)-range(:,1)]';
    h = [(range(:,1)-range(:,2))'*range(:,1);(range(:,2)-range(:,1))'*range(:,2)];
    cvx_begin
        variable z(2);
        dual variable l;
        minimize(norm(z-focus(:,1))+norm(z-focus(:,2)));
        subject to
        n'*z  == d;
        l:G*z <= h;
    cvx_end
else
    cvx_begin
        variable z(2);
        minimize(norm(z-focus(:,1))+norm(z-focus(:,2)));
        subject to
            n'*z  == d;
    cvx_end
end
%% derivative calculation
if der_mode
    L1 = eye(2)/norm(z-focus(:,1))-(z-focus(:,1))*(z-focus(:,1))'/(norm(z-focus(:,1))^3);
    L2 = eye(2)/norm(z-focus(:,2))-(z-focus(:,2))*(z-focus(:,2))'/(norm(z-focus(:,2))^3);
    if ~segment_mode
        LFS = [L1+L2,n;n',0];
        dZVdp = LFS\[L1,L2;zeros(1,4)];
    else
        LFS =  [L1+L2,     G',                 n;...
                diag(l)*G, diag(G*z-h),        zeros(length(l),1);...
                n',        zeros(1,length(l)), 0];
        dZVdp = LFS\[L1,L2;zeros(length(l)+1,4)];
    end
end
%% output
if der_mode
    dZdp = dZVdp(1:2,:);
    out = (z-focus(:,1))'*(dZdp-[eye(2),zeros(2)])/norm(z-focus(:,1))+(z-focus(:,2))'*(dZdp-[zeros(2),eye(2)])/norm(z-focus(:,2));
else
    out = cvx_optval;
end
end


