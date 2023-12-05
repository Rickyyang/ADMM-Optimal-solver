function out = householder_rotation(u_1,type)
if type == 1
    out=hhR(u_1);
elseif type == 2
    out=DHDx(u_1,[1;0]);
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
%         dHdx = dHdv*dvdu*dn_u1du1*[-1,0,1,0;0,-1,0,1];
        dHdx = dHdv*dvdu*dn_u1du1;
    end
end