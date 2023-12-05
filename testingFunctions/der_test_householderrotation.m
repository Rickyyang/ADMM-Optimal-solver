function der_test_householderrotation()
resetRands()

x2 = [1;0];
g=@(x1) householderRotation(diff(x1,[],2),x2);
dg =@(x1) DHDu(diff(x1,[],2),x2)*[-1,0,1,0;0,-1,0,1];
[xt,dxt]=real_randGeodFun(randn(2,2));
gt = @(t) vec(g(xt(t)));
dgt=@(t) vec(dg(xt(t))*vec(dxt(t)));
funCheckDer(gt,dgt);
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
end