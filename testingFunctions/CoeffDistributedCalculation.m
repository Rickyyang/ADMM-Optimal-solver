function [x,z,u] = CoeffDistributedCalculation(W,L,s,xPrev,zPrev,uPrev,p,K)
    %for each agent
    x=0*xPrev;
    u=0*uPrev;
    z=zLocalUpdate(L,s,xPrev,uPrev,K);
    for i = 1:length(W)
        u(i)=uPrev(i)+xPrev(i)-z(i);
    end
    x=xUpdate(x,z,u);
    % test status update
    n = length(W);
%     [F,G] = zMatrixOutput(K,L,s);
%     X = [2*F-eye(n), 2*G, 2*F-eye(n);...
%          F, G, F;...
%          eye(n)-F, -G, eye(n)-F]*[xPrev;zPrev;uPrev]+ [W./p;0*xPrev;0*xPrev];
    function x = xUpdate(x,z,u)
        for j = 1:length(W)
            x(j)=min(max(z(j)-u(j)+W(j)/p(j),0),1);
        end
    end
end