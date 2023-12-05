function [x,z,u,Lag]= MatrixUpdate(W,L,s,xPrev,zPrev,uPrev,p,K)
    n = length(W);
    A = MultiplierMatrixOutput(K,L,s);
    X = A*[xPrev;zPrev;uPrev]+ [W./p;0*xPrev;0*xPrev];
    X = reshape(X,n,3);
    x = min(max(X(:,1),0),1);
    z = X(:,2);
    u = X(:,3);
    %lag
    Lag = -x'*W + p(1)/2*(norm(x-z+u))^2;
end
     