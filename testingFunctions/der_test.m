
resetRands()
Wsize=[3,1];
L = [2 -1 -1;-1 2 -1;-1 -1 2];
s = 0.1;
K = 2;
W = randn(Wsize);
p = 1;
A = MultiplierMatrixOutput(K,L,s);
x_i = 1/Wsize(1)*ones(size(W));
g = @(x) A*x+[W./p;0*x_i;0*x_i];
dg = @(x) 
x = randn(2,20);

[xt,dxt]=real_randGeodFun(randn(2,20));
gt = @(t) vec(g(xt(t)));
dgt=@(t) vec(dg(xt(t))*vec(dxt(t)));
funCheckDer(gt,dgt);
