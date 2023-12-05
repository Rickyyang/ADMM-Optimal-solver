resetRands(1)
W = randn(7,1);
W1 = (W==max(W));
p = 1;
X_s = [W1;W1;W/p];
A=adjgallery(7,'locrand',1);
D = diag(sum(A,2));
L=D-A;
s = 0.01;
K = 100;
A = MultiplierMatrixOutput(K,L,s);
X=A*X_s+[W/p;0*W;0*W];
[X_s,X]'

