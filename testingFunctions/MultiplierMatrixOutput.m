% output matrix F and G for zIteration output with a function of the
% iteration number K. (n is variable size, L is connectivity matrix
function A = MultiplierMatrixOutput(K,L,s)
    n = size(L,1);
    G = (eye(n)-s*L)^K;
    F = 0*L;
    for i = 1:K
        F = F + (eye(n)-s*L)^(i-1);
    end
    F = s*F*L;
    A = [2*F-eye(n), 2*G, 2*F-eye(n);...
         F, G, F;...
         eye(n)-F, -G, eye(n)-F];
end