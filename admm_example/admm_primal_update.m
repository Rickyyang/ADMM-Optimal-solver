%% This is the primal update objective function with gredient
% D is the function D(x)=z
function out = admm_primal_update(g,dg,z,u,p,D,dD)
% primal update function
f = @(x) g(x)+ (p/2)*(sum((D(x)-z+u).^2)');
% primal update function derivative
df = @(x) vec(dg(x)) + (diag(vec([p;p]))*dD(x))'*vec((D(x)-z+u));
    function [l,dl] = internal(x)
        l = f(x);
        if nargout >1   % gradient required
            dl = df(x);
        end
    end
out = @internal;
end
