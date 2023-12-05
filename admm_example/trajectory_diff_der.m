%% d(vec(D(x)))
%  ------------
%   d(vec(x))
function df = trajectory_diff_der()
    function out = generate_df(x)
        l = length(x);
        diag = eye(l);
        ins = zeros(1,l);
        out = [ins;diag]-[diag;ins];
        out = kron(out,eye(2));
    end
df = @generate_df;
end