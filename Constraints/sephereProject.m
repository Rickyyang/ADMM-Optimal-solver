% projected vectorized 2*n variable inside a sephere of radis of r 
function z_out = sephereProject(z,r)
    for i = 2:2:length(z)
        n_zi = norm(z(i-1:i));
        if n_zi > r
            z(i-1:i)=z(i-1:i)/n_zi * r;
        end
    end
    z_out = z;
end