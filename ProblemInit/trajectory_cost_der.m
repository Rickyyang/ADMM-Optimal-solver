function df = trajectory_cost_der(field_prop)
    function der_cost = der_full_cost(x)
        n_gau = length(field_prop);
        n_wp = length(x);
        der_cost = zeros(size(x));
        for i = 1:n_gau
            prop = field_prop(i);
            for j = 1:n_wp
                der_cost(:,j)=der_cost(:,j)-prop.A/sqrt(det(prop.A))*exp(-(x(:,j)-prop.x)'/(prop.Sigma)*(x(:,j)-prop.x)/2)*((prop.Sigma)\(x(:,j)-prop.x));
            end
        end
        der_cost = - der_cost;
    end
df = @der_full_cost;
end