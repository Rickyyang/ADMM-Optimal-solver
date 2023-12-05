function f=gen_gaussian(prop)
f=@(x) prop.A/sqrt(det(prop.A))*exp(-(x-prop.x)'*inv(prop.Sigma)*(x-prop.x)/2);

