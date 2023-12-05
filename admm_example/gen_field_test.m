function f=gen_field_test(field_prop)

n = length(field_prop);
f = gen_gaussian(field_prop(1));
for i = 2:n
    g = gen_gaussian(field_prop(i));
    f = @(x) f(x) + g(x);
end

