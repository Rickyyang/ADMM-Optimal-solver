%% Generate the vectorized trajectory Edge Function and its derivative
function [d,df]=trajectoryEdge(xStart,xEnd)
d=@(x) vec(-diff([xStart x xEnd],[],2));
df= @generate_df;
end

function out = generate_df(x)
l = length(x);
diag = eye(l);
ins = zeros(1,l);
out = [ins;diag]-[diag;ins];
out = kron(out,eye(2));
end