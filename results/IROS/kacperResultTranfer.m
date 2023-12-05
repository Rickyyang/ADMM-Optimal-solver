jsonresult = importdata('init.dat');
result = jsondecode(jsonresult{:});
starts = result.starts;
goals = result.goals;
path = result.control;
n_agent =size(path,2);
p =zeros(length(path)+2,2,n_agent);
for i = 1:n_agent
p(:,:,i) = [starts(i,:);[path(:,i,1),path(:,i,2)];goals(i,:)];
end
figure()
plotPoints(p(:,:,1)',':o');
hold on
grid on
plotPoints(p(:,:,2)','--x');