function plotSafeZone(obstacle)
% get vertex
colors= {'#a57e52','#033500','b','k','r','g','m',[.5 .6 .7],[.8 .2 .6]};
for i = 1:size(obstacle,2)
    normalVec = obstacle(i).normalVec;
    dis = obstacle(i).dis2Origin;
    v = [normalVec;-dis];
    v = [v,v(:,1)];
    vertex = zeros(2,length(normalVec));
    for j = 1:length(normalVec)
        cp = cross(v(:,j),v(:,j+1));
        vertex(:,j) = [cp(1)/cp(3);cp(2)/cp(3)];
    end
    p = polyshape(vertex');
    txt = ['Zone ',num2str(i)];
    plot(p,'DisplayName',sprintf(txt),'FaceColor',colors{i})
    hold on
end