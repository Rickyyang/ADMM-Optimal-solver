
x = initTrajGen([2;0],[10;10],10,2);
focus = x(:,[4,9]);
a = 4;

origin =( focus(:,1)+focus(:,2))/2;
x_e = focus(:,2)-focus(:,1);
x_e = 2*x_e/norm(x_e);
x_e_axis = origin +x_e ;
y_e_axis = origin + [-x_e(2);x_e(1)];
figure('Position', [10 10 700 600])
plotPoints(x,'-o')
hold on
ellipseDraw(focus,a,':r')
plot_arrow( origin(1),origin(2),x_e_axis(1),x_e_axis(2),'headwidth',0.1,'headheight',0.1,'color','b','facecolor','b')
plot_arrow( origin(1),origin(2),y_e_axis(1),y_e_axis(2),'headwidth',0.1,'headheight',0.1,'color','b','facecolor','b')
str ='$$ \mathcal{F}_{\mathcal{E}}$$';
text(origin(1),origin(2)-0.3,str,'Interpreter','latex','FontSize',12)

plot_arrow( 0,0,1.8,0,'headwidth',0.1,'headheight',0.1 )
plot_arrow( 0,0,0,1.8,'headwidth',0.1,'headheight',0.1 )
str ='$$ \mathcal{F}$$';
text(0.05,-0.4,str,'Interpreter','latex','FontSize',12)

l = [4.5;6];
l = ellipseProjection(focus,a,l);
plotPoints([focus(:,1),l,focus(:,2)],'-.g')
l = [9;7];
l = ellipseProjection(focus,a,l);
plotPoints([focus(:,1),l,focus(:,2)],'-.g')

str ='$$ \hat{\nu_{\mathcal{E}}}$$';
text(1.3,-0.4,str,'Interpreter','latex','FontSize',12)
str ='$$ \hat{\nu_{\mathcal{F}}}$$';
text(x_e_axis(1)-0.1,x_e_axis(2)-0.5,str,'Interpreter','latex','FontSize',12)

text(focus(1,1)+0.2,focus(2,1),'x_1','FontSize',12)
text(focus(1,2)+0.2,focus(2,2),'x_2','FontSize',12)

corners = [0,10;0,8;4,8;4,10]';
p = polyshape(corners');
plot(p)
plotPoints(focus,'rx')
axis([-1 12 -1 12])




