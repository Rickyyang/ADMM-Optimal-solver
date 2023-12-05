close all
clc
clear
x = initTrajGen([2;0],[10;10],10,2);
focus = x(:,[4,9]);
a = 4;
ellipseDraw(focus,a,'k');
hold on
plotPoints(x,'-o')
text(focus(1,1)+0.2,focus(2,1),'q_1','FontSize',12)
text(focus(1,2)+0.2,focus(2,2),'q_2','FontSize',12)

%% project fix point
p = [6;4];
plotPoints(p,'bo')
p_e = ellipseProjection(focus,a,p);
text(p(1)+0.2,p(2)-0.2,'q_{avoid}','FontSize',12)
plotPoints([p_e,p],'-.bx')

plot_arrow( p(1),p(2),p_e(1),p_e(2),'headwidth',0.1,'headheight',0.1,'linestyle','-.' )% text(p(1)+0.2,p(2),'f','FontSize',12)
text(p_e(1)+0.2,p_e(2)-0.2,'q_p','FontSize',12)
axis([-1,11,-1,11])

% %% project to plane
% n = [1;1];
% d = 8;
% out = focus2halfplaneProjection(focus,a,n,d);
% t=linspace(-1,10,50);
% l = [0;8]+[1;-1]*t;
% plotPoints(l,'-')