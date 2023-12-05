function d=trajectory_diff(xStart,xEnd)
d=@(x) -diff([xStart x xEnd],[],2);