function [proj_point,proj_percentage] = segmentProject(p,x1,x2)
    vp = p-x1;
    vx = x2-x1;
    proj_length = dot(vp,vx/norm(vx));
    proj_percentage = proj_length/norm(vx);
    if proj_percentage<0
        proj_point = x1;
        proj_percentage = 0;
    elseif proj_percentage>1
        proj_point = x2;
        proj_percentage = 1;
    else
        proj_point = x1+proj_percentage*vx;
    end
        
end