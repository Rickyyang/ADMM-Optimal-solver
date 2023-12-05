
function [projected_point_fcn, projected_point_fcn_grad] = fixPoint2TrajectoryProject(fix_point,nAgent)
    function proj_p = projected_point_fcn_gen(x)
        f = @(x) fixPoint2TrajectoryProjectFcn(fix_point,x,nAgent);
        [proj_p,grad] = f(x);
    end
    function grad = projected_point_gred_gen(x)
        f = @(x) fixPoint2TrajectoryProjectFcn(fix_point,x,nAgent);
        [proj_p,grad] = f(x);
    end
projected_point_fcn = @projected_point_fcn_gen;
projected_point_fcn_grad = @projected_point_gred_gen;
end