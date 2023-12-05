% update the constrain function using using the edge generate function
% z=D(x)
function [D_out,dD_out,zVariableUpdate,min_edge_index]=constrainFcnUpdate(x,x_initial,D,dD,n_agent,fixed_point,r_max,inspection_pair,min_inspection_length,avoid_area)
    TrajDis = [];
    dTrajDis = [];
    D_fix_point = [];
    dD_fix_point = [];
    min_edge_index = [];
    DisLine = [];
    dDisLine = [];
    traj_length = size(x,2);
    if ~isempty(inspection_pair)
        min_edge_index = findInspectionVerticesPair(x,traj_length,inspection_pair);
        [TrajDis,dTrajDis] = traj2TrajDistance(min_edge_index);
    end    
    if ~isempty(fixed_point)
        [projection_vertex_id, projection_percentage] = trajectoryProject(x,fixed_point);
        [D_fix_point,dD_fix_point] = traj2FixPointDistance(fixed_point,length(x),projection_vertex_id, projection_percentage);
    end
    if ~isempty(avoid_area)
        normalVec = avoid_area{1}; Dist2Origin = avoid_area{2};
        [DisLine,dDisLine] = traj2LineDistance(normalVec,Dist2Origin);
    end
    if ~isempty(fixed_point) || ~isempty(inspection_pair)
        [D_out,dD_out,zVariableUpdate] = constrainFcnGenerate(D,dD,r_max,x_initial, 'number of agent',n_agent,...
            'fix point function',D_fix_point,dD_fix_point,...
            'traj diff function',TrajDis,dTrajDis,'min inspection length',min_inspection_length,...
            'avoid area',DisLine,dDisLine);
    else
        D_out = D;
        dD_out = dD;
        r = r_max*ones(1,traj_length+1);
        zVariableUpdate = @(z) sephereProject(z,r);
    end
end