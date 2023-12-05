function constraint = constraintSafeZone(normalVec,dis2Origin,zoneType)
if size(normalVec,2) == 1
    [f_out,df_out] = traj2HalfPlaneDistance(normalVec,dis2Origin);
else
    F_in = cell(size(normalVec,2),1);
    dF_in = cell(size(normalVec,2),1);
    for i = 1:size(normalVec,2)
        [f,df] = traj2HalfPlaneDistance(normalVec(:,i),dis2Origin(i));
        F_in{i} = f;    %input structure to logic or combination function
        dF_in{i} = df;
    end
    if zoneType == 1 % zoneType 1 : forbidded zone
        [f_out,df_out] = combineDxFcnWithLogicOr(F_in,dF_in);
    elseif zoneType == 2 % zoneType 1 : keep in zone
       [f_out,df_out] = combineDxFcnWithLogicAnd(F_in,dF_in);
    end
end

constraint.intermediateVariableDim = [1,1]';  %dimension of z (after vectorization)
constraint.equalityConstrainDx =f_out;  % equality constrain D(x) = z;
constraint.equalityConstrainDer =df_out;
constraint.intermediateVariableUpdateFcn =@(z,x) max(0,z);
constraint.dov = @(z,x) z;  % degree of violation
constraint.type =@(x) 'ineq';
end
