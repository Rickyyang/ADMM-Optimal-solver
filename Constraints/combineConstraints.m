%% vectorize and stack constraints
function constraintOut = combineConstraints(varargin)
ivarargin=1;
while ivarargin<=length(varargin)
    constraintsIn(ivarargin)=varargin{ivarargin};
    ivarargin=ivarargin+1;
end
vectorizedZDim = sum(prod([constraintsIn.intermediateVariableDim]));
constraintOut.intermediateVariableDim = [vectorizedZDim,1]';
% combine all equality constrain lhs
DxAll =@(x) [];   
dDxAll =@(x) [];
for i = 1:(ivarargin-1)
    DxAll =@(x) [DxAll(x);constraintsIn(i).equalityConstrainDx(x)];
    dDxAll =@(x) [dDxAll(x);constraintsIn(i).equalityConstrainDer(x)];
end
constraintOut.equalityConstrainDx = DxAll;  % equality constrain D(x) = z;
constraintOut.equalityConstrainDer = dDxAll; 
% combine z update, x is just to let it fit the regular z update form C(z,x)
    function zOut = CombineIntermediateVariableUpdateFcn(zIn,x)
        zOut = zeros(vectorizedZDim,1);
        startId = 1;
        for j = 1:(ivarargin-1)
            endId = startId + prod(constraintsIn(j).intermediateVariableDim) -1;
            zOut(startId:endId) = constraintsIn(j).intermediateVariableUpdateFcn(zIn(startId:endId),x);
            startId = endId+1;
        end
    end
    function dov = CombineDovFcn(zIn,x)
        dov = [];
        startId = 1;
        for j = 1:(ivarargin-1)
            endId = startId + prod(constraintsIn(j).intermediateVariableDim) -1;
            dov(startId:endId) = constraintsIn(j).dov(zIn(startId:endId),x);
            startId = endId+1;
        end
    end
constraintOut.intermediateVariableUpdateFcn =@CombineIntermediateVariableUpdateFcn;
constraintOut.dov = @CombineDovFcn;
constraintOut.type = 'mixed';
end