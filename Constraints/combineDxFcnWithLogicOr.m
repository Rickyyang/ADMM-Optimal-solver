%% combine multiple inequality constraints using logic or
% logic or : D1(x)>=0 or  D2(x)>=0 or  D3(x)>=0  = D(x)
%    
function [D,dD] = combineDxFcnWithLogicOr(ineqConstraint,ineqConstraint_grad)
nConstraint = size(ineqConstraint,1);
constraintFcn = @(x) [];
constraintFcnDer = @(x) [];
for j = 1:nConstraint
    % input should be in the form of {D(x),dD(x)}
    constraintFcn = @(x) [constraintFcn(x),ineqConstraint{j}(x)];
    constraintFcnDer = @(x) [constraintFcnDer(x);ineqConstraint_grad{j}(x)];
end

    function f = generateConstrainFcn(x)
        valueAll = constraintFcn(x);
        constraintStatus = valueAll>=0; % return a logic matrix representing whether the constraints are active
        %% find inactive constrains (new)
        constraintsStatus = not(any(constraintStatus,2));
        violationScale = constraintsStatus'*valueAll;
        out = max(violationScale);
        f = out;
    end

    function df = generateConstrainFcnDer(x)
        valueAll = constraintFcn(x);
        constraintStatus = valueAll>=0; 
        constraintsStatus = not(any(constraintStatus,2));
        violationScale = constraintsStatus'*valueAll;
        [out,id] = max(violationScale);
        selection = zeros(nConstraint,1);
        selection(id) = 1;
        constraintsStatusAll = kron(selection,constraintsStatus);
        df = constraintsStatusAll'*constraintFcnDer(x);
    end
D = @generateConstrainFcn;
dD = @generateConstrainFcnDer;
end
