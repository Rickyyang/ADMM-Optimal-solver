%% compress constraint function D(x) output to the max one
% D1(x)>= 0 or D2(x)>= 0 or .....or Dn(x)>=0 ----->
% max([D1(x),...Dn(x)])>=0
% what if output of D1(x)..., Dn(x) doesn't have the same dimension
function constraintOut = combineConstraintsWithLogicOr(varargin)
ivarargin=1;
while ivarargin<=length(varargin)
    constraintsIn(ivarargin)=varargin{ivarargin};
    ivarargin=ivarargin+1;
end
%% reshape and stack D(x)
    function out = maxDix(x,out_type,z)
        f_out = zeros(ivarargin-1,1);
        dov = zeros(ivarargin-1,1); %degree of violation 
        dDx_1 = constraintsIn(1).equalityConstrainDer(x);
        df_out = zeros(ivarargin-1,size(dDx_1,2));
        for i = 1:(ivarargin-1)
            Dx_i = constraintsIn(i).equalityConstrainDx(x);
            dov_i = constraintsIn(i).dov(Dx_i,x);
%             if strcmp(constraintsIn(i).type(x),'eq')     % change equality constraint D(x) = 0 to -abs(D(x)) >=0
%                dov_i = -abs(Dx_i); 
%             elseif strcmp(constraintsIn(i).type(x),'ineq')
%                 dov_i = Dx_i;
%             else
%                 error('Undefined constraint type for constraint %d',i);
%             end
            [dov(i),f_out_id] = max(dov_i);    % find the most deviating element 
            f_out(i) = Dx_i(f_out_id);
            dDx_i = constraintsIn(i).equalityConstrainDer(x);
            df_out(i,:) = dDx_i(f_out_id,:);
        end
        [max_dov,max_f_id] = max(dov); % find out the constraint with most deviating element
        f = f_out(max_f_id);
        df = df_out(max_f_id,:);
        % choose f or df as output
        if strcmp(out_type,'fcn')
            out = f;
        elseif strcmp(out_type,'fcnDer')
            out = df;
        elseif strcmp(out_type,'ctType')
            out = constraintsIn(max_f_id).type; % output constraint type
        elseif strcmp(out_type,'ivUpdateFcn')
            out = constraintsIn(max_f_id).intermediateVariableUpdateFcn(z,x);
        elseif strcmp(out_type,'dov')
            out = constraintsIn(min_f_id).dov(z,x);
        end
    end

Dx = @(x) maxDix(x,'fcn',0);
dDx = @(x) maxDix(x,'fcnDer',0);
type_fcn = @(x) maxDix(x,'ctType',0);
iv_update_fcn = @(z,x) maxDix(x,'ivUpdateFcn',z);
dov_fcn = @(z,x) maxDix(x,'dov',z);

%% generate constraint output
constraintOut.intermediateVariableDim = [1;1];
constraintOut.equalityConstrainDx = Dx;  % equality constrain D(x) = z;
constraintOut.equalityConstrainDer = dDx; 
constraintOut.intermediateVariableUpdateFcn =iv_update_fcn;
constraintOut.dov = dov_fcn;
constraintOut.type = type_fcn;
end

   