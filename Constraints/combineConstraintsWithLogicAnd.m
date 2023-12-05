%% compress constraint function D(x) output to the max one
% D1(x)>= 0 and D2(x)>= 0 and ..... Dn(x)>=0 ----->
% min([D1(x),...Dn(x)])>=0
% what if output of D1(x)..., Dn(x) doesn't have the same dimension
function constraintOut = combineConstraintsWithLogicAnd(varargin)
ivarargin=1;
while ivarargin<=length(varargin)
    constraintsIn(ivarargin)=varargin{ivarargin};
    ivarargin=ivarargin+1;
end
%% reshape and stack D(x)
    function out = minDix(x,out_type,z)
        f_out = zeros(ivarargin-1,1);
        dov = zeros(ivarargin-1,1); %degree of violation 
        dDx_1 = constraintsIn(1).equalityConstrainDer(x);
        df_out = zeros(ivarargin-1,size(dDx_1,2));
        for i = 1:(ivarargin-1)
            Dx_i = constraintsIn(i).equalityConstrainDx(x);
            dov_i = constraintsIn(i).dov(Dx_i);
%             if strcmp(constraintsIn(i).type(x),'eq')     % change equality constraint D(x) = 0 to -abs(D(x)) >=0
%                dov_i = -abs(Dx_i); 
%             elseif strcmp(constraintsIn(i).type(x),'ineq')
%                 dov_i = Dx_i;
%             else
%                 error('Undefined constraint type for constraint %d',i);
%             end
            [dov(i),f_out_id] = min(dov_i);
            f_out(i) = Dx_i(f_out_id);
            dDx_i = constraintsIn(i).equalityConstrainDer(x);
            df_out(i,:) = dDx_i(f_out_id,:);
        end
        [min_value,min_f_id] = min(dov);
        f = f_out(min_f_id);
        df = df_out(min_f_id,:);
        % choose f or df as output
        if strcmp(out_type,'fcn')
            out = f;
        elseif strcmp(out_type,'fcnDer')
            out = df;
        elseif strcmp(out_type,'ctType')
            out = constraintsIn(min_f_id).type(x); % output constraint type
        elseif strcmp(out_type,'ivUpdateFcn')
            out = constraintsIn(min_f_id).intermediateVariableUpdateFcn(z,x);
        elseif strcmp(out_type,'dov')
            out = constraintsIn(min_f_id).dov(z,x);
        end
    end

Dx = @(x) minDix(x,'fcn',0);
dDx = @(x) minDix(x,'fcnDer',0);
type_fcn = @(x) minDix(x,'ctType',0);
iv_update_fcn = @(z,x) minDix(x,'ivUpdateFcn',z);
dov_fcn = @(z,x) minDix(x,'dov',z);


%% generate constraint output
constraintOut.intermediateVariableDim = [1;1];
constraintOut.equalityConstrainDx = Dx;  % equality constrain D(x) = z;
constraintOut.equalityConstrainDer = dDx; 
constraintOut.intermediateVariableUpdateFcn =iv_update_fcn;
constraintOut.dov = dov_fcn;
constraintOut.type = type_fcn;
end

   