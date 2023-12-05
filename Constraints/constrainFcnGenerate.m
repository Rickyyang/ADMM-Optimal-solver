function [D_out,dD_out,zVariableUpdate] = constrainFcnGenerate(D_in,dD_in,r_max,x_initial,varargin)
nFixPointFcn = 0;
nTrajDiffFcn = 0;
nAvoidAreaFcn = 0;
ivarargin=1;
min_inspection_length = 0;
flagMultiTraj = false;
traj_length = size(x_initial,2);

%%
% use x_initial to generate the r
%%
n_edges = traj_length+1; % number of constrains
nFixPoints = 0; % number of fix points need to pass
nInspectionPair = 0; % pair of agent need inspection
while ivarargin<=length(varargin)
    switch lower(varargin{ivarargin})
        case 'number of agent'
            ivarargin=ivarargin+1;
            n_agent = varargin{ivarargin};
            if n_agent >1
                flagMultiTraj = true;
            end
            if length(D_in)~=n_agent
                error('number of trajectory edge functions and number of agents does not match ')
            end
        case 'fix point function'                    
            ivarargin=ivarargin+1;
            % check if the input function is empty
            if ~isempty(varargin{ivarargin})
                nFixPointFcn = nFixPointFcn+1;
                fixPointFcn{nFixPointFcn}=varargin{ivarargin};
                ivarargin=ivarargin+1;
                fixPointFcnDer{nFixPointFcn}=varargin{ivarargin};
            else
                % skip the corresponding derivative function 
                ivarargin = ivarargin+1;
            end
        case 'traj diff function'
            ivarargin=ivarargin+1;
            % check if the input function is empty 
            if ~isempty(varargin{ivarargin})
                nTrajDiffFcn = nTrajDiffFcn+1;
                trajDiffFcn{nTrajDiffFcn} = varargin{ivarargin};
                ivarargin=ivarargin+1;
                trajDiffFcnDer{nTrajDiffFcn} = varargin{ivarargin};
            else
                % skip the corresponding derivative function 
                ivarargin=ivarargin+1;
            end
        case 'min inspection length'
            ivarargin=ivarargin+1;
            min_inspection_length = varargin{ivarargin};
        case 'avoid area'
            ivarargin=ivarargin+1;
            if ~isempty(varargin{ivarargin})
                nAvoidAreaFcn = nAvoidAreaFcn+1;
                avoidAreaFcn{nAvoidAreaFcn} = varargin{ivarargin};
                ivarargin=ivarargin+1;
                avoidAreaFcnDer{nAvoidAreaFcn} = varargin{ivarargin};
            else
                ivarargin = ivarargin +1;
            end
        otherwise
            disp(varargin{ivarargin})
            error('Argument not valid!')
    end
    ivarargin=ivarargin+1;
end

% calculating outputs 
r_fixPoint = [];
for h = 1:nFixPointFcn
    nFixPoints = nFixPoints+length(fixPointFcn{h}(x_initial))/2;
    r_fixPoint = zeros(1,nFixPoints);
end
r_trajDiff = [];
for h = 1:nTrajDiffFcn
    nInspectionPair = nInspectionPair + length(trajDiffFcn{h}(x_initial))/2;
    r_trajDiff = min_inspection_length*ones(1,nInspectionPair);
end

%% combine multiple agent traj edge function to one
    function Edge = traj_diff(x)
        % function combine multi agent trajectory cost functions into one
        % single function
        Edge = [];
        for k = 1:n_agent
            start_id = (k-1)*traj_length+1;
            end_id = k*traj_length;
            x_i = x(:,start_id:end_id);
            Edge = [Edge; D_in{k}(x_i)];
        end
    end
    function dEdge = traj_diff_der(x)
        % function combine multi agent trajectory cost functions into one
        % single function
        dEdge = [];
        for k = 1:n_agent
            start_id = (k-1)*traj_length+1;
            end_id = k*traj_length;
            x_i = x(:,start_id:end_id);
            dEdge = blkdiag(dEdge, dD_in{k}(x_i));
        end
    end

if flagMultiTraj
    D = @traj_diff;
    dD = @traj_diff_der;
    n_edges = n_agent*n_edges;
else 
    D = D_in;
    dD = dD_in;
end

%% generate whole functionn (vectorize combine all constrin function )
    function f = x2z(x)
        % function combine trajectory distance and fix point difference
        % with edge generating function D(x)
        f = D(x);
        if nInspectionPair ~=0
            for l = 1:nTrajDiffFcn
                % add trajectory difference function
                f = [f;trajDiffFcn{l}(x)];
            end
        end
        if nFixPoints ~= 0
            for l = 1:nFixPointFcn
                % add fix point difference function
                f = [f;fixPointFcn{l}(x)];
            end
        end
        if nAvoidAreaFcn ~=0
            for l = 1:nAvoidAreaFcn
                % add avoid area function
                f = [f;avoidAreaFcn{l}(x)];
            end
        end
    end

    function df = x2z_der(x)
        % all derivatives are pre-vectorized
        % function generating the derivative of the above function
        df = dD(x);
        if nInspectionPair ~=0
            for l = 1:nTrajDiffFcn
                % add trajectory difference function derivative
                df = [df;trajDiffFcnDer{l}(x)];
            end
        end
        if nFixPoints ~= 0
            for l = 1:nFixPointFcn
                % add fix point difference function derivative
                df = [df;fixPointFcnDer{l}(x)];
            end
        end
        if nAvoidAreaFcn ~=0
            for l = 1:nAvoidAreaFcn
                % add avoid area function derivative
                df = [df;avoidAreaFcnDer{l}(x)];
            end
        end
    end


%% generating output
%
D_out = @x2z;
dD_out = @x2z_der;
r = r_max*ones(1,n_edges);
r = [r,r_trajDiff,r_fixPoint];
zVariableUpdate =@(z) zProj2FeasiableSet(z,r);
end