function [f_out,df_out] = mapUncertaintyModel(map_size,traj_length,varargin)
Sigma = 0.3;
ivarargin=1;
n_agent = 1;
flagWholeMap = false;
b_lse = 1;
while ivarargin<=length(varargin)
    switch lower(varargin{ivarargin})
        case 'sigma'                     % observation quality
            ivarargin=ivarargin+1;
            Sigma=varargin{ivarargin};
        case 'agent number'              % total number of agent
            ivarargin=ivarargin+1;
            n_agent=varargin{ivarargin};
        case 'uncertainty map'
            flagWholeMap = true;
        case 'scale parameter'
            ivarargin=ivarargin+1;
            b_lse=varargin{ivarargin};
        otherwise
            disp(varargin{ivarargin})
            error('Argument not valid!')
    end
    ivarargin=ivarargin+1;
end
% fixed parameter for calculation
x = 0:1:map_size(1);
y = 0:1:map_size(2);
[x,y] = meshgrid(x,y);
map_coord = [x(:),y(:)]';
map_coord_length = length(map_coord);
map_coord_expand = kron(ones(1,traj_length),map_coord');
map_coord_sq = sum(map_coord.^2)'*ones(1,traj_length);
% single agent case
if n_agent == 1 
    f = @(traj) mapUncertaintyModelFcn(traj,map_size,'Y');
    df = @(traj) mapUncertaintyModelFcn(traj,map_size,'Y_der');
else
    % multiple agent case
    for k = 1:n_agent
        f_single{k} = @(traj) mapUncertaintyModelFcn(traj,map_size,'Y');
        df_single{k} = @(traj) mapUncertaintyModelFcn(traj,map_size,'Y_der');
    end
    f = multiAgentFunctionIntegrate(f_single,traj_length,'add');
    df = multiAgentFunctionIntegrate(df_single,traj_length,'horizontal');
end

if ~flagWholeMap
    f_out =@(x) log(sum(exp(-b_lse*f(x))));
    df_out =@(x) -b_lse*exp(-b_lse*f(x))'*df(x)/sum(exp(-b_lse*f(x)));
else
    f_out =@(x) reshape(f(x),map_size(1)+1,[]);
    df_out = df;
end

% functions for uncertainty generate
    function Out = mapUncertaintyModelFcn(traj,map_size,output)
        flagComputeDerivative=strcmp(output,'Y_der');
        %parameter and initilization
        q = 0.01; % process noise cov
        traj = reshape(traj,2,[]);
        n_loop = size(traj,2);
        Y_dim = (map_size(1)+1)*(map_size(2)+1);
        Y = zeros(Y_dim,1);
        Y_der = zeros(Y_dim,numel(traj));
        I = ones(Y_dim,1);
        Q = q*I;
        if ~flagComputeDerivative
            R_inv = scanAccuracy(traj); % scan accuracy for all trajectories
        else
            [R_inv,R_inv_der] = scanAccuracy(traj); % scan accuracy for all trajectories
        end
        for i = 1:n_loop
            Y_prev = Y;
            Y_der_prev = Y_der;
            % Y update
            R_wp_inv = R_inv(:,i);
            Y = Y_prev./(I+Y_prev.*Q)+R_wp_inv;
            if flagComputeDerivative
                % dY/dx
                Y_der_prev_factor=(-Y_prev.*Q./((I+Y_prev.*Q).^2)+(I+Y_prev.*Q).^-1);
                Y_der = Y_der_prev_factor.*Y_der_prev;
                Y_der(:,(2*i-1:2*i)) = R_inv_der(:,(2*i-1:2*i));
            end
        end
        % if output == 'Y'
        %     Out = log(sum(exp(-Y)));
        % elseif output == 'Y_der'
        %     Out = -exp(-Y)'*Y_der/sum(exp(-Y));
        % end
        if ~flagComputeDerivative
            Out = Y;
        else
            Out = Y_der;
        end
        
        function [scan,scan_der] = scanAccuracy(traj)
            flagComputeDerivative=nargout==2;
            flagSkipZeroDis=false;
            dis = map_coord_sq - 2*map_coord'*traj+ones(map_coord_length,1)*sum(traj.^2);
            if ~flagSkipZeroDis
                scan = exp(-dis/(2*Sigma));
            else
                scan=ones(size(dis));
                flagNonZeroDis=dis>1e-10;
                scan(flagNonZeroDis)=exp(-dis(flagNonZeroDis)/(2*Sigma));
            end
            dis_vec = map_coord_expand - traj(:)';
            if flagComputeDerivative
                scan_der = dis_vec/Sigma;
                scan_der(:,1:2:2*traj_length) = scan_der(:,1:2:2*traj_length).*scan;
                scan_der(:,2:2:2*traj_length) = scan_der(:,2:2:2*traj_length).*scan;
            end
%             scan(i+1,j+1)= exp(-([j;i]-traj)'/Sigma*([j;i]-traj)/2);
%             scan_der((j)*scan_map_size(1)+i+1,:) = (exp(-([j;i]-traj)'/Sigma*([j;i]-traj)/2)*(Sigma\([j;i]-traj)))';
%             
%             scanAcc = scan(:);
%             scanAccDer = scan_der;
        end
    end
end

