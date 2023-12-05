function f = multiAgentFunctionIntegrate(f_in,traj_length,varargin)
if length(varargin)>=1
    switch lower(varargin{1})
        case 'add'
            % add all function output
            f = @(x) AddCost(f_in,x);
        case 'horizontal'
            % combine all functions vertically
            f = @(x) HorizontalIntegrate(f_in,x);
        case 'vertical'
            % combine all functions horizontally
            f = @(x) VerticalIntegrate(f_in,x);
        case 'diagonal'
            % combine all functions diagonal
            f = @(x) DiagonalIntegrate(f_in,x);
        otherwise
            disp(varargin{ivarargin})
            error('Argument not valid!')
    end
else
    error('integrate option needed');
end
n_agent = size(f_in,2);
    function out = AddCost(g,x)
        out = 0;
        for i = 1:n_agent
            start_id = (i-1)*traj_length+1;
            end_id = i*traj_length;
            x_i = x(:,start_id:end_id);
            out = out + g{i}(x_i);
        end
    end
    function out = HorizontalIntegrate(g,x)
        out = [];
        for i = 1:n_agent
            start_id = (i-1)*traj_length+1;
            end_id = i*traj_length;
            x_i = x(:,start_id:end_id);
            out = [out, g{i}(x_i)];
        end
    end
    function out = VerticalIntegrate(g,x)
        out = [];
        for i = 1:n_agent
            start_id = (i-1)*traj_length+1;
            end_id = i*traj_length;
            x_i = x(:,start_id:end_id);
            out = [out; g{i}(x_i)];
        end
    end
    function out = DiagonalIntegrate(g,x)
        out = [];
        for i = 1:n_agent
            start_id = (i-1)*traj_length+1;
            end_id = i*traj_length;
            x_i = x(:,start_id:end_id);
            out = blkdiag(out, g{i}(x_i));
        end
    end
end