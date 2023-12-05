function z = zLocalUpdate(L,s,x,uPrev,K)
    zAll=zeros(6,K);
    z_initial = ones(size(x))/length(x);
    zPrev = z_initial;
    zAll(:,1)=zPrev;
    xu = x+uPrev;
    %xu = max(xu,0);
    z = zPrev;
    D = diag(L);
    for i = 1:K
        for j = 1:length(zPrev)
            % for each agent
            L_j = L(j,:);
            %
            if sum(L_j)~=0
                error('L matrix defined wrong')
            end
            %
            index = find(L_j==-1);
            z(j)=zPrev(j)-s*( D(j)*(zPrev(j)-xu(j)) - sum(zPrev(index)-xu(index)));
        end
%         if any(z<0)
%             break
%         end
        zAll(:,i+1)=z;
        zPrev = z;
    end
%     plot(zAll')
end