function Collision = CollisionCheck(q_new,q_near,obstacle)
% get vertex
s=q_new-q_near;
collision=zeros(1,size(obstacle,2));
for i = 1:size(obstacle,2)
    normalVec = obstacle(i).normalVec;
    dis = obstacle(i).dis2Origin;
    v = [normalVec;-dis];
    v = [v,v(:,1)];
    vertex = zeros(2,length(normalVec)+1);
    for j = 1:length(normalVec)
        cp = cross(v(:,j),v(:,j+1));
        vertex(:,j) = [cp(1)/cp(3);cp(2)/cp(3)];
    end
    vertex(:,end)=vertex(:,1);
    intersect=zeros(1,length(normalVec));
    for j = 1:length(normalVec)
        p = vertex(:,j);
        r = vertex(:,j+1)-vertex(:,j);        
        angle=cross2D(r,s);
        dis = cross2D((p-q_near),r);
        t = cross2D((q_near-p),s)/cross2D(r,s);
        u = cross2D((q_near-p),r)/cross2D(r,s);
        if (angle == 0) && (dis~=0)
            %parallel 
            intersect(j)=false;
        elseif (angle == 0) && (dis==0)
            %colinear 
            t1=(q_near-p)/r;
            t2=(q_new-p)/r;
            if (t1>1 && t2<1) || (t1<1 && t2>1) || (t1<0 && t2>0) || (t1>0 && t2<0)
                % intersect
                intersect(j)=true;
            else
                intersect(j)=false;
            end
        elseif (angle~=0) && (t>=0) && (t<=1) && (u>=0) && (u<=1)
            intersect(j)=true;
        else
            intersect(j)=false;
        end        
    end
    collision(i)=any(intersect);
end
Collision=any(collision);
end

