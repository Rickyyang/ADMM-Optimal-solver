function constraintHalfPlane_plot
normalVec = [1,-1,0,0;0,0,1,-1];
dis = [6,-4,6,-4];
halfPlaneConstraint=constraintHalfPlane(normalVec,dis,[2 1]);
funImage(linspace(2,8),linspace(2,8),halfPlaneConstraint.equalityConstrainDx,'method','surf')
