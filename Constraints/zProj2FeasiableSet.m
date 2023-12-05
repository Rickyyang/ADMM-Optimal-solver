function zOut = zProj2FeasiableSet(z,r)
    DisInVec = z(1:2*length(r));
    zOut1 = sephereProject(DisInVec,r);
    DisArea = z(2*length(r)+1:length(z));
    DisArea(DisArea<0) = 0;
    zOut = [zOut1;DisArea];
end