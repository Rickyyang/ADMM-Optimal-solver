load('results\FinalResult.mat')

fileID = fopen('Reference.json','w');
fprintf(fileID,'%s',txt);