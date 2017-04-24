%Function to read in a given log file and extract the data
%Data is extracted into a 3D matrix by Robot
%A robots data can then be accessed using the following:
%       seperatedData(stepNumber,:,RobotID)
function [seperatedData] = SeperateBotData(filePath)

fileID = fopen(filePath,'r');
sizeA = [11 inf];
formatSpec = "%d";

allData = fscanf(fileID,formatSpec,sizeA);
allData = allData';

seperatedData = zeros(9549,11,20);

for i = 1:20
   seperatedData(:,:,i) = allData(allData(:,1) == (i-1),:); 
end