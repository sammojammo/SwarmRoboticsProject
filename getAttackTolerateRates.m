%Function to get the attack/tolerate rates of every agent in the swarm

function [attTolRateMatrix] = getAttackTolerateRates(data)
%Get size of data matrix (equal to number of steps)
numSteps = length(data);

%allocate memory for attackRateMatrix
attTolRateMatrix = zeros(20,1);

%sum proportion of attackers for every time step
for botID = 1:20
    attackTotal = 0;
    for i = 1:numSteps
        temp = data(i,9,botID)/data(i,8,botID);
    
        %Ensure no NaN issues caused by divisions by 0 etc
        if isnan(temp)
            temp = 0;
        end
        attackTotal = attackTotal + temp;
    end
    
    %return average over all time steps    
    attTolRateMatrix(botID,1) = attackTotal/numSteps;
    
    %tolerate rate set in second column
    attTolRateMatrix(botID,2) = 1 - attTolRateMatrix(botID,1);
end