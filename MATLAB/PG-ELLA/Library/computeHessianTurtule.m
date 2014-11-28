function [D]=computeHessianTurtule(data,policy)

sigma=policy.theta.sigma; 
numRollouts=size(data,2);
Hes=zeros(2,2);
for i=1:numRollouts
    VarOne=data(i).x(1,:); 
    Reward=data(i).r; 
    VarOneSquared=sum(VarOne.^2);
    VarTwo=data(i).x(2,:); 
    VarTwoSquared=sum(VarTwo.^2);
    %VarThree=data(i).x(3,:);
    %VarThreeSquared=sum(VarThree.^2);
    %VarFour=data(i).x(4,:); 
    %VarFourSquared=sum(VarFour.^2);
    RewardDum=sum(Reward); 
    Matrix=1./sigma*([VarOneSquared 0; 0 VarTwoSquared]*RewardDum);
    %Matrix=([PosSquare PosVel; PosVel VelSquare]*RewardDum);
    Hes=Hes+Matrix; 
end

D=-Hes*1./numRollouts;
