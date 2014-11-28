function [model]=PGELLAUpdate(model,HessianArray,ParameterArray,taskIndex)
% Update S(:,taskIndex)


Dsqrt=eye(2,2);
%Dsqrt = HessianArray(taskIndex).D^.5;
%Dsqrt = HessianArray(taskIndex).D;
target = Dsqrt*ParameterArray(taskIndex).alpha;
dictTransformed = Dsqrt*model(1).L;

s = full(mexLasso(target,dictTransformed,struct('lambda',model(1).mu_one/2)));

model.S(:,taskIndex)=s; 
% Update L using GD for now... Implemented for consecutive tasks for now
numTasks=model(1).T;
%size(HessianArray);
%size(ParameterArray);
%taskIndex;
termOne=zeros(size(model(1).L));
termTwo=zeros(size(model(1).L));
termThree=zeros(size(model(1).L));

for i=1:numTasks
    %HessianArray(i).D
    %ParameterArray(i).alpha
    %pause
    
    termOne=termOne+HessianArray(i).D*ParameterArray(i).alpha*model(1).S(:,i)';
    termTwo=termTwo+HessianArray(i).D*model(1).L*model(1).S(:,i)*model(1).S(:,i)';
    termThree=termThree+2*model(1).mu_one/2*model(1).L;
end

der=1./numTasks*(-2*termOne+2*termTwo+termThree);

disp('der')
disp(der)
disp('model(1).L')
disp(model(1).L)

model(1).L=model(1).L+model(1).learningrate*der; 



