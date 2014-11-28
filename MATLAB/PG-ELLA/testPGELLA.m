data.x=rand(2,10);
data.u=rand(1,10); 
data.r=rand(1,10); 

K=1;
T=10; % number of tasks
model.L=rand(2,K); % dxk, where d=2, k=1
model.S=rand(K,T);
model.mu_one = 0.001;
model.T = 0; % number of tasks
model.learningrate = 0.1;

for i=1:10
    pg_Param(i).learningrate = 0.00001;
    pg_Param(i).param.N=2;
    pg_Param(i).param.M=1;
    pg_Param(i).param.gamma = 0.9;

    policies(i).policy.theta.k = rand(2,1);
    policies(i).policy.theta.sigma = rand();
    policies(i).policy.type = 3;
    HessianArray(i).D=eye(2,2); 
    ParameterArray(i).alpha=zeros(2,1);
end

for taskIndex=1:10
    [model, HessianArray, ParameterArray]=updatePGELLA(data, model, taskIndex ,pg_Param, policies, HessianArray, ParameterArray)
end

model(1).L 
model.S(:,2)

[policy]=performActionPGELLA(model,1)