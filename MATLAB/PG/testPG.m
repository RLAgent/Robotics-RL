
% Main
h.States = zeros(3,1);
h.Rewards = 0;
h.Actions1 = 0;
h.Actions2 = 0;

h.Policy1.theta.k = zeros(3,1); 
h.Policy1.theta.sigma = rand();
h.Policy1.type = 3;

h.Policy2.theta.sigma = 0;
h.Policy2.theta.k = zeros(3,1); 
h.Policy2.type = 3;

h.Param.N = 3;
h.Param.M = 1;
h.Param.gamma = 0.9;

h.LearningRate = 0.1;

% pginputCallback
for i=1:10
    h.Actions1(1,end+1)=i;
    h.Actions2(1,end+1)=i;
    
    h.Rewards(1,end+1)=i;
    
    h.States(1,end+1) = i;
    h.States(2,end) = i;
    h.States(3,end) = i;
    %h.States(4,end) = 1;
end

% Terminal state
data1.u = h.Actions1(2:end);
data2.u = h.Actions2(2:end);
data1.r = h.Rewards(2:end);
data2.u = h.Rewards(2:end);

data1.x = h.States(:, 2:end);
data2.x = h.States(:, 2:end);

[dJdtheta]=episodicREINFORCE(h.Policy1, data1, h.Param);
h.Policy1.theta.k = h.Policy1.theta.k + h.LearningRate * dJdtheta(1:h.Param.N,1);
h.Policy1.theta.sigma = h.Policy1.theta.sigma + h.LearningRate*dJdtheta(h.Param.N+1,1) * h.Policy1.theta.sigma^2;

h.Policy1.theta.k
h.Policy1.theta.sigma

for i=1:size(data,2)
end
%data.x=rand(4,10);
%data.u=rand(1,10);
%data.r=rand(1,10);
%policy.theta.k(1) = 2;

%param.N = 1;
%param.M = 4;

%param.gamma = 0.9;

%[dJdtheta]=episodicREINFORCE(policy,data,param);

%policy.theta.k=policy.theta.k+learningrate*dJdtheta(1:param.N,1);
%policy.theta.sigma=policy.theta.sigma+learningrate*dJdtheta(param.N+1,1)*policy.theta.sigma^2;


%DlogPiDTheta(policy,1,1,param)

%numIterations=100;
%for i=1:numIterations
  
%[dJdtheta]=episodicREINFORCE(policy,data,param);
%policy.theta.k=policy.theta.k+learningrate*dJdtheta(1:param.N,1);
%policy.theta.sigma=policy.theta.sigma+learningrate*dJdtheta(param.N+1,1)*policy.theta.sigma^2;

%end