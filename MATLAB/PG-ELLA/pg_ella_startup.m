global LinearVelPolicyMsg;
global AngularVelPolicyMsg;
global States1;
global States2;
global Actions1;
global Actions2;
global Rewards;
global numLearningRates;

global numEpisodes;
global numEpisodesTilUpdate;

global numTrajectories;
global numTrajectoriesTilUpdate;

global taskIndex_;
global numTasks;
global dimStateSpace; % dimension of state space

global model1;
global model2;
global pg_Param1;
global pg_Param2;
global policies1;
global policies2;
global HessianArray;
global ParameterArray1;
global ParameterArray2;

global PGOutputPub;
global PGEllaTaskIndexPub;
global TranslationMsg;
global RotationMsg;
global Int16Msg;
global nextTaskExists;

rosMasterIp = 'localhost';
localhostIp = '127.0.1.1';
node = rosmatlab.node('matlab_pg_ella', rosMasterIp, 11311, 'rosIP', localhostIp);

K=2;
numEpisodesTilUpdate = 100; % Length of one trajectory
numEpisodes = 0;
numTasks = 6;
dimStateSpace = 2;
nextTaskExists = true;

numTrajectories = 0;
numTrajectoriesTilUpdate = 10;

% Create a message.
LinearVelPolicyMsg = rosmatlab.message('geometry_msgs/Transform', node);
AngularVelPolicyMsg = rosmatlab.message('geometry_msgs/Transform', node);
TranslationMsg = rosmatlab.message('geometry_msgs/Vector3', node);
RotationMsg = rosmatlab.message('geometry_msgs/Quaternion', node);
Int16Msg = rosmatlab.message('std_msgs/Int16', node);

% Model
model1.L=rand(dimStateSpace,K); % dxk, where d=2, k=1
model1.S=rand(K,numTasks);%zeros(K,numTasks);%rand(K,numTasks);
model1.mu_one = 0.001;
model1.T = 0; % taskIndex
model1.learningrate =0.0000000001;%0.000000000001;

model2.L=rand(dimStateSpace,K); % dxk, where d=2, k=1
model2.S=rand(K,numTasks);%zeros(K,numTasks);%rand(K,numTasks);
model2.mu_one = 0.001;
model2.T = 0; % taskIndex
model2.learningrate = 0.000000000001;%0.0000000000001;

% Policies
for taskIndex=1:numTasks
    pg_Param1(taskIndex).param.N=dimStateSpace;
    pg_Param1(taskIndex).param.M=1;
    pg_Param1(taskIndex).param.gamma = 0.9;
    
    pg_Param2(taskIndex).param.N=dimStateSpace;
    pg_Param2(taskIndex).param.M=1;
    pg_Param2(taskIndex).param.gamma = 0.9;

    policies1(taskIndex).policy.theta.k = rand(dimStateSpace,1)-.5;
    policies2(taskIndex).policy.theta.k = rand(dimStateSpace,1)-.5;    
    policies1(taskIndex).policy.theta.sigma = rand();
    policies1(taskIndex).policy.type = 3;
    policies2(taskIndex).policy.theta.sigma = rand();
    policies2(taskIndex).policy.type = 3;
    
    HessianArray(taskIndex).D = eye(2,2);
    ParameterArray1(taskIndex).alpha = zeros(2,1);
    ParameterArray2(taskIndex).apha = zeros(2,1);
end

disp('policies1(1:5).policy.theta.k')
disp(horzcat(policies1(1).policy.theta.k, policies1(2).policy.theta.k,policies1(3).policy.theta.k, policies1(4).policy.theta.k, policies1(5).policy.theta.k));
disp('policies2(1:5).policy.theta.k')
disp(horzcat(policies2(1).policy.theta.k, policies2(2).policy.theta.k,policies2(3).policy.theta.k,policies2(4).policy.theta.k,policies2(5).policy.theta.k));


pg_Param1(1).learningrate = 0.000001;
pg_Param2(1).learningrate = 0.0000001;
pg_Param1(2).learningrate = 0.000001;
pg_Param2(2).learningrate = 0.000001;
pg_Param1(3).learningrate = 0.0000001;
pg_Param2(3).learningrate = 0.000001;
pg_Param1(4).learningrate = 0.0000001;
pg_Param2(4).learningrate = 0.00001;
pg_Param1(5).learningrate = 0.0000001;
pg_Param2(5).learningrate = 0.00001;
pg_Param1(6).learningrate = 0.000001;
pg_Param2(6).learningrate = 0.000001;

States1 = zeros(2,1);
States2 = zeros(2,1);
Rewards = 0;
Actions1 = 0;
Actions2 = 0;


% Subscribers
PGInputSub = node.addSubscriber('/pg_ella_input', 'geometry_msgs/Transform', 1);
PGInputSub.setOnNewMessageListeners({@pg_ellainputCallback2});

% Publisher
PGOutputPub = node.addPublisher('/pg_ella_output', 'geometry_msgs/Transform');
PGEllaTaskIndexPub = node.addPublisher('/pg_ella_task_index', 'std_msgs/Int16');

%PGInitPolicySub = node.addSubscriber('/pg_ella_init_policy', 'geometry_msgs/Transform', 50);
%PGInitPolicySub.setOnNewMessageListeners({@pg_ellainitpolicyCallback});

taskIndex_ = 1;
sendNextTask(taskIndex_);

% To shutdown: node.Node.shutdown();
