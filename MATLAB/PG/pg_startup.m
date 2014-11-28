global LinearVelPolicyMsg;
global AngularVelPolicyMsg;
global States1;
global States2;
global Actions1;
global Actions2;
global Rewards;
global Policy1;
global Policy2;
global Param;
global LearningRate1;
global LearningRate2;
global numLearningRates;
global numEpisodesTilUpdate;
global numEpisodes;
global PGOutputPub;
global TranslationMsg;
global RotationMsg;

rosMasterIp = 'localhost';
localhostIp = '127.0.1.1';
node = rosmatlab.node('MATLAB_PG', rosMasterIp, 11311, 'rosIP', localhostIp);

numEpisodesTilUpdate = 1;
numEpisodes = 0;

% Publishers
PGOutputPub = node.addPublisher('/pg_output', 'geometry_msgs/Transform');

% Create a message.
LinearVelPolicyMsg = rosmatlab.message('geometry_msgs/Transform', node);
AngularVelPolicyMsg = rosmatlab.message('geometry_msgs/Transform', node);
TranslationMsg = rosmatlab.message('geometry_msgs/Vector3', node);
RotationMsg = rosmatlab.message('geometry_msgs/Quaternion', node);

% Learning rates
LearningRate1 = [0.00000001, 0.0000001, 0.000001, 0.00001];%, 0.0001, 0.001];
LearningRate2 = [0.00000001, 0.0000001, 0.000001, 0.00001];%, 0.0001, 0.001];
numLearningRates = length(LearningRate2);

for i=1:numLearningRates
    Policy1(i).theta.k = zeros(2,1);
    Policy1(i).theta.sigma = rand();
    Policy1(i).type = 3;

    Policy2(i).theta.k = zeros(2,1);
    Policy2(i).theta.sigma = rand();
    Policy2(i).type = 3;
end

% Parameters for episodicREINFORCE
Param.N = 2;
Param.M = 1;
Param.gamma = 0.9;

States1 = zeros(2,1);
States2 = zeros(2,1);
Rewards = 0;
Actions1 = 0;
Actions2 = 0;

% Subscribers
PGInputSub = node.addSubscriber('/pg_input', 'geometry_msgs/Transform', 1);
PGInputSub.setOnNewMessageListeners({@pginputCallback});

PGInitPolicySub = node.addSubscriber('/pg_init_policy', 'geometry_msgs/Transform', 50);
PGInitPolicySub.setOnNewMessageListeners({@pginitpolicyCallback});

%node.setOnShutdownListener(@preShutdownTask);
%node.setOnShutdownCompleteListener(@postShutdownTask);

% To shutdown: node.Node.shutdown();
