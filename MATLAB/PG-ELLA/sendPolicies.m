function sendPolicies()
    global PGOutputPub;
    global LinearVelPolicyMsg;
    global TranslationMsg;
    global RotationMsg;
    global numTasks;
    global model1;
    global model2;
    
    
    for taskIndex=1:model1(1).T
        [policy1]=performActionPGELLA(model1, taskIndex);
        [policy2]=performActionPGELLA(model2, taskIndex);

        % publish to ROS; send new policy
        TranslationMsg.setX(policy1.theta.sigma);
        TranslationMsg.setY(taskIndex);
        TranslationMsg.setZ(0); % linear_vel: msg.translation.z = 0
        RotationMsg.setX(policy1.theta.k(1,:));
        RotationMsg.setY(policy1.theta.k(2,:));
        LinearVelPolicyMsg.setRotation(RotationMsg);
        LinearVelPolicyMsg.setTranslation(TranslationMsg);
        PGOutputPub.publish(LinearVelPolicyMsg);

        TranslationMsg.setX(policy2.theta.sigma);
        TranslationMsg.setY(taskIndex);
        TranslationMsg.setZ(1); % angular_vel: msg.translation.z = 1
        RotationMsg.setX(policy2.theta.k(1,:));
        RotationMsg.setY(policy2.theta.k(2,:));
        LinearVelPolicyMsg.setRotation(RotationMsg);
        LinearVelPolicyMsg.setTranslation(TranslationMsg);
        PGOutputPub.publish(LinearVelPolicyMsg);
    end
    
    disp(['Task # = ', model1(1).T])
    
    disp('model1.L')
    disp(model1.L)
    disp('model1.S')
    disp(model1.S)
    
    disp('model2.L')
    disp(model2.L)
    disp('model2.S')
    disp(model2.S)
    
end