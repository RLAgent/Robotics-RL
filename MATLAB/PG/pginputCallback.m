function pginputCallback (message)
    % Collect trajectories, and compute new policy when episode
    % ends.
    
    global States1;
    global States2;
    global Rewards;
    global Actions1;
    global Actions2;
    global Policy1;
    global Policy2;
    global Param;
    global LearningRate1;
    global LearningRate2;
    global numEpisodes;
    global numEpisodesTilUpdate;
    global numLearningRates;
    global PGOutputPub;
    global LinearVelPolicyMsg;
    global TranslationMsg;
    global RotationMsg;
    
    translation = message.getTranslation();
    rotation = message.getRotation();
    
    Actions1(1,end+1) = translation.getX();  % linear_vel
    Actions2(1,end+1) = translation.getY();  % angular_vel
    Rewards(1,end+1) = translation.getZ();   % reward
    
    % linear_vel = k1*d + k2*|a|
    States1(1,end+1) = rotation.getX();
    States1(2,end) = abs(rotation.getY());
    
    % angular_vel = k1/d + k2*a
    States2(1,end+1) = rotation.getX();
    States2(2,end) = rotation.getY();;
    
    if (rotation.getW() == 1)
        % Terminal state
        
        numEpisodes = numEpisodes+1;
        
        if (numEpisodes == numEpisodesTilUpdate)
            % Update policies
            numEpisodes = 0;
            
            data1.u = Actions1(2:end);
            data2.u = Actions2(2:end);
            data1.r = Rewards(2:end);
            data2.r = Rewards(2:end);

            data1.x = States1(:, 2:end);
            data2.x = States2(:, 2:end);

            for i=1:numLearningRates
                [dJdtheta]=episodicREINFORCE(Policy1(i), data1, Param);
                Policy1(i).theta.k = Policy1(i).theta.k + LearningRate1(i) * dJdtheta(1:Param.N,1);
                Policy1(i).theta.sigma = Policy1(i).theta.sigma + LearningRate1(i)*dJdtheta(Param.N+1,1) * Policy1(i).theta.sigma^2;

                [dJdtheta]=episodicREINFORCE(Policy2(i), data2, Param);
                Policy2(i).theta.k = Policy2(i).theta.k + LearningRate2(i) * dJdtheta(1:Param.N,1);
                Policy2(i).theta.sigma = Policy2(i).theta.sigma + LearningRate2(i)*dJdtheta(Param.N+1,1) * Policy2(i).theta.sigma^2;
            end

            % publish to ROS; send new policy
            %translation = rosmatlab.message('geometry_msgs/Vector3', node);
            %rotation = rosmatlab.message('geometry_msgs/Quaternion', node);

            for i=1:numLearningRates

                TranslationMsg.setX(Policy1(i).theta.sigma);
                TranslationMsg.setY(i);
                TranslationMsg.setZ(0); % linear_vel: msg.translation.z = 0
                RotationMsg.setX(Policy1(i).theta.k(1,:));
                RotationMsg.setY(Policy1(i).theta.k(2,:));
                LinearVelPolicyMsg.setRotation(RotationMsg);
                LinearVelPolicyMsg.setTranslation(TranslationMsg);
                PGOutputPub.publish(LinearVelPolicyMsg);

                TranslationMsg.setX(Policy2(i).theta.sigma);
                TranslationMsg.setY(i);
                TranslationMsg.setZ(1);  % angular_vel: msg.translation.z = 1
                RotationMsg.setX(Policy2(i).theta.k(1,:));
                RotationMsg.setY(Policy2(i).theta.k(2,:));
                LinearVelPolicyMsg.setRotation(RotationMsg);
                LinearVelPolicyMsg.setTranslation(TranslationMsg);
                PGOutputPub.publish(LinearVelPolicyMsg);
                
                if (i == 3)
                    disp(Policy1(i).theta.k)
                elseif (i== 2)
                    disp(Policy2(i).theta.k)
                end
                
            end

            % Clear trajectory
            States1 = zeros(2,1);
            States2 = zeros(2,1);
            Actions1 = 0;
            Actions2 = 0;
            Rewards = 0;
        end
    end
    
    
end
