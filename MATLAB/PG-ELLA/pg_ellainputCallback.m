function pg_ellainputCallback (message)
    % Collect trajectories, and compute new policy when episode
    % ends.
    
    global States1;
    global States2;
    global Rewards;
    global Actions1;
    global Actions2;
    global numEpisodes;
    global numEpisodesTilUpdate;
    global numLearningRates;
    global numTasks;
    
    global model1;
    global model2;
    global taskIndex_;
    global pg_Param1;
    global pg_Param2;
    %global policies;
    global policies1;
    global policies2;
    global HessianArray;
    global ParameterArray1;
    global ParameterArray2;
    
    global nextTaskExists;
    
    if (nextTaskExists)
    
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
        States2(2,end) = rotation.getY();

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

                % Compute new policy
                [model1, HessianArray, ParameterArray1]=updatePGELLA(data1, model1, taskIndex_, pg_Param1, policies1, HessianArray, ParameterArray1);                   
                [model2, HessianArray, ParameterArray2]=updatePGELLA(data2, model2, taskIndex_, pg_Param2, policies2, HessianArray, ParameterArray2);

                % Change to next task
                taskIndex_ = taskIndex_+1;

                if (taskIndex_ < numTasks)
                    sendNextTask(taskIndex_);
                else
                    sendPolicies();
                    nextTaskExists = false;
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
end
