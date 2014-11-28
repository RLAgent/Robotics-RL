function pginitpolicyCallback (message)
    % Set initial theta.

    global Policy1;
    global Policy2;

    translation = message.getTranslation();
    rotation = message.getRotation();

    numPolicy = translation.getY();
    if (numPolicy == 0)
        return;
    end

    if (translation.getZ() == 0)
        Policy1(numPolicy).theta.k = transpose([rotation.getX(), rotation.getY()])
        Policy1(numPolicy).theta.sigma = translation.getX();
    else
        Policy2(numPolicy).theta.k = transpose([rotation.getX(), rotation.getY()])
        Policy2(numPolicy).theta.sigma = translation.getX();
    end

    numPolicy
    Policy1(numPolicy).theta.k
    Policy2(numPolicy).theta.k

end
