function der = DlogPiDTheta(policy, x,u,param)

    %global N M
    N=param.N; 
    M=param.M; 
    k=zeros(N,1);
    xx=zeros(N,1);
    der=zeros(N,1);
    der(N+1,1)=0;
    %if(policy.type == 1)
    % der = zeros(N*(M-1),1);    
     %if(u==M)
     %    actions = selDecBor(x,1):selDecBor(x,M-1);
     %    der(actions,1) = - 1 / (1 - sum(policy.theta(actions)));
     % else
     %    der(selDecBor(x,u),1) = 1 / policy.theta(selDecBor(x,u));
     % end;   
   %elseif(policy.type == 2) 
   %   der = zeros(N*M,1);
   %   for i=1:M
   %      der(selGibbs(x,i),1) = -pi_theta(policy, x, i)/policy.eps;
   %   end;   
   %  der(selGibbs(x,u)) = (1 - pi_theta(policy, x, u))/policy.eps;
   %if(policy.type == 3) 
      
      sigma = max(policy.theta.sigma,0.00001);
      %tempTheta=policy.theta.k; 
      %for l=1:length(tempTheta)
      %    k(l,:)=tempTheta(l,:);
      %   xx(l,:)=x(l,:);
      %end
      k(1:N,1)   = policy.theta.k(1:N);
      xx(1:N,1)  = x(1:N); 
      der(1:N,1) = (u-k'*xx)*xx/(sigma^2);
      der(N+1,1) = ((u-k'*xx)^2-sigma^2)/(sigma^3);      
   %end;   
