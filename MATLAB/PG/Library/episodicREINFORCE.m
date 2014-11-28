function [dJdtheta]= episodicREINFORCE(policy, data, param)

   N=param.N; 
   gamma=param.gamma;
   %dJdtheta = zeros(size(DlogPiDTheta(policy,1,1,param)));
   dJdtheta = zeros(size(DlogPiDTheta(policy,ones(N,1),ones(param.M,1),param))); 
   
	for Trials = 1 : max(size(data))
		%dSumPi = zeros(size(DlogPiDTheta(policy,1,1,param)));
        dSumPi = zeros(size(DlogPiDTheta(policy,ones(N,1),ones(param.M,1),param))); 
        sumR   = 0;
   
      for Steps = 1 : max(size(data(Trials).u))
         dSumPi = dSumPi + DlogPiDTheta(policy, data(Trials).x(:,Steps), data(Trials).u(Steps),param);
         sumR   = sumR   + gamma^(Steps-1)*data(Trials).r(Steps);
      end; 
      
      dJdtheta = dJdtheta + dSumPi * sumR;
   end;
   
   
   if(gamma==1)
      dJdtheta = dJdtheta / i;
	else      
      dJdtheta = (1-gamma)*dJdtheta / max(size(data));
	end;    