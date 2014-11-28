function [model]=initPGELLA(dimTasks,dimLatent,pg_Rate,reg_Rate,numTasks)
% Documentation stuff 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ELLA-Params
model.L=zeros(dimTasks,dimLatent);
model.A=zeros(dimLatent*dimTasks,dimLatent*dimTasks);
model.b=zeros(dimLatent*dimTasks,1);
model.T=0; 
model.mu=reg_Rate; 
model.S=zeros(dimLatent,numTasks);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PG params
% PGparam.learning