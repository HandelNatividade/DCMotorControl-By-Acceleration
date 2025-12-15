close all
clear all
clc
%% train data (PRBS)
data=load(['prbs-completo.txt']);
Ts = mean(diff(data(:,1))); % Calcula a média do intervalo em segundos
N=size(data,1);
u_train=detrend(data(:,2));
y_train=detrend(data(:,3)); 
data_train=iddata(y_train,u_train,Ts);
td_train=0:Ts:(N-1)*Ts;
%% estima FT
Options = tfestOptions;
Gident = tfest(data_train, 3, 2, Options,'Ts',Ts)
yident=lsim(Gident,u_train,td_train);
figure
stairs(td_train,y_train,'linewidth',2,DisplayName='Real')
hold on
stairs(td_train,yident,'linewidth',2,DisplayName='Identificado')
legend
title('Train Dataset')
grid on
save 'Gident' Gident Ts
%%
%pidTuner(Gident,'PD')