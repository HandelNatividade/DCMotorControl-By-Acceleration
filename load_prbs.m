%%
data=load(['prbs-completo.txt']);
Ts = mean(diff(data(:,1))); % Calcula a média do intervalo em segundos
N=size(data,1);
%figure,plot(diff(data(:,1))/1000)
Ts2=floor(mean(diff(data(:,1))))/1000 %get sampling time
if Ts2~=Ts
    warning('samples were lost during acquisition! Ts is not consistent.')
end
N=size(data,1);
time=0:Ts:(N-1)*Ts;
%%
u=data(:,2);
y=data(:,3);
%%
figure
subplot(2,1,1)
hold on
stairs(time,y,DisplayName='Output',linewidth=1.5);
xlabel('Tempo [s]')
ylabel('Potência Média do Sinal [W]')
legend
legend('location','best')
title('Acquired Data')
subplot(2,1,2)
stairs(time,u,DisplayName='Input',linewidth=1.5,color='r');
xlabel('Tempo [s]')
ylabel('PWM [duty]')
