clear all;
close all;

%Fs = 1000000;       % Sampling frequency in Hz
%T = 0.8;           % Total time duration of the chirp in seconds
%t = 0:1/Fs:T-1/Fs; % Time vector

T = readtable('measures/data_enc_pos_timc.csv');
arry=table2array(T);
pos = arry(:,1);
time = arry(:,2);

take_every_n_smaple= 25;
pos=downsample(pos,take_every_n_smaple);
time=downsample(time,take_every_n_smaple);

vel=zeros(size(pos));
for i=2:size(pos)
    vel_temp= (pos(i)-pos(i-1))/(time(i)-time(i-1));
    vel(i)=vel_temp;
end

vel_filtered= movmean(vel,50);
%vel_filtered=vel

% Construct an FDESIGN object and call its CHEBY2 method.




subplot(2,1,1);
plot(time,pos)
xlabel('time');
ylabel('pos');
title('Pos to time');
grid on;


subplot(2,1,2);
%plot(time,vel_filtered,'r')
plot(time,vel_filtered,'r',time,vel,'b')
xlabel('time');
ylabel('rad/s');
legend('vel_filtered','vel')
title('velopcity to time');
grid on;


