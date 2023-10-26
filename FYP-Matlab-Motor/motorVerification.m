clc
clear all

%% SYS_ID_DATA_LOAD.mat

%% Load motor data
% motorData = 'SYS_ID_VALIDATION7.mat';
% motorData = 'SYS_ID_VALIDATION8.mat';
% motorData = 'SYS_ID_VALIDATION9.mat';
motorData = 'SYS_ID_VALIDATION10.mat';
% motorData = 'SYS_ID_VALIDATION11.mat';
% motorData = 'SYS_ID_VALIDATION12.mat';
% motorData = 'SYS_ID_VALIDATION13.mat';
% motorData = 'SYS_ID_VALIDATION14.mat';
% motorData = 'SYS_ID_VALIDATION15.mat';
% motorData = 'SYS_ID_VALIDATION16.mat';
% motorData = 'SYS_ID_VALIDATION17.mat';


[time,current,velocity,Vin,torqueInterp] = motorDataUnpacker2(motorData);

load(motorData);

Td = SysIDData.Td;
Td = Td(:,1:length(time))';
omega = SysIDData.omega;
omega = omega(1:length(time),:);

%% Data Plotting
fig1 = figure;
set(fig1,'Position', [100,100,800,400])
plot(time,Td,'linewidth',2);
hold on
grid on
plot(time,torqueInterp,'linewidth',2);
% plot(time,velocity);
%plot(time,Vin);
title('Torque Reference Verification')
xlabel('Time (s)')
ylabel('Torque (Nm)')
legend('Torque Refernce','Torque Measured')

saveas(fig1,'trefValidation.png')