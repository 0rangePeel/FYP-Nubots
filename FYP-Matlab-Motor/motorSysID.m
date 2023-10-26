clc 
clear all

%% SYS_ID_DATA_LOAD.mat
% 
% phi = 33.8391
% 
% Kw = 2.5159
%


%% Load motor data
load('SYS_ID_DATA_LOAD3.mat')

%% Unpack Data
timeDynamixel   = SysIDData.timeDynamixel;
currentRAW      = SysIDData.current;
velocityRAW     = SysIDData.velocity;
pwmRAW          = SysIDData.pwm;

timeSTM         = SysIDData.timeSTM;
voltage1RAW     = SysIDData.voltage1;
voltage2RAW     = SysIDData.voltage2;

%% Perform Necessary Conversions
currentUnit = 3.36; % mA
current = (currentRAW * currentUnit) / 1000;

velocityUnit = 0.229; % rpm
velocity = velocityRAW * velocityUnit * (pi / 30);

pwmUnit = 0.113; % Unit Percentage
pwm = (pwmRAW * pwmUnit)/100;

Vbat = 12.0;
Vin = pwm * Vbat;

Vsen = 0.3577;
Voffset = 1.6337;
Voffset = 1.6548;
torque = Vsen * (voltage1RAW - Voffset);

%% Interpolate Data

% Create an interpolation function
interpolation_function = @(x) interp1(1:numel(x), x, linspace(1, numel(x), length(timeSTM)), 'linear');
x = 0;
i = 1;
while x < 10
    x = timeDynamixel(1,i);
    i = i+1;
end

% Interpolate the data to the new size
timeInterp = interpolation_function([timeDynamixel(1:(i-2)) 10]);

% Interpolate Current
currentInterp = interpolation_function(current(1:(i-1)));

% Interpolate Velocity
velocityInterp = interpolation_function(velocity(1:(i-1)));

% Interpolate Vin
VinInterp = interpolation_function(Vin(1:(i-1)));

%% Plotting
figure(1);
plot(timeSTM,torque);
hold on
plot(timeDynamixel,current,'*')
hold on
plot(timeInterp,currentInterp);
legend('Torque','Current','CurrentInterp')

figure(2)
plot(timeDynamixel,velocity,'*');
hold on
plot(timeInterp,velocityInterp);
hold on
plot(timeDynamixel,Vin,'*')
hold on
plot(timeInterp,VinInterp)
legend('Velocity','VelocityInterp','Vin','VinInterp')

%% Least Squares Method - Torque

% Gear Ratio
N = 225;

num = [torque/N velocityInterp.'];

[num2] = lsqr(num,VinInterp.');

phi = num2(1) % phi = Ra * (1 / K_I)
Kw = num2(2)

% phi = 307.0052;
% Kw = 2.31;

% phi = 561.2594;
% Kw = 2.3819;

figure(3)
hold on
grid on
plot(timeInterp,VinInterp)
plot(timeSTM,phi*(torque/N) + Kw*velocityInterp.','+')
legend('Vin','Calculation')

%% Least Squares Method - Current

% num = [currentInterp.' velocityInterp.'];
% 
% [num2] = lsqr(num,VinInterp.');
% 
% Ra = num2(1)
% Kw = num2(2)
% 
% % Ra = 678.4393;
% % Kw = 2.3934;
% 
% figure(4)
% hold on
% grid on
% plot(timeSTM,VinInterp)
% plot(timeSTM,Ra*currentInterp.' + Kw*velocityInterp.','+')
