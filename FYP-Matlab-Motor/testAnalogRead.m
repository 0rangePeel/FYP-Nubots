clear
clc

if ~isempty(instrfind('Status', 'open'))
    fclose(instrfind);
end

stm_device = serialport('COM4', 115200, 'Timeout', 0.5);

writeline(stm_device, "logSysID");

N = 2000;

SysIDData.time = zeros(N,1);
SysIDData.voltage1 = zeros(N,1);
SysIDData.voltage2 = zeros(N,1);

for i=1:N
    
    rxStr = readline(stm_device);
    
    seperatedString = split(rxStr,',');
    
    SysIDData.time(i) = str2double(seperatedString{1});
    SysIDData.voltage1(i) = str2double(seperatedString{2});
    SysIDData.voltage2(i) = str2double(seperatedString{3});
    
end

plot(SysIDData.time, SysIDData.voltage1, '-')
grid on
hold on
plot(SysIDData.time, SysIDData.voltage2, '-')
xlabel('Time')
ylabel('Voltage')
legend('PB0', 'PA4')

clear stm_device