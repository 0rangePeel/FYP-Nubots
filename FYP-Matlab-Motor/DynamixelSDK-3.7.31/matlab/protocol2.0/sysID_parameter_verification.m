clc;
clear all;

%% Setup Dynamixel Serial

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_bulk_read.h', 'addheader', 'group_bulk_write.h');
end

% Control table address
ADDR_OPERATING_MODE      = 11;
ADDR_TORQUE_ENABLE       = 64;         % Control table address is different in Dynamixel model
ADDR_GOAL_PWM            = 100;
ADDR_PRESENT_PWM         = 124;
ADDR_PRESENT_CURRENT     = 126;
ADDR_PRESENT_VELOCITY    = 128;
ADDR_PRESENT_POSITION    = 132;

ADDR_START = ADDR_PRESENT_PWM;

% Data Byte Length
LEN_GOAL_PWM                = 2;
LEN_PRESENT_PWM             = 2;
LEN_PRESENT_CURRENT         = 2;
LEN_PRESENT_VELOCITY        = 4;
LEN_PRESENT_POSITION        = 4;

LEN_SUM = LEN_PRESENT_PWM  + ...
          LEN_PRESENT_CURRENT + ... 
          LEN_PRESENT_VELOCITY + ...
          LEN_PRESENT_POSITION;

% Protocol version
PROTOCOL_VERSION                = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL_ID                      = 23;            % Dynamixel ID: 1
BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM6';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'

OPERATING_MODE              = 16;                                            
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_PWM_VALUE       = 25;      % Dynamixel will rotate between this value
DXL_MAXIMUM_PWM_VALUE       = 50;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 2;           % Dynamixel moving status threshold

ESC_CHARACTER                   = 'e';          % Key for escaping loop

COMM_SUCCESS                    = 0;            % Communication Success result value
COMM_TX_FAIL                    = -1001;        % Communication Tx Failed

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

% Initialize groupBulkWrite Struct
groupwrite_num = groupBulkWrite(port_num, PROTOCOL_VERSION);

% Initialize Groupbulkread Structs
groupread_num = groupBulkRead(port_num, PROTOCOL_VERSION);

index = 1;
dxl_comm_result = COMM_TX_FAIL;                 % Communication result
dxl_addparam_result = false;                    % AddParam result
dxl_getdata_result = false;                     % GetParam result
%dxl_goal_pwm = [DXL_MINIMUM_PWM_VALUE DXL_MAXIMUM_PWM_VALUE];         % Goal pwm

t = 10;      % Total Duration (s)
% f = 63;     % Sampling Frequency (Hz) - DO NOT ADJUST
f = 56;     % Sampling Frequency (Hz) - DO NOT ADJUST
p = 4;      % Number of Periods
Unit = 0.113; % Unit Percentage
A = 0.1;     % Amplitude
offset = A*0; % Offset from 0
% dxl_goal_pwm = -A*(cos(linspace(0, p*2*pi, t*f))) + offset;
% dxl_goal_pwm = linspace(10/Unit,10/Unit,t*f);
% dxl_goal_pwm = A*(sin(linspace(0, p*2*pi, t*f))) + offset;
% dxl_goal_pwm = -A*ones(1,t*f);

dxl_goal_pwm = zeros(1,t*f);

Td = [zeros(1,t*f*0.2) A*(sin(linspace(0, p*2*pi, t*f*0.8))) + offset];

phi = 307.0052;
Kw = 2.31;
gearRatio = 10;

% plot(linspace(0,10,t*f),Td)


dxl_error = 0;                                  % Dynamixel error
dxl_present_pwm      = 0;                       % Present pwm
dxl_present_current  = 0;                       % Present current
dxl_present_velocity = 0;                       % Present velocity
dxl_present_position = 0;                       % Present position

% Open port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% Set Operating Mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, OPERATING_MODE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel : Operating Mode set to PWM \n');
end

% Enable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel : Torque Enabled \n');
end

% Add parameter storage for Dynamixel present velocity + present position value
dxl_addparam_result = groupBulkReadAddParam(groupread_num, DXL_ID, ADDR_START, LEN_SUM);
if dxl_addparam_result ~= true
    fprintf('[ID:%03d] groupBulkRead addparam failed', DXL_ID);
    return;
end

N = 2000;
SysIDData.timeSTM = zeros(N,1);
SysIDData.voltage1 = zeros(N,1);
SysIDData.voltage2 = zeros(N,1);
SysIDData.timeDynamixel = linspace(0,10,length(dxl_goal_pwm));
SysIDData.pwm = zeros(length(dxl_goal_pwm),1);
SysIDData.current = zeros(length(dxl_goal_pwm),1);
SysIDData.velocity = zeros(length(dxl_goal_pwm),1);
SysIDData.position =  zeros(length(dxl_goal_pwm),1);

%% Setup STM32 Serial
if ~isempty(instrfind('Status', 'open'))
    fclose(instrfind);
end

stm_device = serialport('COM4', 115200, 'Timeout', 0.5);

writeline(stm_device, "logSysID");

%% Begin Serial Communnication

tic

while 1

    if (SysIDData.velocity(index) > (2^31))
        SysIDData.velocity(index) = SysIDData.velocity(index) - (2^32);
    end

    omega_i = SysIDData.velocity(index) * 0.229 * (pi / 30);


    dxl_goal_pwm(index) = ((phi*(Td(index)) + Kw*omega_i) / Unit);


    % Add parameter storage for Dynamixel goal position
    dxl_addparam_result = groupBulkWriteAddParam(groupwrite_num, DXL_ID, ADDR_GOAL_PWM, LEN_GOAL_PWM, typecast(int16(dxl_goal_pwm(index)), 'uint16'), LEN_GOAL_PWM);
    if dxl_addparam_result ~= true
      fprintf(stderr, '[ID:%03d] groupBulkWrite addparam failed', DXL_ID);
      return;
    end

    % Bulkwrite goal pwm
    groupBulkWriteTxPacket(groupwrite_num);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    end

    % Clear bulkwrite parameter storage
    groupBulkWriteClearParam(groupwrite_num);

    while 1
        % Bulkread present position and velocity
        groupBulkReadTxRxPacket(groupread_num);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        end

        % Check if groupbulkread data of Dynamixel is available
        dxl_getdata_result = groupBulkReadIsAvailable(groupread_num, DXL_ID, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION);
        if dxl_getdata_result ~= true
            fprintf('[ID:%03d] groupBulkRead getdata failed - PRESENT POSITION', DXL_ID);
            return;
        end

        % Get Dynamixel present pwm value
        dxl_present_pwm = groupBulkReadGetData(groupread_num, DXL_ID, ADDR_PRESENT_PWM, LEN_PRESENT_PWM);

        % Get Dynamixel present current value
        dxl_present_current = groupBulkReadGetData(groupread_num, DXL_ID, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);

        % Get Dynamixel present velocity value
        dxl_present_velocity = groupBulkReadGetData(groupread_num, DXL_ID, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);

        % Get Dynamixel present position value
        dxl_present_position = groupBulkReadGetData(groupread_num, DXL_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

%         fprintf('[ID:%03d] Present Position : %d \t Present Velocity: %d \t Present PWM: %d \t Present Current: %d\n', DXL_ID, ...
%                                                                               typecast(uint32(dxl_present_position), 'int32'), ...
%                                                                               typecast(uint32(dxl_present_velocity), 'int32'), ...
%                                                                               typecast(uint16(dxl_present_pwm), 'int16'), ...
%                                                                               typecast(uint16(dxl_present_current), 'int16'));
        
        if ~(abs(dxl_goal_pwm(index)) - abs(dxl_present_pwm) > DXL_MOVING_STATUS_THRESHOLD);
            break;
        end
    end

    % Store Dynamixel Values
    SysIDData.pwm(index) = dxl_present_pwm;
    SysIDData.current(index) = dxl_present_current;
    SysIDData.velocity(index) = dxl_present_velocity;
    SysIDData.position(index) = dxl_present_position;

    index = index + 1;

    if index > length(dxl_goal_pwm)
        break;
    end
end
toc
for i=1:N
    
    rxStr = readline(stm_device);
    
    seperatedString = split(rxStr,',');
    
    SysIDData.timeSTM(i) = str2double(seperatedString{1});
    SysIDData.voltage1(i) = str2double(seperatedString{2});
    SysIDData.voltage2(i) = str2double(seperatedString{3});
    
end

%% Set PWM to 0 to stop motor
% Add parameter storage for Dynamixel goal position
dxl_addparam_result = groupBulkWriteAddParam(groupwrite_num, DXL_ID, ADDR_GOAL_PWM, LEN_GOAL_PWM, typecast(int16(0), 'uint16'), LEN_GOAL_PWM);
if dxl_addparam_result ~= true
  fprintf(stderr, '[ID:%03d] groupBulkWrite addparam failed', DXL_ID);
  return;
end

% Bulkwrite goal pwm
groupBulkWriteTxPacket(groupwrite_num);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
end

% Clear bulkwrite parameter storage
groupBulkWriteClearParam(groupwrite_num);

% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);


%% Data Manipulation

for i = 1:t*f
        if (SysIDData.pwm(i) > 32768)
            SysIDData.pwm(i) = SysIDData.pwm(i) - 65536;
        end
        if (SysIDData.velocity(i) > (2^31))
            SysIDData.velocity(i) = SysIDData.velocity(i) - (2^32);
        end
        if (SysIDData.current(i) > 32768)
            SysIDData.current(i) = SysIDData.current(i) - 65536;
        end
end

%% Plots
fig1 = figure();
plot(SysIDData.timeDynamixel, SysIDData.velocity * 0.229 * (pi / 30), '-')
grid on
hold on
plot(SysIDData.timeDynamixel, (SysIDData.current * 3.36)/1000, '-')
hold on
plot(SysIDData.timeDynamixel, (SysIDData.pwm * Unit)/100, '-')
xlabel('Time (s)')
ylabel('Unit')
legend('Velocity', 'Current', 'PWM')

fig2 = figure();
plot(SysIDData.timeSTM, SysIDData.voltage1, '-')
grid on
hold on
plot(SysIDData.timeSTM, SysIDData.voltage2, '-')
xlabel('Time (s)')
ylabel('Voltage')
legend('PB0', 'PA4')

%% Plots 2
% offset = 1.6337;
offset = mean(SysIDData.voltage1(1:50));
offset2 = mean(SysIDData.voltage2(1:50));
fig3 = figure();
plot(SysIDData.timeDynamixel, -(SysIDData.current * 3.36)/1000, '-')
%hold on
%plot(SysIDData.timeSTM, ((SysIDData.voltage2 - offset2)/0.4), '-')
xlabel('Time (s)')
ylabel('Current (A)')
legend('Dynamixel Current')
%legend('Dynamixel Current', 'STM Current')

%% Torque Plot
fig4 = figure();
plot(SysIDData.timeSTM, -0.3577*(SysIDData.voltage1 - 1.6548), '-')
hold on
plot(SysIDData.timeDynamixel, Td, '-')
xlabel('Time (s)')
ylabel('Torque (Nm)')
legend('Measured Torque','Input Torque')

%% Save Structure
save('Verification.mat', 'SysIDData');




