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

t = 5;      % Total Duration (s)
f = 50;     % Sampling Frequency (Hz)
p = 4;      % Number of Periods
Unit = 0.113; % Unit Percentage
A = 20/Unit;     % Amplitude
offset = A*0; % Offset from 0
% dxl_goal_pwm = A*(sin(linspace(0, p*2*pi, t*f))) + offset;
% dxl_goal_pwm = linspace(10/Unit,10/Unit,t*f);
% dxl_goal_pwm = [zeros(1,t*f*0.2) A*(sin(linspace(0, p*2*pi, t*f*0.6))) + offset zeros(1,t*f*0.2)];
% dxl_goal_pwm = [zeros(1,t*f*0.1) A*(sin(linspace(0, p*2*pi, t*f*0.3))) + offset zeros(1,t*f*0.2) A*(sin(linspace(0, p*2*pi, t*f*0.3))) zeros(1,t*f*0.1)];
% dxl_goal_pwm = -A*ones(1,t*f);
% dxl_goal_pwm = zeros(1,t*f);

T_A = 1;

% Td = [zeros(1,t*f*0.2) T_A*(sin(linspace(0, p*2*pi, t*f*0.6))) + offset zeros(1,t*f*0.2)];
% Td = [T_A*(sin(linspace(0, p*2*pi, t*f*1))) + offset];
Td = [T_A*(sin(linspace(0, p*2*pi, t*f*1)))];
% Td = [T_A*ones(1,t*f)];
Td = [0 Td(1:end - 1)];

plot(linspace(0,t,t*f),Td)

%% Validation Parameters

phi = 877.8865;
Kw = 2.3731;

% Ra = 1.5753;
% Kw = 2.5291;
% KI = 0.0068;
N  = 255;


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

N = f * t; % frequency * duration
SysIDData.time = zeros(N,1);
SysIDData.voltage = zeros(N,1);
SysIDData.pwm = zeros(N,1);
SysIDData.current = zeros(N,1);
SysIDData.velocity = zeros(N,1);
SysIDData.position =  zeros(N,1);
SysIDData.Td = Td;
SysIDData.omega = zeros(N,1);

%% Setup STM32 Serial
if ~isempty(instrfind('Status', 'open'))
    fclose(instrfind);
end

stm_device = serialport('COM4', 115200, 'Timeout', 0.5);
writeline(stm_device, "getSysID");

%% Begin Serial Communnication
elapsed_time = 0;
tic
while elapsed_time < t
    if toc >= 1/f
        % Get the current elapsed seconds
        elapsed_time = toc + elapsed_time;
        % Reset the timer
        tic

        if index == 1
            tempVel = 0;
        else
            tempVel = SysIDData.velocity(index - 1);
        end

        % Get Velocity Reading
        if (tempVel > (2^31))
            temp = tempVel - (2^32);
        else
            temp = tempVel;
        end

        SysIDData.omega(index) = temp * 0.229 * (pi / 30);

        % Calculate needed voltage for torque load
%         Vin = (Td(index)/N)*(Ra/KI) + Kw * SysIDData.omega(index);
        Vin = (Td(index)/N)*(phi) + Kw * SysIDData.omega(index);

        voltageCap = 10;
        if Vin > voltageCap
            Vin = voltageCap;
        end

        if Vin < -voltageCap
            Vin = -voltageCap;
        end

        SysIDData.pwm(index) = ((Vin/12)*100)/0.113;


        % Add parameter storage for Dynamixel goal position
        dxl_addparam_result = groupBulkWriteAddParam(groupwrite_num, DXL_ID, ADDR_GOAL_PWM, LEN_GOAL_PWM, typecast(int16(SysIDData.pwm(index)), 'uint16'), LEN_GOAL_PWM);
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
        SysIDData.pwm(index) = groupBulkReadGetData(groupread_num, DXL_ID, ADDR_PRESENT_PWM, LEN_PRESENT_PWM);
        % Get Dynamixel present current value
        SysIDData.current(index) = groupBulkReadGetData(groupread_num, DXL_ID, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
        % Get Dynamixel present velocity value
        SysIDData.velocity(index) = groupBulkReadGetData(groupread_num, DXL_ID, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
        % Get Dynamixel present position value
        SysIDData.position(index) = groupBulkReadGetData(groupread_num, DXL_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);


        %STM32 measure
        writeline(stm_device, "getSysID");
        SysIDData.voltage(index) = str2double(readline(stm_device));
        SysIDData.time(index) = elapsed_time;
    
        index = index + 1;
    end
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
plot(SysIDData.time, SysIDData.velocity * 0.229 * (pi / 30), '-')
grid on
hold on
plot(SysIDData.time, (SysIDData.current * 3.36)/1000, '-')
hold on
plot(SysIDData.time, (SysIDData.pwm * Unit)/100, '-')
xlabel('Time (s)')
ylabel('Unit')
legend('Velocity', 'Current', 'PWM')

fig2 = figure();
plot(SysIDData.time, SysIDData.voltage, '-')
grid on
xlabel('Time (s)')
ylabel('Voltage')
legend('PB0')

%% Plots 2
% offset = 1.6337;
offset = mean(SysIDData.voltage(1:50));
fig3 = figure();
plot(SysIDData.time, -(SysIDData.current * 3.36)/1000, '-')
xlabel('Time (s)')
ylabel('Current (A)')
legend('Dynamixel Current')

fig4 = figure();
plot(SysIDData.time, 0.3577*(SysIDData.voltage - offset), '-')
xlabel('Time (s)')
ylabel('Torque (Nm)')

%% Validation Plot
fig5 = figure();
plot(SysIDData.time, SysIDData.Td);
hold on
plot(SysIDData.time, -1.2462*(SysIDData.voltage - offset), '-')
xlabel('Time (s)')
ylabel('Torque (Nm)')
legend('Input', 'Output')

%% Save Structure
save('SYS_ID_VALIDATION20.mat', 'SysIDData');

% Loop through each element of the matrix and count NaN values
input_matrix = SysIDData.voltage;
nan_count = 0;
for i = 1:numel(input_matrix)
    if isnan(input_matrix(i))
        nan_count = nan_count + 1;
    end
end

% Display the number of NaN values
fprintf('Number of NaN values: %d\n', nan_count);




