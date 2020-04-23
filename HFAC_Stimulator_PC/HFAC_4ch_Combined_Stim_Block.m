%This script is used 4-channel HFAC Stimulators for biphasic stimulation.
%Stimulation is cathodic phase first.
%This stimulation script has simplified commands for biphasic symmetric
%stimulation, so assymetric stimulation is not possible using this script.
%Ensure that all stimulators connected to the PC System/Rig have been
%initialized (that you have run HFAC_4ch_Stimulator_Initialization)

%PARAMETERS
Stimulators = readtable('Stimulators.xlsx');

%Interface Parameters:
stimulator = 1; %Number of the stimulator to use (See number printed on
%stimulator to identify). Only the stimulator specified here will be made
%active when the script is run regardless of how many are connected to the
%PC system.

%Electrode Parameters
setupChannels = [0 1 2 3;2 2 0 0];

%Block script parameters
block_freq = 10000; %(Hz)
block_amp = 1000; %(uA)
block_start_time = 2500000; %time in microseconds at which to start block
    %once the stimulation command hasbeen received
block_duration = 5; %(s) duration of block
blockChannel = 1; %Which channel to use to output the blocking signal

%Stimulation parameters
stimMode = 0; %0 = monophasic, 1 = biphasic symmetric
stimAmplitude = -500; %Pulse Amplitude in microamps (uA)
pulse_width = 500; %Pulse Width in microseconds (us)
stim_interphase = 20; %Interphase duration between the cathodic and anodic pulses in microseconds (us)
pulse_frequency = 1; %(Hz) frequency of pulses in train (usual values 1,0.5 Hz)
repetitions = 8; %Number of pulses per script execution (F5 is shortcut)
interpulse = 1; %time between the start of each pulse in seconds
stimChannel = 0; %Channel to use for stimulation
%END PARAMETERS

%SECONDARY CALCULATIONS
%These are necessary to calculate the number of block cycles down to
%nearest integer using the specified block frequency and duration.
halfcycle_us = round(1000000/(block_freq*2));
disp('actual frequency will be:')
disp(1000000/(2*halfcycle_us))
disp('actual duration will be:')
numcycles = round(2/(2*halfcycle_us*(10^-6)));
disp(numcycles*2*halfcycle_us*(10^-6))
halfcycles = round(block_duration/(halfcycle_us*10^-6));


%Communication:
uartBaudRate = 115200;
COMPort = Stimulators.COM_Port(stimulator);
%COM Ports identifying each stimulator to the computer. COM Ports for each 
%stimulator can be identified by noting down which COM Port appears when
%plugging in a stimulator's USB-UART connection device on the PC system's
%Device Manager.


%other variables
numChans = 4; %Number of channels on each stimulator (will only change with new hardware)
if stimMode == 0
    stim_interpulse = 1000000/pulse_frequency-pulse_width;
else
    stim_interpulse = 1000000/pulse_frequency-2*pulse_width-stim_interphase;
end


%Identify active UART interfaces, create new ones as needed to communicate
%with the stimulator specified by the user
realtermhandles = 0;
if exist('hrealterm','var')
    [a,b] = size(hrealterm);
    realtermhandles = a*b;
end

if ~(realtermhandles>=1) %There must be at least one active interface
    clear('hrealterm') %Otherwise wipe the hrealterm variable
    hrealterm(1) = actxserver('realterm.realtermintf');
    interface = 1;
        %Create a new realterm instance to connect to the stimulator
else
    %There are active interfaces so we will check to see if one is
    %connected to the COM Port corresponding to the stimulator we want to
    %use
    COMPorts = zeros(1,realtermhandles);
    for i=realtermhandles:-1:1
        %Work backwards because we will remove any dead interfaces as we go
        %along
        try COMPorts(i) = str2num(hrealterm(i).Port);
            %Trying to access the Port on the interface will fail if the
            %interface is dead
        catch
            %If there is an error, catch it and destroy the interface
            %variable
            hrealterm(i) = [];
            COMPorts(i) = [];
        end
    end
    interface = find(COMPorts==COMPort);
    if isempty(interface)
        hrealterm = [hrealterm actxserver('realterm.realtermintf')];
        [~,interface] = size(hrealterm);
        %create a new interface if none are connected to the stimulator's
        %COM Port to preserve interfaces already connected to other devices.
    end
%     try hrealterm(interface).PortOpen=0; %close the comm port (in case it was open)
%     catch %If the instance is dead this will throw error, catch and launch new instance
%         hrealterm(interface) = []; %Destroy the old instance
%         hrealterm = [hrealterm actxserver('realterm.realtermintf')];
%         [~,interface] = size(hrealterm);
%     end
end


hrealterm(interface).baud = uartBaudRate;
hrealterm(interface).flowcontrol=0; %no handshaking currently
hrealterm(interface).Port=num2str(COMPort);
hrealterm(interface).PortOpen=1; %open the comm port
%If the program makes it to this point we assume everything is OK and that
%the connection to the stimulator is active.

%ERASE PREVIOUS STIMULATION INFORMATION
message = uint8(zeros(8,1));

message(1) = 96; %Delete previously stored stimulation information

for i=1:8
    invoke(handles.hrealterm, 'putchar', message(i));
    pause(0.001);
end


%PORTION OF THE SCRIPT PROGRAMMING THE BLOCKING SIGNAL INTO THE STIMULATOR
%Each packet of data sent to the stimulator by UART is composed of eight
%bytes, starting with a command byte followed by seven data bytes.
message = uint8(zeros(8,1));

%Variables
%frequency = block_freq;
microamps = block_amp;

message(1) = 91; %continuous stimulation parameter input part 1
message(2) = blockChannel;
message(3:4) = typecast(uint16(halfcycle_us),'uint8');
message(5:6) = typecast(int16(microamps),'uint8');
for i=1:8
    invoke(handles.hrealterm, 'putchar', message(i));
    pause(0.001);
end

%Variables
message = uint8(zeros(8,1));
microseconds = block_start_time;

message(1) = 92; %continuous stimulation parameter input part 2
message(2) = blockChannel;
message(3:6) = typecast(uint32(microseconds),'uint8');

for i=1:8
    invoke(handles.hrealterm, 'putchar', message(i));
    pause(0.001);
end

%Variables
message = uint8(zeros(8,1));
%halfcycles = 2*block_duration*block_freq;

message(1) = 93; %continuous stimulation parameter input part 3
message(2) = blockChannel;
message(3:6) = typecast(uint32(halfcycles),'uint8');

for i=1:8
    invoke(handles.hrealterm, 'putchar', message(i));
    pause(0.001);
end


%PORTION OF THE SCRIPT PROGRAMMING CONVENTIONAL STIMULATION
for j=1:stim_rep
    %Each packet of data sent to the stimulator by UART is composed of eight
    %bytes, starting with a command byte followed by seven data bytes.
    message = uint8(zeros(8,1));

    %Variables
    value = stimAmplitude;
    time = pulse_width;

    message(1) = 90; %Add discrete current output change time point
    message(2) = stimChannel;
    message(3:4) = typecast(int16(value),'uint8');
    message(5:8) = typecast(uint32(time),'uint8');
    for i=1:8
        invoke(handles.hrealterm, 'putchar', message(i));
        pause(0.001);
    end
    if mode == 1
        %Each packet of data sent to the stimulator by UART is composed of eight
        %bytes, starting with a command byte followed by seven data bytes.
        message = uint8(zeros(8,1));

        %Variables
        value = 0;
        time = stim_interphase;

        message(1) = 90; %Add discrete current output change time point
        message(2) = stimChannel;
        message(3:4) = typecast(int16(value),'uint8');
        message(5:8) = typecast(uint32(time),'uint8');
        for i=1:8
            invoke(hrealterm(interface), 'putchar', message(i));
            pause(0.010);
        end


        %Each packet of data sent to the stimulator by UART is composed of eight
        %bytes, starting with a command byte followed by seven data bytes.
        message = uint8(zeros(8,1));

        %Variables
        value = -stimAmplitude;
        time = pulse_width;

        message(1) = 90; %Add discrete current output change time point
        message(2) = stimChannel;
        message(3:4) = typecast(int16(value),'uint8');
        message(5:8) = typecast(uint32(time),'uint8');
        for i=1:8
            invoke(hrealterm(interface), 'putchar', message(i));
            pause(0.010);
        end
    end
    
    %Each packet of data sent to the stimulator by UART is composed of eight
    %bytes, starting with a command byte followed by seven data bytes.
    message = uint8(zeros(8,1));

    %Variables
    value = 0;
    time = stim_interpulse;

    message(1) = 90; %Add discrete current output change time point
    message(2) = stimChannel;
    message(3:4) = typecast(int16(value),'uint8');
    message(5:8) = typecast(uint32(time),'uint8');
    for i=1:8
        invoke(handles.hrealterm, 'putchar', message(i));
        pause(0.001);
    end
end


%Variables
message = uint8(zeros(8,1));

message(1) = 94; %Stimulation protocol set up command
message(2+stimChannel) = 1; %specify which channel will be used for conventional stimulation
message(2+blockChannel) = 2; %specify which channel will be the blocking channel
for i=1:8
    invoke(handles.hrealterm, 'putchar', message(i));
    pause(0.001);
end


message = uint8(zeros(8,1));

%Variables
chanRoute = zeros(1,4);
for i=1:numChans
    chanRoute(setupChannels(1,i)+1) = setupChannels(2,i);
end
dummyroute = 1;
calibrationroute = 0;

message(1) = 110; %Change routing command
message(2) = 0; %0 or 1 sets the command mode: iterative or complete change
message(3:8) = [chanRoute dummyroute calibrationroute];
for i=1:8
    invoke(handles.hrealterm, 'putchar', message(i));
    pause(0.001);
end

%Variables
message = uint8(zeros(8,1));

message(1) = 95; %Start stimulation command
for i=1:8
    invoke(handles.hrealterm, 'putchar', message(i));
    pause(0.001);
end

%This part of the code was to disconnect electrodes at the end of a block
%trial but the block trial duration depends on the number of stims, the
%duration of the block and where each startsso user discretion is advised
%when using this part of the code.

% if stim_rep/pulse_frequency>block_duration
%     pause(stim_rep/pulse_frequency+1);
% else
%     pause(block_duration+block_start_time/1000000)
% end
% 
% message = uint8(zeros(8,1));
% 
% message(1) = 110; %Change routing command
% message(2) = 0; %0 or 1 sets the command mode: iterative or complete change
% message(3:8) = [0 0 0 0 dummyroute calibrationroute];
% for i=1:8
%     invoke(handles.hrealterm, 'putchar', message(i));
%     pause(0.001);
% end