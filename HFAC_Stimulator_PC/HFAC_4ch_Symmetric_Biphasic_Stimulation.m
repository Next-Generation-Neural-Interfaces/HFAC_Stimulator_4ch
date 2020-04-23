%This script is used 4-channel HFAC Stimulators for biphasic stimulation.
%Stimulation is cathodic phase first.
%This stimulation script has simplified commands for biphasic symmetric
%stimulation, so assymetric stimulation is not possible using this script.
%Ensure that all stimulators connected to the PC System/Rig have been
%initialized (that you have run HFAC_4ch_Stimulator_Initialization)

%PARAMETERS
Stimulators = readtable('Stimulators.xlsx');

%Parameters:
stimulator = 1; %Number of the stimulator to use (See number printed on
%stimulator to identify). Only the stimulator specified here will be made
%active when the script is run regardless of how many are connected to the
%PC system.


amplitude = -40; %Pulse Amplitude in microamps (uA)
pulse_width = 1000; %Pulse Width in microseconds (us)
stim_interphase = 20; %Interphase duration between the cathodic and anodic pulses in microseconds (us)
repetitions = 1; %Number of pulses per script execution (F5 is shortcut)
interpulse = 1; %time between the start of each pulse in seconds
channel = 0; %Channel to use for stimulation
dummy = 0;
%END PARAMETERS


%Communication:
uartBaudRate = 115200;
COMPort = Stimulators.COM_Port(stimulator);
%COM Ports identifying each stimulator to the computer. COM Ports for each 
%stimulator can be identified by noting down which COM Port appears when
%plugging in a stimulator's USB-UART connection device on the PC system's
%Device Manager.


%other variables
numChans = 4; %Number of channels on each stimulator (will only change with new hardware)
stim_interpulse = 1000000-2*pulse_width-stim_interphase;


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


%Stimulator Command Code Begins Here

%Each packet of data sent to the stimulator by UART is composed of eight
%bytes, starting with a command byte followed by seven data bytes.
message = uint8(zeros(8,1));

message(1) = 96; %Delete previously stored stimulation information

for i=1:8
    invoke(hrealterm(interface), 'putchar', message(i));
    pause(0.001);
end

for j=1:repetitions
    %Each packet of data sent to the stimulator by UART is composed of eight
    %bytes, starting with a command byte followed by seven data bytes.
    message = uint8(zeros(8,1));

    %Variables
    value = amplitude;
    time = pulse_width;

    message(1) = 90; %Add discrete current output change time point
    message(2) = channel;
    message(3:4) = typecast(int16(value),'uint8');
    message(5:8) = typecast(uint32(time),'uint8');
    for i=1:8
        invoke(hrealterm(interface), 'putchar', message(i));
        pause(0.001);
    end

    %Each packet of data sent to the stimulator by UART is composed of eight
    %bytes, starting with a command byte followed by seven data bytes.
    message = uint8(zeros(8,1));

    %Variables
    value = 0;
    time = stim_interphase;

    message(1) = 90; %Add discrete current output change time point
    message(2) = channel;
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
    value = -amplitude;
    time = pulse_width;

    message(1) = 90; %Add discrete current output change time point
    message(2) = channel;
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
    value = 0;
    time = stim_interpulse;

    message(1) = 90; %Add discrete current output change time point
    message(2) = channel;
    message(3:4) = typecast(int16(value),'uint8');
    message(5:8) = typecast(uint32(time),'uint8');
    for i=1:8
        invoke(hrealterm(interface), 'putchar', message(i));
        pause(0.001);
    end
end

%Each packet of data sent to the stimulator by UART is composed of eight
%bytes, starting with a command byte followed by seven data bytes.
message = uint8(zeros(8,1));

message(1) = 94; %Set up stimulation
message(2+channel) = 1; %Set corresponding channel as active for conventional stimulation

for i=1:8
    invoke(hrealterm(interface), 'putchar', message(i));
    pause(0.001);
end

%Each packet of data sent to the stimulator by UART is composed of eight
%bytes, starting with a command byte followed by seven data bytes.
message = uint8(zeros(8,1));

%Variables
chanRoute = zeros(1,numChans);
switch dummy
    case 1
        chanRoute(channel+1) = 1;
    otherwise
        chanRoute(channel+1) = 2;
end
dummyroute = 1;
calibrationroute = 0;

message(1) = 110; %Change routing command
message(2) = 0; %0 or 1 sets the command mode: iterative or complete change
message(3:8) = [chanRoute dummyroute calibrationroute];
for k=1:8
    invoke(hrealterm(interface), 'putchar', message(k));
    pause(0.001);
end


%Each packet of data sent to the stimulator by UART is composed of eight
%bytes, starting with a command byte followed by seven data bytes.
message = uint8(zeros(8,1));

message(1) = 95; %Start stimulation protocol

for i=1:8
    invoke(hrealterm(interface), 'putchar', message(i));
    pause(0.001);
end

pause(repetitions*interpulse+2*pulse_width/1000000+0.2)

%Each packet of data sent to the stimulator by UART is composed of eight
%bytes, starting with a command byte followed by seven data bytes.
message = uint8(zeros(8,1));

%Variables
chanRoute = zeros(1,numChans);
dummyroute = 1;
calibrationroute = 0;

message(1) = 110; %Change routing command
message(2) = 0; %0 or 1 sets the command mode: iterative or complete change
message(3:8) = [chanRoute dummyroute calibrationroute];
for k=1:8
    invoke(hrealterm(interface), 'putchar', message(k));
    pause(0.001);
end