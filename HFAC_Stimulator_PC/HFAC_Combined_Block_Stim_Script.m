%This script blocks through one blockChannel and stims through another blockChannel,
%both specified using script parameters

%Electrode Parameters
setupChannels = [0 1 2 3;2 2 0 0];


%Block script parameters
block_freq = 10000; %(Hz)
block_amp = 8000; %(uA)
block_start_time = 2500000; %(us)
block_duration = 15; %(s)
blockChannel = 1;

%Stimulation (monophasic) parameters channel 1
stim_amp = -8000; %(uA)
stim_PW = 200; %(us)
stim_rep = 25; %repetitions
stimChannel_1 = 0;
stim_interpulse = 1000000-stim_PW;

halfcycle_us = round(1000000/(block_freq*2));
disp('actual frequency will be:')
disp(1000000/(2*halfcycle_us))
disp('actual duration will be:')
numcycles = round(2/(2*halfcycle_us*(10^-6)));
disp(numcycles*2*halfcycle_us*(10^-6))
halfcycles = round(block_duration/(halfcycle_us*10^-6));


%Using an existing realterm instance, or create one if it doesn't exist yet
if ~exist('handles','var')
    handles.hrealterm = 0;
end
if (isfield(handles,'hrealterm'))
    try handles.hrealterm.PortOpen=1; %Checks if realterm is alive
    catch
        handles.hrealterm=actxserver('realterm.realtermintf');
        handles.hrealterm.baud=uartBaudRate;
        handles.hrealterm.flowcontrol=0; %no handshaking currently
        handles.hrealterm.Port='\VCP0';
        handles.hrealterm.PortOpen=1; %open the comm port
    end
    is_open=(handles.hrealterm.PortOpen~=0); %check that serial COM port is open
        if (is_open == 0)
            handles.hrealterm.PortOpen=1; %open the comm port
            disp('Opening COM Port...')

        end
else
        handles.hrealterm=actxserver('realterm.realtermintf');
        handles.hrealterm.baud=uartBaudRate;
        handles.hrealterm.flowcontrol=0; %no handshaking currently
        handles.hrealterm.Port='\VCP0';
        handles.hrealterm.PortOpen=1; %open the comm port
end

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
    value = stim_amp;
    time = stim_PW;

    message(1) = 90; %Add discrete current output change time point
    message(2) = stimChannel_1;
    message(3:4) = typecast(int16(value),'uint8');
    message(5:8) = typecast(uint32(time),'uint8');
    for i=1:8
        invoke(handles.hrealterm, 'putchar', message(i));
        pause(0.001);
    end


    %Each packet of data sent to the stimulator by UART is composed of eight
    %bytes, starting with a command byte followed by seven data bytes.
    message = uint8(zeros(8,1));

    %Variables
    value = 0;
    time = stim_interpulse;

    message(1) = 90; %Add discrete current output change time point
    message(2) = stimChannel_1;
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
message(2+stimChannel_1) = 1; %specify which channel will be used for conventional stimulation
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

pause(stim_rep+1);

message = uint8(zeros(8,1));

message(1) = 110; %Change routing command
message(2) = 0; %0 or 1 sets the command mode: iterative or complete change
message(3:8) = [0 0 0 0 dummyroute calibrationroute];
for i=1:8
    invoke(handles.hrealterm, 'putchar', message(i));
    pause(0.001);
end