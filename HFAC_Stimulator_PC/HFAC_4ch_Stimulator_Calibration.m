%This script is used to calibrate the positive and negative gain
%potentiometers of 4-channel HFAC Stimulators, one channel at a time. The
%principle of calibration passes the output current from one channel into
%the dummy capacitor which must charge at a constant rate (constant slope
%for voltage). An imbalance in the positive or negative gains of the
%feedback to the current source will lead to exponential or asymptotic
%behaviour (changing output node voltage will change output current, which
%must be corrected by calibration). Measure the current being output by
%setting up an oscilloscope to measure the voltage between the dummy node
%(SMD probe hoop) and system ground.

%PARAMETERS
Stimulators = readtable('Stimulators.xlsx');


StimulatorToCalibrate = [1];
%Which stimulators to initialize (See number printed on stimulator to
%identify)

%Specify which channel must be calibrated by replacing a '0' with a '1' for
%the corresponding channel (0, 1 ,2 or 3). Only one channel can be
%calibrated at a time.
channelToCalibrate = 0;
%setupChannels = [0 1 2 3;1 0 0 0];

%Number of dummy capacitor charge/discharge cycles to carry out (increase
%if more time is needed to adjust potentiometer values to complete
%calibration).
calibrationCycles = 10;

%Output current to use for charging the calibration capacitor (recommended
%to use small values e.g. +-(10-100) uA)
calibrationCurrent = 5; %(uA)
%How long the calibration pulse should last (adjust depending on
%calibration current)
calibrationPW = 1000000; %(us)
%How long there should be between the  positive and negative pulses during
%calibration (generally should be 1s)
calibrationIP = 1000000; %(us)
%END PARAMETERS



[a,b] = size(StimulatorToCalibrate);
numberOfStimulators = a*b;
%Number of stimulators to initialize as specified by the number of
%identifiers input by the user.
numChans = 4;
%Number of channels for each stimulator (generally 4)

uartBaudRate = 115200;
%UART Baud Rate to use when communicating with stimulators (can change with
%firmware upgrade)

COMPort = Stimulators.COM_Port(StimulatorToCalibrate);
%COM Ports identifying each stimulator to the computer. COM Ports for each 
%stimulator can be identified by noting down which COM Port appears when
%plugging in a stimulator's USB-UART connection device on the PC system's
%Device Manager.


%Here specify setup values for each stimulator in the same order as
%for COM ports designating these stimulators. 

for i=1:numberOfStimulators
    zeropoints{i} = [
        Stimulators.Zeropoint_1(StimulatorToCalibrate(i))
        Stimulators.Zeropoint_2(StimulatorToCalibrate(i))
        Stimulators.Zeropoint_3(StimulatorToCalibrate(i))
        Stimulators.Zeropoint_4(StimulatorToCalibrate(i))
        ];
end
%12-bit DAC codes corresponding to '0' current output

for i=1:numberOfStimulators
    gains{i} = [
        Stimulators.Gain_1(StimulatorToCalibrate(i))
        Stimulators.Gain_2(StimulatorToCalibrate(i))
        Stimulators.Gain_3(StimulatorToCalibrate(i))
        Stimulators.Gain_4(StimulatorToCalibrate(i))
        ];
end
%Value measured at calibration informing the stimulator of the gain (uA/lsb) of each
%channel

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

for i=1:numberOfStimulators
   for j=1:numChans
        %Each packet of data sent to the stimulator by UART is composed of eight
        %bytes, starting with a command byte followed by seven data bytes.
        message = uint8(zeros(8,1));

        %Variables
        zeropoint = zeropoints{i}(channelToCalibrate+1); 
        message(1) = 101; %Change zeropoint command
        message(2) = channelToCalibrate; %selects the channel for which we want to set the zeropoint
        message(3:4) = typecast(int16(zeropoint),'uint8');
        for k=1:8
            invoke(hrealterm(i), 'putchar', message(k));
            pause(0.001);
        end
        
        %Each packet of data sent to the stimulator by UART is composed of eight
        %bytes, starting with a command byte followed by seven data bytes.
        message = uint8(zeros(8,1));

        %Variables
        gain = gains{i}(channelToCalibrate+1); %Note to increase the gain this value must be reduced
        floatGain = cast(gain,'single');
        message(1) = 102; %Change DAC channel gain command
        message(2) = channelToCalibrate; %selects the channel for which we want to set the gain
        message(3:6) = typecast(floatGain,'uint8');
        for k=1:8
            invoke(hrealterm(i), 'putchar', message(k));
            pause(0.001);
        end
        
        %Each packet of data sent to the stimulator by UART is composed of eight
        %bytes, starting with a command byte followed by seven data bytes.
        message = uint8(zeros(8,1));

        %Variables
        value = 0000; 
        message(1) = 100; %Change DAC output command
        message(2) = channelToCalibrate; %selects the channel for which we want to set the DAC output
        message(3:4) = typecast(int16(value),'uint8');
        for k=1:8
            invoke(hrealterm(i), 'putchar', message(k));
            pause(0.001);
        end
        
    end

    %Each packet of data sent to the stimulator by UART is composed of eight
    %bytes, starting with a command byte followed by seven data bytes.
    message = uint8(zeros(8,1));

    %Variables
    chanRoute = zeros(1,numChans);
    for j=1:numChans
        chanRoute(channelToCalibrate+1) = 1;
    end
    dummyroute = 1;
    calibrationroute = 0;

    message(1) = 110; %Change routing command
    message(2) = 0; %0 or 1 sets the command mode: iterative or complete change
    message(3:8) = [chanRoute dummyroute calibrationroute];
    for k=1:8
        invoke(hrealterm(i), 'putchar', message(k));
        pause(0.001);
    end
   
end

%Calibration procedure: alternate charging the dummy capacitor into
%positive and then negative voltage, shorting it through the 1 kohm dummy
%resistor every second. Adjust output current for faster or slower sweep.
%Adjust gain potentiometer values to obtain straight charging slopes. Note
%that minimum-maximum output voltage should be +-16V.

%First set the current output of the channel to '0'
message = uint8(zeros(8,1));

%Variables
value = 0; 
message(1) = 100; %Change DAC output command
message(2) = channelToCalibrate; %selects the channel for which we want to set the DAC output
message(3:4) = typecast(int16(value),'uint8');
for k=1:8
    invoke(hrealterm(i), 'putchar', message(k));
    pause(0.001);
end


for j=1:calibrationCycles
    disp(j)
    %Connect the channel to the dummy node with the dummy capacitor shorted
    message = uint8(zeros(8,1));

    %Variables
    chanRoute = zeros(1,numChans);
    chanRoute(channelToCalibrate+1) = 1;
    dummyroute = 1;
    calibrationroute = 0;

    message(1) = 110; %Change routing command
    message(2) = 0; %0 or 1 sets the command mode: iterative or complete change
    message(3:8) = [chanRoute dummyroute calibrationroute];
    for k=1:8
        invoke(hrealterm(interface), 'putchar', message(k));
        pause(0.001);
    end
    
    message = uint8(zeros(8,1));
    message(1) = 96; %Delete previously stored stimulation information

    for i=1:8
        invoke(hrealterm(interface), 'putchar', message(i));
        pause(0.001);
    end
    
    %Each packet of data sent to the stimulator by UART is composed of eight
    %bytes, starting with a command byte followed by seven data bytes.
    message = uint8(zeros(8,1));

    %Variables
    value = calibrationCurrent;
    time = calibrationPW;

    message(1) = 90; %Add discrete current output change time point
    message(2) = channelToCalibrate;
    message(3:4) = typecast(int16(value),'uint8');
    message(5:8) = typecast(uint32(time),'uint8');
    for i=1:8
        invoke(hrealterm(interface), 'putchar', message(i));
        pause(0.001);
    end


    
    message = uint8(zeros(8,1));

    %Variables
    value = 0;
    time = 100; %Since we are calibrating here we just have 100 us of
                %Stimulator channel downtime, but we have the Matlab
                %script wait 1s between each pulse

    message(1) = 90; %Add discrete current output change time point
    message(2) = channelToCalibrate;
    message(3:4) = typecast(int16(value),'uint8');
    message(5:8) = typecast(uint32(time),'uint8');
    for i=1:8
        invoke(hrealterm(interface), 'putchar', message(i));
        pause(0.001);
    end
    
    message = uint8(zeros(8,1));

    message(1) = 94; %Set up stimulation
    message(2+channelToCalibrate) = 1; %Set corresponding channel as active for conventional stimulation

    for i=1:8
        invoke(hrealterm(interface), 'putchar', message(i));
        pause(0.001);
    end
    
    %set up the dummy node to have the dummy capacitor in series
    message = uint8(zeros(8,1));

    %Variables
    chanRoute = zeros(1,numChans);
    chanRoute(channelToCalibrate+1) = 1;
    dummyroute = 0;
    calibrationroute = 0;

    message(1) = 110; %Change routing command
    message(2) = 0; %0 or 1 sets the command mode: iterative or complete change
    message(3:8) = [chanRoute dummyroute calibrationroute];
    for k=1:8
        invoke(hrealterm(interface), 'putchar', message(k));
        pause(0.001);
    end
    
    %Start the stimulation pulse
    message = uint8(zeros(8,1));
    message(1) = 95; %Start stimulation protocol

    for i=1:8
        invoke(hrealterm(interface), 'putchar', message(i));
        pause(0.001);
    end
    
    %Pause 1 second to allow the dummy capacitor to charge
    pause(2);
    
    %Short the dummy capacitor to return the dummy node's potential to
    %approximately system ground (the dummy resistor is always in the path)
    message = uint8(zeros(8,1));

    %Variables
    chanRoute = zeros(1,numChans);
    chanRoute(channelToCalibrate+1) = 1;
    dummyroute = 1;
    calibrationroute = 0;

    message(1) = 110; %Change routing command
    message(2) = 0; %0 or 1 sets the command mode: iterative or complete change
    message(3:8) = [chanRoute dummyroute calibrationroute];
    for k=1:8
        invoke(hrealterm(interface), 'putchar', message(k));
        pause(0.001);
    end
    
    %Set up a stimulation pulse of the same duration but with inverted
    %polarity
    message = uint8(zeros(8,1));
    message(1) = 96; %Delete previously stored stimulation information

    for i=1:8
        invoke(hrealterm(interface), 'putchar', message(i));
        pause(0.001);
    end
    
    %Each packet of data sent to the stimulator by UART is composed of eight
    %bytes, starting with a command byte followed by seven data bytes.
    message = uint8(zeros(8,1));

    %Variables
    value = -calibrationCurrent;
    time = calibrationPW;

    message(1) = 90; %Add discrete current output change time point
    message(2) = channelToCalibrate;
    message(3:4) = typecast(int16(value),'uint8');
    message(5:8) = typecast(uint32(time),'uint8');
    for i=1:8
        invoke(hrealterm(interface), 'putchar', message(i));
        pause(0.001);
    end


    
    message = uint8(zeros(8,1));

    %Variables
    value = 0;
    time = 100; %Since we are calibrating here we just have 100 us of
                %Stimulator channel downtime, but we have the Matlab
                %script wait 1s between each pulse

    message(1) = 90; %Add discrete current output change time point
    message(2) = channelToCalibrate;
    message(3:4) = typecast(int16(value),'uint8');
    message(5:8) = typecast(uint32(time),'uint8');
    for i=1:8
        invoke(hrealterm(interface), 'putchar', message(i));
        pause(0.001);
    end
    
    message = uint8(zeros(8,1));

    message(1) = 94; %Set up stimulation
    message(2+channelToCalibrate) = 1; %Set corresponding channel as active for conventional stimulation

    for i=1:8
        invoke(hrealterm(interface), 'putchar', message(i));
        pause(0.001);
    end
    
    %set up the dummy node to have the dummy capacitor in series
    message = uint8(zeros(8,1));

    %Variables
    chanRoute = zeros(1,numChans);
    chanRoute(channelToCalibrate+1) = 1;
    dummyroute = 0;
    calibrationroute = 0;

    message(1) = 110; %Change routing command
    message(2) = 0; %0 or 1 sets the command mode: iterative or complete change
    message(3:8) = [chanRoute dummyroute calibrationroute];
    for k=1:8
        invoke(hrealterm(interface), 'putchar', message(k));
        pause(0.001);
    end
    
    %Start the stimulation pulse
    message = uint8(zeros(8,1));
    message(1) = 95; %Start stimulation protocol

    for i=1:8
        invoke(hrealterm(interface), 'putchar', message(i));
        pause(0.001);
    end
    
    %Pause 1 second to allow the dummy capacitor to charge
    pause(2);
    
end

%End calibration: set output to '0' and disconnect dummy.

%Set the current output of the channel to '0'
    message = uint8(zeros(8,1));

    %Variables
    value = 0; 
    message(1) = 100; %Change DAC output command
    message(2) = channelToCalibrate; %selects the channel for which we want to set the DAC output
    message(3:4) = typecast(int16(value),'uint8');
    for k=1:8
        invoke(hrealterm(interface), 'putchar', message(k));
        pause(0.001);
    end
    
    %Disconnect all channels and short the dummy capacitor to revert to
    %default behaviour.
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
