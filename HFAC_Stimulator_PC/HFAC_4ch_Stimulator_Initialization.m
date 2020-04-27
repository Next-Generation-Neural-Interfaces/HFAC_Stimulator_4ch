
% HFAC_4ch_Stimulator_Initialization.m
%  
% 
% Author: Adrien Rapeaux
%  
% Licensing:
% This file is licensed under the GNU GPL v3 license.
% You can access the license at https://www.gnu.org/licenses/gpl-3.0.html
 
%This script is used to initialize 4-channel HFAC Stimulators

%PARAMETERS
Stimulators = readtable('Stimulators.xlsx');

%StimulatorsToInitialize = [1 2];
StimulatorsToInitialize = [1];
%Which stimulators to initialize (See number printed on stimulator to
%identify)

%setupChannels = {[0 1 2 3;0 0 0 0];[0 1 2 3;0 0 0 0]};
setupChannels = {[0 1 2 3;0 2 0 0]};
%For each stimulator in order:
%first row: channel number; second row: channel mode.
%0: channel inactive; 1: connect to dummy; 2: connect to electrode
%Note: per stimulator there should only be one channel connected to the
%dummy at any one time

%END PARAMETERS



[a,b] = size(StimulatorsToInitialize);
numberOfStimulators = a*b;
%Number of stimulators to initialize as specified by the number of
%identifiers input by the user.
numChans = 4;
%Number of channels for each stimulator (generally 4)

uartBaudRate = 115200;
%UART Baud Rate to use when communicating with stimulators (can change with
%firmware upgrade)

COMPorts = Stimulators.COM_Port(StimulatorsToInitialize);
%COM Ports identifying each stimulator to the computer. COM Ports for each 
%stimulator can be identified by noting down which COM Port appears when
%plugging in a stimulator's USB-UART connection device on the PC system's
%Device Manager.


%Here specify setup values for each stimulator in the same order as
%for COM ports designating these stimulators. 

for i=1:numberOfStimulators
    zeropoints{i} = [
        Stimulators.Zeropoint_1(StimulatorsToInitialize(i))
        Stimulators.Zeropoint_2(StimulatorsToInitialize(i))
        Stimulators.Zeropoint_3(StimulatorsToInitialize(i))
        Stimulators.Zeropoint_4(StimulatorsToInitialize(i))
        ];
end
%12-bit DAC codes corresponding to '0' current output

for i=1:numberOfStimulators
    gains{i} = [
        Stimulators.Gain_1(StimulatorsToInitialize(i))
        Stimulators.Gain_2(StimulatorsToInitialize(i))
        Stimulators.Gain_3(StimulatorsToInitialize(i))
        Stimulators.Gain_4(StimulatorsToInitialize(i))
        ];
end
%Value measured at calibration informing the stimulator of the gain (uA/lsb) of each
%channel

realtermhandles = 0;
if exist('hrealterm','var')
    [a,b] = size(hrealterm);
    realtermhandles = a*b;
end

if ~(realtermhandles==numberOfStimulators)
    clear('hrealterm')
    for i=1:numberOfStimulators
        hrealterm(i) = actxserver('realterm.realtermintf');
    end
else
    for i=1:numberOfStimulators
        try hrealterm(i).PortOpen=0; %close the comm port (in case it was open)
        catch %If the instance is dead this will throw error, catch and launch new instance
            hrealterm(i) = actxserver('realterm.realtermintf');
        end
    end
end

for i=1:numberOfStimulators
        hrealterm(i).baud = uartBaudRate;
        hrealterm(i).flowcontrol=0; %no handshaking currently
        hrealterm(i).Port=num2str(COMPorts(i));
        hrealterm(i).PortOpen=1; %open the comm port
end

for i=1:numberOfStimulators
   for j=1:numChans
        %Each packet of data sent to the stimulator by UART is composed of eight
        %bytes, starting with a command byte followed by seven data bytes.
        message = uint8(zeros(8,1));

        %Variables
        zeropoint = zeropoints{i}(setupChannels{i}(1,j)+1); 
        message(1) = 101; %Change zeropoint command
        message(2) = setupChannels{i}(1,j); %selects the channel for which we want to set the zeropoint
        message(3:4) = typecast(int16(zeropoint),'uint8');
        for k=1:8
            invoke(hrealterm(i), 'putchar', message(k));
            pause(0.001);
        end
        
        %Each packet of data sent to the stimulator by UART is composed of eight
        %bytes, starting with a command byte followed by seven data bytes.
        message = uint8(zeros(8,1));

        %Variables
        gain = gains{i}(setupChannels{i}(1,j)+1); %Note to increase the gain this value must be reduced
        floatGain = cast(gain,'single');
        message(1) = 102; %Change DAC channel gain command
        message(2) = setupChannels{i}(1,j); %selects the channel for which we want to set the gain
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
        message(2) = setupChannels{i}(1,j); %selects the channel for which we want to set the DAC output
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
        chanRoute(setupChannels{i}(1,j)+1) = setupChannels{i}(2,j);
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

% handles.hrealterm_1.PortOpen=0; %close the comm port
% is_open=(handles.hrealterm_1.PortOpen~=0);
% if (is_open == 1) % if port is still open for some reason
%     errordlg('Unable to close serial port. Is the port still in use?',...
%         'Invalid Input','modal')
%     uicontrol(hObject)
%     return
% end
% invoke(handles.hrealterm_1,'close'); 
% delete(handles.hrealterm_1);