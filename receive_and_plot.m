%m-file to adquire ERIKA-FLEX FULL data from the serial port using RS232 

clear all
clc
tic 

%------------------------------------------------------------------------
% Configure communication parameter
%------------------------------------------------------------------------
simulation_time=30.00;
sample_time=0.10;
y_max_limit=3.3;
y_min_limit=0.0;
plot_both_values=1;
plot_raw_value_only=0;
plot_filtered_value_only=0;
%------------------------------------------------------------------------

number_of_samples=simulation_time/sample_time; %set the number of adquisitions
offset_samples=0;
                                        %configuring serial port
s=serial('COM5');                      %creates a matlab object from the serial port 'COM1' 
s.BaudRate=115200;                      %115200;%230400;%460800;%baudrate=115200bps
s.Parity='none';                        %no parity
s.DataBits=8;                           %data sended in 8bits format
s.StopBits=1;                           %1 bit to stop
s.FlowControl='none';                   %no flowcontrol
s.Terminator='LF';                      %LineFeed character as terminator
s.Timeout=10;                           %maximum time in seconds since the data is readed
s.InputBufferSize=100000;               %increment this value when data is corrupted
fopen(s)                                %open serial port object

disp('Waiting for initial data... ')
for n=1:offset_samples
    data=fread(s,8,'char');
    if ( data(1) ~= 1 )
        disp('Communication problems....please re-try');
        fclose(s)             
        delete(instrfind)     
        return
    end
end
disp('Capturing data... ')

%------------------------------------------------------------------------
% Capture data from Explorer 16
%------------------------------------------------------------------------
analog_value=[];
raw_data=[];
filtered_value=[];
filtered_data=[];
my_time=[];
for n = 1:number_of_samples             %get data and plot 
    data=fread(s,8,'char');
    my_time(n,1)=(bitshift(data(2),8) + data(3))*sample_time;
    analog_value(n,1)=(bitshift(data(4),8) + data(5))*3.3/4095;
    raw_data(n,1)=bitshift(data(4),8) + data(5);
    filtered_value(n,1)=(bitshift(data(6),8) + data(7))*3.3/4095;
    filtered_data(n,1)=bitshift(data(6),8) + data(7);
    dummy=data(8);
           
    if plot_both_values
        plot(my_time,analog_value,'b.-',my_time,filtered_value,'r-');
    elseif plot_raw_value_only
        plot(my_time,analog_value,'b.-');
    elseif plot_filtered_value_only
        plot(my_time,filtered_value,'r-');
    end
    axis([my_time(1) my_time(1)+simulation_time y_min_limit y_max_limit]);grid on;
    ylabel('Analog Value');xlabel('time (seconds)');
    drawnow; 
end


%------------------------------------------------------------------------
% Close serial port communication
%------------------------------------------------------------------------
fclose(s)                               %close serial port object
delete(instrfind)                       %clean all open ports
