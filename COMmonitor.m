classdef COMmonitor < handle 

    
% COMmonitor:
% written to test serial port output from haptic paddle 
%  - designed to read in 4 values at a time (FloatReadCount), then display to the 
%    command window
%  - **IMPORTANT: search for PortName and type in the correct port number
%
% Original version written by: Nick Rentsch
%
% Modified version by : Ozkan Celik
%
% MEGN 540 Mechatronics
% Colorado School of Mines


properties

    % data props
    Fig                             % figure 
    Port                            % serial port
    DataIn                          % data buffer
    Color = [0.2 0.2 0.2]           % default figure color
    ByteCount = 16;
    FloatReadCount = 4;

    
    Button = [];
    Menu = [];

end
    
methods
          
    
    function app = COMmonitor
    %dlf;

    %% -------------------------- Class Constructor ---------------------------
    %  ------------------------------------------------------------------------

    %app = [];             % empty structure to fill with handles and data

    app.Color = [0.2 0.2 0.2];

    % figure with handle
    app.Fig = figure('Position', [10 31 300 300],...
                     'Name', 'COM Monitor', ...
                     'Color', app.Color, ...
                     'Tag', 'GUI', ...
                     'Resize', 'off', ...
                     'MenuBar', 'none');

    
    app.Button.Connect = uicontrol( ...
                     'Style', 'pushbutton', ...
                     'Parent', app.Fig, ...
                     'Units', 'normalized', ...
                     'Position', [0.1 .1 .8 .1], ...
                     'String', 'Connect', ...
                     'FontSize', 14, ...
                     'FontWeight', 'demi', ...
                     'Callback', {@app.SerialPortConnectCallback});                           
                               
    % Callback assignment for GUI closing
    set(app.Fig, 'CloseRequestFcn', @app.CloseWindowCallback);


    app.DataIn = zeros(app.FloatReadCount,1);
    % Initialize the Serial Port
    app.SerialPortInit();
    
    % end class constructor
    end                     
    
    %% --------------------------- Callbacks -----------------------------
    %*********************************************************************
    %************************ SERIAL BYTES AVAILABLE *********************
    %*********************************************************************
    function BytesAvailableCallback(app, ~, ~)

        try
            app.DataIn = fread(app.Port,app.FloatReadCount,'float');
            disp(app.DataIn);
         catch ME2
            disp(ME2.message);
            disp('Can not read data');
            fclose(app.Port); delete(app.Port); clear app.Port;
        end
    end
    

    %*********************************************************************
    %************************* SERIAL PORT CONNECT ***********************
    %*********************************************************************
    function SerialPortConnectCallback(app, ~, ~)
        
        % check if port is open or closed
        if strncmp(app.Port.Status, 'closed', 4)
            fopen(app.Port);
            set(app.Button.Connect, 'string', 'Disconnect');
        elseif strncmp(app.Port.Status, 'open', 4)
            fclose(app.Port);
            set(app.Button.Connect, 'string', 'Connect');
        end

    end
    
    %*********************************************************************
    %************************* CLOSE WINDOW REQUEST **********************
    %*********************************************************************
    function CloseWindowCallback(app, ~, ~)

        delete(gcf);                            % delete figure

        % check to make sure the port hasn't already been deleted
        if isempty(app.Port)
            % port already deleted
        else
            % check to see if port is still open
            if strncmp(app.Port.Status, 'open', 4)
                fclose(app.Port);
            end
            
            % delete and clear the port
            delete(app.Port); clear app.Port;
        end
        clear app;                              % clear the app
    end
    
    function SerialPortInit(app, ~, ~)
    
    % create/config serial port structure
    
    %** IMPORTANT: Update to correct COM port
    PortName = 'COM3';
    % setup serial port
    try
        app.Port = serial(PortName, ...
                    'BaudRate',9600, ...
                    'Tag', 'serialport');
    catch ME
        disp(ME.message);
        disp('Creating port did not work');
        delete(app.Port); clear app.Port;
    end

    app.Port.BytesAvailableFcnMode = 'byte';
    app.Port.BytesAvailableFcnCount = app.ByteCount;
    app.Port.BytesAvailableFcn = @app.BytesAvailableCallback;
    %app.Port.ByteOrder = 'bigEndian'; %only if float byte order is bigEndian
    
    end
    
end                 % end methods

end                 % end class

