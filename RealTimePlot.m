classdef RealTimePlot < handle

% RealTimePlot    
%       
%
%   **IMPORTANT: search for PortName and type in the correct port number
%                  also make sure you are using same baud rate for Atmel
%                  and Matlab (default here is 9600)
% 
%       You can modify plot y limits to display various signals. (search for ylim)
%       
%       x axis scale depends on how fast values are arriving from the serial port
%       For 100Hz update rate for values, and 9600 baud, 4 sec axis length is correct
%
% Original version written by: Nick Rentsch
%         on: 8/6/13
% Modified by: Abraham Eng
%         on: 10/10/14
% Modified by: Ozkan Celik
%         on: Feb 2015, Feb 2017
%
% MEGN 540 Mechatronics
% Colorado School of Mines

    
properties

    % data props
    DataIn                          % data buffer
    RunController = 0;              % parameter to start/stop system
    RunPlot = 1;
    
    % GUI figure handles
    Fig                             % figure handle

    % GUI objects
    Port;                           % serial port object

        % Serial config/parameters
        ByteCount = 96;                 % No. bytes in buffer to trigger BytesAvailableCallback 
        
        FloatReadCount = 24;            % No. floats to read in each Callback 

    % Controller structure to store Controller Messages/Parameters
    Controller = [];
    
    % GUI structure to store GUI object handles
    Gui = [];

end
    
methods
          
    
    function app = RealTimePlot
        %dlf;

        
        %% -------------------------- Class Constructor ---------------------------
        %  ------------------------------------------------------------------------

        % ************************ Controller[] Population ************************
        
        % Control Variables
        app.Controller.RealTime = zeros(app.FloatReadCount/8, 1);
        
        % ************************* Slider Population *************************
     
                
        % variables for GUI[]
        app.Gui.Panel = [];
        app.Gui.Text = [];
        app.Gui.Plot = [];
        app.Gui.Axes = [];
      

        app.Gui.Panel.yPos = 10;
        app.Gui.Panel.Height = 320; 
        app.Gui.Color = [0.2 0.2 0.2];          % default gui color
        app.Gui.TextColor = [1 1 1];
        
        
        %******************** Gui Plotting Params/Handles *****************
        app.Gui.BUFFERLENGTH = 120;             % No. Samples to buffer 
                                                % for plotting
        app.Gui.Time = linspace(0,...           % time axis
            app.Gui.BUFFERLENGTH/30, app.Gui.BUFFERLENGTH); 
        
        % pre allocate buffers for plotting
        app.Gui.RealTime = zeros(app.Gui.BUFFERLENGTH, 1);
        
        % empty handles for plots
        app.Gui.Plot.RealTime  = NaN;
        
        % plot position coordinates
        app.Gui.Plot.yPos = 75;                % y-pos for plot
        app.Gui.Plot.xPosRealTime = 50;            % x-pos for plot
       
        % empty handles for axes
        app.Gui.Axes.RealTime  = NaN;

        
        app.Gui.Plot.Index = 1;                 % index counter for updating
                                                % plots
        
                                                
                                                
%% --------------------------- Serial Stuff -----------------------
        %  ----------------------------------------------------------------


        
PortName = 'COM3'; %%% ** IMPORTANT: Update to correct COM port

        % create/config serial port structure
        if (isempty(PortName)) 

            disp('No serial port detected! Make sure the Arduino USB cable is plugged in.');
            disp('You may need to restart Matlab *after* you plug the USB cable in.');

        else
            try
                app.Port = serial(PortName, ... 
                            'BaudRate', 9600, ...
                            'Tag', 'SerialPort'); % CHANGE THIS BAUDRATE
             
                % Raw floating point data configuration
                app.Port.BytesAvailableFcnMode = 'byte';
                app.Port.BytesAvailableFcnCount = app.ByteCount;
                % app.Port.ByteOrder = 'bigEndian';
                app.Port.BytesAvailableFcn = @app.BytesAvailableCallback;

                fopen(app.Port);

            catch ME
                disp(ME.message);
                disp('Failed: could not open serial port :(');
                fclose(app.Port); delete(app.Port); clear app.Port;
            end
        end

        disp('Serial port opened.');

        % initialize data buffer size
        app.DataIn = zeros(app.FloatReadCount,1);
        
        
        %% ----------------------- GUI/Figure Creation --------------------
        %  ----------------------------------------------------------------
        % Once serial port open, create and populate GUI window
        % figure with handle
        app.Gui.Fig = figure('Position', [10 20 450 450],...
                         'Name', 'Real Time Plot', ...
                         'Color', app.Gui.Color, ...
                         'Tag', 'Fig', ...
                         'Resize', 'off', ...
                         'MenuBar', 'none');

        % Callback assignment for GUI closing
        set(app.Gui.Fig, 'CloseRequestFcn', @app.CloseWindowCallback);
        
        %% ----------------------- Plot Initialization --------------------
        %  ----------------------------------------------------------------
  
        % plot init and handle assign
        % ---------------- Real Time Plot -------------------%
        %subplot(1,3,1); 
        app.Gui.Plot.RealTime = plot(app.Gui.Time, app.Gui.RealTime);
        %ylim([-200 200]); 
        ylim([-400 400]);    %%% IMPORTANT: can update y axis limits here
        xlim([0 app.Gui.BUFFERLENGTH/30]);  
        set(app.Gui.Plot.RealTime, 'Color', [0.8 0 0.2], ...
                  'linewidth', 2, ...
                  'Tag', 'Plot1', ...
                  'EraseMode', 'normal');
        set(gca,'Color', [0 0 0], ...
                'Units', 'pixels', ...
                'Position', [app.Gui.Plot.xPosRealTime+20 app.Gui.Plot.yPos 300 300], ...
                'YColor', app.Gui.TextColor, ...
                'XColor', app.Gui.TextColor, ...
                'XGrid', 'on', ...
                'YGrid', 'on', ...
                'FontWeight', 'demi');
        app.Gui.Axes.RealTime = gca;
             
       

    end

    %% --------------------------- Callbacks --------------------------- %%
    %  --------------------------------------------------------------------
    
    %**********************************************************************
    %*********************** BYTES AVAILABLE CALLBACK *********************
    %**********************************************************************
    function BytesAvailableCallback(app, ~, ~)

        try
            % raw floating point data method
            app.DataIn = fread(app.Port,app.FloatReadCount,'float');

            % parse DataIn into respective arrays
            app.Controller.RealTime = app.DataIn(1:3:app.FloatReadCount); 
            % every 3rd data point is used for plotting

            % Update Gui arrays
            BlockVector = app.Gui.Plot.Index:(app.Gui.Plot.Index + 8 - 1);
            app.Gui.RealTime(BlockVector)  = app.Controller.RealTime;
 
            % update plots if plots are running
            if (app.RunPlot)
                % update RealTime plot
                set(app.Gui.Plot.RealTime, 'YData', app.Gui.RealTime);

                % increment index to next block of 8 samples
                app.Gui.Plot.Index = app.Gui.Plot.Index + 8;

                % wrap the index variable
                if (app.Gui.Plot.Index > app.Gui.BUFFERLENGTH + 1 - 8)
                    app.Gui.Plot.Index = 1;
                end
            end

        catch ME2
            disp(ME2.message);

        end

    end
 

         
    %**********************************************************************
    %************************* CLOSE WINDOW REQUEST ***********************
    %**********************************************************************
    function CloseWindowCallback(app, ~, ~)

               
        % delete figure (after closing the controller!)
        delete(app.Gui.Fig);
        
        % close and destroy serial port
        if (~isempty(app.Port))
            disp('Closing serial port...');
            fclose(app.Port); delete(app.Port); clear app.Port;
            disp('Closed');
        end

        delete(app);
        clear app;
        clear ans;                              % clean up Matlab workspace
                                                % note: still leaves
                                                % ans, but empty object

    end
    
    %---------------------- End SerialTestApp Methods --------------------%
    
end

    %----------------------- End Class SerialTestApp ---------------------%    
end
