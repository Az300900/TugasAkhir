% Model         :   ACIM Field Oriented Control 
% Description   :   Set Parameters for AC Induction Motor's Field Oriented Control
% File name     :   

%% System Parameters

%% Set PWM Switching frequency
PWM_frequency 	= 20e3;    %Hz          // Converter s/w freq
T_pwm           = 1/PWM_frequency;  %s  // PWM switching time period

%% Set Sample Times
Ts          	= T_pwm;        %sec        // Sample time for control system
Ts_simulink     = T_pwm/2;      %sec        // Simulation time step for model simulation
Ts_motor        = T_pwm/2;      %sec        // Simulation sample time for acim
Ts_inverter     = T_pwm/2;      %sec        // Simulation time step for inverter
Ts_speed        = 30*Ts;        %sec        // Sample time for speed controller

%% Set data type for the model (simulation & code-generation)
dataType = 'single';            % Floating point code-generation 
% dataType = fixdt(1,32,24);    % Fixed point code-generation  

%% System Parameters // Hardware parameters 
% Set inverter parameters
inverter = mcb_SetInverterParameters('BoostXL-DRV8305');

% Set target hardware parameters
target = mcb_SetProcessorDetails('F28379D',PWM_frequency);

% Set motor parameters
acim.model      = 'EM_Synergy-M800006';%// Manufacturer Model Number
acim.sn         = '001';      %         // Manufacturer Serial Number
acim.p          = 2;          % Pole pairs
acim.Rs         = 1.79;       % Stator Resistor
acim.Rr         = 1.05;       % Rotor Resistor
acim.Lls        = 6.81e-3;    % Stator leakage inductance 
acim.Lm         = 30e-3;      % Magnetizing inductance 
acim.Llr        = 6.81e-3;    % Rotor leakage inductance 
acim.J          = 6.28e-4;    % Inertia // measured
acim.B          = 2.48e-4;    % Friction Co-efficient // measured
acim.I_rated    = 1.83;       % Rated current (Phase-peak) 
acim.Id0        = 1.08;       % Magnetizing Current (Phase-peak)
acim.V_rated    = 14.7;       % Rated voltage (line-line, rms)
acim.N_rated    = 1121;       % Rated speed (rpm, speed at full load)
acim.N_base     = 1500;       % Synchronous speed (rpm)
acim.F_rated    = 50;         % Rated frequency (Hz)

%% Calibration section // Uncomment and update relevant parameters

% %Update ADC offsets with manually calibrated values below
% inverter.CtSensAOffset = 2292;
% inverter.CtSensBOffset = 2286;

target.SCI_baud_rate    = 12e6;  % Set baud rate for serial communication
target.ADCCalibEnable   = 1;     % Enable : 1, Disable :0 to Auto-calibrate ADCs

% %Update QEP position sensor details below
acim.PositionOffset = 0.0;       %PU position// QEP Offset // Update for position control
acim.QEPSlits       = 1000;      %           // QEP Encoder Slits

% Update motor parameters
acim.Ls         = acim.Lls + acim.Lm;   % Total stator inductance
acim.Lr         = acim.Llr + acim.Lm;   % Total rotor inductance
acim.sigma      = (1 - ((acim.Lm^2)/(acim.Ls*acim.Lr))); % Leakage factor
acim.Iq0        = sqrt(acim.I_rated^2 - acim.Id0^2); % Rated q-axis current
acim.FluxRated  = acim.Id0*acim.Lm;     % Rated Rotor flux 
acim.T_rated    = (3/2)*acim.p*(acim.Lm/acim.Lr)*acim.FluxRated*acim.Iq0;   %Get T_rated from I_rated

% Computing imaginary/equivalent inductances to set PI controller parameters
acim.Ld = acim.sigma*acim.Ls; 
acim.Lq = acim.sigma*acim.Ls;
acim.FluxPM = (acim.Lm/acim.Lr)*acim.FluxRated;

% Update inverter parameters
% ADC Gain for DRV8305
if acim.I_rated < 5
    inverter.ADCGain = 4;   % ADC Range = +- 4.825A wrt 0-4095 counts
    inverter.SPI_Gain_Setting = 0x502A;
    
elseif acim.I_rated < 10
    inverter.ADCGain = 2;   % ADC Range = +- 9.650A wrt 0-4095 counts
    inverter.SPI_Gain_Setting = 0x5015;

else     
    inverter.ADCGain = 1;   % ADC Range = +- 19.300A wrt 0-4095 counts       
    inverter.SPI_Gain_Setting = 0x5000;        
    
end

% Update inverter voltages
inverter.V_dc = 26; %Required to support this motor with Vdc_reqd >= 14.7*sqrt(3)

% Voltage output of inverter current sense circuit
inverter.ISenseVoltPerAmp = inverter.ISenseVoltPerAmp * inverter.ADCGain;

% Update ISenseMax that is measurable by target ADC
inverter.ISenseMax = inverter.ISenseMax * target.ADC_Vref / inverter.ISenseVref;

% Update ISenseMax according to set ADC gain
inverter.ISenseMax = inverter.ISenseMax/inverter.ADCGain;

% Max and min ADC counts for current sense offsets
inverter.CtSensOffsetMax = 2500; % Maximum permitted ADC counts for current sense offset
inverter.CtSensOffsetMin = 1500; % Minimum permitted ADC counts for current sense offset

%% PU System details // Set base values for pu conversion
PU_System = mcb_SetPUSystem(acim,inverter);

%% Controller design 

% IIR Filter for speed
IIR_filter_speed.type           = 'Low-pass';
IIR_filter_speed.min_speed      = 200; %rpm
IIR_filter_speed.f_cutoff       = IIR_filter_speed.min_speed*acim.p/(120/2); %Hz
IIR_filter_speed.coefficient    = 2*pi*Ts*IIR_filter_speed.f_cutoff/(2*pi*Ts*IIR_filter_speed.f_cutoff + 1);
IIR_filter_speed.time_const     = 1/(2*pi*IIR_filter_speed.f_cutoff);
IIR_filter_speed.delay_ss       = 4*IIR_filter_speed.time_const;

% Sensor Delays
Delays.Current_Sensor           = 10*Ts;                        %Current Sensor Delay
Delays.Speed_Sensor             = Ts;                           %Speed Sensor Delay
Delays.Speed_Filter             = IIR_filter_speed.delay_ss;  %Delay for Speed filter

% Controller Delays
Delays.OM_damping_factor        = 1/sqrt(2);            %Damping factor for current control loop 
Delays.SO_factor_speed          = 2;                  %Speed controller delay factor 1 < x < 20

% Get PI Gains
PI_params = mcb.internal.SetControllerParameters(acim,inverter,PU_System,T_pwm,Ts,Ts_speed,Delays);

%Updating delays for simulation
PI_params.delay_Currents    = 1;
PI_params.delay_Position    = 1;

% %Uncomment for frequency domain analysis
% mcb_getControlAnalysis(acim,inverter,PU_System,PI_params,Ts,Ts_speed);

%% Displaying model variables
disp(acim);
disp(inverter);
disp(target);
disp(PU_System);
