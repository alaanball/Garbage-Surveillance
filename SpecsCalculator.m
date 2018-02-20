% Calculating battery and motor specifications
% Force calculation is based on this reference:
% http://www2.mae.ufl.edu/designlab/motors/EML2322L%20Drive%20Wheel%20Motor%20Torque%20Calculations.pdf
% References for weight estimates is given in SpecificationsAndCalculations.txt

DistanceSpecified = 200;      % Distance between endpoints of path in metres
SpecifiedTime = 20;           % Time taken to complete one round trip of the bot (in minutes)
RoundTrips = 4;               % Number of round trips before battery recharge
AccelerationTime = 1;         % time to accelerate to top speed from rest (in seconds)

DistanceBetweenImageCaptures = 3;     % distance traveled between two image captures (in metres)
ImageProcessingTime = 5;              % time taken in seconds to capture and process the image
NumberOfStops = DistanceSpecified / DistanceBetweenImageCaptures;

% Motor operating time in minutes for 1 round trip
MotorRunTime = SpecifiedTime - 2 * NumberOfStops * ImageProcessingTime / 60; 

TopSpeed = DistanceSpecified / (MotorRunTime / 2 * 60)  % maximum speed of bot
Acceleration = TopSpeed / AccelerationTime;             % acceleration of bot from rest  

masses = ... various masses in the bot (in grams)
[ 250 , ... Chassis without wheels
 120, ... 4 wheels
 4 * 180, ... 4 motors with gearboxes
 45, ... Raspberry Pi
 250, ... Total Battery Weight
 100 ... Miscellaneous
];

RollingFriction = 0.02;
AngleOfInclination = 40;            % maximum slope angle in degrees

MotorVoltage = 12;
WheelRadius = 3.5 / 100;            % Wheel radius in m
RaspberryPiPowerConsumption = 6;    % maximum raspberry pi power in watts

TotalMass = sum(masses) / 1000;     % in kg
GrossWeight = TotalMass * 9.8;      % in N

% Definitions of the following terms is included in the above paper
RollingResistance = GrossWeight * RollingFriction;
GradeResistance = GrossWeight * sind(AngleOfInclination);
AccelerationForce = TotalMass * Acceleration;

TotalTractiveEffort = RollingResistance + GradeResistance + AccelerationForce; 
% in newtons

ResistanceFactor = 1.15;
TotalTorque = TotalTractiveEffort * WheelRadius * ResistanceFactor * 100 % in N-cm
MotorRPM = 60 * TopSpeed / (2 * pi * WheelRadius)

MotorPowerConsumption = TotalTractiveEffort * TopSpeed; % in watts

RaspPiBatteryWH = RaspberryPiPowerConsumption * RoundTrips * SpecifiedTime / 60
RaspPiBatteryMAH = RaspPiBatteryWH * 1000 / 5.1

MotorBatteryWH = MotorPowerConsumption * MotorRunTime * RoundTrips /60
MotorBatteryMAH = MotorBatteryWH * 1000 / MotorVoltage
              
WattHours = RaspPiBatteryWH  + MotorBatteryWH  % total required battery capacity in watt-hours
