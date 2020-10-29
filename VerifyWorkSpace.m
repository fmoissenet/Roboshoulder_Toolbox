% Author       : F. Moissenet
%                Kinesiology Laboratory (K-LAB)
%                University of Geneva
%                https://www.unige.ch/medecine/kinesiology
% License      : Creative Commons Attribution-NonCommercial 4.0 International License 
%                https://creativecommons.org/licenses/by-nc/4.0/legalcode
% Source code  : https://github.com/fmoissenet/Roboshoulder_Toolbox
% Reference    : To be defined
% Date         : March 2020
% -------------------------------------------------------------------------
% Description  : This routine aims to define the work space of a KUKA iiwa
%                robot to manipulation the upper part of a human cadaver. 
%                It is part of the toolkit grouping several methods used in 
%                the RoboShoulder project, a joined project with the HEPIA 
%                school at Geneva.
% -------------------------------------------------------------------------
% Dependencies : - Biomechanical Toolkit (BTK): https://github.com/Biomechanical-ToolKit/BTKCore
%                - PredictMissingMarkers: https://journals.plos.org/plosone/article?id=10.1371/journal.pone.0152616
% -------------------------------------------------------------------------
% This work is licensed under the Creative Commons Attribution - 
% NonCommercial 4.0 International License. To view a copy of this license, 
% visit http://creativecommons.org/licenses/by-nc/4.0/ or send a letter to 
% Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
% INIT THE WORKSPACE
% -------------------------------------------------------------------------
clearvars;
close all;
warning off;
clc;

% -------------------------------------------------------------------------
% SET FOLDERS
% -------------------------------------------------------------------------
toolboxFolder = 'C:\Users\moissene\Documents\Switchdrive\Projets\RoboShoulder\Données\Roboshoulder_toolbox\';
dataFolder    = 'C:\Users\moissene\Documents\Switchdrive\Projets\RoboShoulder\Données\RS001\Mocap\manual_motions\';
exportFolder  = 'C:\Users\moissene\Documents\Switchdrive\Projets\RoboShoulder\Données\RS001\Matlab\';
depFolder     = [toolboxFolder,'dependencies\']; % These binaries are prepared for a recent version of Matlab and Windows 8 or newer
addpath(toolboxFolder);
addpath(genpath(depFolder));
cd(dataFolder);

% -------------------------------------------------------------------------
% SET SUBJECT INFO
% -------------------------------------------------------------------------
Subject.id   = 'RS001';
Subject.side = 'R'; % R or L
disp(['Subject ID: ',Subject.id]);
disp(['Assessed side: ',Subject.side]);

% -------------------------------------------------------------------------
% SET ROBOT PARAMETERS
% -------------------------------------------------------------------------
Robot.heightWorkspaceCentre = 360; % Robot workspace height centre (mm)
Robot.minRadius             = 420; % Robot workspace inferior boundary (mm)
Robot.maxRadius             = Robot.minRadius + 400; % Robot workspace superior boundary (mm)
Robot.lenghtFlange          = 152; % Electrical touch flange length (mm)
Robot.lenghtAssembly        = 15; % Height of the assembly on the flange (mm)
Robot.cutMargin             = 10; % Humerus cut positioning margin (mm)
Robot.tractionMargin        = 5; % Humerus traction positioning margin (mm)
Robot.robotMargin           = 20; % Robot related additional positioning margin (mm)
Robot.lenghtSupporthumerus  = 31; % Length of the humerus support
Robot.DiamSupporthumerus    = 63; % Diameter of the humerus support
Robot.Thicknessflangeplate  = 10; % Thickness of the flange plate

%% ------------------------------------------------------------------------
% GET MARKER TRAJECTORIES
% -------------------------------------------------------------------------
% Trajectories are expressed here in the inertial coordinate system (ICS)
% -------------------------------------------------------------------------

% Load marker trajectories stored in C3D files
% -------------------------------------------------------------------------
% Load C3D data
btkFile = btkReadAcquisition('testRightPostCut01.c3d');
% Get marker trajectories individually for each motion file
Marker = btkGetMarkers(btkFile);
% Keep only the requested side markers
temp.SJN    = Marker.SJN;
temp.SXS    = Marker.SXS;
temp.CV7    = Marker.CV7;
temp.TV8    = Marker.TV8;
temp.EXT01  = Marker.EXT01;
temp.EXT02  = Marker.EXT02;
temp.EXT03  = Marker.EXT03;
temp.INT01  = Marker.INT01;
temp.INT02  = Marker.INT02;
temp.INT03  = Marker.INT03;
temp.INT04  = Marker.INT04;
clear Marker;
Marker      = temp;
markerNames = fieldnames(Marker(1));
clear temp;

% Set marker trajectory parameters
markerNames = fieldnames(Marker); % Marker names in the marker trajectories
nMarker     = length(Marker.CV7); % n frames stored in the marker trajectories
fMarker     = btkGetPointFrequency(btkFile); % Marker trajectories frequency
dMarker     = 14; % Marker diameter (mm)

% Fill gaps and smooth marker trajectories for each motion
% -------------------------------------------------------------------------
% Fill gaps
tMarker = [];
for j = 1:size(markerNames,1)    
    for k = 1:size(Marker.(markerNames{j}),1)
        if Marker.(markerNames{j})(k,:) == [0 0 0] % Replace [0 0 0] by NaN
           Marker.(markerNames{j})(k,:) = nan(1,3);
        end
    end
    tMarker = [tMarker Marker.(markerNames{j})]; % Merge marker data for interpolation
end
tMarker = PredictMissingMarkers(tMarker); % Interpolate markers trajectory
% Smooth trajectories using a lowpass Butterworth filter
for j = 1:size(markerNames,1)
    Marker.(markerNames{j}) = tMarker(:,(3*j)-2:3*j); % Extract interpolated data
    [B,A]                   = butter(2,6/(fMarker/2),'low'); % Low pass filter (Butterworth 4nd order, 6 Hz)
    Marker.(markerNames{j}) = filtfilt(B,A,Marker.(markerNames{j}));
end
clear A B;

% Set the range of time related to each motion
% -------------------------------------------------------------------------
Event = btkGetEvents(btkFile);
iframe_static  = round(Event.static_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                 round(Event.static_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
if strcmp(Subject.side,'R') == 1
    iframe_iFE  = round(Event.R_FE_init*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.R_FE_start*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_FE   = round(Event.R_FE_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.R_FE_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_iAA  = round(Event.R_AA_init*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.R_AA_start*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_AA   = round(Event.R_AA_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.R_AA_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_iIER = round(Event.R_IER_init*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.R_IER_start*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_IER  = round(Event.R_IER_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.R_IER_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_iHFE = round(Event.R_HFE_init*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.R_HFE_start*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_HFE  = round(Event.R_HFE_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.R_HFE_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_iVT  = round(Event.R_VT_init*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.R_VT_start*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_VT   = round(Event.R_VT_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.R_VT_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_iHC  = round(Event.R_HC_init*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.R_HC_start*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_HC   = round(Event.R_HC_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.R_HC_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
elseif strcmp(Subject.side,'L') == 1
    iframe_iFE  = round(Event.L_FE_init*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.L_FE_start*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_FE   = round(Event.L_FE_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.L_FE_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_iAA  = round(Event.L_AA_init*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.L_AA_start*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_AA   = round(Event.L_AA_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.L_AA_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_iIER = round(Event.L_IEL_init*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.L_IEL_start*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_IER  = round(Event.L_IEL_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.L_IEL_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_iHFE = round(Event.L_HFE_init*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.L_HFE_start*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_HFE  = round(Event.L_HFE_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.L_HFE_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_iVT  = round(Event.L_VT_init*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.L_VT_start*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_VT   = round(Event.L_VT_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.L_VT_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_iHC  = round(Event.L_HC_init*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.L_HC_start*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_HC   = round(Event.L_HC_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                  round(Event.L_HC_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
end
clear btkFile;

%% ------------------------------------------------------------------------
% DEFINE FLANGE POSITION AND ORIENTATION
% -------------------------------------------------------------------------

% Define flange orientation                     
X_flange = -(Marker.EXT02-Marker.EXT01); %(Marker.EXT02-Marker.EXT01);
X_flange = X_flange./sqrt(X_flange(:,1).^2+X_flange(:,2).^2+X_flange(:,3).^2);                       
Y_flange = -((Marker.EXT01+Marker.EXT02)/2-Marker.EXT03); %((Marker.EXT01+Marker.EXT02)/2-Marker.EXT03);
Y_flange = Y_flange./sqrt(Y_flange(:,1).^2+Y_flange(:,2).^2+Y_flange(:,3).^2);
Z_flange = -cross(X_flange,Y_flange); %cross(X_flange,Y_flange);
Z_flange = Z_flange./sqrt(Z_flange(:,1).^2+Z_flange(:,2).^2+Z_flange(:,3).^2);
Y_flange = cross(Z_flange,X_flange); % Revise Y axis to ensure orthogonality
Y_flange = Y_flange./sqrt(Y_flange(:,1).^2+Y_flange(:,2).^2+Y_flange(:,3).^2);

% Define INT markers barycenter (while removing the marker height)
temp             = (Marker.INT01+Marker.INT02+Marker.INT03+Marker.INT04)/4;
Marker.INTcenter = temp + Z_flange.*(dMarker/2);
clear temp;

% Store flange position and orientation                  
Marker.FLA         = Marker.INTcenter - Z_flange.*(Robot.Thicknessflangeplate);
Marker.RA6         = Marker.FLA - Z_flange.*(Robot.lenghtFlange); 
radiusRobotAxis6   = sqrt(Marker.RA6(:,1).^2 + ...
                          Marker.RA6(:,2).^2 + ...
                          (Marker.RA6(:,3)-Robot.heightWorkspaceCentre).^2);
markerNames        = fieldnames(Marker);

%% ------------------------------------------------------------------------
% PLOT SOLUTION
% -------------------------------------------------------------------------   

% Plot robot axis 6 radius
% ------------------------------------------------------------------------- 
figure;
hold on; grid on;
title('Distance from iiwa workspace centre of robot axis 6 (mm)');
xlabel('Frames');
plot(radiusRobotAxis6);
line([0 nMarker],[Robot.minRadius Robot.minRadius],'Color','red','LineStyle','--');
line([0 nMarker],[Robot.maxRadius Robot.maxRadius],'Color','red','LineStyle','--');
legend('Robot axis 6 radius','Lower limit','Upper limit');

% Set 3D figure
% ------------------------------------------------------------------------- 
figure;
hold on; grid on; axis equal;
title('Final position of the specimen positioning problem');  

% Plot humerus support trajectories during frames of interest
% -------------------------------------------------------------------------
iframe = 1:nMarker;
plot3(Marker.EXT01(iframe,1),Marker.EXT01(iframe,2),Marker.EXT01(iframe,3),'*','LineWidth',2,'Color','r');
plot3(Marker.EXT02(iframe,1),Marker.EXT02(iframe,2),Marker.EXT02(iframe,3),'*','LineWidth',2,'Color','r');
plot3(Marker.EXT03(iframe,1),Marker.EXT03(iframe,2),Marker.EXT03(iframe,3),'*','LineWidth',2,'Color','r');
plot3(Marker.INT01(iframe,1),Marker.INT01(iframe,2),Marker.INT01(iframe,3),'*','LineWidth',2,'Color','m');
plot3(Marker.INT02(iframe,1),Marker.INT02(iframe,2),Marker.INT02(iframe,3),'*','LineWidth',2,'Color','m');
plot3(Marker.INT03(iframe,1),Marker.INT03(iframe,2),Marker.INT03(iframe,3),'*','LineWidth',2,'Color','m');
plot3(Marker.INT04(iframe,1),Marker.INT04(iframe,2),Marker.INT04(iframe,3),'*','LineWidth',2,'Color','m');
plot3(Marker.FLA(iframe,1),Marker.FLA(iframe,2),Marker.FLA(iframe,3),'+','LineWidth',2);
plot3(Marker.RA6(iframe,1),Marker.RA6(iframe,2),Marker.RA6(iframe,3),'+','LineWidth',2);
plot3(Marker.RA6(:,1),Marker.RA6(:,2),Marker.RA6(:,3),'Color','black','LineStyle','--','LineWidth',1);

% Plot robot geometrical workspace
% -------------------------------------------------------------------------
plot3(sqrt(Robot.maxRadius^2-([-600:10:600].^2)),[-600:10:600],Robot.heightWorkspaceCentre*ones(1,121),...
     'Color','cyan','LineStyle','--','LineWidth',1);
plot3(sqrt(Robot.minRadius^2-([-400:10:400].^2)),[-400:10:400],Robot.heightWorkspaceCentre*ones(1,81),...
      'Color','cyan','LineStyle','--','LineWidth',1);
plot3(sqrt(Robot.maxRadius^2-([-600:10:600].^2)),zeros(1,121),[-600:10:600]+Robot.heightWorkspaceCentre,...
     'Color','cyan','LineStyle','--','LineWidth',1);
plot3(sqrt(Robot.minRadius^2-([-400:10:400].^2)),zeros(1,81),[-400:10:400]+Robot.heightWorkspaceCentre,...
      'Color','cyan','LineStyle','--','LineWidth',1); 
clear iframe;
  
% Plot body segments
% -------------------------------------------------------------------------    
iframe = 1; % static frame at which body segments are plotted
line([Marker.SJN(iframe,1) Marker.SXS(iframe,1)], ...
     [Marker.SJN(iframe,2) Marker.SXS(iframe,2)], ...
     [Marker.SJN(iframe,3) Marker.SXS(iframe,3)], ...
     'Color','green','LineWidth',2);
line([Marker.SJN(iframe,1) Marker.CV7(iframe,1)], ...
     [Marker.SJN(iframe,2) Marker.CV7(iframe,2)], ...
     [Marker.SJN(iframe,3) Marker.CV7(iframe,3)], ...
     'Color','green','LineWidth',2);
line([Marker.TV8(iframe,1) Marker.CV7(iframe,1)], ...
     [Marker.TV8(iframe,2) Marker.CV7(iframe,2)], ...
     [Marker.TV8(iframe,3) Marker.CV7(iframe,3)], ...
     'Color','green','LineWidth',2);
line([Marker.TV8(iframe,1) Marker.SXS(iframe,1)], ...
     [Marker.TV8(iframe,2) Marker.SXS(iframe,2)], ...
     [Marker.TV8(iframe,3) Marker.SXS(iframe,3)], ...
     'Color','green','LineWidth',2);
clear iframe;

% Plot robot coordinate system 
% -------------------------------------------------------------------------  
quiver3(0,0,0,1,0,0,200,'Color','red');
quiver3(0,0,0,0,1,0,200,'Color','green');
quiver3(0,0,0,0,0,1,200,'Color','blue');

%% ------------------------------------------------------------------------
% EXPORT RESULTS
% -------------------------------------------------------------------------
cd(exportFolder);
clear dataFolder depFolder Event exportFolder j k toolboxFolder;
save([Subject.id,'_',Subject.side,'_VerifyWorkSpace.mat']);