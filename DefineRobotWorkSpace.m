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
Subject.id     = 'RS001';
Subject.gender = 'Femme';
Subject.age    = 94; % years
Subject.mass   = 56.4; % kg
Subject.height = 158; % cm
Subject.side   = 'R'; % R or L
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

%% ------------------------------------------------------------------------
% GET MARKER TRAJECTORIES
% -------------------------------------------------------------------------
% Trajectories are expressed here in the inertial coordinate system (ICS)
% -------------------------------------------------------------------------

% Load marker trajectories stored in C3D files
% -------------------------------------------------------------------------
% Load C3D data
btkFile = btkReadAcquisition('testPreCut01.c3d');
% Get marker trajectories individually for each motion file
Marker = btkGetMarkers(btkFile);
% Keep only the requested side markers
temp.SJN = Marker.SJN;
temp.SXS = Marker.SXS;
temp.CV7 = Marker.CV7;
temp.TV8 = Marker.TV8;
if strcmp(Subject.side,'R') == 1
    temp.CAJ = Marker.RCAJ;
    temp.HME = Marker.RHME;
    temp.HLE = Marker.RHLE;
elseif strcmp(Subject.side,'L') == 1
    temp.CAJ = Marker.LCAJ;
    temp.HME = Marker.LHME;
    temp.HLE = Marker.LHLE;
end
clear Marker;
Marker     = temp;
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
    Marker.(markerNames{j}) = [Marker.(markerNames{j})(:,1) ... % Modify the ICS
                                Marker.(markerNames{j})(:,3) ...
                                -Marker.(markerNames{j})(:,2)];
end
clear A B;

% Set the range of time related to each motion
% -------------------------------------------------------------------------
Event = btkGetEvents(btkFile);
iframe_static  = round(Event.static_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                 round(Event.static_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
if strcmp(Subject.side,'R') == 1
    iframe_FE  = round(Event.R_FE_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                 round(Event.R_FE_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_AA  = round(Event.R_AA_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                 round(Event.R_AA_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_IER = round(Event.R_IER_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                 round(Event.R_IER_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_HFE = round(Event.R_HFE_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                 round(Event.R_HFE_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_VT  = round(Event.R_VT_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                 round(Event.R_VT_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_HC  = round(Event.R_HC_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                 round(Event.R_HC_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
elseif strcmp(Subject.side,'L') == 1
    iframe_FE  = round(Event.L_FE_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                 round(Event.L_FE_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_AA  = round(Event.L_AA_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                 round(Event.L_AA_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_IER = round(Event.L_IER_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                 round(Event.L_IER_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_HFE = round(Event.L_HFE_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                 round(Event.L_HFE_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_VT  = round(Event.L_VT_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                 round(Event.L_VT_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
    iframe_HC  = round(Event.L_HC_start*fMarker)+1-btkGetFirstFrame(btkFile)+1:...
                 round(Event.L_HC_stop*fMarker)+1-btkGetFirstFrame(btkFile)+1;
end
clear btkFile;

% Store maximal position indexes
% -------------------------------------------------------------------------
imaxFlex  = iframe_FE(1);
imaxAbd   = iframe_AA(1);
imaxHFlex = iframe_HFE(1);
iminHFlex = iframe_HFE(end);
imaxComp  = iframe_HC(1);
imaxTract = iframe_VT(1);

%% ------------------------------------------------------------------------
% EXPRESS ALL MARKERS IN THE THORAX SEGMENT COORDINATES SYSTEM (SCS)
% -------------------------------------------------------------------------
% Trajectories are expressed in the thorax SCS to remove the components
% related to the thorax motion
% -------------------------------------------------------------------------

% Define the thorax SCS
% Wu et al. 2005 (doi:10.1016/j.jbiomech.2004.05.042)
% -------------------------------------------------------------------------
O6  = Marker.SJN;
tY6 = (Marker.SJN+Marker.CV7)/2 - ...
      (Marker.SXS+Marker.TV8)/2;
Y6  = tY6./sqrt(tY6(:,1).^2+tY6(:,2).^2+tY6(:,3).^2);
tZ6 = cross((Marker.SXS-Marker.TV8), ...
            (Marker.CV7-Marker.TV8));
Z6  = tZ6./sqrt(tZ6(:,1).^2+tZ6(:,2).^2+tZ6(:,3).^2);
X6  = cross(Y6,Z6);
clear tY6 tZ6;

% Express all markers in the thorax SCS
% -------------------------------------------------------------------------
for i = 1:nMarker
    T6 = [X6(i,:)' Y6(i,:)' Z6(i,:)' O6(i,:)'; ...
                 0        0        0        1];    
    for j = 1:size(markerNames,1)         
        temp = inv(T6)*[Marker.(markerNames{j})(i,:) 1]';
        Marker.(markerNames{j})(i,:) = temp(1:3,1);
        clear temp;
    end
end
clear O6 X6 Y6 Z6 T6;
 
%% ------------------------------------------------------------------------
% DEFINE THE HUMERUS LENGTH AND LONGITUDINAL AXIS
% -------------------------------------------------------------------------

% Estimate the elbow joint centre
% -------------------------------------------------------------------------
Marker.EJC  = (Marker.HME+Marker.HLE)/2;
markerNames = fieldnames(Marker);

% Estimate the glenohumeral joint centre per regression
% Dumas and Wojtusch 2018 (doi:10.1007/978-3-319-30808-1_147-1)
% -------------------------------------------------------------------------
% Set technical axes and thorax width
X6 = (Marker.SJN-Marker.CV7) / norm(Marker.SJN-Marker.CV7);
Z6 = (cross(Marker.SXS-Marker.CV7,Marker.SJN-Marker.CV7)) / ...
     norm(cross(Marker.SXS-Marker.CV7,Marker.SJN-Marker.CV7));
Y6 = (cross(Z6,X6)) / norm(cross(Z6,X6));
% Thorax width (distance between CV7 and SJN markers center - 2* marker radius)
W6 = mean(sqrt(sum((Marker.SJN-Marker.CV7).^2))) - dMarker;
% Set regression parameters
if strcmp(Subject.gender,'Femme')
    angle = 5;
    coeff = 0.36;
elseif strcmp(Subject.gender,'Homme')
    angle = 11;
    coeff = 0.33;
end
R6 = [cosd(angle) sind(angle) 0 0; ...
      -sind(angle) cosd(angle) 0 0;
      0 0 1 0; ...
      0 0 0 1];
% Compute the glenohumeral joint centre for each frame
for t = 1:nMarker
    % Assume that the anatomical landmark is at the marker radius distance below the marker along the vertical (Y) axis
    temp = (([X6(t,:)' Y6(t,:)' Z6(t,:)' [Marker.CAJ(t,1) Marker.CAJ(t,2)-dMarker/2 Marker.CAJ(t,3)]'; ...
             [0 0 0 1]]*R6)* ...
             [coeff*W6; 0; 0; 1])';
    Marker.GJC(t,:) = temp(1,1:3);
    clear temp;
end
markerNames = fieldnames(Marker);
clear angle coeff X6 Y6 Z6 W6 R6;

% Define the humerus length and longitudinal axis
% -------------------------------------------------------------------------
humerusLength = mean(sqrt((Marker.EJC(:,1)-Marker.GJC(:,1)).^2+...
                          (Marker.EJC(:,2)-Marker.GJC(:,2)).^2+...
                          (Marker.EJC(:,3)-Marker.GJC(:,3)).^2));
humerusAxis   = (Marker.EJC-Marker.GJC);
humerusAxis   = humerusAxis./sqrt(humerusAxis(:,1).^2+ ...
                                  humerusAxis(:,2).^2+ ...
                                  humerusAxis(:,3).^2);
disp(['Mean humerus length: ',num2str(humerusLength),' mm']);

%% ------------------------------------------------------------------------
% PLOT FULL HUMERUS BOUNDARZ WORKSPACE
% -------------------------------------------------------------------------            

% Set figure
% ------------------------------------------------------------------------- 
figure;
hold on; grid on; axis equal;
title('Humerus boundary workspace');

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
line([Marker.SJN(iframe,1) Marker.CAJ(iframe,1)], ...
     [Marker.SJN(iframe,2) Marker.CAJ(iframe,2)], ...
     [Marker.SJN(iframe,3) Marker.CAJ(iframe,3)], ...
     'Color','red','LineWidth',2);
line([Marker.GJC(iframe,1) Marker.HME(iframe,1)], ...
     [Marker.GJC(iframe,2) Marker.HME(iframe,2)], ...
     [Marker.GJC(iframe,3) Marker.HME(iframe,3)], ...
     'Color','magenta','LineWidth',2);
line([Marker.GJC(iframe,1) Marker.HLE(iframe,1)], ...
     [Marker.GJC(iframe,2) Marker.HLE(iframe,2)], ...
     [Marker.GJC(iframe,3) Marker.HLE(iframe,3)], ...
     'Color','magenta','LineWidth',2);
line([Marker.HME(iframe,1) Marker.HLE(iframe,1)], ...
     [Marker.HME(iframe,2) Marker.HLE(iframe,2)], ...
     [Marker.HME(iframe,3) Marker.HLE(iframe,3)], ...
     'Color','magenta','LineWidth',2);
clear iframe;
 
% Plot GJC and EJC trajectories during frames of interest (not during
% transition between required motions)
% -------------------------------------------------------------------------
iframe = [iframe_static iframe_FE iframe_AA iframe_IER iframe_HFE iframe_VT iframe_HC];
plot3(Marker.GJC(iframe,1),Marker.GJC(iframe,2),Marker.GJC(iframe,3), ...
      'Color','green','LineStyle','--','LineWidth',1);
plot3(Marker.EJC(iframe,1),Marker.EJC(iframe,2),Marker.EJC(iframe,3), ...
      'Color','red','LineStyle','--','LineWidth',1);
clear iframe;

% Plot EJC boundary locations
% -------------------------------------------------------------------------  
plot3(Marker.EJC(imaxFlex,1),Marker.EJC(imaxFlex,2),Marker.EJC(imaxFlex,3) ,'*','LineWidth',2);
plot3(Marker.EJC(imaxHFlex,1),Marker.EJC(iframe_HFE(1),2),Marker.EJC(iframe_HFE(1),3) ,'*','LineWidth',2);
plot3(Marker.EJC(iminHFlex,1),Marker.EJC(iminHFlex,2),Marker.EJC(iminHFlex,3) ,'*','LineWidth',2);
plot3(Marker.EJC(imaxAbd,1),Marker.EJC(imaxAbd,2),Marker.EJC(imaxAbd,3) ,'*','LineWidth',2);
plot3(Marker.EJC(imaxComp,1),Marker.EJC(imaxComp,2),Marker.EJC(imaxComp,3) ,'*','LineWidth',2);
plot3(Marker.EJC(imaxTract,1),Marker.EJC(imaxTract,2),Marker.EJC(imaxTract,3) ,'*','LineWidth',2);

% Plot inertial coordinate system
% -------------------------------------------------------------------------  
quiver3(0,0,0,1,0,0,200,'Color','red');
quiver3(0,0,0,0,1,0,200,'Color','green');
quiver3(0,0,0,0,0,1,200,'Color','blue');

%% ------------------------------------------------------------------------
% STORE BOUNDARZ LOCATIONS OF ALL POTENTIAL HUMERUS CUT POINTS
% -------------------------------------------------------------------------

% Set the range of allowed humerus cut lengths (distance from EJC)
% -------------------------------------------------------------------------
humerusCut = 150;%100:5:210;
disp(['Selected range of humerus cut length (from EJC) (mm): [',num2str(humerusCut(1)),';',num2str(humerusCut(end)),']']);

% Store the boundary locations for all the potential humerus cut points
% -------------------------------------------------------------------------
for h = 1:length(humerusCut)
    LimitPoint.maxFlex(h,:)  = Marker.GJC(iframe_FE(1),:) + ...
                               humerusAxis(iframe_FE(1),:).*(humerusLength - ... % Initial humerus length
                                                             humerusCut(h) + ... % Distance of cut
                                                             Robot.lenghtFlange + ... % Flange length
                                                             Robot.lenghtAssembly + ... % Assembly length
                                                             Robot.cutMargin); % Cut margin
    LimitPoint.maxAbd(h,:)   = Marker.GJC(iframe_AA(1),:) + ...
                               humerusAxis(iframe_AA(1),:).*(humerusLength - ... % Initial humerus length
                                                             humerusCut(h) + ... % Distance of cut
                                                             Robot.lenghtFlange + ... % Flange length
                                                             Robot.lenghtAssembly + ... % Assembly length
                                                             Robot.cutMargin); % Cut margin
    LimitPoint.maxHFlex(h,:) = Marker.GJC(iframe_HFE(1),:) + ...
                               humerusAxis(iframe_HFE(1),:).*(humerusLength - ... % Initial humerus length
                                                              humerusCut(h) + ... % Distance of cut
                                                              Robot.lenghtFlange + ... % Flange length
                                                              Robot.lenghtAssembly + ... % Assembly length
                                                              Robot.cutMargin); % Cut margin
    LimitPoint.minHFlex(h,:) = Marker.GJC(iframe_HFE(end),:) + ...
                               humerusAxis(iframe_HFE(end),:).*(humerusLength - ... % Initial humerus length
                                                                humerusCut(h) + ... % Distance of cut
                                                                Robot.lenghtFlange + ... % Flange length
                                                                Robot.lenghtAssembly + ... % Assembly length
                                                                Robot.cutMargin); % Cut margin
    LimitPoint.maxComp(h,:)  = Marker.GJC(iframe_HC(1),:) + ...
                               humerusAxis(iframe_HC(1),:).*(humerusLength - ... % Initial humerus length
                                                             humerusCut(h) + ... % Distance of cut
                                                             Robot.lenghtFlange + ... % Flange length
                                                             Robot.lenghtAssembly + ... % Assembly length
                                                             Robot.cutMargin); % Cut margin
    LimitPoint.maxTract(h,:) = Marker.GJC(iframe_VT(1),:) + ...
                               humerusAxis(iframe_VT(1),:).*(humerusLength - ... % Initial humerus length
                                                             humerusCut(h) + ... % Distance of cut
                                                             Robot.lenghtFlange + ... % Flange length
                                                             Robot.lenghtAssembly + ... % Assembly length
                                                             Robot.cutMargin); % Cut margin
end
limitPointNames = fieldnames(LimitPoint);

%% ------------------------------------------------------------------------
% SET THE INITIAL SOLUTION FOR THE POSITION OF THE SPECIMEN/ROBOT
% -------------------------------------------------------------------------

% Set the centre of the humerus head at the origin of the ICS
% -------------------------------------------------------------------------
% Define the rigid transformation
Xinit = mean(Marker.SJN(:,1),1);
Yinit = mean(Marker.SJN(:,2),1);
Zinit = mean(Marker.SJN(:,3),1);
Tinit = [Xinit; Yinit; Zinit];
Rinit = [ 1         0          0; ... % Rotation around X in the robot coordinate system
          0 cos(pi/2) -sin(pi/2); ...
          0 sin(pi/2) cos(pi/2)];
% Apply the rigid transformation each marker and limitPoint
for i = 1:(size(markerNames,1))
    Marker.(markerNames{i}) = (Rinit*(Marker.(markerNames{i})'-Tinit))';
end
for i = 1:(size(limitPointNames,1))
    LimitPoint.(limitPointNames{i}) = (Rinit*(LimitPoint.(limitPointNames{i})'-Tinit))';
end
clear Xinit Yinit Zinit Tinit Rinit;

% Update humerus axis
% -------------------------------------------------------------------------
humerusAxis = (Marker.EJC-Marker.GJC);
humerusAxis = humerusAxis./sqrt(humerusAxis(:,1).^2+ ...
                                humerusAxis(:,2).^2+ ...
                                humerusAxis(:,3).^2);
% Define epicondyle axis
% -------------------------------------------------------------------------
epicondyleAxis = (Marker.HLE-Marker.EJC);
epicondyleAxis = epicondyleAxis./sqrt(epicondyleAxis(:,1).^2+ ...
                                      epicondyleAxis(:,2).^2+ ...
                                      epicondyleAxis(:,3).^2);

% Define perpendicular axis, i.e. the axis perpendicular to humerisAxis and
% epicondyleAxis
% -------------------------------------------------------------------------
perpendicularAxis = cross(humerusAxis,epicondyleAxis);

%% ------------------------------------------------------------------------
% PLOT CUT HUMERUS BOUNDARZ WORKSPACE AT INITIAL POSITION
% -------------------------------------------------------------------------            

% Set figure
% ------------------------------------------------------------------------- 
figure;
hold on; grid on; axis equal;
title('Initial position of the specimen positioning problem');

% Plot GJC and EJC trajectories during frames of interest (not during
% transition between required motions)
% -------------------------------------------------------------------------
iframe = [iframe_static iframe_FE iframe_AA iframe_IER iframe_HFE iframe_VT iframe_HC];
plot3(Marker.GJC(iframe,1),Marker.GJC(iframe,2),Marker.GJC(iframe,3), ...
      'Color','green','LineStyle','--','LineWidth',1);
plot3(Marker.EJC(iframe,1),Marker.EJC(iframe,2),Marker.EJC(iframe,3), ...
      'Color','red','LineStyle','--','LineWidth',1);
clear iframe;
   
% Plot maximal humerus cut point boundaries
% -------------------------------------------------------------------------
plot3(LimitPoint.maxFlex(end,1),LimitPoint.maxFlex(end,2),LimitPoint.maxFlex(end,3),'*','LineWidth',2);
plot3(LimitPoint.maxHFlex(end,1),LimitPoint.maxHFlex(end,2),LimitPoint.maxHFlex(end,3),'*','LineWidth',2);
plot3(LimitPoint.minHFlex(end,1),LimitPoint.minHFlex(end,2),LimitPoint.minHFlex(end,3),'*','LineWidth',2);
plot3(LimitPoint.maxAbd(end,1),LimitPoint.maxAbd(end,2),LimitPoint.maxAbd(end,3),'*','LineWidth',2); 
plot3(LimitPoint.maxComp(end,1),LimitPoint.maxComp(end,2),LimitPoint.maxComp(end,3),'*','LineWidth',2);
plot3(LimitPoint.maxTract(end,1),LimitPoint.maxTract(end,2),LimitPoint.maxTract(end,3),'*','LineWidth',2);

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
line([Marker.SJN(iframe,1) Marker.CAJ(iframe,1)], ...
     [Marker.SJN(iframe,2) Marker.CAJ(iframe,2)], ...
     [Marker.SJN(iframe,3) Marker.CAJ(iframe,3)], ...
     'Color','red','LineWidth',2);
line([Marker.GJC(iframe,1) Marker.HME(iframe,1)], ...
     [Marker.GJC(iframe,2) Marker.HME(iframe,2)], ...
     [Marker.GJC(iframe,3) Marker.HME(iframe,3)], ...
     'Color','magenta','LineWidth',2);
line([Marker.GJC(iframe,1) Marker.HLE(iframe,1)], ...
     [Marker.GJC(iframe,2) Marker.HLE(iframe,2)], ...
     [Marker.GJC(iframe,3) Marker.HLE(iframe,3)], ...
     'Color','magenta','LineWidth',2);
line([Marker.HME(iframe,1) Marker.HLE(iframe,1)], ...
     [Marker.HME(iframe,2) Marker.HLE(iframe,2)], ...
     [Marker.HME(iframe,3) Marker.HLE(iframe,3)], ...
     'Color','magenta','LineWidth',2);
clear iframe;

% Plot robot coordinate system 
% -------------------------------------------------------------------------  
quiver3(0,0,0,1,0,0,200,'Color','red');
quiver3(0,0,0,0,1,0,200,'Color','green');
quiver3(0,0,0,0,0,1,200,'Color','blue');

%% ------------------------------------------------------------------------
% LOOP 1: FIND ALL SOLUTIONS OF RIGID TRANSFORMATION IN THE ROBOT (Z VERT.)
%         COORDINATE SYSTEM OF THE SPECIMEN MARKERS AND HUMERUS CUT TO GET 
%         THE HUMERUS EXTRIMITY TRAJECTORZ WITHIN THE ROBOT WORKSPACE WHILE 
%         RESPECTING THE ROBOT MARGIN
% -------------------------------------------------------------------------

% Set the range of allowed specimen positions and orientations
% -------------------------------------------------------------------------
TX = 500:10:800; % Defined empirically
TZ = 655; % Defined physically based on the specimen installation on the test bench
if strcmp(Subject.side,'R')
    TY = -160:10:100; % Defined empirically
    RZ = deg2rad(-150:5:-90); % Defined empirically
elseif strcmp(Subject.side,'L')
    TY = 0:10:160; % Defined empirically
    RZ = deg2rad(90:5:150); % Defined empirically
end
disp(['Selected range of translations (mm): TX [',num2str(TX(1)),';',num2str(TX(end)),'], TY [',num2str(TY(1)),';',num2str(TY(end)),'], TZ [',num2str(TZ(1)),';',num2str(TZ(end)),']']);
disp(['Selected range of rotation (deg): RZ [',num2str(rad2deg(RZ(1))),';',num2str(rad2deg(RZ(end))),']']);

% Compute the radius points related to all available rigid transformations
% and humerus cuts
% -------------------------------------------------------------------------
m = 0;
for i = 1:length(TX)
    for j = 1:length(TY)
        for k = 1:length(TZ)
            for l = 1:length(RZ)                
                % Set the i rigid transformation
                m        = m+1;
                iRT(m,:) = [i,j,k,l]; % Rigid transformations index        
                Ti       = [TX(i); TY(j); TZ(k)] ;                 
                Ri       = [  cos(RZ(l)) sin(RZ(l)) 0; ... % Rotation around Z in the robot coordinate system
                             -sin(RZ(l)) cos(RZ(l)) 0; ...
                                       0          0 1]';                   
                % Test the i rigid transformation to each predefined humerus cut
                for ii = 1:(size(limitPointNames,1))
                    LimitPoint2.(limitPointNames{ii})      = (Ri*(LimitPoint.(limitPointNames{ii}))'+Ti)'; % New position of the limit points
                    RadiusPoint.(limitPointNames{ii})(:,m) = sqrt(LimitPoint2.(limitPointNames{ii})(:,1).^2 + ... % Radius from the centre of the robot workspace to the centre of the limit point
                                                                  LimitPoint2.(limitPointNames{ii})(:,2).^2 + ...
                                                                  (LimitPoint2.(limitPointNames{ii})(:,3)-Robot.heightWorkspaceCentre).^2);    
                end                
                clear Ti Ri;
           end
        end
    end
end

% Identify the radius points included in the robot workspace while
% respecting the robot margin
% -------------------------------------------------------------------------
sol = find(RadiusPoint.maxFlex<(Robot.maxRadius-Robot.robotMargin) & ...
           RadiusPoint.maxFlex>(Robot.minRadius+Robot.robotMargin) & ...
           RadiusPoint.maxAbd<(Robot.maxRadius-Robot.robotMargin) & ...
           RadiusPoint.maxAbd>(Robot.minRadius+Robot.robotMargin) & ...
           RadiusPoint.maxHFlex<(Robot.maxRadius-Robot.robotMargin) & ...
           RadiusPoint.maxHFlex>(Robot.minRadius+Robot.robotMargin) & ...
           RadiusPoint.minHFlex<(Robot.maxRadius-Robot.robotMargin) & ...
           RadiusPoint.minHFlex>(Robot.minRadius+Robot.robotMargin) & ...
           RadiusPoint.maxComp<(Robot.maxRadius-Robot.robotMargin) & ...
           RadiusPoint.maxComp>(Robot.minRadius+Robot.robotMargin) & ...
           RadiusPoint.maxTract<(Robot.maxRadius-Robot.robotMargin) & ...
           RadiusPoint.maxTract>(Robot.minRadius+Robot.robotMargin));

% Store the solutions of this first loop (sorted by humerus cut)
% -------------------------------------------------------------------------
solRow               = sol-(ceil(sol/length(humerusCut))).*length(humerusCut)+length(humerusCut);
solCol               = ceil(sol/length(humerusCut));
[row,irow]           = sort(solRow);
Solution.humerusCut  = humerusCut; % All potential humerus cuts
Solution.ihumerusCut = row; % Selected humerus cuts index after loop1 (e.g. Solution.humerusCut(ihumerusCut) are all the humerus cuts possible after loop1)    
Solution.TX          = TX; % All potential translations /X
Solution.TY          = TY; % All potential translations /Y
Solution.TZ          = TZ; % All potential translations /Z
Solution.RZ          = RZ; % All potential rotations /Y
Solution.iRT1        = iRT; % Indexes of all potential rigid transformations (e.g. Solution.TX(iRT1) are all the potential translations /X)
Solution.iRT2        = solCol(irow); % Selected rigid transformation index after loop1 (e.g. Solution.TX(iRT1(iRT2)) are all the translation /X possible after loop1)     
clear sol solRow solCol row irow RadiusPoint TX TY TZ RZ humerusCut iRT;

%% ------------------------------------------------------------------------
% PLOT CUT HUMERUS BOUNDARY WORKSPACE AT ONE OF THE SOLUTIONS
% -------------------------------------------------------------------------            

% Set figure
% ------------------------------------------------------------------------- 
figure;
hold on; grid on; axis equal;
title('Position of the specimen positioning problem for one potential solution');

% Plot maximal humerus cut point boundaries
% -------------------------------------------------------------------------
plot3(LimitPoint2.maxFlex(end,1),LimitPoint2.maxFlex(end,2),LimitPoint2.maxFlex(end,3),'*','LineWidth',2);
plot3(LimitPoint2.maxHFlex(end,1),LimitPoint2.maxHFlex(end,2),LimitPoint2.maxHFlex(end,3),'*','LineWidth',2);
plot3(LimitPoint2.minHFlex(end,1),LimitPoint2.minHFlex(end,2),LimitPoint2.minHFlex(end,3),'*','LineWidth',2);
plot3(LimitPoint2.maxAbd(end,1),LimitPoint2.maxAbd(end,2),LimitPoint2.maxAbd(end,3),'*','LineWidth',2); 
plot3(LimitPoint2.maxComp(end,1),LimitPoint2.maxComp(end,2),LimitPoint2.maxComp(end,3),'*','LineWidth',2);
plot3(LimitPoint2.maxTract(end,1),LimitPoint2.maxTract(end,2),LimitPoint2.maxTract(end,3),'*','LineWidth',2);
  
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

% Plot robot coordinate system 
% -------------------------------------------------------------------------  
quiver3(0,0,0,1,0,0,200,'Color','red');
quiver3(0,0,0,0,1,0,200,'Color','green');
quiver3(0,0,0,0,0,1,200,'Color','blue');

%% ------------------------------------------------------------------------
% LOOP 2: FIND ACROSS THE PREVIOUS SOLUTIONS THE ONES RESPECTING THE MARGIN
%         CUT AT THE AXIS6 RADIUS
% -------------------------------------------------------------------------
m = 0;
for i = 1:length(Solution.ihumerusCut)
    
    % Define the centre of the Axis6 of the robot along the humerus axis
    % while respecting the cut margin
    % ---------------------------------------------------------------------
    robotAxis6i = Marker.GJC + ...
                  humerusAxis.*(humerusLength - ... % Initial humerus length
                                Solution.humerusCut(Solution.ihumerusCut(i)) + ... % Distance of cut
                                Robot.lenghtFlange + ... % Flange length
                                Robot.lenghtAssembly + ... % Assembly length
                                Robot.cutMargin); % Cut margin
                           
    % Apply the rigid transformation corresponding Solution.irigidTransformations(i)
    % ---------------------------------------------------------------------
    RTi = Solution.iRT1(Solution.iRT2(i),:);
    Ti  = [Solution.TX(RTi(1,1)); Solution.TY(RTi(1,2)); Solution.TZ(RTi(1,3))];
    Ri  = [  cos(Solution.RZ(RTi(1,4))) sin(Solution.RZ(RTi(1,4))) 0;...
            -sin(Solution.RZ(RTi(1,4))) cos(Solution.RZ(RTi(1,4))) 0;... 
                                      0                          0 1]';            
    robotAxis6i2 = (Ri*robotAxis6i'+Ti)';
    RadiusPoint.robotAxis6i2 = sqrt(robotAxis6i2(:,1).^2 + ... % Radius from the centre of the robot workspace to the centre of the Axis6 of the robot
                                    robotAxis6i2(:,2).^2 + ...
                                    (robotAxis6i2(:,3)-Robot.heightWorkspaceCentre).^2);

    % Keep only the solutions included in the robot workspace while
    % respecting the robot margin
    % ---------------------------------------------------------------------
    if RadiusPoint.robotAxis6i2<(Robot.maxRadius-Robot.robotMargin)
       if RadiusPoint.robotAxis6i2>(Robot.minRadius+Robot.robotMargin)
            m = m+1;
            Solution.isolutions(m,1) = i; % Keep solution index
       end
    end

end
clear robotAxis6i RTi Ti Ri RadiusPoint;

%% ------------------------------------------------------------------------
% STORE A SOLUTION
% -------------------------------------------------------------------------

% At this stage, it is mandatory to choose a solution using the following
% constraints:
% 1- Keep the humerus as long as possible (pre-defined in the code)
% 2- Select a humerus cut length that allow versatility in term of specimen
% positionning to allow experimental error in specimen physical placement
% -------------------------------------------------------------------------
% Choose solution
ihumerusCutLength   = Solution.ihumerusCut(Solution.isolutions);
iihumerusCutLength  = max(find(ihumerusCutLength==min(ihumerusCutLength)));
ipotentialPositions = Solution.iRT1(Solution.iRT2(Solution.isolutions(1:iihumerusCutLength)),:);
disp('Potential solutions are:');
potentialPositions  = [Solution.TX(1,ipotentialPositions(:,1)); ...
                       Solution.TY(1,ipotentialPositions(:,2)); ...
                       Solution.TZ(1,ipotentialPositions(:,3)); ...
                       rad2deg(Solution.RZ(1,ipotentialPositions(:,4)))]
s = input('Select a solution that allows +/- 10 cm (give column index): ');
% Apply solution
temp                           = Solution.iRT1(Solution.iRT2(Solution.isolutions(s)),:);
Solution.T                     = [Solution.TX(temp(1,1)); Solution.TY(temp(1,2)); Solution.TZ(temp(1,3))];
Solution.R                     = [ cos(Solution.RZ(temp(1,4))) sin(Solution.RZ(temp(1,4))) 0; ... % Rotation around Z in the robot coordinate system
                                  -sin(Solution.RZ(temp(1,4))) cos(Solution.RZ(temp(1,4))) 0; ...
                                                             0                           0 1]';                                 
Solution.humerusLength         = humerusLength - ... % Initial humerus length
                                 Solution.humerusCut(Solution.ihumerusCut(Solution.isolutions(s))); % Humerus cut related to the selected solution
Solution.specimenOrientationXY = rad2deg(Solution.RZ(temp(1,4))); % Final orientation of the specimen in the robot coordinate system
% Add virtual markers: robot axis 6 centre
Marker.RA6       = Marker.GJC + ...
                   humerusAxis.*(humerusLength - ...
                                 Solution.humerusCut(Solution.ihumerusCut(Solution.isolutions(s))) + ...
                                 Robot.lenghtFlange + ...
                                 Robot.lenghtAssembly);
markerNames      = fieldnames(Marker);
% Apply rigid transformation to boundary points and marker trajectories
for i = 1:(size(limitPointNames,1))
    LimitPoint2.(limitPointNames{i}) = (Solution.R*LimitPoint.(limitPointNames{i})(Solution.ihumerusCut(Solution.isolutions(s)),:)' + Solution.T)';      
end
for i = 1:(size(markerNames,1))
    Marker.(markerNames{i}) = (Solution.R*Marker.(markerNames{i})'+Solution.T)';
end
radiusRobotAxis6 = sqrt(Marker.RA6 (:,1).^2 + ...
                        Marker.RA6 (:,2).^2 + ...
                        (Marker.RA6 (:,3)-Robot.heightWorkspaceCentre).^2);
% Inform about selected solution
disp(['Selected humerus length after cut (mm): ',num2str(Solution.humerusLength)]);
disp(['Selected SJN position (mm): [',num2str(Solution.T(1)),';',num2str(Solution.T(2)),';',num2str(Solution.T(3)),']']);
disp(['Selected CV7 position (mm): [',num2str(round(mean(Marker.CV7(:,1)))),';',num2str(round(mean(Marker.CV7(:,2)))),';',num2str(round(mean(Marker.CV7(:,3)))),']']);

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

% Plot GJC and EJC trajectories during frames of interest (not during
% transition between required motions)
% -------------------------------------------------------------------------
iframe = [iframe_static iframe_FE iframe_AA iframe_IER iframe_HFE iframe_VT iframe_HC];
plot3(Marker.GJC(iframe,1),Marker.GJC(iframe,2),Marker.GJC(iframe,3), ...
      'Color','green','LineStyle','--','LineWidth',1);
plot3(Marker.EJC(iframe,1),Marker.EJC(iframe,2),Marker.EJC(iframe,3), ...
      'Color','red','LineStyle','--','LineWidth',1);    
plot3(Marker.RA6(iframe,1),Marker.RA6(iframe,2),Marker.RA6(iframe,3), ...
      'Color','black','LineStyle','--','LineWidth',1);
clear iframe;  
  
% Plot maximal humerus cut point boundaries
% -------------------------------------------------------------------------
plot3(LimitPoint2.maxFlex(:,1),LimitPoint2.maxFlex(:,2),LimitPoint2.maxFlex(:,3),'*','LineWidth',2);
plot3(LimitPoint2.maxHFlex(:,1),LimitPoint2.maxHFlex(:,2),LimitPoint2.maxHFlex(:,3),'*','LineWidth',2);
plot3(LimitPoint2.minHFlex(:,1),LimitPoint2.minHFlex(:,2),LimitPoint2.minHFlex(:,3),'*','LineWidth',2);
plot3(LimitPoint2.maxAbd(:,1),LimitPoint2.maxAbd(:,2),LimitPoint2.maxAbd(:,3),'*','LineWidth',2); 
plot3(LimitPoint2.maxComp(:,1),LimitPoint2.maxComp(:,2),LimitPoint2.maxComp(:,3),'*','LineWidth',2);
plot3(LimitPoint2.maxTract(:,1),LimitPoint2.maxTract(:,2),LimitPoint2.maxTract(:,3),'*','LineWidth',2);

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
line([Marker.SJN(iframe,1) Marker.CAJ(iframe,1)], ...
     [Marker.SJN(iframe,2) Marker.CAJ(iframe,2)], ...
     [Marker.SJN(iframe,3) Marker.CAJ(iframe,3)], ...
     'Color','red','LineWidth',2);
line([Marker.GJC(iframe,1) Marker.HME(iframe,1)], ...
     [Marker.GJC(iframe,2) Marker.HME(iframe,2)], ...
     [Marker.GJC(iframe,3) Marker.HME(iframe,3)], ...
     'Color','magenta','LineWidth',2);
line([Marker.GJC(iframe,1) Marker.HLE(iframe,1)], ...
     [Marker.GJC(iframe,2) Marker.HLE(iframe,2)], ...
     [Marker.GJC(iframe,3) Marker.HLE(iframe,3)], ...
     'Color','magenta','LineWidth',2);
line([Marker.HME(iframe,1) Marker.HLE(iframe,1)], ...
     [Marker.HME(iframe,2) Marker.HLE(iframe,2)], ...
     [Marker.HME(iframe,3) Marker.HLE(iframe,3)], ...
     'Color','magenta','LineWidth',2);
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
save([Subject.id,'_',Subject.side,'_DefineRobotWorkSpace.mat']);