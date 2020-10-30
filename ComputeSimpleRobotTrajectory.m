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
% Description  : This routine aims to prepare a set of simple trajectories
%                for the end effector of a KUKA iiwa robot (1 DoF) 
% -------------------------------------------------------------------------
% Dependencies : To be defined
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
exportFolder  = 'C:\Users\moissene\Desktop\';
depFolder     = [toolboxFolder,'dependencies\']; % These binaries are prepared for a recent version of Matlab and Windows 8 or newer
addpath(toolboxFolder);
addpath(genpath(depFolder));

% -------------------------------------------------------------------------
% SET THE REQUIRED TRAJECTORY
% -------------------------------------------------------------------------

% Pure translation of the final element
% -------------------------------------------------------------------------
nPoint = 100; % number of trajectory points required
Xreq   = [0.5 0.7]; % starting/stopping points (m)
Yreq   = [0.0 0.0]; % starting/stopping points (m)
Zreq   = [0.5 0.5]; % starting/stopping points (m)
Treq   = [permute(linspace(Xreq(1),Xreq(2),nPoint),[3,1,2]); ...
          permute(linspace(Yreq(1),Yreq(2),nPoint),[3,1,2]); ...
          permute(linspace(Zreq(1),Zreq(2),nPoint),[3,1,2])];
Rreq   = repmat(eye(3,3)*1e-14,[1 1 nPoint]); % Double required (not integer)

% Store flange position and orientation for the required trajectory 
% -------------------------------------------------------------------------
for i = 1:nPoint                
    O_flange(:,:,i) = Treq(:,:,i);
    % Rotation matrix
    R_flange(:,:,i) = Rreq(:,:,i);
    % Translate homogeneous matrix to quaternion representation
    Q_flange(4,:,i) = sqrt(1+R_flange(1,1,i)+R_flange(2,2,i)+R_flange(3,3,i))/2; % W
    Q_flange(1,:,i) = (R_flange(3,2,i)-R_flange(2,3,i))/(4*Q_flange(4,:,i));     % X
    Q_flange(2,:,i) = (R_flange(1,3,i)-R_flange(3,1,i))/(4*Q_flange(4,:,i));     % Y
    Q_flange(3,:,i) = (R_flange(2,1,i)-R_flange(1,2,i))/(4*Q_flange(4,:,i));     % Z
    % Normalise the resulting quaternion
    Q_flange(:,:,i) = ones(4,1).*1e-14+Q_flange(:,:,i)/ ... % Double required (not integer) 
                      sqrt(Q_flange(1,:,i)^2+ ...
                           Q_flange(2,:,i)^2+ ...
                           Q_flange(3,:,i)^2+ ...
                           Q_flange(4,:,i)^2);
end

% -------------------------------------------------------------------------
% EXPORT REQUIRED TRAJECTORY
% -------------------------------------------------------------------------
cd(exportFolder)
save('01_Test.mat','O_flange','Q_flange');
O_temp = O_flange;
Q_temp = Q_flange;
clear O_flange Q_flange;
O_flange = repmat(O_temp(:,:,1),[1 1 10]);
Q_flange = repmat(Q_temp(:,:,1),[1 1 10]);
save('01_Test-init.mat','O_flange','Q_flange');
save('01_Test-init_reversed.mat','O_flange','Q_flange');
clear O_flange Q_flange;