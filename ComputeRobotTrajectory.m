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
% Description  : This routine aims to compute the robot trajectory, i.e.  
%                the robot flange trajectory and quaternion
%                It is part of the toolkit grouping several methods used in 
%                the RoboShoulder project, a joined project with the HEPIA 
%                school at Geneva.
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
dataFolder    = 'C:\Users\moissene\Documents\Switchdrive\Projets\RoboShoulder\Données\RS001\Mocap\manual_motions\';
exportFolder  = 'C:\Users\moissene\Documents\Switchdrive\Projets\RoboShoulder\Données\RS001\Matlab\';
depFolder     = [toolboxFolder,'dependencies\']; % These binaries are prepared for a recent version of Matlab and Windows 8 or newer
addpath(toolboxFolder);
addpath(genpath(depFolder));

% -------------------------------------------------------------------------
% LOAD VERIFYWORKSPACE DATA
% -------------------------------------------------------------------------
cd(exportFolder);
load('RS001_R_VerifyWorkSpace.mat');
disp(['Subject ID: ',Subject.id]);
disp(['Assessed side: ',Subject.side]);

% -------------------------------------------------------------------------
% EXPRESS DATA IN METER (REQUIRED FOR ROS)
% ------------------------------------------------------------------------- 
for i = 1:length(markerNames)
    Marker.(markerNames{i}) = Marker.(markerNames{i})*1e-3; % Markers coordinates (m)
end
dMarker = dMarker*1e-3; % Markers diameter (m)

% -------------------------------------------------------------------------
% PREPARE ROBOT TRAJECTORY CONSTRAINTS FOR EACH MOVEMENT
% ------------------------------------------------------------------------- 
nbMovements = 12;
for mov = 1:nbMovements % Movement file index
    
    % Set movement related timing
    % --------------------------------------------------------------------- 
    switch mov
        case 1
            disp('FE init')
            iframe = iframe_iFE;
        case 2
            disp('FE')
            iframe = iframe_FE;
        case 3
            disp('AA init')
            iframe = iframe_iAA;
        case 4
            disp('AA')
            iframe = iframe_AA;
        case 5
            disp('IER init')
            iframe = iframe_iIER;
        case 6
            disp('IER')
            iframe = iframe_IER;
        case 7
            disp('HFE init')
            iframe = iframe_iHFE;
        case 8
            disp('HFE')
            iframe = iframe_HFE;
        case 9
            disp('VT init')
            iframe = iframe_iVT;
        case 10
            disp('VT')
            iframe = iframe_VT;
        case 11
            disp('HC init')
            iframe = iframe_iHC;
        case 12
            disp('HC')
            iframe = iframe_HC;
    end        

    % Define INT markers barycenter (while removing the marker height)
    % --------------------------------------------------------------------- 
    temp                  = (Marker.INT01(i,:)+Marker.INT02(i,:)+Marker.INT03(i,:)+Marker.INT04(i,:))/4;
    Marker.INTcenter(i,:) = temp + Z_flange(i,:).*(dMarker/2);
    clear temp;

    % Plot flange trajectory
    % --------------------------------------------------------------------- 
%     figure(mov);
%     subplot(2,1,1);
%     for i = iframe(1):10:iframe(end)
%         plot3(Marker.INT01(i,1),Marker.INT01(i,2),Marker.INT01(i,3), ...
%               'Marker','.','MarkerSize',15,'Color','red');
%         hold on;
%         plot3(Marker.INT02(i,1),Marker.INT02(i,2),Marker.INT02(i,3), ...
%               'Marker','.','MarkerSize',15,'Color','magenta');
%         plot3(Marker.INT03(i,1),Marker.INT03(i,2),Marker.INT03(i,3), ...
%               'Marker','.','MarkerSize',15,'Color','green');
%         plot3(Marker.INT04(i,1),Marker.INT04(i,2),Marker.INT04(i,3), ...
%               'Marker','.','MarkerSize',15,'Color','green');
%         plot3(Marker.INTcenter(i,1),Marker.INTcenter(i,2),Marker.INTcenter(i,3), ...
%               'Marker','x','MarkerSize',15,'Color','blue');
%         quiver3(Marker.INTcenter(i,1),Marker.INTcenter(i,2),Marker.INTcenter(i,3), ...
%                 X_flange(i,1),X_flange(i,2),X_flange(i,3), ...
%                 0.2,'red');
%         quiver3(Marker.INTcenter(i,1),Marker.INTcenter(i,2),Marker.INTcenter(i,3), ...
%                 Y_flange(i,1),Y_flange(i,2),Y_flange(i,3), ...
%                 0.2,'green');
%         quiver3(Marker.INTcenter(i,1),Marker.INTcenter(i,2),Marker.INTcenter(i,3), ...
%                 Z_flange(i,1),Z_flange(i,2),Z_flange(i,3), ...
%                 0.2,'blue');    
%         axis equal;
%         grid on;
%         box on;
%         xlabel('X axis');
%         ylabel('Y axis');
%         zlabel('Z axis');
%         xlim([-0.1 1.0]);
%         ylim([-1.0 1.0]);
%         zlim([0 1.0]);
%         view(90,0); 
%         pause(0.1);
%         hold off;
%     end
%     subplot(2,1,2); hold on;
%     plot(cross(Y_flange,Z_flange));
%     plot(cross(Z_flange,X_flange));

    % Store flange position and orientation for the current movement 
    % --------------------------------------------------------------------- 
    t = 1;
    for i = iframe                
        O_flange(:,:,t) = (Marker.INTcenter(i,:) - Z_flange(i,3).*(Robot.Thicknessflangeplate*1e-3))';
        % Rotation matrix
        R_flange(:,:,t) = [X_flange(i,:)' Y_flange(i,:)' Z_flange(i,:)'];
        % Translate homogeneous matrix to quaternion representation
        Q_flange(4,:,t) = sqrt(1+R_flange(1,1,t)+R_flange(2,2,t)+R_flange(3,3,t))/2; % W
        Q_flange(1,:,t) = (R_flange(3,2,t)-R_flange(2,3,t))/(4*Q_flange(4,:,t));     % X
        Q_flange(2,:,t) = (R_flange(1,3,t)-R_flange(3,1,t))/(4*Q_flange(4,:,t));     % Y
        Q_flange(3,:,t) = (R_flange(2,1,t)-R_flange(1,2,t))/(4*Q_flange(4,:,t));     % Z
        % Normalise the resulting quaternion
        Q_flange(:,:,t) = Q_flange(:,:,t)/ ...
                          sqrt(Q_flange(1,:,t)^2+ ...
                               Q_flange(2,:,t)^2+ ...
                               Q_flange(3,:,t)^2+ ...
                               Q_flange(4,:,t)^2);
        t = t+1;
    end
    clear t t1 t2;
    
    % Interpolate flange position and orientation as mxnx30 vectors
    % --------------------------------------------------------------------- 
    k = 1:size(O_flange,3);
    ko = (linspace(1,length(iframe),30))';
    temp = interp1(k,permute(O_flange,[3,1,2]),ko,'makima'); clear O_flange;
    O_flange = permute(temp,[2,3,1]); clear temp;
    temp = interp1(k,permute(Q_flange,[3,1,2]),ko,'makima'); clear Q_flange;
    Q_flange = permute(temp,[2,3,1]); clear temp;
    temp = interp1(k,permute(R_flange,[3,1,2]),ko,'makima'); clear R_flange;
    R_flange = permute(temp,[2,3,1]); clear temp;
    clear k ko;
%     figure; plot(squeeze(O_flange)','Marker','x');
%     figure; plot(squeeze(Q_flange)','Marker','x');
%     figure; plot(rad2deg(squeeze(permute(R_flange(:,3,:),[3,1,2]))),'Marker','x');
    
    % Export results 
    % --------------------------------------------------------------------- 
    cd(exportFolder);
    switch mov
        case 1
            Mvt       = 'FE';
            Ord       = '00';   
            Name      = [Ord,'_',Mvt,'_',Subject.side,'-init.mat'];
            Namefull  = [Mvt,'_',Subject.side,'_','full-init.mat'];
            save(Name,'O_flange','Q_flange');
            save(Namefull);  
            
            Mvt       = 'FE';
            Ord       = '00';   
            Name      = [Ord,'_',Mvt,'_',Subject.side,'-init_reversed.mat'];
            Namefull  = [Mvt,'_',Subject.side,'_','full-init_reversed.mat'];
            O_flange = flip(O_flange,3);
            Q_flange = flip(Q_flange,3);
            save(Name,'O_flange','Q_flange');
            save(Namefull); 
        case 2
            Mvt       = 'FE';  
            Ord       = '00';   
            Name      = [Ord,'_',Mvt,'_',Subject.side,'.mat'];
            Namefull  = [Mvt,'_',Subject.side,'_','full.mat'];
            save(Name,'O_flange','Q_flange');
            save(Namefull);        
        case 3
            Mvt       = 'AA';
            Ord       = '01';   
            Name      = [Ord,'_',Mvt,'_',Subject.side,'-init.mat'];
            Namefull  = [Mvt,'_',Subject.side,'_','full-init.mat'];
            save(Name,'O_flange','Q_flange');
            save(Namefull);  
            
            Mvt       = 'AA';
            Ord       = '01';   
            Name      = [Ord,'_',Mvt,'_',Subject.side,'-init_reversed.mat'];
            Namefull  = [Mvt,'_',Subject.side,'_','full-init_reversed.mat'];
            O_flange = flip(O_flange,3);
            Q_flange = flip(Q_flange,3);
            save(Name,'O_flange','Q_flange');
            save(Namefull);       
        case 4
            Mvt       = 'AA';  
            Ord       = '01';   
            Name      = [Ord,'_',Mvt,'_',Subject.side,'.mat'];
            Namefull  = [Mvt,'_',Subject.side,'_','full.mat'];
            save(Name,'O_flange','Q_flange');
            save(Namefull);           
        case 5
            Mvt       = 'IER';
            Ord       = '02';   
            Name      = [Ord,'_',Mvt,'_',Subject.side,'-init.mat'];
            Namefull  = [Mvt,'_',Subject.side,'_','full-init.mat'];
            save(Name,'O_flange','Q_flange');
            save(Namefull);  
            
            Mvt       = 'IER';
            Ord       = '02';   
            Name      = [Ord,'_',Mvt,'_',Subject.side,'-init_reversed.mat'];
            Namefull  = [Mvt,'_',Subject.side,'_','full-init_reversed.mat'];
            O_flange = flip(O_flange,3);
            Q_flange = flip(Q_flange,3);
            save(Name,'O_flange','Q_flange');
            save(Namefull);         
        case 6
            Mvt       = 'IER';  
            Ord       = '02';   
            Name      = [Ord,'_',Mvt,'_',Subject.side,'.mat'];
            Namefull  = [Mvt,'_',Subject.side,'_','full.mat'];
            save(Name,'O_flange','Q_flange');
            save(Namefull);         
        case 7
            Mvt       = 'HFE';
            Ord       = '03';   
            Name      = [Ord,'_',Mvt,'_',Subject.side,'-init.mat'];
            Namefull  = [Mvt,'_',Subject.side,'_','full-init.mat'];
            save(Name,'O_flange','Q_flange');
            save(Namefull);  
            
            Mvt       = 'HFE';
            Ord       = '03';   
            Name      = [Ord,'_',Mvt,'_',Subject.side,'-init_reversed.mat'];
            Namefull  = [Mvt,'_',Subject.side,'_','full-init_reversed.mat'];
            O_flange = flip(O_flange,3);
            Q_flange = flip(Q_flange,3);
            save(Name,'O_flange','Q_flange');
            save(Namefull);         
        case 8
            Mvt       = 'HFE';  
            Ord       = '03';   
            Name      = [Ord,'_',Mvt,'_',Subject.side,'.mat'];
            Namefull  = [Mvt,'_',Subject.side,'_','full.mat'];
            save(Name,'O_flange','Q_flange');
            save(Namefull);          
        case 9
            Mvt       = 'VT';
            Ord       = '04';   
            Name      = [Ord,'_',Mvt,'_',Subject.side,'-init.mat'];
            Namefull  = [Mvt,'_',Subject.side,'_','full-init.mat'];
            save(Name,'O_flange','Q_flange');
            save(Namefull);  
            
            Mvt       = 'VT';
            Ord       = '04';   
            Name      = [Ord,'_',Mvt,'_',Subject.side,'-init_reversed.mat'];
            Namefull  = [Mvt,'_',Subject.side,'_','full-init_reversed.mat'];
            O_flange = flip(O_flange,3);
            Q_flange = flip(Q_flange,3);
            save(Name,'O_flange','Q_flange');
            save(Namefull);           
        case 10
            Mvt       = 'VT';  
            Ord       = '04';   
            Name      = [Ord,'_',Mvt,'_',Subject.side,'.mat'];
            Namefull  = [Mvt,'_',Subject.side,'_','full.mat'];
            save(Name,'O_flange','Q_flange');
            save(Namefull);         
        case 11
            Mvt       = 'HC';
            Ord       = '05';   
            Name      = [Ord,'_',Mvt,'_',Subject.side,'-init.mat'];
            Namefull  = [Mvt,'_',Subject.side,'_','full-init.mat'];
            save(Name,'O_flange','Q_flange');
            save(Namefull);  
            
            Mvt       = 'HC';
            Ord       = '05';   
            Name      = [Ord,'_',Mvt,'_',Subject.side,'-init_reversed.mat'];
            Namefull  = [Mvt,'_',Subject.side,'_','full-init_reversed.mat'];
            O_flange = flip(O_flange,3);
            Q_flange = flip(Q_flange,3);
            save(Name,'O_flange','Q_flange');
            save(Namefull);          
        case 12
            Mvt       = 'HC';  
            Ord       = '05';   
            Name      = [Ord,'_',Mvt,'_',Subject.side,'.mat'];
            Namefull  = [Mvt,'_',Subject.side,'_','full.mat'];
            save(Name,'O_flange','Q_flange');
            save(Namefull);           
    end  
    clear Name Namefull Num Side Ord Mvt;

end