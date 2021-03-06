% Author     :   F. Moissenet
%                Kinesiology Laboratory (K-LAB)
%                University of Geneva
%                https://www.unige.ch/medecine/kinesiology
% License    :   Creative Commons Attribution-NonCommercial 4.0 International License 
%                https://creativecommons.org/licenses/by-nc/4.0/legalcode
% Source code:   https://github.com/fmoissenet/Roboshoulder_Toolbox
% Reference  :   To be defined
% Date       :   March 2020
% -------------------------------------------------------------------------
% Description:   This routine aims to get the bones geometry and related
%                pose/kinematics from CTscan derived STL images and mocap 
%                marker trajectories, respectively. 
%                It is part of the toolkit grouping several methods used in 
%                the RoboShoulder project, a joined project with the HEPIA 
%                school at Geneva.
% -------------------------------------------------------------------------
% Dependencies : - SphereFit: http://www.mathworks.com/matlabcentral/fileexchange/22643
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
toolboxFolder = 'C:\Users\moissene\Documents\Switchdrive\Projets\RoboShoulder\Donn�es\Roboshoulder_toolbox\';
dataFolder    = 'C:\Users\moissene\Documents\Switchdrive\Projets\RoboShoulder\Donn�es\RS001\Imagerie\STL\';
exportFolder  = 'C:\Users\moissene\Documents\Switchdrive\Projets\RoboShoulder\Donn�es\RS001\Matlab\';
depFolder     = [toolboxFolder,'dependencies\']; % these binaries are prepared for a recent version of Matlab and Windows 8 or newer
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
% SET 3D FIGURE
% -------------------------------------------------------------------------
figure(); hold on; axis equal;
legend off;

%% ------------------------------------------------------------------------
% DEFINE RIGHT HUMERUS
% -------------------------------------------------------------------------

% Get the bone geometry from STL file
% -------------------------------------------------------------------------
stlFile                   = [Subject.id,'_',Subject.side,'_Mesh_HumerusWithCluster.stl'];
Bone(1).name              = 'RHumerus';
temp                      = stlread(stlFile);
Bone(1).Geometry.vertices = temp.Points;
Bone(1).Geometry.faces    = temp.ConnectivityList;
clear temp;
patch('Faces',Bone(1).Geometry.faces(:,1:3),...
      'Vertices',Bone(1).Geometry.vertices(:,1:3),...
      'FaceColor', [0.7 0.7 0.7],'EdgeColor','none','FaceLighting','gouraud');
xlabel('x'); ylabel('y'); zlabel('z');
light('position',[1.5 1.5 0.75],'Style','infinite');

% Set the humeral head centre by sphere fitting
% Set a proximo-distal view first to click many on top nodes
% -------------------------------------------------------------------------
disp('Right humerus head centre definition');
pause();
nPoints = 50; % Number of points to be selected for the sphere fitting
for i = 1:nPoints
    dcm_obj        = datacursormode(1);
    set(dcm_obj,'DisplayStyle','window','SnapToDataVertex','off',...
        'Enable','on');
    waitforbuttonpress;
    c_info{i,1}    = getCursorInfo(dcm_obj);
    positions(i,:) = eval('c_info{i,1}.Position');
end
[Bone(1).Marker.GJC(1,:),~] = sphereFit(positions);
clear dcm_obj c_info positions nPoints;
plot3(Bone(1).Marker.GJC(1,1),...
      Bone(1).Marker.GJC(1,2),...
      Bone(1).Marker.GJC(1,3),...
      'Marker','o','MarkerSize',150,'LineWidth',2,'Color','blue');
  
% Set the first cluster marker by sphere fitting (cRHUM01)
% -------------------------------------------------------------------------
disp('Right humerus cluster marker 1');
pause();
nPoints = 20; % Number of points to be selected for the sphere fitting
for i = 1:nPoints
    dcm_obj        = datacursormode(1);
    set(dcm_obj,'DisplayStyle','window','SnapToDataVertex','off',...
        'Enable','on');
    waitforbuttonpress;
    c_info{i,1}    = getCursorInfo(dcm_obj);
    positions(i,:) = eval('c_info{i,1}.Position');
end
[Bone(1).Marker.cRHUM01(1,:),~] = sphereFit(positions);
clear dcm_obj c_info positions nPoints;
plot3(Bone(1).Marker.cRHUM01(1,1),...
      Bone(1).Marker.cRHUM01(1,2),...
      Bone(1).Marker.cRHUM01(1,3),...
      'Marker','o','MarkerSize',35,'LineWidth',2,'Color','red');  
  
% Set the second cluster marker by sphere fitting (cRHUM02)
% -------------------------------------------------------------------------
disp('Right humerus cluster marker 2');
pause();
nPoints = 20; % Number of points to be selected for the sphere fitting
for i = 1:nPoints
    dcm_obj        = datacursormode(1);
    set(dcm_obj,'DisplayStyle','window','SnapToDataVertex','off',...
        'Enable','on');
    waitforbuttonpress;
    c_info{i,1}    = getCursorInfo(dcm_obj);
    positions(i,:) = eval('c_info{i,1}.Position');
end
[Bone(1).Marker.cRHUM02(1,:),~] = sphereFit(positions);
clear dcm_obj c_info positions nPoints;
plot3(Bone(1).Marker.cRHUM02(1,1),...
      Bone(1).Marker.cRHUM02(1,2),...
      Bone(1).Marker.cRHUM02(1,3),...
      'Marker','o','MarkerSize',35,'LineWidth',2,'Color','red');   
  
% Set the third cluster marker by sphere fitting (cRHUM03)
% -------------------------------------------------------------------------
disp('Right humerus cluster marker 3');
pause();
nPoints = 20; % Number of points to be selected for the sphere fitting
for i = 1:nPoints
    dcm_obj        = datacursormode(1);
    set(dcm_obj,'DisplayStyle','window','SnapToDataVertex','off',...
        'Enable','on');
    waitforbuttonpress;
    c_info{i,1}    = getCursorInfo(dcm_obj);
    positions(i,:) = eval('c_info{i,1}.Position');
end
[Bone(1).Marker.cRHUM03(1,:),~] = sphereFit(positions);
clear dcm_obj c_info positions nPoints;
plot3(Bone(1).Marker.cRHUM03(1,1),...
      Bone(1).Marker.cRHUM03(1,2),...
      Bone(1).Marker.cRHUM03(1,3),...
      'Marker','o','MarkerSize',35,'LineWidth',2,'Color','red'); 
  
% Set the fourth cluster marker by sphere fitting (cRHUM04)
% -------------------------------------------------------------------------
disp('Right humerus cluster marker 4');
pause();
nPoints = 20; % Number of points to be selected for the sphere fitting
for i = 1:nPoints
    dcm_obj        = datacursormode(1);
    set(dcm_obj,'DisplayStyle','window','SnapToDataVertex','off',...
        'Enable','on');
    waitforbuttonpress;
    c_info{i,1}    = getCursorInfo(dcm_obj);
    positions(i,:) = eval('c_info{i,1}.Position');
end
[Bone(1).Marker.cRHUM04(1,:),~] = sphereFit(positions);
clear dcm_obj c_info positions nPoints;
plot3(Bone(1).Marker.cRHUM04(1,1),...
      Bone(1).Marker.cRHUM04(1,2),...
      Bone(1).Marker.cRHUM04(1,3),...
      'Marker','o','MarkerSize',35,'LineWidth',2,'Color','red'); 

%% ------------------------------------------------------------------------
% DEFINE RIGHT CLAVILE
% -------------------------------------------------------------------------

% Get the bone geometry from STL file
% -------------------------------------------------------------------------
stlFile                   = [Subject.id,'_',Subject.side,'_Mesh_ClavicleWithCluster.stl'];
Bone(2).name              = 'RClavicule';
temp                      = stlread(stlFile);
Bone(2).Geometry.vertices = temp.Points;
Bone(2).Geometry.faces    = temp.ConnectivityList;
clear temp;
patch('Faces',Bone(2).Geometry.faces(:,1:3),...
      'Vertices',Bone(2).Geometry.vertices(:,1:3),...
      'FaceColor', [0.7 0.7 0.7],'EdgeColor','none','FaceLighting','gouraud');
xlabel('x'); ylabel('y'); zlabel('z');
light('position',[1.5 1.5 0.75],'Style','infinite');

% Set the first cluster marker by sphere fitting (cRCLA01)
% -------------------------------------------------------------------------
disp('Right clavicle cluster marker 1');
pause();
nPoints = 20; % Number of points to be selected for the sphere fitting
for i = 1:nPoints
    dcm_obj        = datacursormode(1);
    set(dcm_obj,'DisplayStyle','window','SnapToDataVertex','off',...
        'Enable','on');
    waitforbuttonpress;
    c_info{i,1}    = getCursorInfo(dcm_obj);
    positions(i,:) = eval('c_info{i,1}.Position');
end
[Bone(2).Marker.cRCLA01(1,:),~] = sphereFit(positions);
clear dcm_obj c_info positions nPoints;
plot3(Bone(2).Marker.cRCLA01(1,1),...
      Bone(2).Marker.cRCLA01(1,2),...
      Bone(2).Marker.cRCLA01(1,3),...
      'Marker','o','MarkerSize',35,'LineWidth',2,'Color','red');  
  
% Set the second cluster marker by sphere fitting (cRCLA02)
% -------------------------------------------------------------------------
disp('Right clavicle cluster marker 2');
pause();
nPoints = 20; % Number of points to be selected for the sphere fitting
for i = 1:nPoints
    dcm_obj        = datacursormode(1);
    set(dcm_obj,'DisplayStyle','window','SnapToDataVertex','off',...
        'Enable','on');
    waitforbuttonpress;
    c_info{i,1}    = getCursorInfo(dcm_obj);
    positions(i,:) = eval('c_info{i,1}.Position');
end
[Bone(2).Marker.cRCLA02(1,:),~] = sphereFit(positions);
clear dcm_obj c_info positions nPoints;
plot3(Bone(2).Marker.cRCLA02(1,1),...
      Bone(2).Marker.cRCLA02(1,2),...
      Bone(2).Marker.cRCLA02(1,3),...
      'Marker','o','MarkerSize',35,'LineWidth',2,'Color','red');   
  
% Set the third cluster marker by sphere fitting (cRCLA03)
% -------------------------------------------------------------------------
disp('Right clavicle cluster marker 3');
pause();
nPoints = 20; % Number of points to be selected for the sphere fitting
for i = 1:nPoints
    dcm_obj        = datacursormode(1);
    set(dcm_obj,'DisplayStyle','window','SnapToDataVertex','off',...
        'Enable','on');
    waitforbuttonpress;
    c_info{i,1}    = getCursorInfo(dcm_obj);
    positions(i,:) = eval('c_info{i,1}.Position');
end
[Bone(2).Marker.cRCLA03(1,:),~] = sphereFit(positions);
clear dcm_obj c_info positions nPoints;
plot3(Bone(2).Marker.cRCLA03(1,1),...
      Bone(2).Marker.cRCLA03(1,2),...
      Bone(2).Marker.cRCLA03(1,3),...
      'Marker','o','MarkerSize',35,'LineWidth',2,'Color','red');
  
% Set the fourth cluster marker by sphere fitting (cRCLA04)
% -------------------------------------------------------------------------
disp('Right clavicle cluster marker 4');
pause();
nPoints = 20; % Number of points to be selected for the sphere fitting
for i = 1:nPoints
    dcm_obj        = datacursormode(1);
    set(dcm_obj,'DisplayStyle','window','SnapToDataVertex','off',...
        'Enable','on');
    waitforbuttonpress;
    c_info{i,1}    = getCursorInfo(dcm_obj);
    positions(i,:) = eval('c_info{i,1}.Position');
end
[Bone(2).Marker.cRCLA04(1,:),~] = sphereFit(positions);
clear dcm_obj c_info positions nPoints;
plot3(Bone(2).Marker.cRCLA04(1,1),...
      Bone(2).Marker.cRCLA04(1,2),...
      Bone(2).Marker.cRCLA04(1,3),...
      'Marker','o','MarkerSize',35,'LineWidth',2,'Color','red');     

%% ------------------------------------------------------------------------
% DEFINE RIGHT SCAPULA
% -------------------------------------------------------------------------

% Get the bone geometry from STL file
% -------------------------------------------------------------------------
stlFile                   = [Subject.id,'_',Subject.side,'_Mesh_ScapulaWithCluster.stl';
Bone(3).name              = 'RScapula';
temp                      = stlread(stlFile);
Bone(3).Geometry.vertices = temp.Points;
Bone(3).Geometry.faces    = temp.ConnectivityList;
clear temp;
patch('Faces',Bone(3).Geometry.faces(:,1:3),...
      'Vertices',Bone(3).Geometry.vertices(:,1:3),...
      'FaceColor', [0.7 0.7 0.7],'EdgeColor','none','FaceLighting','gouraud');
xlabel('x'); ylabel('y'); zlabel('z');
light('position',[1.5 1.5 0.75],'Style','infinite');

% Set the first cluster marker by sphere fitting (cRSCA01)
% -------------------------------------------------------------------------
disp('Right scapula cluster marker 1');
pause();
nPoints = 20; % Number of points to be selected for the sphere fitting
for i = 1:nPoints
    dcm_obj        = datacursormode(1);
    set(dcm_obj,'DisplayStyle','window','SnapToDataVertex','off',...
        'Enable','on');
    waitforbuttonpress;
    c_info{i,1}    = getCursorInfo(dcm_obj);
    positions(i,:) = eval('c_info{i,1}.Position');
end
[Bone(3).Marker.cRSCA01(1,:),~] = sphereFit(positions);
clear dcm_obj c_info positions nPoints;
plot3(Bone(3).Marker.cRSCA01(1,1),...
      Bone(3).Marker.cRSCA01(1,2),...
      Bone(3).Marker.cRSCA01(1,3),...
      'Marker','o','MarkerSize',35,'LineWidth',2,'Color','red');  

% Set the second cluster marker by sphere fitting (cRSCA02)
% -------------------------------------------------------------------------
disp('Right scapula cluster marker 2');
pause();
nPoints = 20; % Number of points to be selected for the sphere fitting
for i = 1:nPoints
    dcm_obj        = datacursormode(1);
    set(dcm_obj,'DisplayStyle','window','SnapToDataVertex','off',...
        'Enable','on');
    waitforbuttonpress;
    c_info{i,1}    = getCursorInfo(dcm_obj);
    positions(i,:) = eval('c_info{i,1}.Position');
end
[Bone(3).Marker.cRSCA02(1,:),~] = sphereFit(positions);
clear dcm_obj c_info positions nPoints;
plot3(Bone(3).Marker.cRSCA02(1,1),...
      Bone(3).Marker.cRSCA02(1,2),...
      Bone(3).Marker.cRSCA02(1,3),...
      'Marker','o','MarkerSize',35,'LineWidth',2,'Color','red');  

% Set the third cluster marker by sphere fitting (cRSCA03)
% -------------------------------------------------------------------------
disp('Right scapula cluster marker 3');
pause();
nPoints = 20; % Number of points to be selected for the sphere fitting
for i = 1:nPoints
    dcm_obj        = datacursormode(1);
    set(dcm_obj,'DisplayStyle','window','SnapToDataVertex','off',...
        'Enable','on');
    waitforbuttonpress;
    c_info{i,1}    = getCursorInfo(dcm_obj);
    positions(i,:) = eval('c_info{i,1}.Position');
end
[Bone(3).Marker.cRSCA03(1,:),~] = sphereFit(positions);
clear dcm_obj c_info positions nPoints;
plot3(Bone(3).Marker.cRSCA03(1,1),...
      Bone(3).Marker.cRSCA03(1,2),...
      Bone(3).Marker.cRSCA03(1,3),...
      'Marker','o','MarkerSize',35,'LineWidth',2,'Color','red');  

% Set the fourth cluster marker by sphere fitting (cRSCA04)
% -------------------------------------------------------------------------
disp('Right scapula cluster marker 4');
pause();
nPoints = 20; % Number of points to be selected for the sphere fitting
for i = 1:nPoints
    dcm_obj        = datacursormode(1);
    set(dcm_obj,'DisplayStyle','window','SnapToDataVertex','off',...
        'Enable','on');
    waitforbuttonpress;
    c_info{i,1}    = getCursorInfo(dcm_obj);
    positions(i,:) = eval('c_info{i,1}.Position');
end
[Bone(3).Marker.cRSCA04(1,:),~] = sphereFit(positions);
clear dcm_obj c_info positions nPoints;
plot3(Bone(3).Marker.cRSCA04(1,1),...
      Bone(3).Marker.cRSCA04(1,2),...
      Bone(3).Marker.cRSCA04(1,3),...
      'Marker','o','MarkerSize',35,'LineWidth',2,'Color','red'); 

%% ------------------------------------------------------------------------
% EXPORT RESULTS
% -------------------------------------------------------------------------
cd(exportFolder);
save([Subject.id,'_',Subject.side,'_GetBonePose.mat']);
  
%% ------------------------------------------------------------------------
% LOAD FOLLOWING LINES INSTEAD OF PREVIOUS ONES IF BONE STRUCTURE ALREADY
% BUILT
% -------------------------------------------------------------------------
clearvars;
matFile = 'C:\Users\moissene\Documents\Switchdrive\Projets\RoboShoulder\Donn�es\RS001\Matlab\RS001_R_GetBonePose.mat';
load(matFile);
figure(); hold on; axis equal; legend off;

for i = 1:length(Bone)
    patch('Faces',Bone(i).Geometry.faces(:,1:3),...
          'Vertices',Bone(i).Geometry.vertices(:,1:3),...
          'FaceColor', [0.7 0.7 0.7],'EdgeColor','none',...
          'FaceLighting','gouraud','FaceAlpha',0.7);
    xlabel('x'); ylabel('y'); zlabel('z');
    light('position',[1.5 1.5 0.75],'Style','infinite');
    nMarker = fieldnames(Bone(i).Marker);
    for j = 1:length(nMarker)
        if i == 1
            if j == 1 % humerus head centre
                plot3(Bone(i).Marker.(nMarker{j})(1,1),...
                      Bone(i).Marker.(nMarker{j})(1,2),...
                      Bone(i).Marker.(nMarker{j})(1,3),...
                      'Marker','+','MarkerSize',15,'LineWidth',2,'Color','blue');        
            else
                plot3(Bone(i).Marker.(nMarker{j})(1,1),...
                      Bone(i).Marker.(nMarker{j})(1,2),...
                      Bone(i).Marker.(nMarker{j})(1,3),...
                      'Marker','o','MarkerSize',20,'LineWidth',2,'Color','red');         
            end
        else
            plot3(Bone(i).Marker.(nMarker{j})(1,1),...
                  Bone(i).Marker.(nMarker{j})(1,2),...
                  Bone(i).Marker.(nMarker{j})(1,3),...
                  'Marker','o','MarkerSize',15,'LineWidth',2,'Color','red');                
        end
    end
end