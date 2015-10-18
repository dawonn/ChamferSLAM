clc


% Scan ROI Settings
step          = 1;       % Scans
%start         = 100;    % Scan Index (Set in setup.m)
%stop          = 10000;  % Scan Index (Set in setup.m)
skip          = 1;       % Points


% Framework Options
verbose              = false;
debugplots           = true;         

usePrevOffsetAsGuess = false;        % Constant Velocity assumption
useScan2World        = true;         % Scan to Map vs Scan to Scan

MaxVelocityLin       = 4;            % (Meters  / second   )
MaxVelocityRot       = deg2rad(90);  % (Radians / second   )
MaxAccelLin          = 0.5;          % (Meters  / second^2 )
MaxAccelRot          = deg2rad(60);  % (Radians / second^2 )

MapBorderSize        = 1;            % (Meters )
MapPixelSize         = 0.05;         % (Meters )

% ChamferSLAM parameters
SearchResolutionLin  = 0.05;         % (Meters )
SearchResolutionRot  = deg2rad(0.5); % (Radians )
MaxItterations       = 30;           % (Itterations)
MaxDepth             = 3;            % (Search Depth)

% Update map after distance traveled. 
UpdateMapDT          = 0.1;          % (Meters)
UpdateMapDR          = deg2rad(5);   % (Radians)




% Initialize State Variables
nScanIndex = unique(Lidar_ScanIndex);

map        = [];
pose       = [0 0 0];
path       = [0 0 0];
world      = [];
T          = [0 0 0];
init_guess = [0 0 0];
LastMapUpdatePose = [0 0 0];
SearchRange = [];
WorldUpdated = true;


% Clear all figures before running
for i = 1:3
    clearallplots( i );
end



% Scan Matching Loop
startTime = tic;
stopIdx = min(stop,size(nScanIndex,1));
for scanIdx = start:step:stopIdx
    
    % Display current scan index
    fprintf('ScanMatcher: Scan %d / %d\n', scanIdx, stopIdx);
    
    
    % Get Current Scan
    [scan, pol] = getLidarXY(scanIdx, nScanIndex, Lidar_Angles, Lidar_Ranges, Lidar_ScanIndex, ...
                                     'LidarRange', 30);

    
    % Timestamp (missing data compensation)
    stamp = Lidar_Timestamp_Sensor(scanIdx);
       
    
    % Get the orientation from the IMU for each hit
    %Fusion_Q = interp1(IMU_Timestamp, IMU_Q, stamp);

    % Rotate all points by their orientation
    %Fusion_scan = quatrotate(Fusion_Q, [scan, zeros(size(scan,1),1)]);
    
    % Remove points that are probably on the ground or ceiling
    %I = abs(Fusion_scan(:,3)) < 0.2;
    %scan = scan(I, [1,2]);
    
    % Init World
    if isempty(map)
        % Init map and world
        path  = pose;
        world = map;
        map   = scan;
        
        % Skip to next scan
        prev_stamp = stamp;
        continue
    end
    
    
    % Generate a local map from the world map
    if useScan2World
        
        if WorldUpdated
          % Translate current scan to map coordinates
          dx    = init_guess(1);
          dy    = init_guess(2);
          theta = init_guess(3);
          
          M = [ cos(theta) -sin(theta) dx ;
                sin(theta)  cos(theta) dy ;
                0           0          1  ];
          
          scanWorldFrame = [scan ones(size(scan,1), 1)];
          scanWorldFrame = scanWorldFrame * M';
          scanWorldFrame = scanWorldFrame(:,[1,2]);
          
          % extract points around the current scan for a reference map
          map = map(map(:,1) > min(scanWorldFrame(:,1)) - MapBorderSize, :);
          map = map(map(:,1) < max(scanWorldFrame(:,1)) + MapBorderSize, :);
          map = map(map(:,2) > min(scanWorldFrame(:,2)) - MapBorderSize, :);
          map = map(map(:,2) < max(scanWorldFrame(:,2)) + MapBorderSize, :);
        end
    end
    
    
    % Search area
    if usePrevOffsetAsGuess
        rmax = MaxAccelRot; %#ok<UNRCH>
        tmax = MaxAccelLin;
    else
        rmax = MaxVelocityRot;
        tmax = MaxVelocityLin;
    end
  
    dt   = min(stamp - prev_stamp, 0.5);    % limit data loss time.
    rmax = rmax * dt;                       
    tmax = tmax * dt;                       
    rmax = max(rmax, SearchResolutionRot);  
    tmax = max(tmax, SearchResolutionLin);  
    
    % Rotation range limit
    rmax = min(rmax,  deg2rad(180));
    rmax = max(rmax, -deg2rad(180));
  
    % Search area history
    SearchRange = [SearchRange; tmax rmax]; %#ok<AGROW>
    
    
    % Initial Guess
    T = init_guess;
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Chmafer SLAM 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if WorldUpdated
         % Generate Occupancy Grid from map
        ogrid = oGrid(map, MapPixelSize);

        % Generate chamfer distance map
        Dmap  = bwdist(ogrid.grid);
    end

    [T, hits] = chamferMatch(T, scan, ogrid, Dmap,   ...
                              'SearchRot'      , SearchResolutionRot, ...
                              'SearchLin'      , SearchResolutionLin, ...
                              'MaxItterations' , MaxItterations, ...
                              'MaxDepth'       , MaxDepth );
    score  = sum(hits);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    
    
    % Update current pose
    if useScan2World
        pose = T;
    else
        % Rotate translation into map frame
        theta = pose(3); %#ok<UNRCH>
        Trans = [ cos(theta) -sin(theta) 0 ;

                  sin(theta)  cos(theta) 0 ;
                  0           0          1 ];
        mapT  = Trans * T';


        % Add previous scan to pose
        pose = pose + [mapT(1:2)', T(3)];
    end

    path(end+1,:) = pose; %#ok<SAGROW>
    
    % Make Sure Scan in proper Coordinates
    tempScan = scan;
    
    
    % Current World Pose Transform
    dx    = pose(1);
    dy    = pose(2);
    theta = pose(3);
    
    Trans = [ cos(theta) -sin(theta) dx ;
              sin(theta)  cos(theta) dy ;
              0           0          1  ];
    temp =  [tempScan ones(size(tempScan,1),1)] * Trans';
    
    
    % Current Scan Transformation
    dx    = T(1);
    dy    = T(2);
    theta = T(3);
    
    LTrans = [ cos(theta) -sin(theta) dx ;
               sin(theta)  cos(theta) dy ;
               0           0          1  ];
    tempL = [tempScan ones(size(tempScan,1),1)] * LTrans';
    
    
    
    % Add transformed data to world
    if useScan2World
                
        % Update map after distance traveled. 
        dp = abs(LastMapUpdatePose - path(end, :));
        if (dp(1) > UpdateMapDT) || ...
           (dp(2) > UpdateMapDT) || ... 
           (dp(3) > UpdateMapDR)
            LastMapUpdatePose = path(end, :);
                
            % Only add new points to the map 
            I = ~logical(hits);
            newpts = tempL(I, 1:2);            
            world = [world; newpts]; %#ok<AGROW>

            WorldUpdated = true;
        else
            WorldUpdated = false;
        end
        
    else
        world = [world; temp(:,1:2)]; %#ok<UNRCH>
    end
    
    
    
    % Debug Plots
    if debugplots  && mod(length(path), 400) == 0 
  
        % Limit number of points in the map
        MaxMapSize = 100000;
        if size(world,1) > MaxMapSize           
            I = randsample(size(world,1), MaxMapSize);
            %I = (size(map,1)-MaxMapSize):size(map,1);
            tmpW = world(I,:);
        else
            tmpW = world;
        end
        
        % Plot World
        change_current_figure(1);
        clf
        plot(tmpW(:,1), tmpW(:,2), 'k.', 'MarkerSize', 1)
        hold on
        plot(path(:,1), path(:,2), 'r.')
        axis equal
        title(['Scan: ' num2str(scanIdx)]);
        drawnow
  
        %set(gcf,'PaperUnits','inches','PaperPosition', [0 0 8.5 11]);
        print([ OutPath DatasetName '-dbg'],'-dpdf');
    
        tmpW = [];
        
        % Plot Transformed and Map and Scans
        tempMap = map; 
        
        change_current_figure(2);
        clf
        hold on
        plot(tempMap(:,1),tempMap(:,2),'r.', 'MarkerSize', 1)
        if useScan2World
            plot(scanWorldFrame(:,1),scanWorldFrame(:,2),'b.', 'MarkerSize', 1)
        else
            plot(tempScan(:,1),tempScan(:,2),'b.', 'MarkerSize', 1) %#ok<UNRCH>
        end
        plot(tempL(:,1),tempL(:,2),'g.', 'MarkerSize', 1)
        hold off
        axis equal
        title(['Scan: ' num2str(scanIdx)]);
        legend('Reference', 'Current Scan', 'Registered Scan')
        
    end
    
    
    % Select the map for the next scan
    if useScan2World
    
        if WorldUpdated
            map = world;
        end
        
    else
        map = scan; %#ok<UNRCH>
    end
    
    % Set next search starting location
    if useScan2World
        if usePrevOffsetAsGuess
            init_guess = T + (path(end, :) - path(end-1, :)); %#ok<UNRCH>
        else
            init_guess = T;
        end
    else
        if usePrevOffsetAsGuess %#ok<UNRCH>
            init_guess = T;
        end
    end 
    
    % Timestamp 
    prev_stamp = stamp;
    
    
end


exectime = toc(startTime);
realTime = Lidar_Timestamp_Sensor(scanIdx) - Lidar_Timestamp_Sensor(start);
fprintf('ScanMatcher: %.2f / %.2f (exe/log) seconds = %0.4f \n', ...
         exectime, realTime, exectime / realTime)



% Plot World

% Limit number of points in the map
MaxMapSize = 100000;
if size(world,1) > MaxMapSize           
    I = randsample(size(world,1), MaxMapSize);
    map = world(I,:);
else
    map = world;
end
 
change_current_figure(1);
clf
hold on
plot(map(:,1), map(:,2), 'k.', 'MarkerSize', 1)
plot(path(:,1), path(:,2), 'r.', 'MarkerSize', 2);
axis equal
title(['Scan: ' num2str(scanIdx)]);

hgsave([ OutPath DatasetName])

set(gcf,'PaperUnits','inches','PaperPosition', [0 0 8.5 11]);
print([ OutPath DatasetName],'-dpdf');


% Plot dT
fl = 5;
n = 1;
change_current_figure(2);
clf
subplot(3,1,1);
plot(diff(path(:,1),n) , 'r.')
hold on
plot( SearchRange(:,1), 'b.')
plot(-SearchRange(:,1), 'b.')
title(['X: diff(path(:,1),' num2str(n) ')'])
tmp  = diff(path(:,1),n);
tmp2 = conv(tmp, ones(fl,1) / fl);
plot(tmp2, '-b')

subplot(3,1,2);
plot(diff(path(:,2),n), 'r.')
hold on
plot( SearchRange(:,1), 'b.')
plot(-SearchRange(:,1), 'b.')
title(['Y: diff(path(:,2),' num2str(n) ')'])
tmp  = diff(path(:,2),n);
tmp2 = conv(tmp, ones(fl,1) / fl);
plot(tmp2, '-b')

subplot(3,1,3);
plot(rad2deg(diff(path(:,3),n)), 'r.')
hold on
plot( rad2deg(SearchRange(:,2)), 'b.')
plot(-rad2deg(SearchRange(:,2)), 'b.')
title(['Z: diff(path(:,3),' num2str(n) ')'])
tmp  = rad2deg(diff(path(:,3),n));
tmp2 = conv(tmp, ones(fl,1) / fl);
plot(tmp2, '-r')

print( [OutPath DatasetName '-pathDiff1'],'-dpng');




