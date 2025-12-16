function final_code_v2()
% Phase-1 demo v2: Image-based RRT* + LIDAR + Local Path Following
% + runtime metrics (success rate, collision count, replanning latency, clearance, path length)
%
% Save as: phase1_demo_final_v2.m

clc; clear; close all;

disp('🌍 Select a floor-plan image (PNG/JPG/BMP)...');
[filename, pathname] = uigetfile({'*.png;*.jpg;*.bmp','Image Files'});
if isequal(filename,0)
    disp('❌ No image selected.'); return;
end

%% Load and preprocess image
imgPath = fullfile(pathname, filename);
img = imread(imgPath);
if size(img,3)==3, grayImg = rgb2gray(img); else, grayImg = img; end
grayImg = double(grayImg)/255;
bw = grayImg < 0.5;
bw = imresize(bw, [300 400]);      % rows x cols
disp('✅ Map loaded successfully.');

figure; imshow(~bw); title('Processed Binary Occupancy Map');
disp('📍 Select START point'); [sx, sy] = ginput(1);
disp('🎯 Select GOAL point'); [gx, gy] = ginput(1);
start = [sx, sy]; goal = [gx, gy];
fprintf('Start = [%.1f %.1f], Goal = [%.1f %.1f]\n', start, goal);

[rows, cols] = size(bw);
mapSize = [0 cols; 0 rows]; % [xmin xmax; ymin ymax]
xgv = linspace(mapSize(1,1), mapSize(1,2), cols);
ygv = linspace(mapSize(2,1), mapSize(2,2), rows);
isFree = @(pt) ~isOccupied(pt, bw, xgv, ygv);

%% RRT* initial plan
disp('🧭 Running initial RRT*...');
tic; 
path = rrtstar(start, goal, isFree, [mapSize(1,:) mapSize(2,:)], 2000, 12, 25);
initPlanTime = toc;
if isempty(path)
    warning('No path found; using straight line.'); path = [start; goal];
end
path = smoothdata(path,'gaussian',7);

%% Simulation setup
robotPose = [start, atan2(path(2,2)-path(1,2), path(2,1)-path(1,1))];
traj = robotPose(1:2);
dt = 0.2; v_nom = 5; maxSteps = 1200;
numBeams = 72; lidarRange = 60;
replanThreshold = 18;    % if obstacle closer than this in front, trigger replan
replanCooldown = 10;     % steps between replanning attempts
lastReplanStep = -inf;

% Metrics initialization
metrics.collisionCount = 0;
metrics.steps = 0;
metrics.minClearances = []; % per step
metrics.replans = 0;
metrics.replanLatencies = [];
metrics.pathLengths = path_length(path);
metrics.success = false;

figure('Position',[100 100 1000 600]);

% Video setup
v = VideoWriter('phase1_demo_final_v2.mp4','MPEG-4'); v.FrameRate = 30; open(v);

for step=1:maxSteps
    metrics.steps = metrics.steps + 1;
    % Simulated LIDAR
    angles = linspace(-pi,pi,numBeams);
    lidarPts = nan(numBeams,2);
    lidarDists = nan(numBeams,1);
    for i=1:numBeams
        a = angles(i)+robotPose(3);
        r=0;
        hit=false;
        while r<lidarRange
            r=r+1; % step resolution 1 px
            pt = robotPose(1:2)+r*[cos(a),sin(a)];
            if ~isFree(pt)
                hit=true; break;
            end
        end
        if hit
            lidarPts(i,:) = pt;
            lidarDists(i) = r;
        else
            lidarPts(i,:) = robotPose(1:2) + lidarRange*[cos(a),sin(a)];
            lidarDists(i) = lidarRange;
        end
    end

    % record min front clearance
    frontIdx = abs(angles) < pi/6;
    minFront = min(lidarDists(frontIdx));
    metrics.minClearances(end+1) = minFront;

    % Visualization
    clf; imshow(~bw); hold on;
    plot(path(:,1),path(:,2),'-b','LineWidth',2);
    plot(goal(1),goal(2),'g*','MarkerSize',10);
    plot(traj(:,1),traj(:,2),'-r','LineWidth',1.5);
    plot(lidarPts(:,1),lidarPts(:,2),'.y','MarkerSize',3);
    theta = linspace(0,2*pi,40);
    plot(robotPose(1)+5*cos(theta), robotPose(2)+5*sin(theta),'Color',[0 0.6 0.2],'LineWidth',1);
    title(sprintf('Step %d',step));
    drawnow;

    % Save frame
    frame = getframe(gcf); writeVideo(v, frame);

    %% Path-following: pure pursuit
    lookahead = 20;
    dists = vecnorm(path - robotPose(1:2),2,2);
    [~,nearestIdx] = min(dists);
    targetIdx = min(nearestIdx + round(lookahead/5), size(path,1));
    target = path(targetIdx,:);
    dirVec = target - robotPose(1:2);

    desiredTheta = atan2(dirVec(2), dirVec(1));
    angErr = wrapToPi(desiredTheta - robotPose(3));

    % Mild reactive avoidance
    frontDists = lidarDists(frontIdx);
    minFront = min(frontDists);
    avoidTerm = 0;
    if minFront < 20
        % steer away from side with denser obstacles
        leftMean = mean(lidarDists(angles>0 & abs(angles)<pi/2));
        rightMean = mean(lidarDists(angles<0 & abs(angles)<pi/2));
        sideSign = sign(rightMean - leftMean); % positive -> steer right
        avoidTerm = sideSign * (20 - minFront)/20;
    end

    wcmd = 0.6*angErr + 0.35*avoidTerm;
    vcmd = v_nom * exp(-abs(angErr));

    % Update pose (simple unicycle)
    nextPose = robotPose;
    nextPose(1) = robotPose(1) + vcmd*dt*cos(robotPose(3));
    nextPose(2) = robotPose(2) + vcmd*dt*sin(robotPose(3));
    nextPose(3) = wrapToPi(robotPose(3) + wcmd*dt);

    % collision check for next pose
    if isOccupied(nextPose(1:2),bw,xgv,ygv)
        metrics.collisionCount = metrics.collisionCount + 1;
        vcmd=0;
        nextPose(3)=wrapToPi(robotPose(3)+0.3*randn);
    end

    robotPose = nextPose;
    traj=[traj; robotPose(1:2)];

    % Replanning trigger (only every replanCooldown steps)
    if minFront < replanThreshold && (step - lastReplanStep) > replanCooldown
        fprintf('⚠️ Obstacle close (%.2f). Triggering replanning at step %d...\n', minFront, step);
        tstart = tic;
        newPath = rrtstar(robotPose(1:2), goal, isFree, [mapSize(1,:) mapSize(2,:)], 1200, 12, 25);
        replTime = toc(tstart);
        if ~isempty(newPath)
            metrics.replans = metrics.replans + 1;
            metrics.replanLatencies(end+1) = replTime;
            path = smoothdata(newPath,'gaussian',7);
            metrics.pathLengths(end+1) = path_length(path);
            lastReplanStep = step;
            fprintf('🔁 Replan succeeded (%.3f s). New path length %.2f\n', replTime, metrics.pathLengths(end));
        else
            fprintf('❌ Replan failed (%.3f s). Keeping old path.\n', replTime);
        end
    end

    % goal check (threshold in pixels)
    if norm(robotPose(1:2)-goal) < 10
        disp('🎯 Goal reached!');
        metrics.success = true;
        break;
    end

end

% Close video
close(v);

% Final metrics calculation
totalSteps = metrics.steps;
collisionRate = metrics.collisionCount / totalSteps;
meanClearance = mean(metrics.minClearances);
avgReplanLatency = nan;
if ~isempty(metrics.replanLatencies)
    avgReplanLatency = mean(metrics.replanLatencies);
end
finalPathLength = metrics.pathLengths(end);

% Print metrics
fprintf('\n=== Simulation Metrics ===\n');
fprintf('Success: %d\n', metrics.success);
fprintf('Total steps: %d\n', totalSteps);
fprintf('Collision count: %d\n', metrics.collisionCount);
fprintf('Collision rate (per step): %.4f\n', collisionRate);
fprintf('Mean min-front clearance: %.2f px\n', meanClearance);
fprintf('Number of replans: %d\n', metrics.replans);
fprintf('Average replanning latency: %.3f s\n', avgReplanLatency);
fprintf('Final path length: %.2f px\n', finalPathLength);
fprintf('Video saved as phase1_demo_final_v2.mp4\n');

% Save metrics to CSV
metricsTable = table(metrics.success, totalSteps, metrics.collisionCount, collisionRate, meanClearance, metrics.replans, avgReplanLatency, finalPathLength, 'VariableNames', ...
    {'Success','TotalSteps','CollisionCount','CollisionRate','MeanMinFrontClearance','NumReplans','AvgReplanLatency_s','FinalPathLength'});
writetable(metricsTable,'phase1_metrics.csv');
disp('✅ Metrics saved to phase1_metrics.csv');

end

%% ---------------- Helper functions ----------------
function occ=isOccupied(pt,bw,xgv,ygv)
% Improved occupancy: safe index rounding & out-of-bounds = occupied
[rows,cols]=size(bw);
% Map continuous coordinates to image indices
% clamp to limits and mark outside as occupied
xi = round( (pt(1)-xgv(1)) / (xgv(end)-xgv(1)) * (cols-1) ) + 1;
yi = round( (pt(2)-ygv(1)) / (ygv(end)-ygv(1)) * (rows-1) ) + 1;
if xi<1 || xi>cols || yi<1 || yi>rows
    occ = true;
else
    occ = bw(yi, xi) == 1;
end
end

function L = path_length(path)
% simple path length in Euclidean pixels
if isempty(path), L = inf; return; end
d = diff(path,1,1);
L = sum(sqrt(sum(d.^2,2)));
end

function path=rrtstar(start,goal,mapFcn,bounds,nSamples,stepSize,neighR)
% Simple RRT* implementation with safer parent/cost bookkeeping
% nodes: Nx2
nodes = start;
parent = 0;          % parent(1) = 0 for start
cost = 0;            % cost(1) = 0
foundGoal = false;

for iter=1:nSamples
    if rand < 0.08
        sample = goal + 0.5*(rand(1,2)-0.5);
    else
        sample = [bounds(1) + rand*(bounds(2)-bounds(1)), bounds(3) + rand*(bounds(4)-bounds(3))];
    end

    % nearest
    dif = nodes - sample;
    dists = sqrt(sum(dif.^2,2));
    [~, idx] = min(dists);

    direction = sample - nodes(idx,:);
    if norm(direction)==0, continue; end
    extension = nodes(idx,:) + stepSize * direction / norm(direction);

    % collision check: check midpoint and extension point
    if ~mapFcn((nodes(idx,:) + extension)/2) || ~mapFcn(extension)
        continue;
    end

    % add node
    nodes = [nodes; extension];
    parent(end+1) = idx;
    cost(end+1) = cost(idx) + norm(extension - nodes(idx,:));

    % rewiring: neighbors (excluding the new node itself)
    neighIdx = find(sqrt(sum((nodes - extension).^2,2)) < neighR);
    neighIdx(neighIdx == size(nodes,1)) = []; % remove self if present
    for ni = neighIdx'
        potCost = cost(end) + norm(nodes(ni,:) - extension);
        if potCost < cost(ni) && mapFcn((nodes(ni,:) + extension)/2)
            parent(ni) = size(nodes,1);
            cost(ni) = potCost;
        end
    end

    % check close to goal
    if norm(extension - goal) < stepSize && mapFcn((extension + goal)/2) && mapFcn(goal)
        nodes = [nodes; goal];
        parent(end+1) = size(nodes,1)-1;
        foundGoal = true;
        break;
    end
end

if foundGoal
    p = size(nodes,1);
    path = nodes(p,:);
    while parent(p) > 0
        p = parent(p);
        path = [nodes(p,:); path];
    end
else
    path = [];
end
end
