%% Sensors and Control - Control and Grasping of Dobot
% Edan Anonuevo - 13925490 (Contribution - 50%)
% Ricardo Rios - 13208335 (Contribution - 50%)
% Shan-E Hassan Syed Jaffery - 13844105 (Contribution - 0%)

close all;

%% Camera Calibration
% Read Rosbag
calibBag = rosbag('calibration.bag');
calibBagImagesRGB = select(calibBag, 'Topic', '/device_0/sensor_1/Color_0/image/data');

totalFrames = calibBagImagesRGB.NumMessages;
numImages = 35;
numFrames = round(totalFrames/numImages)-1;
calibImage = cell(1,numImages);
calibImageRGB = cell(1,numImages);

for i = 1:numImages
    calibImage{i} = readMessages(calibBagImagesRGB, i*numFrames);
    calibImageRGB{i} = readImage(calibImage{i}{1});
end

ImageFolder = '/home/ricrios/snc/Assignment 2';
for i = 1:numImages
    filename = sprintf('image%d.jpg',i);
    imgName = fullfile(ImageFolder,filename);
    imwrite(calibImageRGB{i},imgName);
end

% Calibration
files = cell(1,numImages);
for i = 1:numImages
    files{i} = fullfile(ImageFolder,sprintf('image%d.jpg', i));
end

I = imread(files{1});
figure;
imshow(I);

[imagePoints, boardSize] = detectCheckerboardPoints(files);

squareSize = 25;
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

imageSize = [size(I, 1), size(I,2)];
cameraParams = estimateCameraParameters(imagePoints, worldPoints, ImageSize=imageSize);
intrinsics = cameraParams.Intrinsics;

focalLength = cameraParams.Intrinsics.FocalLength;
principalPoint = cameraParams.Intrinsics.PrincipalPoint;

%% Feature Extraction
% Read rosbags
coordBag = rosbag('coordinate.bag');
coordBagImages = select(coordBag, 'Topic', '/device_0/sensor_1/Color_0/image/data');
coordBagDepth = select(coordBag, 'Topic', '/device_0/sensor_0/Depth_0/image/data');

% Depth image
depthMSG = readMessages(coordBagDepth,1);
depthImage = readImage(depthMSG{1});

% RGB image
coordImage = readMessages(coordBagImages, 1);
coordImageRGB = readImage(coordImage{1});

% Save as jpg and read
filename = sprintf('coordImage.jpg');
imgName = fullfile(ImageFolder,filename);
imwrite(coordImageRGB,imgName);
featureImage = imread('coordImage.jpg');

% Create mask of red to get coords
[BW,maskedRGBImage] = createMask(featureImage);
edges = edge(BW, 'Canny');                          % Edge detection
cc = bwconncomp(edges);                             % Blob detection
stats = regionprops(cc, 'Area', 'Centroid');
centroids = cat(1, stats.Centroid);                 % Find the centroids of all regions

% Display the centroids on the original image
figure
imshow(maskedRGBImage);
figure
imshow(BW);
figure
imshow(featureImage);
hold on;
plot(centroids(1, 1), centroids(1, 2), 'go', 'MarkerSize', 10);     % Cube
plot(centroids(4, 1), centroids(4, 2), 'go', 'MarkerSize', 10);     % End-Effector
hold off;

%% Coordinates
% Image coordinates
u_DoBot = (centroids(2,1));
v_DoBot = (centroids(2,2));
u_Cube = (centroids(1,1));
v_Cube = (centroids(1,2));

% Calibration points
cx = principalPoint(1,1);   % principal point
cy = principalPoint(1,2);   % principal point
fx = focalLength(1,1);      % focal length
fy = focalLength(1,2);      % focal length

% Depth values
Z_DoBot = double(depthImage(round(u_DoBot),round(v_DoBot)))/1000;   % depth value end-effector point
Z_Cube = double(depthImage(round(u_Cube),round(v_Cube)))/1000;      % depth value cube point 

% Known global frame (end-effector)
x_known = 0;
y_known = -0.200;
z_known = 0.010;

% Cube coordinates
x_norm = (u_Cube - cx) / fx;
y_norm = (v_Cube - cy) / fy;
x_cam = x_norm * Z_Cube;
y_cam = y_norm * Z_Cube;
x_global = x_known + x_cam;
y_global = y_known + y_cam;
z_global = z_known + Z_Cube;

% Dobot coordinates
dobotX = -y_global;
dobotY = x_global;
dobotZ = z_global;
dobotXYZ = [dobotX, dobotY, dobotZ];

disp(dobotXYZ);

%% Dobot Control
% Initialise ROS
rosshutdown;                            % Shutdown 
rosinit;                                % Start Dobot Magician Node
dobot = DobotMagician();                

% Publish custom end-effector pose
rotation = [0,0,0];
startPos = [0.2,0,0.01];
pickupPos = [dobotX,dobotY,0.00];
depositPos = [0.2,0.2,0.00];

pickupOver = [pickupPos(1),pickupPos(2),pickupPos(3)+0.03];
pickupDown = pickupPos;
depositOver = [depositPos(1),depositPos(2),depositPos(3)+0.03];
depositDown = depositPos;

% Tool Variables
toolOpen = 0;
toolOn = 1;
toolClose = 1;
toolOff = 0;

% Pickup Cube
dobot.PublishEndEffectorPose(pickupOver,rotation);          % Move over cube
pause(8)
dobot.PublishToolState(toolOn,toolOpen);                    % Start tool open
pause(5)
dobot.PublishEndEffectorPose(pickupDown,rotation);          % Move onto cube
pause(6)
dobot.PublishToolState(toolOn,toolClose);                   % Pick up and close tool
pause(5)
dobot.PublishEndEffectorPose(pickupOver,rotation);          % Move up
pause(6)

% Deposit Cube
dobot.PublishEndEffectorPose(depositOver,rotation);         % Move over zone
pause(10)
dobot.PublishEndEffectorPose(depositDown,rotation);         % Move onto zone
pause(6)
dobot.PublishToolState(toolOn,toolOpen);                    % Deposit and open tool
pause(5)
dobot.PublishEndEffectorPose(depositOver,rotation);         % Move up
pause(6)

% Return To Start Turn Off Tool
dobot.PublishEndEffectorPose(startPos,rotation);            % Move onto zone
pause(8)
dobot.PublishToolState(toolOff,toolOpen);                   % Deposit and open tool
pause(5)

%% Functions
function [BW,maskedRGBImage] = createMask(RGB) 

    % Convert RGB image to HSV image
    I = rgb2hsv(RGB);
    
    % Define thresholds for 'Hue'. Modify these values to filter out different range of colors.
    channel1Min = 0.965;
    channel1Max = 0.188;
    
    % Define thresholds for 'Saturation'
    channel2Min = 0.400;
    channel2Max = 1.000;
    
    % Define thresholds for 'Value'
    channel3Min = 0.450;
    channel3Max = 1.000;
    
    % Create mask based on chosen histogram thresholds
    BW = ((I(:,:,1)>=channel1Min) | (I(:,:,1)<=channel1Max)) & ...
        (I(:,:,2)>=channel2Min) & (I(:,:,2)<=channel2Max) & ...
        (I(:,:,3)>=channel3Min) & (I(:,:,3)<=channel3Max);
    
    % Initialize output masked image based on input image.
    maskedRGBImage = RGB;
    
    % Set background pixels where BW is false to zero.
    maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
end