function laneDetectionRealtime_dynamicThresholdWithOccupancyMap
% Lane line detection based on camera images, with dynamic threshold adjustment and occupancy map generation
% Processing flow:
% 1. Apply bilateral filtering on the single-channel grayscale image to reduce noise;
% 2. Determine a threshold value based on the average brightness of the image (linear interpolation):
%       If avgBrightness <= 13, set threshold to 12;
%       If avgBrightness >= 150, set threshold to 175;
%       Otherwise: greythreshold = 1.0788 * avgBrightness + 4.0248;
% 3. Perform morphological operations on the thresholded image to obtain candidate lane regions;
% 4. Generate an edge overlay image: invert the mask so that lane regions appear red [255,0,0], and background is black;
% 5. Use camera intrinsic and extrinsic parameters (camera position (0,0,0.8), pitch=-30°) to construct a homography matrix,
%    and use a custom OutputView (assuming the ground plane range x ∈ [-1,3] meters, y ∈ [-1,1] meters, resolution 0.05 m/pixel)
%    to perform a perspective transform on the red portion in the overlay, generating an occupancy map (lane = 1, others = 0);
% 6. Display using a 2×4 subplot layout: original image, filtered image, thresholding result, morphological result,
%    edge overlay, occupancy map, and real-time textual info.

%% Initialize ROS2 node and subscriber
node = ros2node("test_node");
imageSub = ros2subscriber(node, "/multisense/left/image_rect", "sensor_msgs/Image", ...
    "Reliability", "besteffort", "Durability", "volatile", "History", "keeplast", "Depth", 10);

%% Create a real-time display figure (2 rows × 4 columns)
hFig = figure('Name', 'Real-time Lane Detection, Dynamic Threshold & Occupancy Map', 'NumberTitle', 'off');

while isvalid(hFig)
    overallTimer = tic;
    
    %% 1. Image reception
    try
        tPreReceive = tic;
        imgMsg = receive(imageSub, 5);  % Wait up to 5 seconds
        rawImg = rosReadImage(imgMsg);  % Single-channel grayscale, size 1024×544
        tReceive = toc(tPreReceive);
    catch ME
        warning('Failed to receive image: %s', ME.message);
        continue;
    end
    
    %% 2. Bilateral filtering: denoise while preserving edges
    tPreBlur = tic;
    blurredImg = imbilatfilt(rawImg);
    tBlur = toc(tPreBlur);
    
    %% 3. Dynamic thresholding (using linear interpolation)
    tPreThreshold = tic;
    avgBrightness = mean(rawImg(:));

    if avgBrightness <= 13
        greythreshold = 12;
    elseif avgBrightness >= 150
        greythreshold = 175;
    else
        % Example polynomial was commented out in Chinese code; replaced by a simpler linear form
        greythreshold = 1.0788 * avgBrightness + 4.0248;
    end

    extractGraph = rawImg < greythreshold;
    tThreshold = toc(tPreThreshold);
    
    %% 4. Morphological operations: enhance the continuity of lane line regions
    tPreMorph = tic;
    laneLineMask = processEdges(extractGraph);
    tMorph = toc(tPreMorph);
    
    %% 5. Generate the edge overlay image
    % Assume laneLineMask indicates lane line regions as false
    % Invert the mask so lane regions are red [255,0,0], background black
    tPreOverlay = tic;
    overlay = createOverlay(laneLineMask);
    tOverlay = toc(tPreOverlay);
    
    %% 6. Generate occupancy map (perspective transform onto ground plane)
    tPreOcc = tic;
    % Custom OutputView: assume ground plane range x ∈ [-1, 3] m, y ∈ [-1, 1] m, resolution 0.05 m/pixel
    xWorldLimits = [-1, 3];
    yWorldLimits = [-1, 1];
    pixelExtent = 0.05;
    outputCols = round(diff(xWorldLimits)/pixelExtent);
    outputRows = round(diff(yWorldLimits)/pixelExtent);
    outputRef = imref2d([outputRows, outputCols], xWorldLimits, yWorldLimits);
    
    % Camera intrinsics (in pixels), adapted for MultiSense S7 2MP to current image size 1024×544
    K = [595,   0, 512;
         0,   590, 272;
         0,     0,   1];
    
    % Camera extrinsics: camera location C = (0,0,0.8) in meters, Euler angles (roll, pitch, yaw) = (0, -30, 0)
    theta = -30 * pi/180;  % Convert -30° to radians
    % Rotation about y-axis (pitch rotation)
    R = [ cos(theta),  0, sin(theta);
          0,           1, 0;
         -sin(theta),  0, cos(theta) ];
    C = [0; 0; 0.8];
    t_extr = -R * C;
    
    % Construct homography H: for points on the ground plane (Z=0) [X; Y; 1],
    % s*[u; v; 1] = K * [r1, r2, t_extr] * [X; Y; 1]
    H = K * [R(:,1:2), t_extr];
    tform = projective2d(H');
    % Extract the red portion (lane region) from overlay: pick red channel > 200 for a binary image
    redChannel = overlay(:,:,1);
    occupancyInput = redChannel > 200;
    
    % Perspective transform of occupancyInput to ground plane
    [occupancyMap, ~] = imwarp(occupancyInput, tform, 'OutputView', outputRef);
    occupancyMap = occupancyMap > 0.5;
    tOcc = toc(tPreOcc);
    
    overallTime = toc(overallTimer);
    
    %% 7. Display each processing stage result (2 rows × 4 columns)
    clf(hFig);
    
    subplot(2,4,1);
    imshow(rawImg, []);
    title(sprintf('Original Image\n(Receive: %.3f s)', tReceive));
    
    subplot(2,4,2);
    imshow(blurredImg, []);
    title(sprintf('Bilateral Filtering\n(%.3f s)', tBlur));
    
    subplot(2,4,3);
    imshow(extractGraph, []);
    title(sprintf('Threshold ( <%d )\n(%.3f s)', round(greythreshold), tThreshold));
    
    subplot(2,4,4);
    imshow(laneLineMask, []);
    title(sprintf('Morphological Ops\n(%.3f s)', tMorph));
    
    subplot(2,4,5);
    imshow(overlay);
    title(sprintf('Edge Overlay\n(%.3f s)', tOverlay));
    
    subplot(2,4,6);
    imshow(occupancyMap, outputRef);
    title(sprintf('Occupancy Map\n(%.3f s)', tOcc));
    
    subplot(2,4,7);
    txt = sprintf(['Entire Frame Processing: %.3f s\n' ...
                   'Receive: %.3f s\nBilateral: %.3f s\nThreshold: %.3f s\nMorph: %.3f s\n' ...
                   'Average Brightness: %d\nDynamic Threshold: %d'], ...
                   overallTime, tReceive, tBlur, tThreshold, tMorph, round(avgBrightness), round(greythreshold));
    text(0.1, 0.5, txt, 'FontSize', 12);
    axis off;
    
    disp(['Entire Frame: ', num2str(overallTime), ' s  |  Image Receive: ', num2str(tReceive), ' s']);
end
end

%% Helper function: Morphological operations (enhance continuity of lane lines)
function enhancedEdges = processEdges(edgeImg)
    % Perform erosion, reconstruction, and dilation on the binary image to enhance lane continuity
    se = strel('rectangle', [1, 2]);
    marker = imerode(edgeImg, se);
    reconstructedEdges = imreconstruct(marker, edgeImg);
    enhancedEdges = imdilate(reconstructedEdges, strel('rectangle', [2, 6]));
end

%% Helper function: Generate edge overlay image
function overlay = createOverlay(laneLineMask)
    % Generate color overlay:
    % Assume laneLineMask indicates lane line regions as false
    % Invert the mask so lane regions appear red [255,0,0], background black
    overlay = zeros([size(laneLineMask), 3], 'uint8');
    overlay(:,:,1) = 255 * uint8(~laneLineMask);  % Red channel
    overlay(:,:,2) = 0;
    overlay(:,:,3) = 0;
end
