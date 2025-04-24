function laneDetectionRealtime_dynamicThresholdWithOccupancyMap
% Lane line detection based on camera images, with dynamic threshold adjustment and occupancy map generation
% Process:
% 1. Apply bilateral filtering on a single-channel grayscale image to reduce noise while preserving edges;
% 2. Dynamically adjust the threshold based on the average brightness of the whole image (linear interpolation):
%       If avgBrightness <= 13, threshold = 12;
%       If avgBrightness >= 150, threshold = 175;
%       Otherwise: greythreshold = 1.0788 * avgBrightness + 4.0248;
% 3. Apply morphological operations to the binary result to obtain candidate lane areas;
% 4. Construct an "inverse mask" based on the morphological result, making the lane area red [255,0,0], and the background black;
% 5. Use the camera's intrinsic and extrinsic parameters (camera position (0,0,0.8), pitch angle = -30°) to construct a homography matrix (H). Since imwarp does an "image-to-ground" transformation, it is necessary to invert the previously constructed "ground-to-image" H;
% 6. Map the result to a set ground coordinate range (x ∈ [-1,3] meters, y ∈ [-1,1] meters, resolution 0.05 meters/pixel) to generate an occupancy map (lane = 1, others = 0);
% 7. Display in a 2-row × 4-column subplot in real-time: original image, filtered result, thresholded image, intermediate morphological result, overlay image, occupancy map, and text information.

%% 1. Initialize ROS2 node and Subscriber
node = ros2node("test_node");
imageSub = ros2subscriber(node, "/multisense/left/image_rect", "sensor_msgs/Image", ...
    "Reliability", "besteffort", "Durability", "volatile", "History", "keeplast", "Depth", 10);

%% 2. Create real-time display window (2 rows × 4 columns)
hFig = figure('Name', 'Real-time Lane Detection, Dynamic Threshold & Occupancy Map', 'NumberTitle', 'off');

while isvalid(hFig)
    overallTimer = tic;
    
    %% 1. Image reception
    try
        tPreReceive = tic;
        imgMsg = receive(imageSub, 5);  % Wait for up to 5 seconds
        rawImg = rosReadImage(imgMsg);  % Single-channel grayscale image, size 1024×544
        tReceive = toc(tPreReceive);
    catch ME
        warning('Failed to receive image: %s', ME.message);
        continue;
    end
    
    %% 2. Bilateral filtering: reduce noise while preserving edges
    tPreBlur = tic;
    blurredImg = imbilatfilt(rawImg);
    tBlur = toc(tPreBlur);
    
    %% 3. Dynamic threshold (linear interpolation)
    tPreThreshold = tic;
    avgBrightness = mean(rawImg(:));
    
    if avgBrightness <= 13
        greythreshold = 12;
    elseif avgBrightness >= 150
        greythreshold = 175;
    else
        % Simple linear relation
        greythreshold = 1.0788 * avgBrightness + 4.0248;
    end
    
    extractGraph = rawImg < greythreshold; 
    tThreshold = toc(tPreThreshold);
    
    %% 4. Morphological operation: enhance continuity of lane line areas
    tPreMorph = tic;
    laneLineMask = processEdges(extractGraph);
    tMorph = toc(tPreMorph);
    
    %% 5. Construct "overlay image", make lane areas red
    tPreOverlay = tic;
    overlay = createOverlay(laneLineMask);
    tOverlay = toc(tPreOverlay);
    
    %% 6. Generate occupancy grid map (perspective transformation to ground coordinates)
    tPreOcc = tic;
    % 6.1 Set output ground coordinate range and resolution
    xWorldLimits = [-1, 5];   % x-direction [-1, 5] meters
    yWorldLimits = [-2, 2];   % y-direction [-2, 2] meters
    pixelExtent = 0.01;       % Resolution = 0.01 meters/pixel
    outputCols = round(diff(xWorldLimits)/pixelExtent);  % Number of pixels in width direction
    outputRows = round(diff(yWorldLimits)/pixelExtent);  % Number of pixels in height direction
    outputRef = imref2d([outputRows, outputCols], xWorldLimits, yWorldLimits);
    
    % 6.2 Camera intrinsic parameters (approximate values for MultiSense S7 2MP scaled to 1024×544)
    K = [595,   0,   512;
          0,  590,   272;
          0,    0,     1];
    
    % 6.3 Camera extrinsic parameters: camera pose: C = (0,0,0.8), pitch = -30°
    theta = -30 * pi/180;
    % Rotation matrix around y-axis (only considering pitch)
    R = [ cos(theta),  0, sin(theta);
          0,           1, 0;
         -sin(theta),  0, cos(theta) ];
    C = [0; 0; 0.8];
    
    % Translation vector (since after rotation, the camera position is translated to the origin, so t_extr = -R*C)
    t_extr = -R * C;
    
    % 6.4 Ground-to-image homography matrix H_plane2image
    % On the ground (Z=0): [X; Y; 1] → s*[u; v; 1] = K * [R(:,1:2), t_extr] * [X; Y; 1]
    H_plane2image = K * [R(:,1:2), t_extr];
    
    % -------- Key modification: inverse transformation from image to ground --------
    % The original H_plane2image is a ground-to-image mapping; imwarp needs the image-to-ground mapping
    % Therefore, we need to take H_image2plane = inv(H_plane2image)
    H_image2plane = inv(H_plane2image);
    % When transforming to projective2d, use the transpose: projective2d(H'). So the matrix is passed transposed
    tform = projective2d(H_image2plane');
    
    % 6.5 Extract red pixels (lane area) from the overlay image, and project to the ground
    redChannel = overlay(:,:,1);
    occupancyInput = redChannel > 200;  % Lane area
    
    % Use imwarp to map to the "ground coordinate system" and output to the custom range outputRef
    [occupancyMap, ~] = imwarp(occupancyInput, tform, 'OutputView', outputRef);
    occupancyMap = occupancyMap > 0.5; 
    tOcc = toc(tPreOcc);
    
    overallTime = toc(overallTimer);
    
    %% 7. Display results in blocks (2 rows × 4 columns)
    clf(hFig);
    
    subplot(2,4,1);
    imshow(rawImg, []);
    title(sprintf('Original Image\n(Receive: %.3f s)', tReceive));
    
    subplot(2,4,2);
    imshow(blurredImg, []);
    title(sprintf('Bilateral Filtering\n(%.3f s)', tBlur));
    
    subplot(2,4,3);
    imshow(extractGraph, []);
    title(sprintf('Threshold (<%d)\n(%.3f s)', round(greythreshold), tThreshold));
    
    subplot(2,4,4);
    imshow(laneLineMask, []);
    title(sprintf('Morphological Processing\n(%.3f s)', tMorph));
    
    subplot(2,4,5);
    imshow(overlay);
    title(sprintf('Overlay Image\n(%.3f s)', tOverlay));
    
    subplot(2,4,6);
    imshow(occupancyMap, outputRef);
    title(sprintf('Occupancy Map\n(%.3f s)', tOcc));
    
    subplot(2,4,7);
    txt = sprintf(['Total Frame Processing Time: %.3f s\n' ...
                   'Receive: %.3f s\nBilateral Filtering: %.3f s\nThreshold: %.3f s\nMorphological: %.3f s\n' ...
                   'Average Brightness: %d\nDynamic Threshold: %d'], ...
                   overallTime, tReceive, tBlur, tThreshold, tMorph, ...
                   round(avgBrightness), round(greythreshold));
    text(0.1, 0.5, txt, 'FontSize', 12);
    axis off;
    
    disp(['Total Frame Time: ', num2str(overallTime), ' s  |  Image Receive: ', num2str(tReceive), ' s']);
end
end

%% ----------------------- Auxiliary Functions -------------------------------
%% Morphological processing to enhance continuity of lane lines
function enhancedEdges = processEdges(edgeImg)
    % First erode, then reconstruct, then dilate
    se = strel('rectangle', [1, 2]);
    marker = imerode(edgeImg, se);
    reconstructedEdges = imreconstruct(marker, edgeImg);
    enhancedEdges = imdilate(reconstructedEdges, strel('rectangle', [2, 6]));
end

%% Generate overlay image: make lane areas red
function overlay = createOverlay(laneLineMask)
    % Note: laneLineMask is true for the background, false for the lane
    % We want the lane to be red → invert (~laneLineMask) and set the red channel
    overlay = zeros([size(laneLineMask), 3], 'uint8');
    overlay(:,:,1) = 255 * uint8(~laneLineMask);  % Red channel
    % Other channels are 0
end
