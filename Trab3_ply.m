clear all
close all
clc

% run('Calibracao.m');
% close all
% clc

cameraParams = open('cameraParams.mat');

%Get the intrinsic matrix
IntriMatrix = cameraParams.cameraParams.IntrinsicMatrix;

cam = webcam;
squareSize = 24; %in mm

ptCloud = pcread('Armadillo.ply');
stepSize = round(ptCloud.Count/10900);
indices = 1:stepSize:ptCloud.Count;
ptCloudOut = select(ptCloud, indices);
verts = ptCloudOut.Location;
verts = verts*[1 0 0;0 -1 0;0 0 -1];
maxi = max( verts(:,3));
verts(:,1) = verts(:,1)+3*24; %posição inicial em x
vertstemp = verts(:,2);
verts(:,2) = verts(:,3)+1*24; %posição inicial em y
verts(:,3) = vertstemp-maxi; %posição inicial em z

% ptCloud = pcread('teapot.ply');
% stepSize = round(ptCloud.Count/10900);
% indices = 1:stepSize:ptCloud.Count;
% ptCloudOut = select(ptCloud, indices);
% verts = ptCloudOut.Location;
% verts = verts.*15;
% verts = verts*[1 0 0;0 1 0;0 0 -1];
% maxi = max( verts(:,3));
% verts(:,1) = verts(:,1)+3*24; %posição inicial em x
% verts(:,2) = verts(:,2)+1*24; %posição inicial em y
% verts(:,3) = verts(:,3)-maxi; %posição inicial em z

while(true)
    % Acquire a single image.
    rgbImage = snapshot(cam);
    rgbImage = undistortImage(rgbImage, cameraParams.cameraParams);
    bwImage = imono(rgbImage);
    
    %imagePoints = icorner(rgbImage);
    [imagePoints,boardSize] = detectCheckerboardPoints(rgbImage);
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    
    %Calculate the extrinsic matrix
    [rotationMatrix, translationVector] = extrinsics(imagePoints,worldPoints, cameraParams.cameraParams);
    
    P = cameraMatrix(cameraParams.cameraParams,rotationMatrix,translationVector)';
    
    PontoInicial = [worldPoints(1,1),worldPoints(1,2)];
    
    vertsX = [verts , ones(size(verts,1),1)];
    
    vertsHom = P*vertsX(:,:)';
    
    for i=1:size(vertsHom,2)
        vertsUV(i,1) = vertsHom(1,size(vertsHom,2)-i+1)/vertsHom(3,size(vertsHom,2)-i+1);
        vertsUV(i,2) = vertsHom(2,size(vertsHom,2)-i+1)/vertsHom(3,size(vertsHom,2)-i+1);
    end
    
    % Display the image.
    imshow(bwImage);
    hold on;
    
    plot(vertsUV(:,1),vertsUV(:,2),'.');
    
end