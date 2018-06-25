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
Aresta = 4;
Rota = 0;
PontoInicial = [2*squareSize,2*squareSize];

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
    
    PontosHom = P*[PontoInicial(1);PontoInicial(2);0;1];
    PontosUV = [PontosHom(1)/PontosHom(3) PontosHom(2)/PontosHom(3)];
    
    PontosHom2 = P*[PontoInicial(1)+(cosd(Rota)*(squareSize*Aresta));PontoInicial(2)+(sind(Rota)*(squareSize*Aresta));0;1];
    PontosUV2 = [PontosHom2(1)/PontosHom2(3) PontosHom2(2)/PontosHom2(3)];
    
    PontosHom3 = P*[PontoInicial(1)+((cosd(Rota)-sind(Rota))*(squareSize*Aresta));PontoInicial(2)+((cosd(Rota)+sind(Rota))*(squareSize*Aresta));0;1];
    PontosUV3 = [PontosHom3(1)/PontosHom3(3) PontosHom3(2)/PontosHom3(3)];
    
    PontosHom4 = P*[PontoInicial(1)-(sind(Rota)*(squareSize*Aresta));PontoInicial(2)+(cosd(Rota)*(squareSize*Aresta));0;1];
    PontosUV4 = [PontosHom4(1)/PontosHom4(3) PontosHom4(2)/PontosHom4(3)];
    
    PontosHom5 = P*[PontoInicial(1);PontoInicial(2);-(squareSize*Aresta);1];
    PontosUV5 = [PontosHom5(1)/PontosHom5(3) PontosHom5(2)/PontosHom5(3)];
    
    PontosHom6 = P*[PontoInicial(1)+(cosd(Rota)*(squareSize*Aresta));PontoInicial(2)+(sind(Rota)*(squareSize*Aresta));-(squareSize*Aresta);1];
    PontosUV6 = [PontosHom6(1)/PontosHom6(3) PontosHom6(2)/PontosHom6(3)];
    
    PontosHom7 = P*[PontoInicial(1)+((cosd(Rota)-sind(Rota))*(squareSize*Aresta));PontoInicial(2)+((cosd(Rota)+sind(Rota))*(squareSize*Aresta));-(squareSize*Aresta);1];
    PontosUV7 = [PontosHom7(1)/PontosHom7(3) PontosHom7(2)/PontosHom7(3)];
    
    PontosHom8 = P*[PontoInicial(1)-(sind(Rota)*(squareSize*Aresta));PontoInicial(2)+(cosd(Rota)*(squareSize*Aresta));-(squareSize*Aresta);1];
    PontosUV8 = [PontosHom8(1)/PontosHom8(3) PontosHom8(2)/PontosHom8(3)];
    
    x = [PontosUV(1);PontosUV5(1);PontosUV6(1);PontosUV2(1)];
    y = [PontosUV(2);PontosUV5(2);PontosUV6(2);PontosUV2(2)];
    
    x2 = [PontosUV6(1);PontosUV2(1);PontosUV3(1);PontosUV7(1)];
    y2 = [PontosUV6(2);PontosUV2(2);PontosUV3(2);PontosUV7(2)];
    
    x3 = [PontosUV3(1);PontosUV7(1);PontosUV8(1);PontosUV4(1)];
    y3 = [PontosUV3(2);PontosUV7(2);PontosUV8(2);PontosUV4(2)];
    
    x4 = [PontosUV8(1);PontosUV4(1);PontosUV(1);PontosUV5(1)];
    y4 = [PontosUV8(2);PontosUV4(2);PontosUV(2);PontosUV5(2)];
    
    x5 = [PontosUV5(1);PontosUV6(1);PontosUV7(1);PontosUV8(1)];
    y5 = [PontosUV5(2);PontosUV6(2);PontosUV7(2);PontosUV8(2)];
    
    % Display the image.
    imshow(rgbImage);
    hold on;
    
    patch(x3,y3,'b', 'LineStyle', 'none')
    patch(x4,y4,'b', 'LineStyle', 'none')
    patch(x2,y2,'b', 'LineStyle', 'none')
    patch(x,y,'b', 'LineStyle', 'none')
    patch(x5,y5,'b', 'LineStyle', 'none')
    
end