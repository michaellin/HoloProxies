%% Authors: Michael Lin and Alexa Siu for CS231A final project
%% create the region of 3D voxel
x = -99:100;
y = -99:100;
z = -99:100;

[X, Y, Z] = meshgrid(x, y, z);

voxels = [reshape(X, [200^3, 1]), reshape(Y, [200^3, 1]), reshape(Z, [200^3, 1])];

%% Load the .obj file
Obj = read_wobj('objs/redbox.obj');
V = Obj.vertices;
F = Obj.objects(3).data.vertices;
%% Calculate the signed distance function

[D, S, C] = signed_distance_direction(voxels, V, F);
% Outputs:
%   D  #P by dim list of normalized directions
%   S  #P signed distances
%   C  #P by dim list of closest points
%% write result to binary file
fileID = fopen('SDFs/redbox.bin', 'w');
fwrite(fileID, S, 'float32');
fclose(fileID);