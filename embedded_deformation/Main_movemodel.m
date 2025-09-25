%   Move the model to user definded direction
% Open one model

[filename, pathName] = uigetfile('*.stl', 'Select first model');
[fout1, vout1, cout1] = ReadSTLACSII(filename) ;
figure(1);
trisurf ( fout1, vout1(:,1), vout1(:,2), vout1(:,3));
axis equal;
xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis');

center_mass = mean(vout1);
vout1 = vout1 - repmat(center_mass,[size(vout1,1),1]);
%filename = 'new';
stlwrite(filename,fout1,vout1,'mode','ascii');