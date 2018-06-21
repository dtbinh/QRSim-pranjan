%YY = randi(400, 2, 3) - 200;
%a = YY(1, :)
%b = YY(2, :)
%
% a = [-22  8  -12];
% b = [-10 -0  -48];

function draw_spheroid(a, b, len_minor)
c = (a+b) / 2;
D = norm(c-a);
major_ax = b-a;
len_major = sqrt(power(D,2) + power(len_minor, 2)); %Major axis length
a1 = a(1); a2 = a(2); a3 = a(3);
b1 = b(1); b2 = b(2); b3 = b(3);
num_points = 20; %number of points on ellipse
%Project on XY plane
% scatter3(a1, a2, a3, 'Magenta', 'filled');
% scatter3(b1, b2, b3, 'Magenta', 'filled');
% scatter3(c(1), c(2), c(3), 'Magenta', 'filled');
line([a1 b1], [a2, b2], [a3, b3]);

v1 = [1 0];
v2 = [major_ax(1) major_ax(2)];
theta_z = rad2deg(acos(dot(v1, v2)/(norm(v1) * norm(v2))));

%Project on XZ plane
v1 = [1 0];
v2 = [major_ax(1) major_ax(3)];
theta_y = rad2deg(acos(dot(v1, v2)/(norm(v1) * norm(v2))));

if a2 < b2
    theta_z = - theta_z;
    theta_y = -theta_y;
end
[x, y, z] = ellipsoid(c(1), c(2), c(3), len_major,len_minor,len_minor,num_points);
S = surf(x, y, z, 'FaceAlpha',0.2, 'edgecolor','none');

% alpha 0.2;
% shading interp;

%To fix final rotation :
%Minor axis:
m1 = sin(deg2rad(-theta_y));
m3 = cos(deg2rad(-theta_y));
m11 = m1*cos(deg2rad(-theta_z)); %m2*..
m22 = m1*sin(deg2rad(-theta_z));
m33 = m3;

%Rotation vector
r = cross([m11 m22 m33], major_ax);


%Major axis:
m1 = cos(deg2rad(-theta_y));
m3 = -sin(deg2rad(-theta_y));
m11 = m1*cos(deg2rad(-theta_z)) - 0; %m2*..
m22 = m1*sin(deg2rad(-theta_z));
m33 = m3;

max_i = [m11 m22 m33];

%rotate(S, [1 0 0], 45, c)
rot_angle = rad2deg(atan2(norm(cross(max_i, major_ax)), dot(max_i, major_ax)));
if a1 > b1
    rot_angle = - rot_angle;
end
if a2 < b2
    rot_angle = - rot_angle;
end

%Rotate along Y and Z axis
rotate(S, [0,1,0], -theta_y, c)
rotate(S, [0,0,1], -theta_z, c)
rotate(S, r, rot_angle, c);
end