clc;
clear all;

%Change the A and B values here for different cases
a1 = 10; a2 = 4; a3 = 20;
b1 = 0; b2 = -11; b3 = 10;
len_major = 5.9; %Major axis length
len_minor = 3.5; %Minor axis length (applied to both minor axes)
num_points = 30; %number of points on ellipse

a = [a1 a2 a3];
b = [b1 b2 b3];
c = (a+b)/2;
major_ax = b-a;



%Project on XY plane
v1 = [1 0];
v2 = [major_ax(1) major_ax(2)];
theta_z = rad2deg(acos(min(1,max(-1, v1(:).' * v2(:) / norm(v1) / norm(v2) ))));

%Project on XZ plane
v1 = [1 0];
v2 = [major_ax(1) major_ax(3)];
theta_y = rad2deg(acos(min(1,max(-1, v1(:).' * v2(:) / norm(v1) / norm(v2) ))));

if a2 < b2
    theta_z = - theta_z;
    theta_y = -theta_y;
end



%Initial plot
%{
[x, y, z] = ellipsoid(c(1), c(2), c(3), len_major,len_minor,len_minor,num_points);
figure;
S1 = surfl(x, y, z);
alpha 0.2;
colormap copper
axis equal
xlabel('X')
ylabel('Y')
zlabel('Z')
%pause 
hold on;
v=[a;b];
plot3(v(:,1),v(:,2),v(:,3),'r')

%}



%%Now rotate
%hold off;
figure;
[x, y, z] = ellipsoid(c(1), c(2), c(3), len_major,len_minor,len_minor,num_points);
S = surfl(x, y, z);
alpha 0.2;
colormap copper
axis equal
xlabel('X')
ylabel('Y')
zlabel('Z')


%Rotate along Y and Z axis
rotate(S, [0,1,0], -theta_y, c)
rotate(S, [0,0,1], -theta_z, c)


%To fix final rotation :
%Minor axis:
m1 = sin(deg2rad(-theta_y));
m2 = 0;
m3 = cos(deg2rad(-theta_y));
m11 = m1*cos(deg2rad(-theta_z)) - 0; %m2*..
m22 = m1*sin(deg2rad(-theta_z));
m33 = m3;

%Rotation vector
r = cross([m11 m22 m33], major_ax);


%Major axis:
m1 = cos(deg2rad(-theta_y));
m2 = 0;
m3 = -sin(deg2rad(-theta_y));
m11 = m1*cos(deg2rad(-theta_z)) - 0; %m2*..
m22 = m1*sin(deg2rad(-theta_z));
m33 = m3;

max = [m11 m22 m33];

%rotate(S, [1 0 0], 45, c)
rot_angle = rad2deg(atan2(norm(cross(max, major_ax)), dot(max, major_ax)));
if a1 > b1
    rot_angle = - rot_angle;
end
if a2 < b2
    rot_angle = - rot_angle;
end
rotate(S, r, rot_angle, c);
hold on;

v=[a;b];
plot3(v(:,1),v(:,2),v(:,3),'r')




%%Verify that the new mean of the ellipsoid is still at point c
x = S.XData; 
y = S.YData;
z = S.ZData;
mean(mean(x))
mean(mean(y))
mean(mean(z))
c