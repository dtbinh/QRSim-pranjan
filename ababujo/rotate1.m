% a = obj.sloc';
% b = obj.dloc';
% c = (a+b) / 2;
% major_ax = b-a;
% len_major = obj.major_axis/simState.dist_scale;
% len_minor =  obj.minor_axis/simState.dist_scale;
% 
% v1 = [1 0];
% v2 = [major_ax(1) major_ax(2)];
% theta_z = rad2deg(acos(min(1,max(-1, v1(:).' * v2(:) / norm(v1) / norm(v2)))));
% 
% v1 = [1 0];
% v2 = [major_ax(1) major_ax(3)];
% theta_y = rad2deg(acos(min(1,max(-1, v1(:).' * v2(:) / norm(v1) / norm(v2) ))));
% 
% if a(2) < b(2)
%     theta_z = - theta_z;
%     theta_y = -theta_y;
% end
% 
% scatter3(obj.sloc(1), obj.sloc(2), obj.sloc(3)-2, 60, 'Magenta', 'filled');
% scatter3(obj.dloc(1), obj.dloc(2), obj.dloc(3)-2, 60, "*", 'Magenta');
% [x, y, z] = ellipsoid(c(1), c(2), c(3), len_major,len_minor,len_minor,20);
% S = surfl(x, y, z);
% 
% alpha 0.2;
% colormap copper
% 
% rotate(S, [0,1,0], -theta_y, c)
% rotate(S, [0,0,1], -theta_z, c)
% 
% 
% m1 = sin(deg2rad(-theta_y));
% m2 = 0;
% m3 = cos(deg2rad(-theta_y));
% m11 = m1*cos(deg2rad(-theta_z)) - 0; %m2*..
% m22 = m1*sin(deg2rad(-theta_z));
% m33 = m3;
% 
% 
% r = cross([m11 m22 m33], major_ax);
% m1 = cos(deg2rad(-theta_y));
% m2 = 0;
% m3 = -sin(deg2rad(-theta_y));
% m11 = m1*cos(deg2rad(-theta_z)) - 0; %m2*..
% m22 = m1*sin(deg2rad(-theta_z));
% m33 = m3;
% 
% max = [m11 m22 m33];
% rot_angle = rad2deg(atan2(norm(cross(max, major_ax)), dot(max, major_ax)));
% if a(1) > b(1)
%     rot_angle = - rot_angle;
% end
% if a(2) < b(2)
%     rot_angle = - rot_angle;
% end
% rotate(S, r, rot_angle, c);
% hold on;
% 
% v=[a;b];
% plot3(v(:,1),v(:,2),v(:,3),'r')
% 
% Node density sphere plot
mfig = figure();
fontsize = 14;
[x,y,z] = sphere;
hold on;
s = surf(x*20, y*20, z * 20, 'FaceAlpha', 0.1);
s2 = surf(x*20 + 20, y*20, z * 20, 'FaceAlpha', 0.1);

s1 = surf(x*10 , y*10, z*10, 'FaceAlpha', 0.3);
s3 = surf(x*10 + 20, y*10, z*10, 'FaceAlpha', 0.3);

shading interp;
grid on;

scatter3(0,0,0, 'Magenta', 'filled');
scatter3(20,0,0, 'Magenta', 'filled');
line([0, 20], [0, 0], [0, 0]);
text(-7, 5, 0, "\boldmath$$  v = \frac{4}{3} \cdot \pi \cdot \Big(\frac{T_r}{2}\Big)^3 $$" , 'Interpreter','latex', 'FontSize', fontsize);
text(10, 2, 0, "\boldmath $$ T_r $$", 'Interpreter' , 'latex', 'FontSize', 14);
text(-2, -2, 0, "S", 'FontSize', fontsize);
text(22, 2, 0, "D", 'FontSize', fontsize);
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
