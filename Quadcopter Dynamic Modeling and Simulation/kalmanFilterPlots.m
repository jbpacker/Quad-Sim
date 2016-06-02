X = yout(:,10)*3.28; 
Y = yout(:,11)*3.28; 
Z = yout(:,12)*3.28;  

phiq = yout(:,4);
thetaq = yout(:,5);
psiq = yout(:,6);

xEstimate = yout(:,34);
yEstimate = yout(:,35);
zEstimate = yout(:,36);

cam_r = yout(:,31);
cam_theta = yout(:,32);
cam_psi = yout(:,33);

c_x = cam_r.*sin(cam_psi).*cos(cam_theta);
c_y = cam_r.*sin(cam_psi).*sin(cam_theta);
c_z = cam_r.*cos(cam_psi);
cam_x = zeros(length(c_x), 1);
cam_y = zeros(length(c_x), 1);
cam_z = zeros(length(c_x), 1);

for i = 1:length(c_x)
    Rback = [cos(psiq(i))*cos(thetaq(i)), cos(psiq(i))*sin(thetaq(i))*sin(phiq(i))-sin(psiq(i))*cos(phiq(i)), cos(psiq(i))*sin(thetaq(i))*cos(phiq(i))+sin(psiq(i))*sin(phiq(i));
              sin(psiq(i))*cos(thetaq(i)), sin(psiq(i))*sin(thetaq(i))*sin(phiq(i))+cos(psiq(i))*cos(phiq(i)), sin(psiq(i))*sin(thetaq(i))*cos(phiq(i))-cos(psiq(i))*sin(phiq(i));
              -sin(thetaq(i)),         cos(thetaq(i))*sin(phiq(i)),                            cos(thetaq(i))*cos(phiq(i))];
    human_cam = Rback*[c_x(i); c_y(i); c_z(i)];

    cam_x(i,1) = human_cam(1);
    cam_y(i,1) = human_cam(2);
    cam_z(i,1) = human_cam(3);
end


human_x = yout(:,27);
human_y = yout(:,28);
human_z = yout(:,29);

figure(1)
plot(tout, X-cam_x, tout, human_x, tout, X-xEstimate, 'LineWidth', 2)
legend('cam transformed with noise', 'human GT', 'kalman estimate');
title('kalman x performance')

figure(2)
plot(tout, Y-cam_y, tout, human_y, tout, Y-yEstimate, 'LineWidth', 2)
legend('cam transformed with noise', 'human GT', 'kalman estimate');
title('kalman y performance')

figure(3)
plot(tout, Z-cam_z, tout, human_z, tout, Z-zEstimate, 'LineWidth', 2)
legend('cam transformed with noise', 'human GT', 'kalman estimate');
title('kalman z performance')