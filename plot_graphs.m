figure(2);

vx = xHistory(1,:);
vy = xHistory(2,:);
vpsi = xHistory(3, :);
vV = xHistory(4,:);

va = uHistory(1,:);
vdlt = uHistory(2,:);

subplot(2,2,1);
plot(vx, vy);
title('Position - (Horizon 10)');
xlabel('distance');
ylabel('lane position');
grid on;
axis equal;
ylim([-10 10])
xlim([0 60])

subplot(2,2,2);
plot(vx, vV);
title('Velocity - (Horizon 10)');
xlabel('distance');
ylabel('Velocity');
grid on;
axis equal;
ylim([-10 20])
xlim([0 60])

subplot(2,2,3);
plot(vx, vpsi);
title('Heading angle - (Horizon 10)');
xlabel('distance');
ylabel('heading');
grid on;
% axis equal;
ylim([-pi/2 pi/2])
xlim([0 60])

subplot(2,2,4);
plot(vx, va);
title('Acceleration - (Horizon 10)');
xlabel('distance');
ylabel('acceleration');
grid on;
axis equal;
ylim([-11 11])
xlim([0 60])