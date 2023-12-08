function plotting_curve(ref, obs, xHistory, time_obs, lane_length, nlobj, waypoints)
figure(1);
H = nlobj.PredictionHorizon;
[numObs, cc, ii] = size(obs);
obsXY = zeros(numObs,8);

[x, y, offset_x, offset_y, offsetN_x, offsetN_y, heading] = curved_path(waypoints);

x_list = xHistory(1,:)';
y_list = xHistory(2,:)';
h_list = xHistory(3,:)';

for i=1:length(x_list)
    
    clf;
    [carx, cary] = vehicle_drawing(x_list(i,1), y_list(i,1), h_list(i,1));
    [parkx, parky] = vehicle_drawing(ref(1,1), ref(1,2), ref(1,3));
    
    for o=1:numObs
    [obsXY(o,1:4), obsXY(o,5:8)] = vehicle_drawing(obs(o,1,i), obs(o,2,i), 0);
    end


    patch(carx, cary, 'r', 'EdgeColor', 'r', 'LineWidth', 2);
    hold on;
    line(offsetN_x, offsetN_y, 'Color', 'k', 'LineStyle', '-');
    line(offset_x, offset_y, 'Color', 'k', 'LineStyle', '-');
    line(x, y, 'Color', 'k', 'LineStyle', '--');
    pathx = x_list(1:i,1);
    pathy = y_list(1:i,1);
    line(pathx, pathy, 'Color', 'b');
    patch(parkx, parky, 'b', 'EdgeColor', 'k', 'LineWidth', 1);
    

    for o=1:numObs
        % if i<time_obs && o == 3
        %     continue;
        % end
    patch(obsXY(o,1:4), obsXY(o,5:8), 'k', 'EdgeColor', 'k', 'LineWidth', 1);
    end
    

    hold off;
    grid on;
    % xlim([0 40]);
    ylim([-5, 15]);
    axis equal;
    title(['Horizon: ', num2str(H),'   Time: ', num2str(i*0.1), 'sec']);
    
    drawnow;
    pause(0.1);
end

end