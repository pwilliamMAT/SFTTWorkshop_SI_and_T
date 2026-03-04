function helperPlotRMS(results)

figure;
timeVec = 1:length(results.posRMSE);
subplot(2,2,1)
scatter(timeVec,results.posRMSE,'filled');
xlabel('Time step');
ylabel('RMS Error in Position (m)');
grid('on');

subplot(2,2,2)
scatter(timeVec,results.velRMSE,'filled');
xlabel('Time step');
ylabel('RMS Error in Velocity (m/s)');
grid('on');

subplot(2,2,3)
scatter(timeVec,results.posANEES,'filled');
xlabel('Time step');
ylabel('Average Normalized Error in Position');
grid('on');

subplot(2,2,4)
scatter(timeVec,results.velANEES,'filled');
xlabel('Time step');
ylabel('Average Normalized Error in Velocity');
grid('on');

sgtitle('Cumulative errors for all fused tracks','FontWeight','bold');
end