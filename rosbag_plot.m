close,clear
clc

bag=rosbag('bag_nuova.bag');

topic = '/fra2mo/pose';
poseStampedMsgs = readMessages(select(bag, 'Topic', topic));

timestamps = cellfun(@(msg) double(msg.Header.Stamp.Sec) + double(msg.Header.Stamp.Nsec)/1e9, poseStampedMsgs);
x = cellfun(@(msg) msg.Pose.Position.X, poseStampedMsgs);
y = cellfun(@(msg) msg.Pose.Position.Y, poseStampedMsgs);
z = cellfun(@(msg) msg.Pose.Position.Z, poseStampedMsgs);

figure;
plot(timestamps, x, '-o', timestamps, y, '-o', timestamps, z, '-o');
legend('X', 'Y', 'Z');
xlabel('Time (sec)');
ylabel('Position');
title('PoseStamped Data Plot');

% Accedi ai dati dell'orientamento (quaternion) nei messaggi e plotta
orientation = cellfun(@(msg) msg.Pose.Orientation, poseStampedMsgs);

% Estrai componenti quaternion
quatW = cellfun(@(msg) msg.Pose.Orientation.W, poseStampedMsgs);
quatX = cellfun(@(msg) msg.Pose.Orientation.X, poseStampedMsgs);
quatY = cellfun(@(msg) msg.Pose.Orientation.Y, poseStampedMsgs);
quatZ = cellfun(@(msg) msg.Pose.Orientation.Z, poseStampedMsgs);
% Converte quaternion in angoli di Eulero (opzionale)
eulerAngles = quat2eul([quatW, quatX, quatY, quatZ], 'XYZ');

% Plotta i dati
figure;

plot(timestamps, eulerAngles(:, 1), '.', timestamps, eulerAngles(:, 2), '.', timestamps, eulerAngles(:, 3), '.');
legend('Roll', 'Pitch', 'Yaw');
xlabel('Time (sec)');
ylabel('Euler Angles (rad)');
title('Euler Angles Plot');
