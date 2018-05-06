rosinit
tftree = rostf;
pause(1);

N = 5;
solution = zeros(N, 6);

for i = 1 : 5
    cam_to_marker = getTransform(tftree, 'camera', 'ar_marker_0');
    base_to_marker = getTransform(tftree, 'base_link', 'tool_ar_tag');

    translation = cam_to_marker.Transform.Translation;
    t_cam = [translation.X; translation.Y; translation.Z];
    rotation = cam_to_marker.Transform.Rotation;
    q_cam = [rotation.W, rotation.X, rotation.Y, rotation.Z];
    rot_cam = quat2rotm(q_cam);
    tf_cam = [rot_cam t_cam; 0 0 0 1;];
    
    translation = base_to_marker.Transform.Translation;
    t_base = [translation.X; translation.Y; translation.Z];
    rotation = base_to_marker.Transform.Rotation;
    q_base = [rotation.W, rotation.X, rotation.Y, rotation.Z];
    rot_base = quat2rotm(q_base);
    tf_base = [rot_base t_base; 0 0 0 1;];
    
    x = tf_base / tf_cam;
    q = rotm2eul(x(1:3, 1:3));
    solution(i, :) = [x(1, 4), x(2, 4), x(3, 4), q]
    
    disp('move to next location');
    k = waitforbuttonpress;
end
average_solution = mean(solution);
quat = eul2quat(average_solution(4:6));

solution = [average_solution(1:3), quat(2:4), quat(1)];

fileID = fopen('../yaml/camera_transformation.yaml', 'w');
fprintf(fileID, 'transformation: [');
for i = 1 : 7
    fprintf(fileID, '%f', solution(i));
    if (i ~= 7)
       fprintf(fileID, ', '); 
    else
       fprintf(fileID, ']\n');
    end
end
fclose(fileID);

rosshutdown;
