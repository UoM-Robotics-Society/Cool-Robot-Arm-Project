t = (0:0.2:10)'; % Time
count = length(t);
center = [0.15 0.15 0.2];
radius = 0.1;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];
Arm_Model;

q0 = homeConfiguration(robot);
ndof = length(q0);
qs = repmat(q0,count,1);


ik = inverseKinematics('RigidBodyTree', robot,'SolverAlgorithm','BFGSGradientProjection');
ik.SolverParameters.AllowRandomRestarts = false;
weights = [0, 0, 0, 1, 1, 1];
endEffector = 'body6';

qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

figure
show(robot,qs(1,:));
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot3(points(:,1),points(:,2),points(:,3));

framesPerSecond = 30;
r = rateControl(framesPerSecond);
i = 1;
for j = 0:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
    i = i + 1;
    if i == count
        i = 1; 
    end
end

