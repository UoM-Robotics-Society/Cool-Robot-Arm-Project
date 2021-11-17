% Scale to work with arm
chess_scalar = 0.05;

% Positions
rest_pos = chess_scalar*[0 4.5 3];
piece_init = chess_scalar*[2.5 2.5 3];
piece_fin = chess_scalar*[1.5 4.5 3];

% Generating the three parts of the route
route1 = path(rest_pos, piece_init);
stationary1 = more_vals(rest_pos);
down1 = drop_down(piece_init);
route2 = path(piece_init, piece_fin);
down2 = drop_down(piece_fin);
route3 = path(piece_fin, rest_pos);

% Combine seperate parts of route
full_route = vertcat(stationary1, route1, down1, route2, down2, route3);

disp(full_route)

points = full_route;

count = length(points);

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

% Generate the chess board
ynumbers = chess_scalar*[2 3 4 5 6 7 8 9 10];
xnumbers = chess_scalar*[-4 -3 -2 -1 0 1 2 3 4];
board_squares = 0;


figure
show(robot,qs(1,:));
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot3(points(:,1),points(:,2),points(:,3));

% Draw chess board y's
for i = ynumbers
    plot3([xnumbers(1), xnumbers(9)],[i i], [0 0], color='black');
end
% Draw chess board x's
for i = xnumbers
    plot3([i i], [ynumbers(1) ynumbers(9)], [0 0], color='black');
end

framesPerSecond = 30;
r = rateControl(framesPerSecond);
i = 1;
for j = 0:(count)
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
    i = i + 1;
    if i == count
        i = 1; 
    end
end

% Function to find the step size
function [step_vals] = step_size(val_1, val_2)

	step_vals = ((val_2 - val_1)/50);
end

% Require function to find paths between 2 positions
function [route_vals] = path(start_pos, end_pos)
	
	% Starting Positions
	xs = start_pos(1);
	ys = start_pos(2);
	zs = start_pos(3);

	% Final Positions
	xf = end_pos(1);
	yf = end_pos(2);
	zf = end_pos(3);

	% Values in each coordinate for the route
    
    if xs == xf
        N = 51;
        x_route = zeros(N,1) + xs;
    else
        x_route = (xs:(step_size(xs,xf)):xf)';
    end
        
    if ys == yf
        N = 51;
        y_route = zeros(N,1) + ys;
    else
        y_route = (ys:(step_size(ys,yf)):yf)';
    end   
        
    if zs == zf
        N = 51;
        z_route = zeros(N,1) + zs;
    else
        z_route = (zs:(step_size(zs,zf)):zf)';
    end
    
	% Combine into 1 matrix
	route_vals = horzcat(x_route, y_route, z_route);

end

function[down_route_vals] = drop_down(pos)
    x = pos(1);
    y = pos(2);
    z_start = pos(3);
    z_end = pos(3)-0.15;
    
    z_route1 = (z_start:(step_size(z_start,z_end)):z_end)';
    z_route2 = (z_end:(step_size(z_end,z_start)):z_start)';
    z_route = vertcat(z_route1,z_route2);
    
    N = size(z_route);
    x_route = zeros(N(1),1) + x;
    y_route = zeros(N(1),1) + y;
    
    down_route_vals = horzcat(x_route,y_route,z_route);
end

function[stack_of_vals] = more_vals(pos)
    x = pos(1);
    y = pos(2);
    z = pos(3);
    
    n = 100;
    x_route = zeros(n,1) + x;
    y_route = zeros(n,1) + y;
    z_route = zeros(n,1) + z;
    stack_of_vals = horzcat(x_route,y_route,z_route);
end