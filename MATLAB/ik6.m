% Scale to work with arm
chess_scalar = 0.03;

% Positions
REST = convert_position([4.5 1.5 3], chess_scalar);
piece_init = convert_position([4 4 3], chess_scalar);
piece_fin = convert_position([5 5 3], chess_scalar);

%=======================================
% List of Functions
%=======================================
% 1) move_piece DONE
% 2) take_piece DONE
% 3) castle long/short
% 4) Pawn Promote
% 5) en_passant
%=======================================
% convert_position
full_route = take_piece(piece_init, piece_fin, REST, chess_scalar);

points = full_route;

% Amount of points in array
count = length(points);
disp(count)

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
ynumbers = chess_scalar*[3.5 4.5 5.5 6.5 7.5 8.5 9.5 10.5 11.5];
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
    plot3([xnumbers(1), xnumbers(9)],[i i], [0 0]);
end
% Draw chess board x's
for i = xnumbers
    plot3([i i], [ynumbers(1) ynumbers(9)], [0 0]);
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
    % Finds linear path between 2 positions
	
	% Starting Positions
	xs = start_pos(1);
	ys = start_pos(2);
	zs = start_pos(3);

	% Final Positions
	xf = end_pos(1);
	yf = end_pos(2);
	zf = end_pos(3);

	% Values in each coordinate for the route
    % 
    
    
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

function[full_drop_route] = drop_down(pos, drop)
    % Generates an array of points to drop the arm down
    % to the board from the current position
    x = pos(1);
    y = pos(2);
    z_start = pos(3);
    z_end = pos(3) - drop;
    
    z_route1 = (z_start:(step_size(z_start,z_end)):z_end)';
    z_route2 = (z_end:(step_size(z_end,z_start)):z_start)';
    
    N = size(z_route1);
    x_route = zeros(N(1),1) + x;
    y_route = zeros(N(1),1) + y;
    
    down_route_vals = horzcat(x_route,y_route,z_route1);
    up_route_vals = horzcat(x_route, y_route, z_route2);
    full_drop_route = vertcat(down_route_vals, up_route_vals);
end

function[stack_of_vals] = more_vals(pos)
    % This generates an array of identical points to keep
    % the arm stationary
    x = pos(1);
    y = pos(2);
    z = pos(3);
    
    n = 100;
    x_route = zeros(n,1) + x;
    y_route = zeros(n,1) + y;
    z_route = zeros(n,1) + z;
    stack_of_vals = horzcat(x_route,y_route,z_route);
end

function[full_route] = move_piece(int_pos, final_pos, REST)
    % Code to produce a full array of the route to take
    % a piece
    route1 = path(REST, int_pos);
    down1 = drop_down(int_pos, 0.1);
    route2 = path(int_pos, final_pos);
    down2 = drop_down(final_pos, 0.1);
    route3 = path(final_pos, REST);
    
    full_route = vertcat(route1, down1, route2, down2, route3);

end

function[full_route] = take_piece(int_pos, final_pos, REST, chess_scalar)
    % Code to produce a full array of the route required
    % by the arm to take another piece
    disp(chess_scalar)
    the_pit = chess_scalar*[5 6 4];
    route1 = move_piece(final_pos, the_pit, REST);
    route2 = move_piece(int_pos, final_pos, REST);
    full_route = vertcat(route1, route2);
end

function[conv_pos] = convert_position(pos, chess_scalar)
    % Function to convert from 2d coords to the coords the arm uses
    x = chess_scalar*(pos(1)-4.5);
    y = chess_scalar*(pos(2)+3);
    z = chess_scalar*(pos(3));
    conv_pos = horzcat(x,y,z);
    disp(conv_pos)
end