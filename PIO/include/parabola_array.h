// Patrick Evans Trajectory Planning
// Robotics society

#include <cmath>
#include <iostream>

double point_convert(double position[3], double chess_scalar, int i) {
    // Function to convert from chess coordinates into
    // a range that the arm understands.
    double conv_pos[3];

    conv_pos[0] = chess_scalar * (position[0]);
    conv_pos[1] = chess_scalar * (position[1]);
    conv_pos[2] = chess_scalar * (position[2]);

    return conv_pos[i];
}

double xy_parabolic(double xy_start, double xy_fin, int i, int num_point) {
    // x and y values for the parabolic trajectory
    // Both done in one function as done in same way.

    double difference = (xy_fin - xy_start) / num_point;
    double xy_val = xy_start + (i * difference);

    return xy_val;
}

double z_parabolic(double start_point[3], double end_point[3], double x_i,
                   double y_i, double chess_scalar, int i) {
    // Outputs the z-values of the parabolic trajectory
    // Nope not gonna do this bit now its scary.

    double xs = start_point[0];
    double xf = end_point[0];
    double ys = start_point[1];
    double yf = end_point[1];
    double z_i;

    z_i = chess_scalar * (1 - (4 / (pow(xs, 2) + pow(xf, 2) - 2 * xf * xs +
                                    pow(yf, 2) + pow(ys, 2) - 2 * yf * ys)) *
                                  (pow((x_i - (xs + xf) / 2), 2) +
                                   pow((y_i - (ys + yf) / 2), 2)));
    z_i += start_point[2];
    return z_i;
}

double parabola_func(double point1[3], double point2[3], int point_num,
                     int cartesian, double chess_scalar) {
    int num_of_points = 6;  // Points per parabola path

    // Convert point 1 to "arm coordinates"
    double pos1[3];
    pos1[0] = point_convert(point1, chess_scalar, 0);
    pos1[1] = point_convert(point1, chess_scalar, 1);
    pos1[2] = point_convert(point1, chess_scalar, 2);

    // Convert point 2 to "arm coordinates"
    double pos2[3];
    pos2[0] = point_convert(point2, chess_scalar, 0);
    pos2[1] = point_convert(point2, chess_scalar, 1);
    pos2[2] = point_convert(point2, chess_scalar, 2);

    // Create space for parabolic paths
    double x_path[num_of_points - 1], y_path[num_of_points - 1],
        z_path[num_of_points - 1];

    // X Path
    //    std::cout << "======X Points======" << std::endl;
    for (int i = 0; i < num_of_points; i++) {
        x_path[i] = xy_parabolic(pos1[0], pos2[0], i, num_of_points);
    }

    // Y Path
    //  std::cout << "======Y Points======" << std::endl;
    for (int i = 0; i < num_of_points; i++) {
        y_path[i] = xy_parabolic(pos1[1], pos2[1], i, num_of_points);
        //    std::cout << y_path[i] << std::endl;
    }

    // Z Path
    // std::cout << "======Z Points======" << std::endl;
    for (int i = 0; i < num_of_points; i++) {
        z_path[i] =
            z_parabolic(pos1, pos2, x_path[i], y_path[i], chess_scalar, i);
        //    std::cout << z_path[i] << std::endl;
    }

    switch (cartesian) {
        case 0:
            return x_path[point_num];
            break;
        case 1:
            return y_path[point_num];
            break;
        case 2:
            return z_path[point_num];
            break;
        default:
            std::cout << "You broke it :(" << std::endl;
            return 1;
    }
}

double move_piece(double piece_start[3], double piece_end[3], bool take,
                  int array_x, int array_y) {
    double chess_scalar = 1;

    // Rest Position of the Arm PUT CORRECT ONE HERE!
    double rest[3] = {0, 0, 0};
    rest[0] = point_convert(rest, chess_scalar, 0);
    rest[1] = point_convert(rest, chess_scalar, 1);
    rest[2] = point_convert(rest, chess_scalar, 2);

    // Position for taken pieces PUT CORRECT ONE HERE!
    double piece_bin[3] = {9, 2, 2};
    piece_bin[0] = point_convert(piece_bin, chess_scalar, 0);
    piece_bin[1] = point_convert(piece_bin, chess_scalar, 1);
    piece_bin[2] = point_convert(piece_bin, chess_scalar, 2);

    double coord_array[3][30];

    // Generate Set Size Arrays depending on whether taking or moving
    if (take) {
        double coord_array[3][31];
        double number_of_points = 30;

        // Find coordinates
        for (int i = 0; i < 6; i++) {
            // Values
            coord_array[array_x][i] =
                parabola_func(rest, piece_end, i, array_x, chess_scalar);
            coord_array[array_x][i + 6] =
                parabola_func(piece_end, piece_bin, i, array_x, chess_scalar);
            coord_array[array_x][i + 12] = parabola_func(
                piece_bin, piece_start, i, array_x, chess_scalar);
            coord_array[array_x][i + 18] = parabola_func(
                piece_start, piece_end, i, array_x, chess_scalar);
            coord_array[array_x][i + 24] = parabola_func(
                piece_start, piece_end, i, array_x, chess_scalar);
            coord_array[array_x][31] = rest[array_x];
        }

        return coord_array[array_x][array_y];

    } else {
        double coord_array[3][19];
        double number_of_points = 18;

        // Find coordinates
        for (int i = 0; i < 6; i++) {
            // Values
            coord_array[array_x][i] =
                parabola_func(rest, piece_start, i, array_x, chess_scalar);
            coord_array[array_x][i + 6] = parabola_func(
                piece_start, piece_end, i, array_x, chess_scalar);
            coord_array[array_x][i + 12] =
                parabola_func(piece_end, rest, i, array_x, chess_scalar);
            coord_array[array_x][19] = rest[array_x];
        }

        return coord_array[array_x][array_y];
    }
}
