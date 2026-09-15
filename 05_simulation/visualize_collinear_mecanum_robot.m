function visualize_collinear_mecanum_robot(result)
%VISUALIZE_COLLINEAR_MECANUM_ROBOT Animate a CMD simulation result in 3D.
%
% SPDX-License-Identifier: MIT
%
% Run the nonlinear simulation first
%   simulate_collinear_mecanum_robot
%
% Then run the visualizer
%   visualize_collinear_mecanum_robot(result)
%
% This function is deliberately separate from the plant simulation. It reads
% the result structure but does not change the dynamics, controller, or safety
% state. Visual geometry is illustrative and does not replace mechanical CAD.

%% USER-EDITABLE VISUALIZATION PARAMETERS
% Change these values to resemble the physical robot. They affect only the
% drawing and have no effect on the simulated physics.

v.body_width_m = 0.220;
v.body_depth_m = 0.160;
v.body_height_m = 0.560;
v.body_bottom_above_axle_m = 0.015;

v.wheel_width_m = 0.045;
v.wheel_surface_segments = 28;
v.axle_radius_m = 0.012;
v.heading_arrow_length_m = 0.220;
v.roller_marker_length_m = 0.070;

v.frame_rate_hz = 30.0;
v.playback_speed = 1.0;
v.world_margin_m = 0.350;
v.show_full_actual_path = true;
v.show_reference_path = true;
v.show_body_axes = true;
v.show_wheel_labels = true;

v.export_video = false;
v.video_filename = "cmd_robot_visualization.mp4";
v.video_quality_percent = 95;

% Display colours
v.colour.ground = [0.93, 0.94, 0.95];
v.colour.reference = [0.15, 0.15, 0.15];
v.colour.path_complete = [0.65, 0.75, 0.90];
v.colour.path_progress = [0.00, 0.35, 0.85];
v.colour.body_disarmed = [0.45, 0.48, 0.52];
v.colour.body_balancing = [0.10, 0.45, 0.90];
v.colour.body_fallen = [0.85, 0.15, 0.12];
v.colour.wheel = [0.12, 0.12, 0.14];
v.colour.axle = [0.30, 0.30, 0.32];
v.colour.roller_positive = [0.95, 0.45, 0.05];
v.colour.roller_negative = [0.00, 0.65, 0.75];

%% VALIDATE INPUT AND PREPARE ANIMATION DATA
validate_visualization_input(result, v);

p = result.parameters;
t = result.time_s(:).';
state = result.state;
wheel_angle_rad = cumtrapz(t, result.wheel_speed_radps, 2);

simulation_step_s = median(diff(t));
simulation_seconds_per_frame = v.playback_speed / v.frame_rate_hz;
frame_stride = max(1, round(simulation_seconds_per_frame / simulation_step_s));
frame_indices = unique([1:frame_stride:numel(t), numel(t)]);

mode_colours = [v.colour.body_disarmed;
                v.colour.body_balancing;
                v.colour.body_fallen];

%% CREATE FIGURE AND STATIC PLOTS
figure_handle = figure('Name', 'Collinear Mecanum robot visualization', ...
                       'Color', 'w', ...
                       'Position', [80, 80, 1420, 780]);
layout = tiledlayout(figure_handle, 2, 2, ...
                     'TileSpacing', 'compact', ...
                     'Padding', 'compact');

robot_axes = nexttile(layout, [2, 1]);
hold(robot_axes, 'on');
grid(robot_axes, 'on');
axis(robot_axes, 'equal');
view(robot_axes, 42.0, 24.0);
xlabel(robot_axes, 'World X [m]');
ylabel(robot_axes, 'World Y [m]');
zlabel(robot_axes, 'World Z [m]');
title(robot_axes, 'Three-dimensional robot view');

[x_limits, y_limits, z_limits] = visualization_limits(result, p, v);
xlim(robot_axes, x_limits);
ylim(robot_axes, y_limits);
zlim(robot_axes, z_limits);

ground_vertices = [x_limits(1), y_limits(1), 0.0;
                   x_limits(2), y_limits(1), 0.0;
                   x_limits(2), y_limits(2), 0.0;
                   x_limits(1), y_limits(2), 0.0];
patch(robot_axes, 'Vertices', ground_vertices, ...
      'Faces', [1, 2, 3, 4], ...
      'FaceColor', v.colour.ground, ...
      'EdgeColor', [0.75, 0.77, 0.80], ...
      'FaceAlpha', 0.70);

if v.show_reference_path
    plot3(robot_axes, ...
          result.reference_q(1, :), ...
          result.reference_q(2, :), ...
          zeros(1, numel(t)) + 0.003, ...
          '--', ...
          'Color', v.colour.reference, ...
          'LineWidth', 1.4);
end

if v.show_full_actual_path
    plot3(robot_axes, ...
          state(1, :), ...
          state(2, :), ...
          zeros(1, numel(t)) + 0.006, ...
          '-', ...
          'Color', v.colour.path_complete, ...
          'LineWidth', 1.0);
end

path_progress_handle = plot3(robot_axes, nan, nan, nan, ...
                             'Color', v.colour.path_progress, ...
                             'LineWidth', 2.2);

lean_axes = nexttile(layout);
hold(lean_axes, 'on');
grid(lean_axes, 'on');
plot(lean_axes, t, rad2deg(state(4, :)), ...
     'Color', [0.75, 0.12, 0.10], ...
     'LineWidth', 1.4);
yline(lean_axes, rad2deg(p.fall_lean_limit_rad), 'k:');
yline(lean_axes, -rad2deg(p.fall_lean_limit_rad), 'k:');
lean_marker = plot(lean_axes, t(1), rad2deg(state(4, 1)), ...
                   'o', ...
                   'MarkerSize', 8, ...
                   'MarkerFaceColor', [0.75, 0.12, 0.10], ...
                   'MarkerEdgeColor', 'w');
xlabel(lean_axes, 'Time [s]');
ylabel(lean_axes, 'Lean [deg]');
title(lean_axes, 'Balance state');
xlim(lean_axes, [t(1), t(end)]);

torque_axes = nexttile(layout);
hold(torque_axes, 'on');
grid(torque_axes, 'on');
plot(torque_axes, t, state(9:12, :).', 'LineWidth', 1.1);
yline(torque_axes, p.motor_torque_limit_Nm, 'k:');
yline(torque_axes, -p.motor_torque_limit_Nm, 'k:');
torque_time_marker = xline(torque_axes, t(1), 'k--', 'LineWidth', 1.3);
xlabel(torque_axes, 'Time [s]');
ylabel(torque_axes, 'Actual motor torque [N m]');
title(torque_axes, 'Actuator response');
xlim(torque_axes, [t(1), t(end)]);
legend(torque_axes, 'M1', 'M2', 'M3', 'M4', ...
       'Location', 'eastoutside');

%% OPTIONAL VIDEO OUTPUT
if v.export_video
    video_writer = VideoWriter(v.video_filename, 'MPEG-4');
    video_writer.FrameRate = v.frame_rate_hz;
    video_writer.Quality = v.video_quality_percent;
    open(video_writer);
end

%% ANIMATE THE SAVED SIMULATION
robot_handles = gobjects(0);

for frame_number = 1:numel(frame_indices)
    k = frame_indices(frame_number);

    if ~isempty(robot_handles)
        delete(robot_handles(isgraphics(robot_handles)));
    end

    mode_index = double(result.mode(k)) + 1;
    if mode_index < 1 || mode_index > size(mode_colours, 1)
        mode_index = 1;
    end

    q = state(1:4, k);
    robot_handles = draw_robot(robot_axes, ...
                               q, ...
                               wheel_angle_rad(:, k), ...
                               p, ...
                               v, ...
                               mode_colours(mode_index, :));

    set(path_progress_handle, ...
        'XData', state(1, 1:k), ...
        'YData', state(2, 1:k), ...
        'ZData', zeros(1, k) + 0.010);

    set(lean_marker, ...
        'XData', t(k), ...
        'YData', rad2deg(state(4, k)));
    torque_time_marker.Value = t(k);

    mode_name = mode_name_at(result, mode_index);
    title(robot_axes, sprintf( ...
        't = %.2f s   mode = %s   X = %.2f m   Y = %.2f m', ...
        t(k), mode_name, q(1), q(2)));

    drawnow;

    if v.export_video
        writeVideo(video_writer, getframe(figure_handle));
    else
        pause(1.0 / v.frame_rate_hz);
    end
end

if v.export_video
    close(video_writer);
    fprintf('Saved visualization to %s\n', v.video_filename);
end
end

function handles = draw_robot(ax, q, wheel_angles, p, v, body_colour)
%DRAW_ROBOT Draw the robot at one saved configuration.

    position_xy = q(1:2);
    yaw = q(3);
    lean = q(4);

    R_yaw = [cos(yaw), -sin(yaw), 0.0;
             sin(yaw),  cos(yaw), 0.0;
             0.0,       0.0,      1.0];
    R_lean = [1.0, 0.0,        0.0;
              0.0, cos(lean), -sin(lean);
              0.0, sin(lean),  cos(lean)];
    R_body = R_yaw * R_lean;

    axle_origin = [position_xy; p.wheel_radius_m];
    handles = gobjects(0);

    % The body cuboid is anchored near the axle and leans with theta.
    body_z_min = v.body_bottom_above_axle_m;
    body_z_max = body_z_min + v.body_height_m;
    [body_vertices_local, body_faces] = cuboid_geometry( ...
        v.body_width_m, v.body_depth_m, body_z_min, body_z_max);
    body_vertices_world = axle_origin + R_body * body_vertices_local;

    handles(end + 1) = patch(ax, ...
        'Vertices', body_vertices_world.', ...
        'Faces', body_faces, ...
        'FaceColor', body_colour, ...
        'FaceAlpha', 0.88, ...
        'EdgeColor', 0.45 * body_colour, ...
        'LineWidth', 0.8);

    % The COM marker uses the physical model parameter, not the visual body
    % centre. This makes a mismatch between CAD geometry and model data visible.
    com_world = axle_origin + R_body * [0.0;
                                       0.0;
                                       p.body_com_height_m];
    handles(end + 1) = plot3(ax, ...
        [axle_origin(1), com_world(1)], ...
        [axle_origin(2), com_world(2)], ...
        [axle_origin(3), com_world(3)], ...
        'k-', ...
        'LineWidth', 1.2);
    handles(end + 1) = plot3(ax, ...
        com_world(1), com_world(2), com_world(3), ...
        'o', ...
        'MarkerSize', 8, ...
        'MarkerFaceColor', [1.0, 0.85, 0.10], ...
        'MarkerEdgeColor', 'k');

    % Draw the common axle as a thick line. The visual axle radius is encoded
    % through line width because a cylinder adds little useful information.
    axle_half_length = max(abs(p.wheel_offsets_x_m)) + 0.5 * v.wheel_width_m;
    axle_points_world = axle_origin + ...
        R_yaw * [-axle_half_length, axle_half_length;
                  0.0,              0.0;
                  0.0,              0.0];
    axle_line_width = max(2.0, 300.0 * v.axle_radius_m);
    handles(end + 1) = plot3(ax, ...
        axle_points_world(1, :), ...
        axle_points_world(2, :), ...
        axle_points_world(3, :), ...
        '-', ...
        'Color', v.colour.axle, ...
        'LineWidth', axle_line_width);

    for wheel_index = 1:4
        wheel_centre = axle_origin + ...
            R_yaw * [p.wheel_offsets_x_m(wheel_index); 0.0; 0.0];

        [wheel_x, wheel_y, wheel_z] = wheel_surface( ...
            wheel_centre, ...
            R_yaw, ...
            p.wheel_radius_m, ...
            v.wheel_width_m, ...
            wheel_angles(wheel_index), ...
            v.wheel_surface_segments);

        handles(end + 1) = surf(ax, wheel_x, wheel_y, wheel_z, ...
            'FaceColor', v.colour.wheel, ...
            'EdgeColor', [0.30, 0.30, 0.32], ...
            'FaceAlpha', 0.98);

        % A rotating radial marker makes wheel rotation observable.
        radial_direction_body = [0.0;
            cos(wheel_angles(wheel_index));
            sin(wheel_angles(wheel_index))];
        marker_end = wheel_centre + ...
            R_yaw * (p.wheel_radius_m * radial_direction_body);
        handles(end + 1) = plot3(ax, ...
            [wheel_centre(1), marker_end(1)], ...
            [wheel_centre(2), marker_end(2)], ...
            [wheel_centre(3), marker_end(3)], ...
            '-', ...
            'Color', [0.92, 0.92, 0.95], ...
            'LineWidth', 2.0);

        % The coloured line above each wheel shows roller-axis handedness.
        roller_angle = p.roller_angles_rad(wheel_index);
        roller_direction_body = [cos(roller_angle);
                                 sin(roller_angle);
                                 0.0];
        roller_centre = wheel_centre + [0.0;
                                       0.0;
                                       1.10 * p.wheel_radius_m];
        roller_half_vector = 0.5 * v.roller_marker_length_m * ...
                             R_yaw * roller_direction_body;

        if roller_angle >= 0.0
            roller_colour = v.colour.roller_positive;
        else
            roller_colour = v.colour.roller_negative;
        end

        handles(end + 1) = plot3(ax, ...
            [roller_centre(1) - roller_half_vector(1), ...
             roller_centre(1) + roller_half_vector(1)], ...
            [roller_centre(2) - roller_half_vector(2), ...
             roller_centre(2) + roller_half_vector(2)], ...
            [roller_centre(3), roller_centre(3)], ...
            '-', ...
            'Color', roller_colour, ...
            'LineWidth', 3.0);

        if v.show_wheel_labels
            handles(end + 1) = text(ax, ...
                roller_centre(1), ...
                roller_centre(2), ...
                roller_centre(3) + 0.035, ...
                sprintf('M%d', wheel_index), ...
                'HorizontalAlignment', 'center', ...
                'FontWeight', 'bold', ...
                'Color', [0.05, 0.05, 0.05]);
        end
    end

    % Body-forward arrow. Body x lies along the common wheel axle and body y
    % is the forward direction used by the nonlinear model.
    forward_vector = R_yaw * [0.0; v.heading_arrow_length_m; 0.0];
    handles(end + 1) = quiver3(ax, ...
        axle_origin(1), axle_origin(2), axle_origin(3), ...
        forward_vector(1), forward_vector(2), forward_vector(3), ...
        0.0, ...
        'Color', [0.10, 0.65, 0.15], ...
        'LineWidth', 2.0, ...
        'MaxHeadSize', 0.8);

    if v.show_body_axes
        axis_length = 0.12;
        body_axes_world = R_body * (axis_length * eye(3));
        axis_colours = [0.85, 0.10, 0.10;
                        0.10, 0.65, 0.15;
                        0.10, 0.25, 0.90];
        for axis_index = 1:3
            handles(end + 1) = quiver3(ax, ...
                axle_origin(1), axle_origin(2), axle_origin(3), ...
                body_axes_world(1, axis_index), ...
                body_axes_world(2, axis_index), ...
                body_axes_world(3, axis_index), ...
                0.0, ...
                'Color', axis_colours(axis_index, :), ...
                'LineWidth', 1.5, ...
                'MaxHeadSize', 0.7);
        end
    end
end

function [vertices, faces] = cuboid_geometry(width, depth, z_min, z_max)
%CUBOID_GEOMETRY Return local cuboid vertices as a 3-by-8 matrix.

    x_min = -0.5 * width;
    x_max = 0.5 * width;
    y_min = -0.5 * depth;
    y_max = 0.5 * depth;

    vertices = [x_min, x_max, x_max, x_min, x_min, x_max, x_max, x_min;
                y_min, y_min, y_max, y_max, y_min, y_min, y_max, y_max;
                z_min, z_min, z_min, z_min, z_max, z_max, z_max, z_max];

    faces = [1, 2, 3, 4;
             5, 8, 7, 6;
             1, 5, 6, 2;
             2, 6, 7, 3;
             3, 7, 8, 4;
             4, 8, 5, 1];
end

function [X_world, Y_world, Z_world] = wheel_surface( ...
    centre_world, R_yaw, radius, width, wheel_angle, segment_count)
%WHEEL_SURFACE Return a cylinder aligned with the body x axis.

    circumference_angle = linspace(0.0, 2.0 * pi, segment_count + 1).' ...
                          + wheel_angle;
    local_x = repmat([-0.5 * width, 0.5 * width], ...
                     segment_count + 1, 1);
    local_y = radius * repmat(cos(circumference_angle), 1, 2);
    local_z = radius * repmat(sin(circumference_angle), 1, 2);

    local_points = [local_x(:).'; local_y(:).'; local_z(:).'];
    world_points = centre_world + R_yaw * local_points;

    surface_size = size(local_x);
    X_world = reshape(world_points(1, :), surface_size);
    Y_world = reshape(world_points(2, :), surface_size);
    Z_world = reshape(world_points(3, :), surface_size);
end

function [x_limits, y_limits, z_limits] = visualization_limits(result, p, v)
%VISUALIZATION_LIMITS Select fixed axes so the camera does not jump.

    all_x = [result.state(1, :), result.reference_q(1, :)];
    all_y = [result.state(2, :), result.reference_q(2, :)];

    x_limits = padded_limits(all_x, v.world_margin_m);
    y_limits = padded_limits(all_y, v.world_margin_m);

    wheel_span = max(abs(p.wheel_offsets_x_m)) ...
                 + p.wheel_radius_m ...
                 + v.world_margin_m;
    x_limits = [min(x_limits(1), min(all_x) - wheel_span), ...
                max(x_limits(2), max(all_x) + wheel_span)];

    maximum_height = p.wheel_radius_m ...
                     + v.body_bottom_above_axle_m ...
                     + v.body_height_m;
    z_limits = [-0.03, maximum_height + 0.15];
end

function limits = padded_limits(data, padding)
%PADDED_LIMITS Return non-degenerate limits around finite data.

    finite_data = data(isfinite(data));
    if isempty(finite_data)
        limits = [-padding, padding];
        return;
    end

    data_min = min(finite_data);
    data_max = max(finite_data);
    if data_max - data_min < 1.0e-6
        limits = [data_min - padding, data_max + padding];
    else
        limits = [data_min - padding, data_max + padding];
    end
end

function mode_name = mode_name_at(result, mode_index)
%MODE_NAME_AT Return a printable mode name with a safe fallback.

    if isfield(result, 'mode_names') && ...
       mode_index >= 1 && mode_index <= numel(result.mode_names)
        mode_name = result.mode_names{mode_index};
    else
        mode_name = sprintf('MODE_%d', mode_index - 1);
    end
end

function validate_visualization_input(result, v)
%VALIDATE_VISUALIZATION_INPUT Check the simulation-to-visualizer contract.

    required_result_fields = {'time_s', ...
                              'state', ...
                              'wheel_speed_radps', ...
                              'reference_q', ...
                              'mode', ...
                              'parameters'};

    for field_index = 1:numel(required_result_fields)
        field_name = required_result_fields{field_index};
        if ~isfield(result, field_name)
            error('The result structure is missing field %s.', field_name);
        end
    end

    sample_count = numel(result.time_s);
    if sample_count < 2
        error('At least two simulation samples are required.');
    end

    if size(result.state, 1) < 12 || size(result.state, 2) ~= sample_count
        error('result.state must contain at least 12 rows and one column per time sample.');
    end

    if ~isequal(size(result.wheel_speed_radps), [4, sample_count])
        error('result.wheel_speed_radps must be 4-by-N.');
    end

    if size(result.reference_q, 1) ~= 4 || ...
       size(result.reference_q, 2) ~= sample_count
        error('result.reference_q must be 4-by-N.');
    end

    if numel(result.mode) ~= sample_count
        error('result.mode must contain one entry per time sample.');
    end

    p = result.parameters;
    required_parameter_fields = {'wheel_radius_m', ...
                                 'wheel_offsets_x_m', ...
                                 'roller_angles_rad', ...
                                 'body_com_height_m', ...
                                 'fall_lean_limit_rad', ...
                                 'motor_torque_limit_Nm'};

    for field_index = 1:numel(required_parameter_fields)
        field_name = required_parameter_fields{field_index};
        if ~isfield(p, field_name)
            error('result.parameters is missing field %s.', field_name);
        end
    end

    if numel(p.wheel_offsets_x_m) ~= 4 || numel(p.roller_angles_rad) ~= 4
        error('The visualizer requires four wheel offsets and four roller angles.');
    end

    if any(diff(result.time_s) <= 0.0)
        error('result.time_s must be strictly increasing.');
    end

    if v.frame_rate_hz <= 0.0 || v.playback_speed <= 0.0
        error('Frame rate and playback speed must be positive.');
    end
end
