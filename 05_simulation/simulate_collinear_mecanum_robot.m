%% SIMULATE_COLLINEAR_MECANUM_ROBOT
% Nonlinear simulation of a four-wheel Collinear Mecanum Drive (CMD)
% balancing robot on flat ground.
%
% SPDX-License-Identifier: MIT
%
% This file is intentionally self-contained. The user-editable parameters
% are collected at the beginning, while the plant, controller, safety state
% machine, validation, plotting, and animation are implemented as local
% functions below the main script.
%
% Coordinate convention
%   World X, Y       planar position [m]
%   Body x           right, along the common wheel axle
%   Body y           forward
%   Body z           up when the body is upright
%   psi              yaw about world z, positive by the right-hand rule
%   theta            unstable body lean about body x
%   alpha_i          Mecanum roller-axis angle measured from body x
%
% The word "lean" is used instead of pitch or roll because a rotation about
% body x is often called roll in aerospace conventions. Map theta to the
% firmware pitch or roll field only after confirming the installed IMU axes.
%
% Degrees of freedom
%   Full coordinates    4 platform + 4 wheel + 4 roller = 12
%   Contact constraints 2 ideal velocity constraints per wheel = 8
%   Independent DOF     12 - 8 = 4: X, Y, psi, theta
%   Mechanical states   4 positions + 4 velocities = 8
%   Simulation states   8 mechanical + 4 actuator torques = 12
%
% Independent actuation has rank three. Four motor torques contain one
% redundant internal combination because forward force and body-lean
% reaction torque are mechanically coupled in this collinear arrangement.
%
% Model basis and improvements
%   - The upright inverted-pendulum and LQR workflow follows the engineering
%     pattern used by C. T. Refvem, "Design, Modeling and Control of a
%     Two-Wheel Balancing Robot Driven by BLDC Motors," 2019.
%   - The four-coordinate reduction follows the CMD formulation described by
%     M. T. Watson, D. T. Gladwin, and T. J. Prescott, IEEE TRO, 2020,
%     DOI 10.1109/TRO.2020.2977878.
%   - This implementation adds four-wheel Mecanum allocation, a nonlinear
%     mass matrix, motor torque lag, torque and slew limits, a sampled
%     controller, smooth planar/yaw references, validation checks, and a
%     latched simulation safety state.
%
% Required software
%   Base MATLAB is sufficient. If Control System Toolbox is installed, lqr()
%   is used. Otherwise, a Hamiltonian-matrix CARE solution is used.

clear;
clc;
close all;

%% USER-EDITABLE ROBOT PARAMETERS
% Replace the example values below with measured values before using results
% in the thesis or transferring gains to hardware.

% Fundamental constant
p.gravity_mps2 = 9.80665;

% Geometry
p.wheel_diameter_m = 0.120;
p.wheel_radius_m = p.wheel_diameter_m / 2.0;
p.wheel_offsets_x_m = [-0.240; -0.080; 0.080; 0.240];
p.roller_angles_deg = [45.0; -45.0; 45.0; -45.0];
p.roller_angles_rad = deg2rad(p.roller_angles_deg);
p.body_com_height_m = 0.280;

% Lumped mass properties
% p.body_mass_kg is the leaning structure above the wheel axle.
% p.base_mass_kg contains wheels, rotors, axle-mounted parts, and any mass
% whose centre remains at axle height in this reduced model.
p.body_mass_kg = 6.000;
p.base_mass_kg = 2.000;

% Leaning-body inertia at its centre of mass, resolved in body axes.
% Measure or obtain these values from CAD. The x value governs lean motion.
p.body_inertia_kgm2 = diag([0.180, 0.120, 0.160]);

% Base yaw inertia excludes wheel spin inertia, which is added separately.
p.base_yaw_inertia_kgm2 = 0.060;
p.wheel_spin_inertia_kgm2 = 8.0e-4;

% Effective losses
% Planar drag is expressed in the body x and body y directions.
p.planar_viscous_Ns_per_m = [4.0; 5.0];
p.yaw_viscous_Nms_per_rad = 0.350;
p.lean_viscous_Nms_per_rad = 0.080;
p.wheel_bearing_viscous_Nms_per_rad = 0.004;

% Actuator approximation
% Torque is output-shaft torque. Obtain the real limit and closed-loop time
% constant from AK40-10 step-response tests in the intended control mode.
p.motor_torque_limit_Nm = 8.0;
p.motor_torque_slew_limit_Nmps = 200.0;
p.motor_torque_time_constant_s = 0.012;

%% USER-EDITABLE CONTROLLER PARAMETERS
% LQR is designed from a numerical linearisation at the upright equilibrium.
% State order is [X Y psi theta Xdot Ydot psidot thetadot].
p.lqr_Q = diag([35.0, 50.0, 30.0, 900.0, ...
                 6.0, 10.0,  8.0, 120.0]);
p.lqr_R = 0.8 * eye(4);
p.controller_period_s = 0.005;  % 200 Hz sample-and-hold controller

%% USER-EDITABLE SAFETY AND SIMULATION PARAMETERS
p.arm_time_s = 0.20;
p.arm_lean_limit_rad = deg2rad(10.0);
p.arm_lean_rate_limit_rps = deg2rad(80.0);
p.fall_lean_limit_rad = deg2rad(30.0);

p.simulation_time_s = 10.0;
p.integration_step_s = 0.001;
p.enable_animation = false;
p.animation_speed = 1.0;
p.save_results = false;
p.results_filename = "cmd_simulation_results.mat";

% Initial state
p.initial_position_m = [0.0; 0.0];
p.initial_yaw_rad = deg2rad(0.0);
p.initial_lean_rad = deg2rad(3.0);
p.initial_velocity = zeros(4, 1);

% Smooth reference manoeuvre
p.reference.forward_distance_m = 0.350;
p.reference.forward_start_s = 1.0;
p.reference.forward_end_s = 3.0;
p.reference.lateral_distance_m = 0.200;
p.reference.lateral_start_s = 3.0;
p.reference.lateral_end_s = 5.0;
p.reference.yaw_angle_rad = deg2rad(20.0);
p.reference.yaw_start_s = 5.0;
p.reference.yaw_end_s = 7.0;

% Numerical validation and linearisation steps
p.numerics.state_step = 1.0e-6;
p.numerics.input_step = 1.0e-5;
p.numerics.rank_tolerance = 1.0e-8;

%% MODEL ASSUMPTIONS
p.model_assumptions = {
    'The ground is flat, rigid, stationary, and horizontal.'
    'The wheel centres and wheel axes are collinear along body x.'
    'The robot has one unstable lean coordinate about body x.'
    'The base stays at axle height, with no heave or independent base tilt.'
    'The leaning body and base are rigid bodies.'
    'The body centre of mass is directly above the axle origin when upright.'
    'Wheel contact is continuous, with no wheel lift or impact.'
    'Each wheel enforces ideal Mecanum rolling constraints.'
    'Rollers are massless and their individual spin dynamics are neglected.'
    'Slip and roller compliance are represented only by effective damping.'
    'Wheel spin inertia is included, while other wheel inertia is lumped into the base.'
    'Motor torque acts on each wheel with an equal and opposite reaction on the body.'
    'Actuator dynamics are first-order torque lag with magnitude and slew saturation.'
    'Motor electrical dynamics, battery voltage sag, and regeneration are neglected.'
    'The controller receives the complete state without estimator noise or delay.'
    'LQR is designed at the stationary upright equilibrium and used on the nonlinear plant.'
    'Reference commands are smooth and remain within the local controller operating region.'
    'The safety mode is a simulation guard and is not a certified robot safety function.'
    };

%% VALIDATE MODEL AND DESIGN THE CONTROLLER
validate_parameters(p);

x_equilibrium = zeros(8, 1);
tau_equilibrium = zeros(4, 1);
[A, B] = linearise_mechanical_model(x_equilibrium, tau_equilibrium, p);
[K, lqr_source] = design_lqr(A, B, p.lqr_Q, p.lqr_R);

wheel_map_rank = rank(wheel_jacobian(zeros(4, 1), p), ...
                      p.numerics.rank_tolerance);
input_map_rank = rank(input_matrix(zeros(4, 1), p), ...
                      p.numerics.rank_tolerance);
controllability_rank = matrix_controllability_rank(A, B, ...
                                                    p.numerics.rank_tolerance);
closed_loop_poles = eig(A - B * K);

fprintf('\nCollinear Mecanum robot model\n');
fprintf('  Independent configuration DOF : 4\n');
fprintf('  Mechanical state dimension    : 8\n');
fprintf('  Simulation state dimension    : 12\n');
fprintf('  Wheel kinematic map rank      : %d\n', wheel_map_rank);
fprintf('  Independent actuation rank    : %d\n', input_map_rank);
fprintf('  Linear controllability rank   : %d of %d\n', ...
        controllability_rank, size(A, 1));
fprintf('  LQR solution                  : %s\n\n', lqr_source);

if wheel_map_rank ~= 3
    error('The wheel geometry does not provide full planar mobility.');
end

if input_map_rank ~= 3
    error('The expected CMD input-map rank is three, but the model produced %d.', ...
          input_map_rank);
end

if controllability_rank ~= size(A, 1)
    error('The upright linear model is not fully controllable.');
end

if any(real(closed_loop_poles) >= 0.0)
    error('The designed continuous-time closed loop is not asymptotically stable.');
end

%% RUN FIXED-STEP NONLINEAR SIMULATION
MODE_DISARMED = uint8(0);
MODE_BALANCING = uint8(1);
MODE_FALLEN = uint8(2);

dt = p.integration_step_s;
time = 0.0:dt:p.simulation_time_s;
sample_count = numel(time);

% Augmented state order
%   1:4    q          [X Y psi theta]
%   5:8    qdot       [Xdot Ydot psidot thetadot]
%   9:12   tau_actual four output-shaft motor torques
x = zeros(12, sample_count);
x(:, 1) = [p.initial_position_m;
           p.initial_yaw_rad;
           p.initial_lean_rad;
           p.initial_velocity;
           zeros(4, 1)];

tau_command = zeros(4, sample_count);
wheel_speed = zeros(4, sample_count);
reference_q = zeros(4, sample_count);
reference_qdot = zeros(4, sample_count);
mode_log = zeros(1, sample_count, 'uint8');

mode = MODE_DISARMED;
held_tau_command = zeros(4, 1);
next_control_time = 0.0;

for k = 1:(sample_count - 1)
    t = time(k);
    mode = update_safety_mode(mode, t, x(:, k), p, ...
                              MODE_DISARMED, MODE_BALANCING, MODE_FALLEN);

    [q_reference, qdot_reference] = reference_trajectory(t, p);
    reference_q(:, k) = q_reference;
    reference_qdot(:, k) = qdot_reference;

    if t + 0.5 * dt >= next_control_time
        if mode == MODE_BALANCING
            held_tau_command = state_feedback_command(x(1:8, k), ...
                                                      q_reference, ...
                                                      qdot_reference, K);
        else
            held_tau_command = zeros(4, 1);
        end

        held_tau_command = saturate_vector(held_tau_command, ...
                                            p.motor_torque_limit_Nm);
        next_control_time = next_control_time + p.controller_period_s;
    end

    tau_command(:, k) = held_tau_command;
    wheel_speed(:, k) = wheel_jacobian(x(1:4, k), p) * x(5:8, k);
    mode_log(k) = mode;

    x(:, k + 1) = rk4_step(x(:, k), held_tau_command, dt, p);

    if any(~isfinite(x(:, k + 1)))
        error('Simulation became non-finite at t = %.6f s.', time(k + 1));
    end
end

[q_reference, qdot_reference] = reference_trajectory(time(end), p);
reference_q(:, end) = q_reference;
reference_qdot(:, end) = qdot_reference;
tau_command(:, end) = held_tau_command;
wheel_speed(:, end) = wheel_jacobian(x(1:4, end), p) * x(5:8, end);
mode_log(end) = update_safety_mode(mode, time(end), x(:, end), p, ...
                                   MODE_DISARMED, MODE_BALANCING, MODE_FALLEN);

result = struct();
result.time_s = time;
result.state = x;
result.state_names = { ...
    'X_m', 'Y_m', 'yaw_rad', 'lean_rad', ...
    'Xdot_mps', 'Ydot_mps', 'yaw_rate_rps', 'lean_rate_rps', ...
    'motor_1_torque_Nm', 'motor_2_torque_Nm', ...
    'motor_3_torque_Nm', 'motor_4_torque_Nm'};
result.torque_command_Nm = tau_command;
result.wheel_speed_radps = wheel_speed;
result.reference_q = reference_q;
result.reference_qdot = reference_qdot;
result.mode = mode_log;
result.mode_names = {'DISARMED', 'BALANCING', 'FALLEN'};
result.A = A;
result.B = B;
result.K = K;
result.closed_loop_poles = closed_loop_poles;
result.parameters = p;

plot_results(result, p);

if p.enable_animation
    animate_planar_motion(result, p);
end

if p.save_results
    save(p.results_filename, 'result');
    fprintf('Saved simulation result to %s\n', p.results_filename);
end

final_lean_deg = rad2deg(x(4, end));
maximum_lean_deg = rad2deg(max(abs(x(4, :))));
fprintf('Final planar pose             : X %.3f m, Y %.3f m, yaw %.2f deg\n', ...
        x(1, end), x(2, end), rad2deg(x(3, end)));
fprintf('Final lean                    : %.3f deg\n', final_lean_deg);
fprintf('Maximum absolute lean         : %.3f deg\n', maximum_lean_deg);
fprintf('Final simulation mode         : %s\n', ...
        result.mode_names{double(mode_log(end)) + 1});

%% LOCAL FUNCTIONS

function validate_parameters(p)
%VALIDATE_PARAMETERS Reject inconsistent geometry or nonphysical values.

    must_be_positive = [p.gravity_mps2;
                        p.wheel_diameter_m;
                        p.body_com_height_m;
                        p.body_mass_kg;
                        p.base_mass_kg;
                        diag(p.body_inertia_kgm2);
                        p.base_yaw_inertia_kgm2;
                        p.wheel_spin_inertia_kgm2;
                        p.motor_torque_limit_Nm;
                        p.motor_torque_slew_limit_Nmps;
                        p.motor_torque_time_constant_s;
                        p.controller_period_s;
                        p.integration_step_s;
                        p.simulation_time_s];

    if any(~isfinite(must_be_positive)) || any(must_be_positive <= 0.0)
        error(['All physical, actuator, timing, and inertia parameters ' ...
               'must be finite and positive.']);
    end

    if numel(p.wheel_offsets_x_m) ~= 4 || numel(p.roller_angles_rad) ~= 4
        error('Exactly four wheel offsets and four roller angles are required.');
    end

    if any(abs(sin(p.roller_angles_rad)) < 1.0e-6)
        error('Roller angles must not be zero or 180 degrees.');
    end

    if ~isequal(size(p.body_inertia_kgm2), [3, 3]) || ...
       norm(p.body_inertia_kgm2 - p.body_inertia_kgm2.', 'fro') > 1.0e-12
        error('p.body_inertia_kgm2 must be a symmetric 3-by-3 matrix.');
    end

    if min(eig(p.body_inertia_kgm2)) <= 0.0
        error('p.body_inertia_kgm2 must be positive definite.');
    end

    if p.integration_step_s > p.controller_period_s
        error('The integration step must not exceed the controller period.');
    end

    ratio = p.controller_period_s / p.integration_step_s;
    if abs(ratio - round(ratio)) > 1.0e-9
        error('The controller period must be an integer multiple of the integration step.');
    end

    [M_upright, ~] = mass_matrix_and_derivatives(zeros(4, 1), p);
    if min(eig(M_upright)) <= 0.0
        error('The upright mass matrix is not positive definite. Check mass and inertia values.');
    end
end

function dx = augmented_dynamics(x, tau_command, p)
%AUGMENTED_DYNAMICS Nonlinear mechanical plant plus actuator torque lag.

    mechanical_state = x(1:8);
    tau_actual = x(9:12);

    mechanical_derivative = mechanical_dynamics(mechanical_state, ...
                                                tau_actual, p);

    tau_target = saturate_vector(tau_command, p.motor_torque_limit_Nm);
    tau_rate = (tau_target - tau_actual) / p.motor_torque_time_constant_s;
    tau_rate = saturate_vector(tau_rate, p.motor_torque_slew_limit_Nmps);

    dx = [mechanical_derivative; tau_rate];
end

function dx = mechanical_dynamics(x, motor_torque, p)
%MECHANICAL_DYNAMICS Evaluate M(q)qdd + c(q,qdot) + g(q) = Q.

    q = x(1:4);
    qdot = x(5:8);

    [M, dM_dq] = mass_matrix_and_derivatives(q, p);
    A_wheel = wheel_jacobian(q, p);
    B_motor = input_matrix(q, p);

    % Coriolis and centrifugal vector obtained directly from the kinetic
    % energy identity for T = 0.5*qdot'*M(q)*qdot. This avoids selecting a
    % non-unique C matrix while retaining the exact energy-consistent term.
    M_dot = zeros(4, 4);
    kinetic_gradient = zeros(4, 1);
    for i = 1:4
        M_dot = M_dot + dM_dq(:, :, i) * qdot(i);
        kinetic_gradient(i) = 0.5 * qdot.' * dM_dq(:, :, i) * qdot;
    end
    coriolis_vector = M_dot * qdot - kinetic_gradient;

    gravity_vector = [0.0;
                      0.0;
                      0.0;
                     -p.body_mass_kg * p.gravity_mps2 * ...
                      p.body_com_height_m * sin(q(4))];

    % Body-frame anisotropic drag is rotated into the world frame.
    R_body_to_world = planar_rotation(q(3));
    D_planar_world = R_body_to_world * ...
                     diag(p.planar_viscous_Ns_per_m) * ...
                     R_body_to_world.';
    drag_force = -[D_planar_world * qdot(1:2);
                   p.yaw_viscous_Nms_per_rad * qdot(3);
                   p.lean_viscous_Nms_per_rad * qdot(4)];

    % Bearing loss depends on wheel speed relative to the leaning body.
    % The equal and opposite reaction is applied to the lean coordinate.
    wheel_speed = A_wheel * qdot;
    bearing_torque_on_wheel = ...
        -p.wheel_bearing_viscous_Nms_per_rad * (wheel_speed - qdot(4));
    bearing_generalised_force = A_wheel.' * bearing_torque_on_wheel;
    bearing_generalised_force(4) = bearing_generalised_force(4) ...
                                      - sum(bearing_torque_on_wheel);

    generalised_force = B_motor * motor_torque ...
                        + drag_force ...
                        + bearing_generalised_force;

    qddot = M \ (generalised_force - coriolis_vector - gravity_vector);
    dx = [qdot; qddot];
end

function [M, dM_dq] = mass_matrix_and_derivatives(q, p)
%MASS_MATRIX_AND_DERIVATIVES Build M(q) and its analytic derivatives.
%
% The mass matrix comes from the translational and rotational kinetic
% energies of the axle-height base, leaning body, and constrained wheel
% spins. Only yaw and lean change the matrix, so dM/dX and dM/dY are zero.

    psi = q(3);
    theta = q(4);
    c_psi = cos(psi);
    s_psi = sin(psi);
    c_theta = cos(theta);
    s_theta = sin(theta);
    h = p.body_com_height_m;

    % Jacobian from qdot to body COM translational velocity in world axes.
    J_com = [1.0, 0.0, h * c_psi * s_theta,  h * s_psi * c_theta;
             0.0, 1.0, h * s_psi * s_theta, -h * c_psi * c_theta;
             0.0, 0.0, 0.0,                     -h * s_theta];

    % Jacobian from qdot to body angular velocity in leaning-body axes.
    J_omega = [0.0, 0.0, 0.0,     1.0;
               0.0, 0.0, s_theta, 0.0;
               0.0, 0.0, c_theta, 0.0];

    A_wheel = wheel_jacobian(q, p);

    M_base = diag([p.base_mass_kg, ...
                   p.base_mass_kg, ...
                   p.base_yaw_inertia_kgm2, ...
                   0.0]);

    M = M_base ...
        + p.body_mass_kg * (J_com.' * J_com) ...
        + J_omega.' * p.body_inertia_kgm2 * J_omega ...
        + p.wheel_spin_inertia_kgm2 * (A_wheel.' * A_wheel);

    dM_dq = zeros(4, 4, 4);

    dJ_com_dpsi = zeros(3, 4);
    dJ_com_dpsi(:, 3) = [-h * s_psi * s_theta;
                          h * c_psi * s_theta;
                          0.0];
    dJ_com_dpsi(:, 4) = [h * c_psi * c_theta;
                         h * s_psi * c_theta;
                         0.0];

    cot_alpha = cot(p.roller_angles_rad);
    dA_wheel_dpsi = zeros(4, 4);
    dA_wheel_dpsi(:, 1) = ...
        (cot_alpha * s_psi - c_psi) / p.wheel_radius_m;
    dA_wheel_dpsi(:, 2) = ...
        (-s_psi - cot_alpha * c_psi) / p.wheel_radius_m;

    dM_dq(:, :, 3) = ...
        p.body_mass_kg * (dJ_com_dpsi.' * J_com ...
                          + J_com.' * dJ_com_dpsi) ...
        + p.wheel_spin_inertia_kgm2 * ...
          (dA_wheel_dpsi.' * A_wheel ...
           + A_wheel.' * dA_wheel_dpsi);

    dJ_com_dtheta = zeros(3, 4);
    dJ_com_dtheta(:, 3) = [h * c_psi * c_theta;
                           h * s_psi * c_theta;
                           0.0];
    dJ_com_dtheta(:, 4) = [-h * s_psi * s_theta;
                            h * c_psi * s_theta;
                           -h * c_theta];

    dJ_omega_dtheta = [0.0, 0.0,  0.0,     0.0;
                       0.0, 0.0,  c_theta, 0.0;
                       0.0, 0.0, -s_theta, 0.0];

    dM_dq(:, :, 4) = ...
        p.body_mass_kg * (dJ_com_dtheta.' * J_com ...
                          + J_com.' * dJ_com_dtheta) ...
        + dJ_omega_dtheta.' * p.body_inertia_kgm2 * J_omega ...
        + J_omega.' * p.body_inertia_kgm2 * dJ_omega_dtheta;

    % Limit floating-point asymmetry before solving M*qddot = force.
    M = 0.5 * (M + M.');
end

function A_wheel = wheel_jacobian(q, p)
%WHEEL_JACOBIAN Map reduced qdot to the four wheel speeds [rad/s].
%
% For wheel i
%   omega_i = (-cot(alpha_i)*v_x + v_y + l_i*psi_dot) / r_w
%
% where v_x and v_y are body-frame planar velocities. Using measured roller
% handedness and motor polarity is essential. If a physical wheel spins in
% the opposite direction, correct its alpha or introduce a documented motor
% sign only after a single-wheel commissioning test.

    psi = q(3);
    c_psi = cos(psi);
    s_psi = sin(psi);
    cot_alpha = cot(p.roller_angles_rad);

    A_wheel = zeros(4, 4);
    A_wheel(:, 1) = -(cot_alpha * c_psi + s_psi) / p.wheel_radius_m;
    A_wheel(:, 2) = (c_psi - cot_alpha * s_psi) / p.wheel_radius_m;
    A_wheel(:, 3) = p.wheel_offsets_x_m / p.wheel_radius_m;
end

function B_motor = input_matrix(q, p)
%INPUT_MATRIX Map four motor torques to reduced generalised forces.
%
% A_wheel'*tau follows from power consistency. The last row includes the
% equal and opposite motor reaction torque applied to the leaning body.

    A_wheel = wheel_jacobian(q, p);
    lean_reaction = [0.0; 0.0; 0.0; 1.0] * ones(1, 4);
    B_motor = A_wheel.' - lean_reaction;
end

function R = planar_rotation(yaw)
%PLANAR_ROTATION Rotate a body-frame planar vector into the world frame.

    R = [cos(yaw), -sin(yaw);
         sin(yaw),  cos(yaw)];
end

function [A, B] = linearise_mechanical_model(x_eq, u_eq, p)
%LINEARISE_MECHANICAL_MODEL Central-difference linearisation about equilibrium.

    nx = numel(x_eq);
    nu = numel(u_eq);
    A = zeros(nx, nx);
    B = zeros(nx, nu);

    for i = 1:nx
        delta = zeros(nx, 1);
        delta(i) = p.numerics.state_step;
        f_plus = mechanical_dynamics(x_eq + delta, u_eq, p);
        f_minus = mechanical_dynamics(x_eq - delta, u_eq, p);
        A(:, i) = (f_plus - f_minus) / (2.0 * p.numerics.state_step);
    end

    for i = 1:nu
        delta = zeros(nu, 1);
        delta(i) = p.numerics.input_step;
        f_plus = mechanical_dynamics(x_eq, u_eq + delta, p);
        f_minus = mechanical_dynamics(x_eq, u_eq - delta, p);
        B(:, i) = (f_plus - f_minus) / (2.0 * p.numerics.input_step);
    end
end

function [K, source] = design_lqr(A, B, Q, R)
%DESIGN_LQR Solve the continuous-time LQR problem.

    if exist('lqr', 'file') == 2
        K = lqr(A, B, Q, R);
        source = 'Control System Toolbox lqr()';
        return;
    end

    % Base-MATLAB fallback using the stable invariant subspace of the
    % Hamiltonian matrix. This solves the continuous algebraic Riccati
    % equation without hiding fixed, robot-specific gains in the script.
    n = size(A, 1);
    H = [A,             -B * (R \ B.');
        -Q,             -A.'];
    [V, D] = eig(H);
    stable_columns = find(real(diag(D)) < -1.0e-9);

    if numel(stable_columns) ~= n
        error(['Unable to isolate the stable Hamiltonian subspace. ' ...
               'Install Control System Toolbox or revise Q, R, and the model parameters.']);
    end

    V1 = V(1:n, stable_columns);
    V2 = V((n + 1):(2 * n), stable_columns);
    if rcond(V1) < 1.0e-12
        error('The Hamiltonian CARE solution is ill-conditioned.');
    end

    P = real(V2 / V1);
    P = 0.5 * (P + P.');
    K = R \ (B.' * P);
    source = 'base-MATLAB Hamiltonian CARE fallback';
end

function rank_value = matrix_controllability_rank(A, B, tolerance)
%MATRIX_CONTROLLABILITY_RANK Evaluate rank([B AB ... A^(n-1)B]).

    n = size(A, 1);
    controllability_matrix = B;
    A_power_B = B;
    for i = 2:n
        A_power_B = A * A_power_B;
        controllability_matrix = [controllability_matrix, A_power_B]; %#ok<AGROW>
    end
    rank_value = rank(controllability_matrix, tolerance);
end

function tau_command = state_feedback_command(x, q_reference, ...
                                               qdot_reference, K)
%STATE_FEEDBACK_COMMAND Upright LQR with planar errors resolved in body axes.
%
% Rotating planar position and velocity errors into body axes gives the
% equilibrium gain useful yaw invariance without gain scheduling.

    q = x(1:4);
    qdot = x(5:8);
    R_body_to_world = planar_rotation(q(3));

    planar_position_error_body = ...
        R_body_to_world.' * (q(1:2) - q_reference(1:2));
    planar_velocity_error_body = ...
        R_body_to_world.' * (qdot(1:2) - qdot_reference(1:2));

    state_error = [planar_position_error_body;
                   wrap_to_pi(q(3) - q_reference(3));
                   q(4) - q_reference(4);
                   planar_velocity_error_body;
                   qdot(3) - qdot_reference(3);
                   qdot(4) - qdot_reference(4)];

    tau_command = -K * state_error;
end

function mode = update_safety_mode(mode, t, x, p, ...
                                   MODE_DISARMED, MODE_BALANCING, MODE_FALLEN)
%UPDATE_SAFETY_MODE Minimal latched simulation safety state machine.

    if mode == MODE_FALLEN
        return;
    end

    if any(~isfinite(x)) || abs(x(4)) >= p.fall_lean_limit_rad
        mode = MODE_FALLEN;
        return;
    end

    if mode == MODE_DISARMED && ...
       t >= p.arm_time_s && ...
       abs(x(4)) <= p.arm_lean_limit_rad && ...
       abs(x(8)) <= p.arm_lean_rate_limit_rps
        mode = MODE_BALANCING;
    end
end

function [q_reference, qdot_reference] = reference_trajectory(t, p)
%REFERENCE_TRAJECTORY Smooth forward, lateral, and yaw test manoeuvres.

    [s_forward, ds_forward] = smooth_step(t, ...
        p.reference.forward_start_s, p.reference.forward_end_s);
    [s_lateral, ds_lateral] = smooth_step(t, ...
        p.reference.lateral_start_s, p.reference.lateral_end_s);
    [s_yaw, ds_yaw] = smooth_step(t, ...
        p.reference.yaw_start_s, p.reference.yaw_end_s);

    q_reference = [p.reference.lateral_distance_m * s_lateral;
                   p.reference.forward_distance_m * s_forward;
                   p.reference.yaw_angle_rad * s_yaw;
                   0.0];

    qdot_reference = [p.reference.lateral_distance_m * ds_lateral;
                      p.reference.forward_distance_m * ds_forward;
                      p.reference.yaw_angle_rad * ds_yaw;
                      0.0];
end

function [s, ds] = smooth_step(t, start_time, end_time)
%SMOOTH_STEP Quintic interpolation with zero endpoint velocity and acceleration.

    if t <= start_time
        s = 0.0;
        ds = 0.0;
        return;
    end

    if t >= end_time
        s = 1.0;
        ds = 0.0;
        return;
    end

    duration = end_time - start_time;
    xi = (t - start_time) / duration;
    s = 10.0 * xi^3 - 15.0 * xi^4 + 6.0 * xi^5;
    ds = (30.0 * xi^2 - 60.0 * xi^3 + 30.0 * xi^4) / duration;
end

function x_next = rk4_step(x, tau_command, dt, p)
%RK4_STEP One fixed-step fourth-order Runge-Kutta integration step.

    k1 = augmented_dynamics(x, tau_command, p);
    k2 = augmented_dynamics(x + 0.5 * dt * k1, tau_command, p);
    k3 = augmented_dynamics(x + 0.5 * dt * k2, tau_command, p);
    k4 = augmented_dynamics(x + dt * k3, tau_command, p);
    x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
end

function y = saturate_vector(x, absolute_limit)
%SATURATE_VECTOR Apply a symmetric scalar limit element by element.

    y = min(max(x, -absolute_limit), absolute_limit);
end

function angle = wrap_to_pi(angle)
%WRAP_TO_PI Wrap radians to [-pi, pi) without Mapping Toolbox.

    angle = mod(angle + pi, 2.0 * pi) - pi;
end

function plot_results(result, p)
%PLOT_RESULTS Produce controller, actuator, and safety validation plots.

    t = result.time_s;
    x = result.state;

    figure('Name', 'CMD planar motion', 'Color', 'w');
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(result.reference_q(1, :), result.reference_q(2, :), 'k--', ...
         'LineWidth', 1.5);
    hold on;
    plot(x(1, :), x(2, :), 'b', 'LineWidth', 1.5);
    axis equal;
    grid on;
    xlabel('World X [m]');
    ylabel('World Y [m]');
    title('Planar path');
    legend('Reference', 'Nonlinear plant', 'Location', 'best');

    nexttile;
    plot(t, rad2deg(result.reference_q(3, :)), 'k--', 'LineWidth', 1.2);
    hold on;
    plot(t, rad2deg(x(3, :)), 'b', 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Yaw [deg]');
    title('Yaw tracking');
    legend('Reference', 'Actual', 'Location', 'best');

    nexttile;
    plot(t, rad2deg(x(4, :)), 'r', 'LineWidth', 1.5);
    hold on;
    yline(rad2deg(p.fall_lean_limit_rad), 'k:');
    yline(-rad2deg(p.fall_lean_limit_rad), 'k:');
    grid on;
    xlabel('Time [s]');
    ylabel('Lean [deg]');
    title('Balance coordinate');

    nexttile;
    stairs(t, double(result.mode), 'LineWidth', 1.5);
    grid on;
    yticks([0, 1, 2]);
    yticklabels(result.mode_names);
    ylim([-0.25, 2.25]);
    xlabel('Time [s]');
    ylabel('Mode');
    title('Latched simulation safety state');

    figure('Name', 'CMD actuator states', 'Color', 'w');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(t, result.torque_command_Nm.', '--', 'LineWidth', 1.0);
    hold on;
    color_order = colororder;
    for i = 1:4
        plot(t, x(8 + i, :), 'Color', color_order(i, :), ...
             'LineWidth', 1.4);
    end
    yline(p.motor_torque_limit_Nm, 'k:');
    yline(-p.motor_torque_limit_Nm, 'k:');
    grid on;
    xlabel('Time [s]');
    ylabel('Output torque [N m]');
    title('Dashed command and solid actuator torque');
    legend('M1 cmd', 'M2 cmd', 'M3 cmd', 'M4 cmd', ...
           'M1 actual', 'M2 actual', 'M3 actual', 'M4 actual', ...
           'Location', 'eastoutside');

    nexttile;
    plot(t, result.wheel_speed_radps.', 'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('Wheel speed [rad/s]');
    title('Ideal constrained wheel speeds');
    legend('M1', 'M2', 'M3', 'M4', 'Location', 'eastoutside');
end

function animate_planar_motion(result, p)
%ANIMATE_PLANAR_MOTION Lightweight top-view and lean animation.

    t = result.time_s;
    x = result.state;
    frame_period = 1.0 / 30.0;
    frame_stride = max(1, round(frame_period / p.integration_step_s));
    half_length = max(abs(p.wheel_offsets_x_m)) + p.wheel_radius_m;
    lean_scale = p.body_com_height_m;

    figure('Name', 'CMD animation', 'Color', 'w');
    tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax_plan = nexttile;
    hold(ax_plan, 'on');
    grid(ax_plan, 'on');
    axis(ax_plan, 'equal');
    plot(ax_plan, result.reference_q(1, :), result.reference_q(2, :), 'k--');
    path_line = plot(ax_plan, nan, nan, 'b', 'LineWidth', 1.2);
    axle_line = plot(ax_plan, nan, nan, 'r', 'LineWidth', 4.0);
    heading_line = plot(ax_plan, nan, nan, 'k', 'LineWidth', 2.0);
    xlabel(ax_plan, 'World X [m]');
    ylabel(ax_plan, 'World Y [m]');
    title(ax_plan, 'Top view');
    xlim(ax_plan, [min(result.reference_q(1, :)) - 0.4, ...
                   max(result.reference_q(1, :)) + 0.4]);
    ylim(ax_plan, [min(result.reference_q(2, :)) - 0.4, ...
                   max(result.reference_q(2, :)) + 0.4]);

    ax_lean = nexttile;
    hold(ax_lean, 'on');
    grid(ax_lean, 'on');
    axis(ax_lean, 'equal');
    lean_line = plot(ax_lean, nan, nan, 'b', 'LineWidth', 5.0);
    plot(ax_lean, [-0.2, 0.2], [0.0, 0.0], 'k');
    xlabel(ax_lean, 'Forward displacement from axle [m]');
    ylabel(ax_lean, 'Height [m]');
    title(ax_lean, 'Lean view');
    xlim(ax_lean, 1.2 * [-lean_scale, lean_scale]);
    ylim(ax_lean, [-0.05, 1.2 * lean_scale]);

    for k = 1:frame_stride:numel(t)
        position = x(1:2, k);
        yaw = x(3, k);
        lean = x(4, k);
        R = planar_rotation(yaw);

        axle_endpoints = position + ...
            R * [-half_length, half_length; 0.0, 0.0];
        heading_endpoint = position + ...
            R * [0.0; 0.65 * half_length];

        set(path_line, 'XData', x(1, 1:k), 'YData', x(2, 1:k));
        set(axle_line, 'XData', axle_endpoints(1, :), ...
                       'YData', axle_endpoints(2, :));
        set(heading_line, 'XData', [position(1), heading_endpoint(1)], ...
                          'YData', [position(2), heading_endpoint(2)]);
        set(lean_line, 'XData', [0.0, lean_scale * sin(lean)], ...
                       'YData', [0.0, lean_scale * cos(lean)]);
        title(ax_lean, sprintf('Lean view, t = %.2f s', t(k)));
        drawnow;
        pause(frame_period / max(p.animation_speed, eps));
    end
end
