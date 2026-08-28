%% =========================================================================
%  SYMBOLIC FORWARD KINEMATICS - DOBOT MAGICIAN (CLASSICAL DH CONVENTION)
%  =========================================================================
%  Computes the Total Homogeneous Transformation Matrix (T1_7) and extracts
%  the closed-form algebraic equations for Px, Py, Pz relative to the 
%  displaced base reference frame {1}, strictly accounting for the 
%  Tool Center Point (TCP) offsets.
%  =========================================================================
clc; clear; close all;

%% 1. DECLARATION OF SYMBOLIC VARIABLES
syms theta1 theta2 theta3 theta4 theta5 real
syms L0 L1 L2 L3 L4 L5 L6 real

% Mechanical constraint of the 4-bar parallelogram linkage enforces 
% that the end-effector remains parallel to the horizontal plane.
% Therefore, the controllable wrist pitch is mathematically zeroed.
theta4 = sym(0); 

% Exact symbolic Pi constant to prevent floating-point residual errors
pi_sym = sym(pi);

%% 2. JOINT ANGLE MAPPING (q_n)
% Corrected hybrid-to-serial joint mapping to preserve absolute orientations
% within an open-chain evaluation matrix.
q1 = theta1;
q2 = -pi_sym/2 + theta2;
q3 =  pi_sym/2 - theta2 + theta3; % Forearm absolute angle accurately tracks theta3
q4 = theta4 - theta3;             % For theta4 = 0 -> q4 = -theta3
q5 = theta5;                      % Gripper roll action

%% 3. CLASSICAL DENAVIT-HARTENBERG HOMOGENEOUS TRANSFORMATION MATRIX
% Standard algebraic structure: A_i = Rot_z(q) * Trans_z(d) * Trans_x(r) * Rot_x(alpha)
dh_mat = @(r, alpha, d, q) [
    cos(q), -sin(q)*cos(alpha),  sin(q)*sin(alpha), r*cos(q);
    sin(q),  cos(q)*cos(alpha), -cos(q)*sin(alpha), r*sin(q);
    sym(0),  sin(alpha),         cos(alpha),        d;
    sym(0),  sym(0),             sym(0),            sym(1)
];

%% 4. INDIVIDUAL HOMOGENEOUS TRANSFORMATION MATRICES
T0_1 = dh_mat(sym(0),  sym(0), L0, 0);   % Link 2: Base Rotation
T1_2 = dh_mat(sym(0), -pi_sym/2, L1, q1);   % Link 2: Base Rotation
T2_3 = dh_mat(L2,      sym(0),    sym(0), q2);   % Link 3: Shoulder Pitch
T3_4 = dh_mat(L3,      sym(0),    sym(0), q3);   % Link 4: Elbow Pitch
T4_5 = dh_mat(L4, -pi_sym/2,  sym(0), q4);   % Link 5: Wrist Pitch (Compensated)
T5_6 = dh_mat(sym(0),  sym(0),    L5, sym(0));   % Link 6: Flange Z-Offset
T6_7 = dh_mat(sym(0),  sym(0),    L6, q5);   % Link 7: Tool Extension & Roll

%% 5. TOTAL HOMOGENEOUS TRANSFORMATION (FRAME {1} TO FRAME {7})
% Multiply sequential transformation frames algebraically
T1_7 = simplify(T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7);

% Extract independent Cartesian position coordinates (Px, Py, Pz) for the TCP
p_x = vpa(simplify(T1_7(1, 4)),2);
p_y = vpa(simplify(T1_7(2, 4)),2);
p_z = vpa(simplify(T1_7(3, 4)),2);

%% 6. DISPLAY SYMBOLIC RESULTS
fprintf('=================================================================\n');
fprintf('  CLOSED-FORM POSITION EXPRESSIONS IN DISPLACED FRAME {1} (TCP)  \n');
fprintf('=================================================================\n');
fprintf('Px = %s\n', char(p_x));
fprintf('Py = %s\n', char(p_y));
fprintf('Pz = %s\n', char(p_z));
fprintf('=================================================================\n\n');

%% 7. NUMERICAL EVALUATION EXAMPLE (NOMINAL HARDWARE DIMENSIONS)
% Measured structural constraints (millimeters)
L0_val = 130; L1_val = 8; L2_val = 135; L3_val = 147; 
L4_val = 59.7; L5_val = 8; L6_val = 74; % L6=0 represents Flange Face

% Test evaluation at Home / Zero configuration
T1_7_num = double(subs(T1_7, ...
    [L1, L2, L3, L4, L5, L6, theta1, theta2, theta3, theta5], ...
    [L1_val, L2_val, L3_val, L4_val, L5_val, L6_val, 0, 0, 0, 0]));

fprintf('Numerical Coordinates at Home [0, 0, 0, 0] with Tool L6 = %d mm:\n', L6_val);
fprintf('Px: %.2f mm\n', T1_7_num(1, 4));
fprintf('Py: %.2f mm\n', T1_7_num(2, 4));
fprintf('Pz: %.2f mm\n', T1_7_num(3, 4));