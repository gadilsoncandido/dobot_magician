% =========================================================================
% DOBOT MAGICIAN 7-FRAME KINEMATIC MODEL (CLASSICAL DH CONVENTION)
% =========================================================================
% This script implements a high-fidelity kinematic model of the Dobot 
% Magician manipulator in MATLAB using rigidBodyTree.
% It strictly adheres to the 7-link Classical Denavit-Hartenberg (DH) table,
% incorporating the base offset L0, intermediate frames, flange, and tool.
%
% Features:
% - Exact 7-frame classical DH parameter mapping.
% - Closed-form and forward kinematics visualization.
% - Academic paper export mode and headless clipboard export option.
% =========================================================================
clc; clear; close all;

% =========================================================================
% 1. USER INPUTS: JOINT ANGLES & ATTACHMENTS
% =========================================================================
% Define the input angles for each joint (in degrees):
theta1 = 0;   % Base Pan Rotation (J1)
theta2 = 0;   % Shoulder Pitch (J2)
theta3 = 0;   % Elbow Pitch (J3)
theta4 = 0;   % Wrist Pitch (J4) - Parallelism constraint (theta4 = 0)
theta5 = 0;   % Tool/Gripper Roll (J5)

% Transform relative input angles into absolute kinematic states (radians)
% Equation:
% q1 = theta1
% q2 = -90 + theta2
% q3 =  90 - theta2 + theta3
% q4 = theta4 - theta3
% q5 = theta5
angles_degrees = [theta1, -90 + theta2, 90 - theta2 + theta3, theta4 - theta3, theta5]; 
angles_rad = deg2rad(angles_degrees);

% End-Effector / Tool extension presence toggle
has_gripper = true; 

% =========================================================================
% 2. ROBOT KINEMATIC DIMENSIONS (in mm)
% =========================================================================
L0 = 130;    % Base Pedestal Lower Section (Frame 0 -> Frame 1 offset)
L1 = 8;      % Base Pedestal Upper Section (Frame 1 -> Frame 2 offset)
L2 = 135;    % Rear Arm / Shoulder Link Length
L3 = 147;    % Forearm / Elbow Link Length
L4 = 59.7;   % Wrist Link Length
L5 = 8;      % Standard Mounting Flange Thickness
L6 = 74;     % End-Effector / Tool Extension Length

% =========================================================================
% 3. VISUALIZATION AND REFERENCE FRAME PREFERENCES
% =========================================================================
ctrl_viz.display_text_position = true; 

% Target point for forward kinematics: 'end_extension' (TCP) or 'flange' (Frame 6)
if has_gripper
    ctrl_viz.measurement_point = 'end_extension'; 
else
    ctrl_viz.measurement_point = 'flange'; 
end

% Reference frame computation behavior:
% 'base'         -> Global inercial frame {0} at table level (Z = 0)
% 'frame1'       -> Displaced base frame {1} at Z = L0 (130 mm)
% 'shoulder'     -> Shoulder joint axis {2}
% 'offset_base'  -> Table reference plane at Z = L0
ctrl_viz.reference_frame = 'offset_base'; 

ctrl_viz.paper_mode = false;     % Suppress floating labels for clean export
ctrl_viz.headless_export = false; % Headless execution to clipboard only

% --- GRAPHICAL FRAME CONTROL (8 Explicit Frames: 0 to 7) ---
% 1: Base Origin {0}
% 2: Displaced Base Frame {1}
% 3: Shoulder Joint Frame {2}
% 4: Elbow Joint Frame {3}
% 5: Wrist Joint Frame {4}
% 6: Flange Input Frame {5}
% 7: Flange Tool Face {6}
% 8: Tool Tip / TCP {7}
ctrl_viz.active_frames = [1, 1, 1, 1, 1, 1, 1, 1]; 
ctrl_viz.active_axes   = ones(8, 3); 
ctrl_viz.active_text   = ones(8, 3); 
ctrl_viz.show_body_names = true;

% =========================================================================
% 4. COLOR DEFINITIONS FOR RIGID BODY VISUALS
% =========================================================================
link_colors        = [0.9 0.9 0.9];
detail_colors      = [0.2 0.2 0.2];
joint_colors       = [0.2 0.2 0.2];
gripper_base_color = [0.3 0.3 0.3];
gripper_fingers_color = [0.8 0.8 0.8];
extension_color    = [0.1 0.1 0.1];
flange_color       = [0.6 0.6 0.6];

% =========================================================================
% 5. DH MATRIX CONSTRUCTION AND ROBOT ASSEMBLY
% =========================================================================
robot = rigidBodyTree('DataFormat', 'column');

% CLASSICAL DH PARAMETERS MATRIX: [r_n, alpha_n, d_n, q_n]
dhParams = [
    0,      0,       L0,   0;   % Link 1: Base Offset {0} -> {1} (Fixed)
    0,     -pi/2,    L1,   0;   % Link 2: Base Rotation {1} -> {2} (J1: q1)
    L2,     0,       0,    0;   % Link 3: Shoulder Pitch {2} -> {3} (J2: q2)
    L3,     0,       0,    0;   % Link 4: Elbow Pitch {3} -> {4} (J3: q3)
    L4,    -pi/2,    0,    0;   % Link 5: Wrist Pitch {4} -> {5} (J4: q4)
    0,      0,       L5,   0;   % Link 6: Tool Flange {5} -> {6} (Fixed)
];

bodyNames  = {'Base_Offset_Link', 'Base_Link', 'Shoulder_Link', 'Elbow_Link', 'Wrist_Link', 'Flange_Link'};
jointNames = {'Joint_Offset', 'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Flange_Joint'};
jointTypes = {'fixed', 'revolute', 'revolute', 'revolute', 'revolute', 'fixed'};

% Link 7: Tool Extension / Gripper Roll {6} -> {7} (J5: q5)
if has_gripper
    dhParams(7,:) = [0, 0, L6, 0];
    bodyNames{7}  = 'Extension_Link';
    jointNames{7} = 'Gripper_Roll_Joint';
    jointTypes{7} = 'revolute';
end

% Assemble kinematic tree
parentName = 'base';
num_links = size(dhParams, 1);

for i = 1:num_links
    body = rigidBody(bodyNames{i});
    jnt  = rigidBodyJoint(jointNames{i}, jointTypes{i});
    
    setFixedTransform(jnt, dhParams(i,:), 'dh');
    body.Joint = jnt;
    
    r_dh = dhParams(i, 1); 
    d_dh = dhParams(i, 3); 
    alpha_dh = dhParams(i, 2);
    
    % --- Graphical Visual Attachments ---
    if i == 1 % Base pedestal base section
        if d_dh > 0
            t_vis = trvec2tform([0, 0, -d_dh/2]);
            addVisual(body, 'Cylinder', [22, d_dh], t_vis, 'FaceColor', detail_colors);
            addVisual(body, 'Box', [140 140 10], trvec2tform([0, 0, -d_dh]), 'FaceColor', detail_colors);
        end
    elseif i == 2 % Upper pedestal
        addVisual(body, 'Sphere', 22*1.1, eye(4), 'FaceColor', joint_colors);
        if d_dh > 0
            rot_c = axang2tform([1 0 0 -alpha_dh]);
            t_vis = rot_c * trvec2tform([0, 0, -d_dh/2]);
            addVisual(body, 'Cylinder', [20, d_dh], t_vis, 'FaceColor', detail_colors);
        end
    elseif i >= 3 && i <= 5 % Articulated links (Shoulder, Elbow, Wrist)
        rad_map = [18, 14, 12];
        r_cyl = rad_map(i-2);
        addVisual(body, 'Sphere', r_cyl*1.1, eye(4), 'FaceColor', joint_colors);
        if r_dh > 0
            t_vis = trvec2tform([-r_dh/2, 0, 0]) * axang2tform([0 1 0 pi/2]);
            addVisual(body, 'Cylinder', [r_cyl, r_dh], t_vis, 'FaceColor', link_colors);
        end
    elseif i == 6 % Flange
        flange_radius = 16;
        t_vis = trvec2tform([0, 0, -d_dh/2]);
        addVisual(body, 'Cylinder', [flange_radius, d_dh], t_vis, 'FaceColor', flange_color);
    elseif i == 7 % Tool / Gripper
        ext_radius = 6;
        t_vis_ext = trvec2tform([0, 0, -d_dh/2]);
        addVisual(body, 'Cylinder', [ext_radius, d_dh], t_vis_ext, 'FaceColor', extension_color);
        
        baseDim = [18, 45, 12];
        addVisual(body, 'Box', baseDim, eye(4), 'FaceColor', gripper_base_color);
        fingerDim = [4, 6, 25];
        fingerOffsetY = baseDim(2)/2 - fingerDim(2)/2;
        fingerOffsetZ = baseDim(3)/2 + fingerDim(3)/2;
        addVisual(body, 'Box', fingerDim, trvec2tform([0,  fingerOffsetY, fingerOffsetZ]), 'FaceColor', gripper_fingers_color);
        addVisual(body, 'Box', fingerDim, trvec2tform([0, -fingerOffsetY, fingerOffsetZ]), 'FaceColor', gripper_fingers_color);
    end
    
    addBody(robot, body, parentName);
    parentName = bodyNames{i};
end

% =========================================================================
% 6. KINEMATIC CONFIGURATION SETUP
% =========================================================================
config = homeConfiguration(robot);

% Map active revolute joint states:
% Joint1 -> angles_rad(1) (q1)
% Joint2 -> angles_rad(2) (q2)
% Joint3 -> angles_rad(3) (q3)
% Joint4 -> angles_rad(4) (q4)
for k = 1:4
    config(k) = angles_rad(k);
end
if has_gripper && length(config) >= 5
    config(5) = angles_rad(5); % Gripper_Roll_Joint (q5)
end

% =========================================================================
% 7. FORWARD KINEMATICS & TARGET POSITION COMPUTATION
% =========================================================================
if strcmp(ctrl_viz.measurement_point, 'end_extension') && has_gripper
    targetBody = 'Extension_Link';
    point_name = "Tool Tip (TCP - Frame 7)";
else
    targetBody = 'Flange_Link';
    point_name = "Flange Face (Frame 6)";
end

tform_target = getTransform(robot, config, targetBody);

switch ctrl_viz.reference_frame
    case 'base'
        final_pos = tform_target(1:3, 4);
        ref_name = "Global Base Frame {0}";
        
    case 'frame1'
        tform_f1 = getTransform(robot, config, 'Base_Offset_Link');
        final_pos = (tform_f1 \ tform_target) * [0; 0; 0; 1];
        final_pos = final_pos(1:3);
        ref_name = "Displaced Base Frame {1}";
        
    case 'shoulder'
        tform_f2 = getTransform(robot, config, 'Base_Link');
        final_pos = (tform_f2 \ tform_target) * [0; 0; 0; 1];
        final_pos = final_pos(1:3);
        ref_name = "Shoulder Frame {2}";
        
    case 'offset_base'
        tform_ref = trvec2tform([0, 0, L0]);
        final_pos = (tform_ref \ tform_target) * [0; 0; 0; 1];
        final_pos = final_pos(1:3);
        ref_name = sprintf("Offset Plane Z = %d mm", L0);
        
    otherwise
        final_pos = [0;0;0]; 
        ref_name = "Undefined Frame";
end

% =========================================================================
% 8. HEADLESS EXPORT EXECUTION
% =========================================================================
if ctrl_viz.headless_export
    dobot_pos = [final_pos(1), final_pos(2), final_pos(3)];
    excel_formatted_vector = strrep(sprintf('%0.3f\t%0.3f\t%0.3f', dobot_pos), '.', ',');
    
    fprintf('\n=================== KINEMATIC EXPORT RESULT ===================\n');
    fprintf('Target Point : %s\n', point_name);
    fprintf('Ref Frame    : %s\n', ref_name);
    fprintf('Excel Vector : %s\n', excel_formatted_vector);
    fprintf('Status       : Copied to Clipboard! Ready to paste in Excel.\n');
    fprintf('===============================================================\n\n');
    
    clipboard('copy', excel_formatted_vector);
    return;
end

% =========================================================================
% 9. SIMULATION SCENE GENERATION
% =========================================================================
fig = figure('Name', 'Dobot Magician 7-Frame Kinematic Simulation', 'Color', 'w');
hold on; grid on;

show(robot, config, 'Visuals', 'on', 'Collision', 'off', 'Frames', 'off');
view(45, 30);
axis equal;
camlight('headlight'); lighting gouraud; material dull;

% Render Z=L0 reference plane if offset_base is selected
if strcmp(ctrl_viz.reference_frame, 'offset_base')
    w = 400; X = [-w w w -w]/2; Y = [-w -w w w]/2; Z = [L0 L0 L0 L0];
    patch(X, Y, Z, 'm', 'FaceAlpha', 0.1, 'EdgeColor', 'm', 'LineStyle', '--');
    for c = 1:4, plot3([X(c) X(c)], [Y(c) Y(c)], [0 L0], 'k:', 'LineWidth', 0.5); end
    plot3(0, 0, L0, 'mo', 'MarkerFaceColor', 'm');
end

% --- RENDER LOCAL AXES AND KINEMATIC FRAMES ---
scale = 45; txt_offset = 1.2; lw = 1.5; fs_axis = 9; fs_label = 7;
allBodies = [{'base'}, bodyNames];
frameLabels = {'{0} BASE', '{1} BASE_OFFSET', '{2} SHOULDER', '{3} ELBOW', '{4} WRIST', '{5} FLANGE_IN', '{6} FLANGE_OUT', '{7} TCP'};

for i = 1:length(allBodies)
    if i <= length(ctrl_viz.active_frames) && ctrl_viz.active_frames(i) == 0
        continue;
    end
    
    bName = allBodies{i};
    try
        tform = getTransform(robot, config, bName);
    catch
        continue;
    end
    
    P = tform(1:3, 4); 
    R = tform(1:3, 1:3);
    uX = R(:,1)*scale; uY = R(:,2)*scale; uZ = R(:,3)*scale;
    
    frame_axes = ctrl_viz.active_axes(min(i, size(ctrl_viz.active_axes, 1)), :);
    frame_text = ctrl_viz.active_text(min(i, size(ctrl_viz.active_text, 1)), :);
    
    % Draw frame orientation arrows
    if frame_axes(1), quiver3(P(1),P(2),P(3), uX(1),uX(2),uX(3), 'r', 'LineWidth', lw, 'AutoScale', 'off'); end
    if frame_axes(2), quiver3(P(1),P(2),P(3), uY(1),uY(2),uY(3), 'g', 'LineWidth', lw, 'AutoScale', 'off'); end
    if frame_axes(3), quiver3(P(1),P(2),P(3), uZ(1),uZ(2),uZ(3), 'b', 'LineWidth', lw, 'AutoScale', 'off'); end
    
    % Draw axis identifiers
    if frame_text(1), text(P(1)+uX(1)*txt_offset, P(2)+uX(2)*txt_offset, P(3)+uX(3)*txt_offset, 'X', 'Color','r','FontSize',fs_axis,'FontWeight','bold'); end
    if frame_text(2), text(P(1)+uY(1)*txt_offset, P(2)+uY(2)*txt_offset, P(3)+uY(3)*txt_offset, 'Y', 'Color','g','FontSize',fs_axis,'FontWeight','bold'); end
    if frame_text(3), text(P(1)+uZ(1)*txt_offset, P(2)+uZ(2)*txt_offset, P(3)+uZ(3)*txt_offset, 'Z', 'Color','b','FontSize',fs_axis,'FontWeight','bold'); end
    
    if ctrl_viz.show_body_names && i <= length(frameLabels)
        text(P(1), P(2), P(3)-12, frameLabels{i}, 'Interpreter','none','FontSize',fs_label,...
            'Color','k','HorizontalAlignment','center','BackgroundColor',[1 1 1 0.7]);
    end
end

% --- DISPLAY TARGET METRICS ---
if ctrl_viz.display_text_position && ~ctrl_viz.paper_mode
    P_draw = tform_target(1:3, 4);
    txt_info = sprintf('Target Point: %s\nReference: %s\n----------------------\nPx: %.2f mm\nPy: %.2f mm\nPz: %.2f mm', ...
        point_name, ref_name, final_pos(1), final_pos(2), final_pos(3));
    
    text(P_draw(1), P_draw(2), P_draw(3) + 160, txt_info, ...
        'FontSize', 9, 'Color', 'k', 'FontWeight', 'bold', ...
        'BackgroundColor', [1 1 0.85], 'EdgeColor', 'k', 'Margin', 5);
    
    plot3([P_draw(1), P_draw(1)], [P_draw(2), P_draw(2)], [P_draw(3), P_draw(3)+80], 'k:', 'LineWidth', 1);
    plot3(P_draw(1), P_draw(2), P_draw(3), 'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'y');
end

if ~ctrl_viz.paper_mode
    title(sprintf('Dobot Kinematics - Measured at %s', char(point_name)));
end

xlabel('X Axis (mm)'); ylabel('Y Axis (mm)'); zlabel('Z Axis (mm)');
xlim([-350 450]); ylim([-350 450]); zlim([0 550]);
hold off;