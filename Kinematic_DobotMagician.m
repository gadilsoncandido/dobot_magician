% =========================================================================
% 4-DOF DOBOT MAGICIAN KINEMATICS MODEL: CORRECTED PROPORTIONS & DIMENSIONS
% =========================================================================
% This script builds a high-fidelity kinematic model of the Dobot Magician 
% manipulator. It establishes a rigidBodyTree architecture relying on 
% rigorous Denavit-Hartenberg (DH) parameters, bridging the gap between 
% the theoretical calculations and the physical dimensions of the robot.
% =========================================================================
clc; clear; close all;

% =========================================================================
% 1. USER INPUTS: JOINT ANGLES & ATTACHMENTS
% =========================================================================
% Define the input angles for each joint (in degrees). 
% For the physical Dobot Magician, a four-bar linkage mechanism forces the 
% end-effector to remain parallel to the ground surface. However, when 
% modeling the robot as a serial open-chain in MATLAB, we mathematically 
% compensate for this mechanical coupling by linking the wrist pitch (theta4) 
% counter-rotations to the shoulder (theta2) and elbow (theta3) movements.
theta1 = 0;   % Base Rotation Pan (J1)
theta2 = 0;   % Shoulder Pitch (J2)
theta3 = 0; % Elbow Pitch (J3)
theta4 = 0;   % Wrist Pitch (J4) - Physically unactuated, mimics J2/J3 coupling
theta5 = 0;   % Gripper/Tool Roll (J5)

% Transform relative input angles into absolute kinematic states (radians)
% The mathematical offsets (e.g., -90, 90) account for the initial 
% resting configuration of the mechanical hardware relative to the 
% orthogonal Denavit-Hartenberg mathematical frames.
angles_degrees = [0+theta1, -90+theta2, 90-theta2+theta3, theta4-theta3, 0+theta5]; 
angles_rad = deg2rad(angles_degrees);

% --- END-EFFECTOR (GRIPPER & EXTENSION) CONFIGURATION ---
% Toggle the presence of an extended tool (suction cup or pneumatic gripper)
has_gripper = true; 

% =========================================================================
% 2. ROBOT KINEMATIC DIMENSIONS 
% =========================================================================
% The following dimensional constants constitute the physical lengths of 
% the manipulator links. Dimensional accuracy here is strictly required to 
% ensure that the Forward and Inverse Kinematic matrices map perfectly 
% to the real-world TCP (Tool Center Point) coordinates.
% All units are strictly in millimeters (mm).

% --- MAIN BODY DIMENSIONS ---
% L1 (Base Pedestal): 138mm. Defines the absolute distance from the physical 
% resting plane (the table) up to the cross-sectional axis of J2 (Shoulder).
% Replacing the older simplistic 85mm measurement with 138mm ensures collision 
% avoidance algorithms accurately account for the robot's lower chassis.
L1 = 138;   

% L2 (Rear Arm / Shoulder Link): 135mm. The pivotal sweeping arm.
L2 = 135;   

% L3 (Forearm / Elbow Link): 147mm. Along with L2, determines the 320mm 
% maximum theoretical workspace envelope radius.
L3 = 147;   

% L4 (Wrist Offset): 59.7mm. The linear distance from the final rotational 
% elbow joint to the flat mounting face of the structural body.
L4 = 59.7;  

% --- END-EFFECTOR (EoF) DIMENSIONS ---
% LEoF_Standard: 8mm. The thickness of the metallic structural flange 
% where external peripherals are mounted. Acts as a fixed Z-offset.
LEoF_Standard = 8;  

% L_Extension: 74mm. The physical length of the external tool (e.g., 
% the stem of the suction cup or the pneumatic gripper structure). 
% Modifies the final TCP, heavily altering singular Jacobians and tracking.
L_Extension = 74;   

% =========================================================================
% 3. VISUALIZATION AND REFERENCE FRAME PREFERENCES
% =========================================================================
ctrl_viz.display_text_position = true; 

% Measurement Target: 'end_extension' (Tool Tip) or 'start_extension' (Flange)
ctrl_viz.measurement_point = 'start_extension'; 

% Positional Reference Frame computation behavior:
% 'base' (Global origin 0,0,0)
% 'shoulder' (Relative to the J2 Joint axis)
% 'offset_base' (Simulates a table surface shifted on the Z-plane)
ctrl_viz.reference_frame = 'offset_base'; 

% Graphical control flags [Base, Shoulder, Elbow, Wrist, Flange, Extension]
ctrl_viz.active_frames = [1, 1, 1, 1, 1, 1, 1]; 
ctrl_viz.active_axes = [1, 1, 1];    % Render X, Y, Z unit vectors
ctrl_viz.active_text = [1, 1, 1];    % Render X, Y, Z character labels
ctrl_viz.show_body_names = true;     % Show floating titles over links

% =========================================================================
% 4. COLOR DEFINITIONS FOR RIGID BODY VISUALS
% =========================================================================
% Colors mimic the classic industrial aesthetic of the physical Dobot.
link_colors = [0.9 0.9 0.9];     % Light grey for main articulated arms
detail_colors = [0.2 0.2 0.2];   % Dark grey for the base foundation block
joint_colors = [0.2 0.2 0.2];    % Dark grey for the rotational spheres
gripper_base_color = [0.3 0.3 0.3];
gripper_fingers_color = [0.8 0.8 0.8];
extension_color = [0.1 0.1 0.1]; % Matte black for the pneumatic rods
flange_color = [0.6 0.6 0.6];    % Metallic grey tone

% =========================================================================
% 5. DH MATRIX CONSTRUCTION AND ROBOT ASSEMBLY
% =========================================================================
% Initialize the Rigid Body Tree. Setting DataFormat to 'column' is highly 
% recommended for computational efficiency when handling kinematic state 
% vectors during lagrangian dynamics or Jacobian iterations in MATLAB.
robot = rigidBodyTree('DataFormat', 'column'); 

% STANDARD DENAVIT-HARTENBERG (DH) PARAMETERS
% Format per row: [a (Link Length), alpha (Link Twist), d (Offset), theta (Angle)]
% Note: The 'theta' values populated in this array are placeholders. 
% For any joint designated as 'revolute', MATLAB's setFixedTransform method 
% dynamically ignores this column since theta depends on the live configuration.
dhParams = [
    0,      -pi/2, L1,   0;   % Joint 1 (Base Pan)
    L2,     0,     0,     0;   % Joint 2 (Shoulder Pitch)
    L3,     0,     0,     0;   % Joint 3 (Elbow Pitch)
    L4,     -pi/2, 0,     0;   % Joint 4 (Wrist Pitch)
];

bodyNames = {'Base_Link', 'Shoulder_Link', 'Elbow_Link', 'Wrist_Link'};
jointNames = {'Joint1', 'Joint2', 'Joint3', 'Joint4'};

% --- ADDING END-EFFECTOR MATRICES (STATIC LINKS) ---
% 5. Add Row 5: The 8mm connecting flange. Mathematically modeled as a 
% fixed translational d-offset along the Z-axis of the wrist frame.
dhParams(5,:) = [0, 0, LEoF_Standard, 0];
bodyNames{5} = 'Flange_Link';
jointNames{5} = 'Flange_Joint';

% 6. Add Row 6: The 74mm tool extension (Condition Dependent).
if has_gripper
    dhParams(6,:) = [0, 0, L_Extension, 0]; 
    bodyNames{6} = 'Extension_Link';
    jointNames{6} = 'Gripper_Roll_Joint';
end

% --- KINEMATIC CHAIN CREATION LOOP ---
parentName = 'base';
num_links = size(dhParams, 1);

for i = 1:num_links
    body = rigidBody(bodyNames{i});
    
    % The physical Dobot arms are revolute, but structural extensions 
    % like the terminal flange are mathematically treated as 'fixed' joints.
    if i == 5 
        jnt = rigidBodyJoint(jointNames{i}, 'fixed'); 
    else
        jnt = rigidBodyJoint(jointNames{i}, 'revolute');
    end
    
    % Attach the transformation matrix to the joint using the DH row
    setFixedTransform(jnt, dhParams(i,:), 'dh');
    body.Joint = jnt;
    
    % --- ATTACHING GRAPHICAL VISUALS (CORRECTED PROPORTIONS) ---
    % Extract DH elements to orient cylinders and geometric boxes accurately.
    % Theoretical DH frames often misalign with aesthetic 3D rendering origins,
    % requiring supplementary rotational matrices (t_vis) for alignment.
    L = dhParams(i, 1); D = dhParams(i, 3); alpha_dh = dhParams(i, 2);
    
    if i <= 4
        % MAIN ROBOT LINKS
        % Decreasing radius for a tapering aesthetic toward the wrist payload.
        if i==1, r=22; elseif i==2, r=18; elseif i==3, r=14; else, r=12; end
        
        % Render the joint pivot sphere
        addVisual(body, 'Sphere', r*1.1, eye(4), 'FaceColor', joint_colors);
        
        if L > 0 % Horizontal Links (Shoulder and Elbow driven by DH parameter a)
            % Transform visual to align the physical cylinder along the DH X-axis
            t_vis = trvec2tform([-L/2, 0, 0]) * axang2tform([0 1 0 pi/2]);
            addVisual(body, 'Cylinder', [r, L], t_vis, 'FaceColor', link_colors);
            
        elseif D > 0 % Vertical Links (Base and Wrist driven by offset parameter d)
            % Counter-rotate utilizing the alpha angle twist to plumb the solids
            rot_correction = axang2tform([1 0 0 -alpha_dh]);
            t_vis = rot_correction * trvec2tform([0, 0, -D/2]);
            
            if i == 1 % Base Pedestal Visual
                addVisual(body, 'Cylinder', [r, D], t_vis, 'FaceColor', detail_colors);
                addVisual(body, 'Box', [140 140 10], rot_correction * trvec2tform([0,0,-D]), 'FaceColor', detail_colors);
            else % Wrist Visual
                addVisual(body, 'Cylinder', [r, D], t_vis, 'FaceColor', link_colors);
            end
        end
        
    elseif i == 5
        % FLANGE VISUAL (8mm Fixed Link)
        flange_radius = 16; 
        t_vis = trvec2tform([0, 0, -D/2]); 
        addVisual(body, 'Cylinder', [flange_radius, D], t_vis, 'FaceColor', flange_color);
        
    elseif i == 6
        % TOOL EXTENSION & GRIPPER VISUAL (74mm Fixed Link)
        ext_radius = 6; 
        t_vis_ext = trvec2tform([0, 0, -D/2]); 
        addVisual(body, 'Cylinder', [ext_radius, D], t_vis_ext, 'FaceColor', extension_color);
        
        % Gripper Chassis Rectangular Construction
        baseDim = [18, 45, 12]; 
        addVisual(body, 'Box', baseDim, eye(4), 'FaceColor', gripper_base_color);

        % Gripper Fingers offset outwards from the main chassis
        fingerDim = [4, 6, 25]; 
        fingerOffsetY = baseDim(2)/2 - fingerDim(2)/2; 
        fingerOffsetZ = baseDim(3)/2 + fingerDim(3)/2; 
        addVisual(body, 'Box', fingerDim, trvec2tform([0, fingerOffsetY, fingerOffsetZ]), 'FaceColor', gripper_fingers_color);
        addVisual(body, 'Box', fingerDim, trvec2tform([0, -fingerOffsetY, fingerOffsetZ]), 'FaceColor', gripper_fingers_color);
    end
    
    addBody(robot, body, parentName);
    parentName = bodyNames{i};
end

% =========================================================================
% 6. KINEMATIC CONFIGURATION SETUP
% =========================================================================
% Pre-allocate the home configuration state vector, then inject user angles
config = homeConfiguration(robot);

for k=1:4
    config(k) = angles_rad(k);
end

if has_gripper && size(dhParams, 1) >= 6 && length(angles_rad) >= 5
    config(5) = angles_rad(5); 
end

% =========================================================================
% 7. SIMULATION SCENE GENERATION
% =========================================================================
figure('Name', 'Dobot Kinematic Simulation', 'Color', 'w');
hold on; grid on;

% Render the rigid body tree without displaying opaque collision meshes
show(robot, config, 'Visuals', 'on', 'Collision', 'off', 'Frames', 'off');
view(45, 30);
axis equal;
camlight('headlight'); lighting gouraud; material dull; 

% --- FORWARD KINEMATICS: DEFINE TARGET TCP ---
if has_gripper
    if strcmp(ctrl_viz.measurement_point, 'end_extension')
        targetBody = 'Extension_Link'; 
        point_name = "Gripper Tip (TCP)";
    else
        targetBody = 'Flange_Link'; 
        point_name = "Standard Connection (8mm Flange)";
    end
else
    targetBody = 'Flange_Link';
    point_name = "Flange Face (No Gripper)";
end

% Extract the absolute Homogeneous Transformation Matrix of the Target Frame
tform_target = getTransform(robot, config, targetBody);

% --- RELATIVE POSITION COMPUTATION ---
% Determine the position of the TCP relative to different coordinate references
if strcmp(ctrl_viz.reference_frame, 'base')
    % Absolute cartesian coordinates mapping to the world origin
    final_pos = tform_target(1:3, 4); 
    ref_name = "Global Base Origin";
    
elseif strcmp(ctrl_viz.reference_frame, 'shoulder')
    % Position metrics relative solely to the Shoulder (J2) Local Frame
    tform_ref = getTransform(robot, config, 'Shoulder_Link');
    final_pos = (inv(tform_ref) * tform_target) * [0;0;0;1]; 
    final_pos = final_pos(1:3);
    ref_name = "Shoulder Frame";
    
elseif strcmp(ctrl_viz.reference_frame, 'offset_base')
    % Simulated interaction relative to an artificially shifted horizontal plane
    z_offset = 130;
    tform_ref = trvec2tform([0, 0, z_offset]); 
    final_pos = (inv(tform_ref) * tform_target) * [0;0;0;1];
    final_pos = final_pos(1:3);
    ref_name = sprintf("Target Plane Z=%d mm", z_offset);
    
    % Render the virtual intersection plane visually
    w = 400; X=[-w w w -w]/2; Y=[-w -w w w]/2; Z=[z_offset z_offset z_offset z_offset];
    patch(X, Y, Z, 'm', 'FaceAlpha', 0.1, 'EdgeColor', 'm', 'LineStyle', '--');
    for c=1:4, plot3([X(c) X(c)], [Y(c) Y(c)], [0 z_offset], 'k:', 'LineWidth',0.5); end
    plot3(0,0,z_offset,'mo','MarkerFaceColor','m');
else
    final_pos = [0;0;0]; ref_name = "Matrix Error";
end

% --- RENDER LOCAL AXES AND KINEMATIC VECTORS ---
scale = 50; txt_offset = 1.2; lw = 1.5; fs_axis = 9; fs_label = 7;      
allBodies = [{'base'}, bodyNames]; 

for i = 1:length(allBodies)
    if i <= length(ctrl_viz.active_frames) && ctrl_viz.active_frames(i) == 0, continue; end
    
    bName = allBodies{i};
    try
        tform = getTransform(robot, config, bName);
    catch
        continue;
    end
    
    % Extract the translation vector (P) and orientation rotation matrix (R)
    P = tform(1:3, 4); R = tform(1:3, 1:3);
    uX = R(:,1)*scale; uY = R(:,2)*scale; uZ = R(:,3)*scale;
    
    if ctrl_viz.active_axes(1), quiver3(P(1),P(2),P(3), uX(1),uX(2),uX(3), 'r','LineWidth',lw,'AutoScale','off'); end
    if ctrl_viz.active_axes(2), quiver3(P(1),P(2),P(3), uY(1),uY(2),uY(3), 'g','LineWidth',lw,'AutoScale','off'); end
    if ctrl_viz.active_axes(3), quiver3(P(1),P(2),P(3), uZ(1),uZ(2),uZ(3), 'b','LineWidth',lw,'AutoScale','off'); end
    
    if ctrl_viz.active_text(1)
        text(P(1)+uX(1)*txt_offset, P(2)+uX(2)*txt_offset, P(3)+uX(3)*txt_offset, 'X', 'Color','r','FontSize',fs_axis,'FontWeight','bold'); 
    end
    if ctrl_viz.active_text(2)
        text(P(1)+uY(1)*txt_offset, P(2)+uY(2)*txt_offset, P(3)+uY(3)*txt_offset, 'Y', 'Color','g','FontSize',fs_axis,'FontWeight','bold'); 
    end
    if ctrl_viz.active_text(3)
        text(P(1)+uZ(1)*txt_offset, P(2)+uZ(2)*txt_offset, P(3)+uZ(3)*txt_offset, 'Z', 'Color','b','FontSize',fs_axis,'FontWeight','bold'); 
    end
    
    if ctrl_viz.show_body_names
        labelName = bName;
        if strcmp(bName, 'Extension_Link'), labelName = 'TCP'; end
        if strcmp(bName, 'Flange_Link'), labelName = 'FLANGE'; end
        if strcmp(bName, 'base'), labelName = 'BASE'; end
        text(P(1), P(2), P(3)-15, labelName, 'Interpreter','none','FontSize',fs_label,'Color','k','HorizontalAlignment','center','BackgroundColor',[1 1 1 0.6]);
    end
end

% --- DISPLAY QUANTITATIVE KINEMATIC RESULTS IN SCENE ---
if ctrl_viz.display_text_position
    P_draw = tform_target(1:3, 4); 
    txt_info = sprintf('TCP Measurement: %s\nRelative To: %s\n----------------------\nX: %.1f mm\nY: %.1f mm\nZ: %.1f mm', ...
        point_name, ref_name, final_pos(1), final_pos(2), final_pos(3));
    
    text(P_draw(1), P_draw(2), P_draw(3) + 220, txt_info, ...
        'FontSize', 10, 'Color', 'k', 'FontWeight', 'bold', ...
        'BackgroundColor', [1 1 0.8], 'EdgeColor', 'k', 'Margin', 5);
    
    plot3([P_draw(1), P_draw(1)], [P_draw(2), P_draw(2)], [P_draw(3), P_draw(3)+120], 'k:', 'LineWidth', 1);
    plot3(P_draw(1), P_draw(2), P_draw(3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'y');
end

title(['Dobot Simulation - Target: ' char(point_name)]);
xlabel('X Axis (mm)'); ylabel('Y Axis (mm)'); zlabel('Z Axis (mm)');
xlim([-350 450]); ylim([-350 450]); zlim([0 550]);

hold off;