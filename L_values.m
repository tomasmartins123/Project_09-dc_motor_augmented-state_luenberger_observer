clear
clc

% -------------------------------------------------------------------------
% Desired observer poles
% (must be inside the unit circle)
% -------------------------------------------------------------------------

observer_poles = [0.9 0.95];

%% ========================================================================
%% LEFT MOTOR
%% ========================================================================

% Identified discrete model
A_L = 0.810;
B_L = 0.255;

% Augmented system
A_aug_L = [
    A_L  B_L
    0    1
];

C_aug = [1 0];

% Observer gain
L_L = place(A_aug_L', C_aug', observer_poles)';

L1_L = L_L(1)
L2_L = L_L(2)

%% ========================================================================
%% RIGHT MOTOR
%% ========================================================================

% Identified discrete model
A_R = 0.833;
B_R = 0.225;

% Augmented system
A_aug_R = [
    A_R  B_R
    0    1
];

% Observer gain
L_R = place(A_aug_R', C_aug', observer_poles)';

L1_R = L_R(1)
L2_R = L_R(2)