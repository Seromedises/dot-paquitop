close all
clear
clc

%% inputs
base_link5 = [0.00162790062116 -0.999986697429 -0.00489437479274 0.287550172603 
-0.999998080483 -0.00163321699142 0.00108241997908 -0.0950272847574 
-0.00109039915618 0.00489260332574 -0.999987436652 0.276491180265 
0.0 0.0 0.0 1.0    ];

base_link5 = [0.000105380375945 -0.999995543364 -0.00298364675211 0.287182060177 
-0.999999606726 -0.000108007243339 0.000880274277228 -0.0913292161286 
-0.000880592609626 0.00298355281508 -0.999995161473 0.27396423245 
0.0 0.0 0.0 1.0  ];
     
link5_camera = readmatrix('Calib_Matrix.txt');
 
 camera_object = [-0.99997469  0.11220616  0.00610771  0.12355725;...
     -0.00168546  0.94931707  0.29481157  0.07884091;...
     -0.00691178  0.29361009 -0.95553589  0.32331252;...
     0.          0.          0.          1.        ];
 base_camera = base_link5*link5_camera;
 
 %% traslation and rotation rest
 trasl_base_link5 = base_link5(1:3,4)';
 trasl_base_camera = base_camera(1:3,4)';
 
 quat_base_link5 = rotm2quat(base_link5(1:3,1:3));
 quat_base_camera = rotm2quat(base_camera(1:3,1:3));
 
 plot_trasl = [trasl_base_link5;trasl_base_camera;[0 0 0]];
 plot_rot = [quat_base_link5;quat_base_camera;rotm2quat([1 0 0; 0 1 0; 0 0 1])];
 
 figure()
 bt5 = plotTransforms(plot_trasl,plot_rot,'FrameSize',.1);
 axis equal
 grid on
 xlabel("x [m]")
 ylabel("y [m]")
 zlabel("z [m]")
 
 %% Measuring Aruko markers
 pose_aruko = [-0.0821113071237 -0.975507630393 -0.204413589175 0.092912353511; 
-0.950641715134 0.0142269782493 0.309924252149 -0.00328215425092; 
-0.299229114042 0.219505025308 -0.928526813016 0.454000021564;
0.0 0.0 0.0 1.0 ]; 
 
 trasl_camera_posearuko = pose_aruko(1:3,4)';
 quat_camera_posearuko = rotm2quat(pose_aruko(1:3,1:3));
 
 figure()
 bt5 = plotTransforms([trasl_camera_posearuko;[0 0 0]],[quat_camera_posearuko;[1 0 0 0]],'FrameSize',.1);
 axis equal
 grid on
 xlabel("x [m]")
 ylabel("y [m]")
 zlabel("z [m]")
 
 %% Calcolo simbolico della posa
 
%  clear
%  close all
%  clc
%  
%  syms alpha beta gamma real
%  syms x y z real
%  
%  R05 = Rotz(gamma)*Roty(beta)*Rotx(alpha);
%  R05 = simplify(R05);
%  T05 = [R05 [x y z]'; 0 0 0 1]
 
 function R = Rotx(th)
    R = [1  0       0;
         0  cos(th) -sin(th);
         0  sin(th) cos(th)];
 end
 
function R = Roty(th)
    R = [cos(th)    0       sin(th);
         0          1       0;
         -sin(th)   0       cos(th)];
  end
 
function R = Rotz(th)
    R = [cos(th) -sin(th)       0;
         sin(th) cos(th)        0;
         0       0              1];
 end