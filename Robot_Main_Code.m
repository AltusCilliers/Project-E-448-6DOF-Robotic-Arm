clc % Clear command window
clear % Removes variables from workspace
startup_rvc % Starts robotic toolbox

s = serialport('COM5',115200);
save ('serial','s');
string = "SP";
serial_send(string,s);


%%DH Params in mm
%mm           a    alpha_angle d       theta
%dhparams = [38,   pi/2,	     67.81, 0;
%           300,     0,       0,       pi/2;
%           0,       pi/2,    0,       0;
%           0,   	-pi/2,	  200,     0;
%           0,       pi/2,	  0,       0;
%           0,       0,       0,    0]; %d6 is 0 I think, because it is wrist centre point
       
% cm          a    alpha_angle d       theta
dhparams = [0.038,  pi/2,	 0.06781, 0;
           0.3,     0,       0,       pi/2;
           0,       pi/2,    0,       0;
           0,   	-pi/2,	 0.2,     0;
           0,       pi/2,	 0,       0;
           0,       0,       0,       0]; %d6 is 0 I think, because it is wrist centre point

a = dhparams(:,1);
alpha_angle = dhparams(:,2);
d = dhparams(:,3);
theta = dhparams(:,4);

% dh values for each link
% L = Link([Theta d a alpha])

L(1) = Link([theta(1) d(1) a(1) pi/2]);
L(1).qlim = pi/180*[-170 170];
L(2) = Link([theta(2) d(2) a(2) 0]);
L(3) = Link([theta(3) d(3) 0    pi/2 ]);
L(4) = Link([theta(4) d(4) 0    -pi/2 ]);
L(5) = Link([theta(5) d(5) 0    pi/2]);
L(6) = Link([theta(6) d(6) 0    0]);


% makes robot SerialLink object 
robot = SerialLink(L);
robot.name = 'AltusBot';
robot.n

robot.isspherical()
robot.tool = SE3(0,0,0.09); %used to go to tool centre point eg. 90mm (0.09cm)

%robot.plot([0 pi/2 0 0 0 0])
q = [0 pi/2 0 0 0 0];
robot.teach(q)


