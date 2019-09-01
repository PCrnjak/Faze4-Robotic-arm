%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%
clc
clear

%% Starts robotic toolbox
startup_rvc
%%dh values for each link
%%L = Link([Theta d a alpha])
%%gripping_point = 0.057 ;
gripping_point = 0 ;
L(1) = Link([0 0.23682 0 pi/2]);
L(2) = Link([0  0 0.32 0]);
L(3) = Link([0 0 0.0735 pi/2 ]);
L(4) = Link([0 0.2507 0 -pi/2 ]);
L(5) = Link([0 0 0 pi/2]);
L(6) = Link([0 gripping_point 0 0]);

%% makes robot SerialLink object 
robot = SerialLink(L);
robot.name='faze4'
%% test if robot has spherical wrist
robot.isspherical()
%% zero position of robot defined in kinematic diagram
zero = [0 pi/2 0 0 0 0];

n = 30; 

%%Limits in deg
Joint1_lim = [-120 120];
Joint2_lim = [-16 176]; 
Joint3_lim = [-85 180];

Joint1_range = abs(Joint1_lim(1)) + abs(Joint1_lim(2));
Joint2_range = abs(Joint2_lim(1)) + abs(Joint2_lim(2));
Joint3_range = abs(Joint3_lim(1)) + abs(Joint3_lim(2));

Joint1_steps = Joint1_range / (n -1);
Joint2_steps = Joint2_range / (n -1);
Joint3_steps = Joint3_range / (n -1);

envelope_3d = 0;

%% create n segments of joint 1 range with joint1 steps
for i = 1 : n
    if i == 1
     num1(i) = Joint1_lim(1);
    else
       num1(i) = num1(i-1) + Joint1_steps;
    end
   
end

%% create n segments of joint 2 range with joint2 steps
for i = 1 : n
    if i == 1
     num2(i) = Joint2_lim(1);
    else
       num2(i) = num2(i-1) + Joint2_steps;
    end
   
end

%% create n segments of joint 3 range with joint3 steps
for i = 1 : n
    if i == 1
     num3(i) = Joint3_lim(1);
    else
       num3(i) = num3(i-1) + Joint3_steps;
    end
   
end

k=0;
%%axis([-1 1 -1 1 -1 1]);
hold on 
%% Creates dot map that represents Work envelope in x,y plane
tic
%%plot(0,0.2368,'-o','Color','b','MarkerSize',15)
%%robot.plot([0 pi/2 0 0 0 0]);
for i = 1 : n
    for j = 1 : n
        k = k + 1;
        Point_Cloud(k,:) = robot.fkine([0 deg2rad(num2(i)) deg2rad(num3(j)) 0 0 0])
        a=deg2rad(num2(i))
        b=deg2rad(num3(j))
        [R,t(:,k)] =  tr2rt(Point_Cloud(k,:));
        if envelope_3d == 0
        %%plot3(t(1,k),t(2,k),t(3,k),'.r')
        plot(0,0.2368,'-o','Color','b','MarkerSize',15) %% plot dot that represents joint 2
        plot(t(1,k),t(3,k),'.r')
        %%robot.plot([0 a b 0 0 0]);
        end
                 
    end
end
toc

hold off

%% Creates dot map that represents Work envelope in x,y,z plane

if envelope_3d == 1
axis([-1 1 -1 1 -1 1]);   
hold on
robot.plot([0 pi/2 0 0 0 0]); 
 
t_ = t';
k_ = k

for i = 1 : (n)
    for j = 1 : k_
    k = k + 1;
    T=rpy2r(0,0,num1(i),'deg');
    t(:,k) = (t_(j,:) * T)';
    end
    
end


for i = 1 : k 
    plot3(t(1,i),t(2,i),t(3,i),'.r')  
end


end

   