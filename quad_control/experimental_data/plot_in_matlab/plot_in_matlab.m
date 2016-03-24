%% Useful operations

% restoredefaultpath; matlabrc
% close all
% clear all
clc

%% Include functions
addpath(genpath('./functions'))

%%
disp('testing')
% disp(filename)
format long
% M          = dlmread('_1458747408_test_plot_1.txt');
M          = dlmread(file_name);
time       = M(:,1);
state      = M(:,3:11);
stated     = M(:,12:20);
input_quad = M(:,21:23);

%% set some definitions
set(0,'DefaultTextInterpreter','latex',...
      'DefaultLineLineWidth',1,...
      'defaulttextfontsize',12,...
      'defaultaxesfontsize',12);

%% Breaking data into pieces 

clc
close all

% remove offset from time instants (so it starts at zero)
time = time - time(1);
tEND = time(end);

% position and velocity of quad
pQ  = state(:,1:3);
vQ  = state(:,4:6);
% quarotor euler angles
eeQ = state(:,7:9)*pi/180;

% desired position, velocity and acceleration of quad
pd = stated(:,1:3);
vd = stated(:,4:6);
% ad = stated(:,7:9);


%% Plot trajectory for all time

close all
plot_quad_trajectory(time,pQ,pd,eeQ)

%% Plot trajectory in intervals of Delta_Time length

close all

Delta_Time = 5;
for i=1:floor(tEND/Delta_Time)
    aux = time<=i*Delta_Time & time>=(i-1)*Delta_Time;
    figure()
    plot_quad_trajectory(time(aux),pQ(aux,:),pd(aux,:),eeQ(aux,:))
end


%% Plot position/velocity all time

close all

figure()
plot_compare(time,pQ,pd,'p')
figure()
plot_compare(time,vQ,vd,'v')

%% Plot position/velocity in intervals of Delta_Time length

close all

Delta_Time = 5;
for i=1:floor(tEND/Delta_Time)
    aux = time<=i*Delta_Time & time>=(i-1)*Delta_Time;
    figure()
    plot_compare(time(aux),pQ(aux,:),pd(aux,:),'p')
    figure()
    plot_compare(time(aux),vQ(aux,:),vd(aux,:),'v')    
end

%% Plot position/velocity tracking error for all time

close all

figure()
plot_error(time,pQ,pd,'p')
figure()
plot_error(time,vQ,vd,'v')

%% Plot position tracking error in intervals of Delta_Time length

close all

Delta_Time = 5;
for i=1:floor(tEND/Delta_Time)
    aux = time<=i*Delta_Time & time>=(i-1)*Delta_Time;
    figure()
    plot_error(time(aux),pQ(aux,:),pd(aux,:),'p')
    figure()
    plot_error(time(aux),vQ(aux,:),vd(aux,:),'v')    
end


%% Plot euler angles all time

% quarotor euler angles
% eeQ = state(:,7:9)*pi/180;

eeQd = [];
ades = stated(:,7:9);
gravity = 9.81;

for i= 1:length(ades)
    psi   = eeQ(i,3);
    ades_ = ades(i,:);
    n     = (gravity*[0;0;1] + ades_')/norm(gravity*[0;0;1] + ades_'); 
    n     = rot_matrix([0;0;-psi])*n;
    
    phid   = asin(max(-1,min(1,-n(2))));
    
    sin_thetad = max(-1,min(1,n(1)/cos(phid)));
    cos_thetad = max(-1,min(1,n(3)/cos(phid)));
    thetad     = atan2(sin_thetad,cos_thetad);
    
    psid  = 0; 
    
    eeQd_ = [phid;thetad;psid];
    

    eeQd = [eeQd; eeQd_'];  
end

close all

figure()
plot_compare_angles(time,eeQ,eeQd)

%% Plot position/velocity in intervals of Delta_Time length

close all

Delta_Time = 5;
for i=1:floor(tEND/Delta_Time)
    aux = time<=i*Delta_Time & time>=(i-1)*Delta_Time;
    figure()
    plot_compare_angles(time(aux),eeQ(aux,:),eeQd(aux,:))  
end


%% Plot inputs for all time

clc

eeQd = [];
ades = stated(:,7:9);
gravity = 9.81;

force3d_input = input_quad;
thrust = [];
phid   = [];
thetad = [];

for i= 1:length(force3d_input)
    
    force3d_input_ = force3d_input(i,:)';
    psi   = eeQ(i,3);
    
    n     = force3d_input_/norm(force3d_input_); 
    n     = rot_matrix([0;0;-psi])*n;
    
    thrust_ = norm(force3d_input_);
    thrust = [thrust;thrust_];
    
    phid_  = asin(max(-1,min(1,-n(2))));
    phid   = [phid;phid_];
    
    sin_thetad = max(-1,min(1,n(1)/cos(phid_)));
    cos_thetad = max(-1,min(1,n(3)/cos(phid_)));
    thetad_    = atan2(sin_thetad,cos_thetad);

    thetad = [thetad;thetad_];
  
end

close all 
plot_force3d_input(time,thrust,phid,thetad)


%% Plot inputs in intervals of Delta_Time length

close all

Delta_Time = 5;
for i=1:floor(tEND/Delta_Time)
    aux = time<=i*Delta_Time & time>=(i-1)*Delta_Time;
    figure()
    plot_force3d_input(time(aux),thrust(aux),phid(aux),thetad(aux))
end
