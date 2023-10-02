% A nonlinear planar engagement simulation of target and pursuer kinematics with proportional navigation. This is a 
%main driver script that can be modified to explore various engagements. 
% In this project, a head on collision with pursuer heading error is simulated.

%state is y = [beta, RTx, RTz, RMx, RMZ, VTx, VTz, VMx, VMz]

% beta - target flight path angle
% RTx - horizontal position of target wrt inertial cs
% RTZ - vertical position of target wrt inertial cs
% RMx - horizontal position of missile wrt inertial cs
% RMz - vertical position of missile wrt inertial cs
% VTx - horizontal velocity of target wrt inertial cs 
% VTZ - vertical velocity of target wrt inertial cs
% VMx - horizontal velocity of missile wrt inertial cs 
% VMz - vertical velocity of missile wrt inertial cs

% User defined inputs:
%--------------------------------------------------------------------------------------------------------------
% plot settings - various selections with 'on' or 'off' values
% viz - animations 'on' or 'off'
% PN type - 'Pure' or 'True' proportional navigation
% aT - Target acceleration [ft/sec^2] (constant throughout sim)
% Np - Navigation gain
% tf- Final time of simulation [s]
% h - Time step size of integration 
% beta_rad - Initial target flight path angle [rad]
% RTx - Initial horizontal position of target wrt inertial cs [ft]
% Rtz - Initial vertical position of target vit inertial cs [ft]
% RMx - Initial horizontal position of missile wrt inertial cs [ft]
% Rmz - Initial vertical position of missile wet inertial cs [ft]
% VT - Target velocity magnitude [ft/s)
% VM - Initial missile velocity magnitude (ft/s)
%--------------------------------------------------------------------------------------------------------------
clc
clear all
close all

% User Defined Inputs (Specify or Compute Engagement Parameters)
%--------------------------------------------------------------------------------------------------------------
viz = 'on';

% Select which animations or plots will be shown
engagement_plots = 'on';
full_scale_engagement_viz = 'off';
zoomed_engagement_viz = 'off';
late_engagement_viz = 'off';
later_engagement_viz = 'off';
control_effort_plots = 'off';



vid_file='off';
% ProNav Type I
PN_type = 'Pure';
% PN_type ='True':

%ProNav & Engagement Parameters
aT = 0;
HE_rad = -20*pi/180;
Np = 3;
tf = 20.18;
h = 1e-2;

% Engagement Initial Conditions
beta_rad = 0;
RTx = 40000;
RTz = 10000;
RMx = 0;
RMz = 10000;
VM = 3000;
VT = 1000;

% Initial calculations
%--------------------------------------------------------------------------------------------------------------

%resolve target velocity components in inertial cs
VTx = -VT*cos(beta_rad);
VTz =  VT*sin(beta_rad);

% relative positions and velocities 
RTMx = RTx-RMx;
RTMz = RTz-RMz;

% range
RTM = sqrt(RTMx^2 + RTMz^2);

%line of sight angle
lambda = atan2(RTMz, RTMx);

%missile lead angle
L = asin(VT*sin(beta_rad+lambda)/VM);

% missile velocity components
VMx = VM*cos(lambda + L + HE_rad);
VMz = VM*sin(lambda + L + HE_rad);

% Initial condition vector
%-------------------------------------------------------------------------------------------------------------

% IC vector
y0 =  [beta_rad, RTx, RTz, RMx, RMz, VTx, VTz, VMx, VMz]';

%Simulation- integrate nonlinear 2-D engagement with RK4
%-------------------------------------------------------------------------------------------------------------

%Discretized time line
t0 = 0;
y = y0;
%yout=y;
t = t0:h:(tf-h) ;
nt = length(t);

% Preallocate solution matrix for speed
yout = zeros(length(y0),nt);

%Assign initial condition
yout(:,1) = y0;

% Integrate with constant time step, h [s] 
%y = y0;
for j = 1:nt-1
     s1 = nlinpronav_sim(t(j), y, HE_rad, Np, aT, VM, PN_type) ;
     s2 = nlinpronav_sim(t(j)+h/2, y+h*s1/2, HE_rad, Np, aT, VM, PN_type);
     s3 = nlinpronav_sim(t(j)+h/2, y+h*s2/2, HE_rad, Np, aT, VM, PN_type);
     s4 = nlinpronav_sim(t(j)+h,   y+h*s3,   HE_rad, Np, aT, VM, PN_type);
     y = y+(h*(s1+(2*s2)+(2*s3)+s4)/6)
     yout(:,j)=y;
end
y = yout';

%Post-process simulation results
% Pointers to states
sel_beta=1; 
sel_RTx = 2;
sel_RTz = 3;
sel_RMx= 4;
sel_RMz=5;
sel_VTx = 6;
sel_VTz=7;
sel_VMx = 8;
sel_VMz = 9;
% relative positions and velocities 
RTMx = y(:,sel_RTx)-y(:,sel_RMx);
RTMz = y(:,sel_RTz)-y(:,sel_RMz);
VTM1 = y(:,sel_VTx)-y(:,sel_VMx);
VTM2 = y(:, sel_VTz)-y(:,sel_VMz);

% range
RTM = sqrt(RTMx.^2 + RTMz.^2);

%line of sight angle and time derivative
lambda     = atan2(RTMz,RTMx);
lambda_dot = (RTMx.*VTM2-RTMz.*VTM1)./RTM.^2;

% closing velocity
VC = -(RTMx.*VTM1+ RTMz.*VTM2)./RTM;

% Compute acc commands
if strcmp(PN_type, 'True')
   aM = Np*VC.*lambda_dot;
elseif strcmp(PN_type,'Pure')
   aM=Np*VM.*lambda_dot;
else
   disp('Error: ProNav type not recognized.')
   return
end

% Visualize Results
%=====================================================
%Number of time steps per plot time step
dt_index = .1/h;
% Number of raw time steps
nt = length(t);

%Time index of tf (time to intercept)
miss_index = find (min(abs(RTM))==abs(RTM));
%Basic engagement plots
if strcmp(engagement_plots, 'on')
     figure (1)
     plot(y(1:miss_index, sel_RTx),y(1:miss_index, sel_RTz),'r--' ,'linewidth', 2); 
     hold on 
     plot(y(1:miss_index, sel_RMx), y(1:miss_index, sel_RMz),'linewidth', 2); 
     hold on
     plot(y(1,sel_RMx),y(1,sel_RMz),'ob','linewidth',2);
     plot(y(1,sel_RTx),y(1,sel_RTz),'or','linewidth',2);
     xlabel('Downrange [ft]', 'fontsize', 16);
     ylabel('Altitude [ft]', 'fontsize', 16);
     set(gca, 'fontsize', 16);
     set(gcf, 'color', 'w');
     grid on
    
     figure (2)
     plot(t(1:miss_index),aM(1:miss_index)./32.2, 'linewidth', 2); 
     hold on
     xlabel('Time[s]', 'fontsize', 16);
     ylabel('Acceleration [G]', 'fontsize', 16); 
     set(gca, 'fontsize', 16, 'ylim', [-2 20]); 
     set(gcf, 'color', 'w'); 
     grid on

     figure (3)
     plot(t, VC, 'linewidth', 2);
     xlabel('Time [sec]', 'fontsize', 16);
     ylabel('V_C [ft/s]', 'fontsize', 16);
     set(gca,'fontsize', 16, 'xlim', [0 11], 'ylim', [3700 4850]);
     set(gcf, 'color', 'w');
     grid on

     figure (4)
     plot(t,lambda_dot*180/pi,'linewidth',2);
     xlabel('Time[sec]', 'fontsize', 16);
     ylabel('LOS Rate [deg/s]', 'fontsize', 16);
     set(gca, 'fontsize', 16, 'xlim', [0 11], 'ylim', [-5 5]);
     set(gcf, 'color', 'w');
     grid on

     figure (5)
     semilogy(t, RTM, 'b', 'linewidth', 2);
     xlabel('Time [sec]', 'fontsize', 16);
     ylabel('Range [ft]', 'fontsize', 16);
     set(gcf, 'color', 'w');
     grid on

elseif ~(strcmp(engagement_plots,'on')|| strcmp(engagement_plots, 'off'))
        disp('Acceptable values for Engagement Plots are ''on'' or ''off''');
end

% Plot engagement at full scale
%----------------------------------------------------------------------
if strcmp(full_scale_engagement_viz, 'on')

% Engagement full scale
    k = 1; 
for ii = 1 : dt_index : (miss_index + dt_index)
    if ii == 1
       figure (6)
       plot(y(1,sel_RMx), y(1,sel_RMz), 'ob', 'linewidth', 1,'markerfacecolor', 'b');
       hold on
       plot (y(1,sel_RTx), y(1,sel_RTz), 'or', 'linewidth', 1,'markerfacecolor', 'r');
       title([PN_type 'ProNav, -20 Deg HE, N = ' num2str(Np)],'fontsize',16);
       xlabel('Downrange [ft]', 'fontsize', 16);
       ylabel('Altitude [ft]', 'fontsize', 16);
       set(gca,'fontsize', 16, 'xlim',[0 40000], 'ylim', [6000 12000],'position', [0.1220  0.1381 0.8388  0.7119]);
       set(gcf,'color','w','position', [10, 80, 1212, 298]);
       grid on
       axis equal
    end
    if ii >= 2
     figure(6)
     plot([y(ii,sel_RTx), y(ii-dt_index, sel_RTx)], [y(ii,sel_RTz), y(ii-dt_index, sel_RTz)], 'r-' ,'linewidth', 2);
     plot([y(ii,sel_RMx), y(ii-dt_index, sel_RMx)], [y(ii,sel_RMz), y(ii-dt_index, sel_RMz)], 'b-','linewidth', 2);
     set(gca, 'fontsize', 16, 'xlim', [0 40000],'ylim', [6000 12000],'position', [0.1220, 0.1381, 0.8388,0.7119]);
    end
    pause(0.1)
    if strcmp(vid_file, 'on')
        F1(k)=getframe(gcf);
        k=k+1
    end
end 

if strcmp(vid_file, 'on')
   movie2avi(F1, ['Full_Engagement_20deg_HE' PN_type],'fps',30);
end


elseif ~(strcmp(full_scale_engagement_viz, 'on')||strcmp(full_scale_engagement_viz, 'off'))
         disp(['Acceptable values for ''full_scale_engagement_viz'' are ''''on'' or ''off'''])

end
k=1
for ii = 1:dt_index :(miss_index + dt_index)
            %Plot pursuer velocity vector 
            figure(7)
            VMx = y(ii, sel_VMx);
            VMz = y(ii, sel_VMz);
            ph1 = quiver(y(ii,sel_RMx),y(ii,sel_RMz), VMx, VMz, 'b', 'linewidth', 2);
            hold on 
            grid on
            %Plot position trace as time steps forward
            if ii >= 2
                     plot([y(ii, sel_RTx), y(ii-dt_index, sel_RTx)], [y(ii, sel_RTz), y(ii-dt_index, sel_RTz)], 'r-', 'linewidth', 2);
                     plot([y(ii, sel_RMx), y(ii-dt_index, sel_RMx)], [y(ii, sel_RMz), y(ii-dt_index, sel_RMz)], 'b-','linewidth', 2);
            end
            %Determine pursuer acceleration vector
            if strcmp(PN_type, 'Pure')
                  Heading_pursuer =atan2(VMz,VMx);
                  aMx=-aM(ii)*sin(Heading_pursuer)*8;
                  aMz = aM(ii)*cos(Heading_pursuer)*8;
            elseif strcmp(PN_type, 'True')
                  %Plot range vector
                  ph2= plot([y(ii,sel_RMx), y(ii,sel_RMx)+RTMx(ii)], [y(ii, sel_RMz), y(ii, sel_RMz) + RTMz(ii)],'k--', 'linewidth', 2);
                  aMx=-aM(ii)*sin(lambda(ii))*8;
                  aMz = aM(ii)*cos(lambda(ii))*8;
            else
                  disp('Error, PN type must be string True or Pure');
            end
            % Plot pursuer acceleration vector
            ph3 =quiver(y(ii, sel_RMx), y(ii,sel_RMz), aMx, aMz,'k', 'linewidth', 2);
            %Plot pursuer point
            ph4= plot(y(ii,sel_RMx),y(ii, sel_RMz),'b.', 'linewidth', 2, 'markersize', 20);
            %Plot formatting
           % set(gca,'xlim',[y(ii,sel_RMx)-4e3, y(ii, sel_RMx) +4e3], 'ylim', [y(ii, sel_RMz)-4e3, y(ii, sel_RMz)+4e3]);
           % set (gcf, 'color', 'w', 'position', [360 278 560 420]);
            title(['Engagement Visualization-' PN_type ' ProNav'], 'fontsize',14);
            pause(0.1);

            
            if strcmp(vid_file, 'on')
                  % Store video frames
                  F2(k)= getframe(gcf);
                  k = k+1;
            end

            %Delete plots for next frame
            if strcmp(PN_type, 'True') 
                delete (ph2)
            end
            delete(ph1)
            delete(ph3)
            delete(ph4)
end

%if strcmp(vid_file, 'on')
 %     movie2avi(F2,['aM_and_VM' PN_type], 'fps', 30);

%end
%elseif ~(strcmp(full_scale_engagement_viz,'on')||~(strcmp(full_scale_engagement_viz,'off'))
 %    disp(['acceptable values for ''full_scale_engagement_viz''are 'on'or 'off'])    
%end
