
clear all
close all
global start goal;
global nbStanja;
global TauN;
global nbTau;
global TauAttract TauReject;
global Tori;
global oriVariability;
global posW;
global nbIter;


%% DEFINING THE INPUTS

% task_p_learning_igra.m - the file with the learning algorithm
% cost_trajektorija_igra.m - the cost function for trajectory optimization

% folders demo1, demo2, demo3... contain the input data for the algorithm
% script(1)(2)(3).txt - contain the demonstrated trajectories by kinesthetic
%                       teaching
% TauN(1)(2)(3).txt - contain the object positions and orientations before
%                     kinesthetic teaching (initial configuration)
% scriptX.txt - contain the initial position of the robot for a new
%               situation
% TauNX.txt - contains the initial position of the objects for a new
%             situation


nbStanja=15;     % number of samples for trajectory

%nbDemo          % number of demonstrations available - script(1)(2)(3).txt
                 % number of objects in the robot workspace - TauN(1)(2)(3).txt
           
demo='demo5'      %which demo to run

%% demo1 - 2 demonstrations, 2 objects

if demo == 'demo1'
    
    nbDemo=2
    demonstracije = 'demo1/';
    situacija = 'konfig1/';
    
    numIter = 3000 
    iniSTD = 4

    %supplementary points

    %object 1
    DO1x = false
    DO1y = false

    %object 2
    DO2x = false
    DO2y = false

    %object 3
    DO3x = false
    DO3_x = false
    DO3y = false
    DO3_y = false
    
end

%% demo2 - 3 demonstrations, 3 objects


if demo == 'demo2'
    
    nbDemo=3
    demonstracije = 'demo2/';
    situacija = 'konfig1/';
    
    numIter = 10000 
    iniSTD = 4

    %supplementary points

    %object 1
    DO1x = true
    DO1y = false

    %object 2
    DO2x = true
    DO2y = false

    %object 3
    DO3x = true
    DO3_x = true
    DO3y = true
    DO3_y = true
    
end

%% demo3 - 3 demonstrations, 3 objects, obstacle avoidance

if demo == 'demo3'
    nbDemo=3
    demonstracije = 'demo3/';
    situacija = 'konfig1/';
    
    numIter = 15000 
    iniSTD = 5

    %supplementary points

    %object 1
    DO1x = false
    DO1y = false

    %object 2
    DO2x = false
    DO2y = false

    %object 3
    DO3x = false
    DO3_x = false
    DO3y = false
    DO3_y = false
end

%% demo4 - 3 demonstrations, 3 objects, position constraints 

if demo == 'demo4'
    nbDemo=3
    demonstracije = 'demo4/';
    situacija = 'konfig1/';
    
    numIter = 14000 
    iniSTD = 100

    %supplementary points

    %object 1
    DO1x = false
    DO1y = false

    %object 2
    DO2x = false
    DO2y = false

    %object 3
    DO3x = false
    DO3_x = false
    DO3y = false
    DO3_y = false
end

%% demo5 - sweeping task

if demo == 'demo5'
    nbDemo=3
    demonstracije = 'demo5/';
    situacija = 'konfig1/';
    
    numIter = 14000 
    iniSTD = 10

    %supplementary points

    %object 1
    DO1x = true
    DO1y = false

    %object 2
    DO2x = false
    DO2y = true

    %object 3
    DO3x = true
    DO3_x = true
    DO3y = true
    DO3_y = true
end
%%

% demo1 - 2 demonstracije, 2 predmeta, paralelno sa x-osi
% demo2 - 3 demonstracije, 3 predmeta, paralelno sa x-osi
% demo3 - izbjegavanje prepreke sa 2 attractor pointa
% demo4 - 3 attractor pointa
% demo5 - sweeping task
% demo6 -
% demo7 -



%% ***************************************TRAJECTORIES
%% reading the trajectories

att=[];
TT_att={};
br=0;

for ii=1:nbDemo
    
    y_des=[];
    
    y_des=dlmread([demonstracije,'script',num2str(ii),'.txt'])
    
    %% transforming angle-axis representation to euler
    theta=[];
    ex=[];
    ey=[];
    ez=[];
    rot_robot_sve=[];
    robot_ori=[];
    Y_des_euler=[];
    
    theta=sqrt(y_des(:,4).^2 + y_des(:,5).^2 + y_des(:,6).^2)

    ex = y_des(:,4)./theta;
    ey = y_des(:,5)./theta;
    ez = y_des(:,6)./theta;

    rot_robot_sve = [ex, ey, ez, theta]

    robot_ori=axang2rotm(rot_robot_sve);
    

    for i=1:size(robot_ori,3)

        Robot_pos = [robot_ori(:,:,i); 0 0 0]; 
        Robot_pos = [Robot_pos, [y_des(i,1:3), 1]']
        Y_euler = tform2eul(Robot_pos)
        Y_des_euler(i,:) = [y_des(i,1:3), Y_euler]

    end

    X=Y_des_euler(:,1)*1000   % X component
    Y=Y_des_euler(:,2)*1000   % Y component
    thet_=Y_des_euler(:,4)*-1 % rotation (around Z)    
    
    
    %figure(1)
    figure(ii)
    
    xlabel('X[mm]','FontSize',12)
    ylabel('Y[mm]','FontSize',12)
    
    set(gca,'FontSize',13)
    
    hold on
    plot(X,Y)

    y_des=[X, Y, thet_]';


    %% trajectory meshing

    d = hypot(diff(X), diff(Y));             % Distance Of Each Segment
    d_tot = sum(d)                           % Total Distance

    Dist_func=[];

    for i=1:size(d)
        A=d(1:i);
        Dist_func(i) = sum(A); 
    end

    Dist_func = [0, Dist_func]'

     for i=1:size(Dist_func)
       Dist_func(i)=Dist_func(i)+i/1000
     end

    xm=[];
    xm=linspace(0, d_tot, nbStanja)

    tm=[];
    tm=linspace(0, size(X,1), size(X,1))

    T_sample=interp1(Dist_func, tm , xm)

    X_sampled=interp1(tm, X, T_sample)
    Y_sampled=interp1(tm, Y, T_sample)
    Theta_sampled=interp1(tm, thet_, T_sample)
    
    %%nadopuna mesha ukoliko fali tocka na pocetku
    X_sampled(1) = X(1)
    Y_sampled(1) = Y(1)
    Theta_sampled(1) = thet_(1)


    figure(ii)
    %figure(1)
    hold on
    scatter(X_sampled, Y_sampled)


    %% storing the sampled trajectories into cells

    att(1,:,ii)=X_sampled;
    att(2,:,ii)=Y_sampled;
    att(3,:,ii)=Theta_sampled;


    privremena=[]
    privremena = att(:,:,ii)
    privremena(isnan(privremena)) = privremena(isnan(privremena),2)
    att(:,:,ii) = privremena


    % figure(2)
    % hold on
    % xlabel('x1','FontSize',12)
    % ylabel('x2','FontSize',12)
    % set(gca,'FontSize',13)
    % %legend(demo_scat,'demonstration')


    %% transformation to TF matrix

    for j=1:nbStanja

        theta=att(3,j,ii);

        T=[cos(theta) sin(theta) att(1,j,ii)
           -sin(theta) cos(theta) att(2,j,ii)
           0    0   1];
       
        T3D=[cos(theta) sin(theta)  0  att(1,j,ii)
           -sin(theta) cos(theta) 0  att(2,j,ii)
                0           0     1      0
                0           0     0      1];

        %trplot2(T,'color', 'g','length', 10)
        %trplot2(T,'color', 'black','length',25, 'thick', 4,'LineWeight', 'bold', 'text_opts', {'FontSize',10, 'FontWeight', 'bold'})
        L = 'XY ' 
        trplot(T3D,'length', 20,'thick', 1.6,'color','black','labels', L,'text_opts', {'FontSize',11, 'FontWeight', 'bold'})
        
        br=br+1;

        %angleSve(br)=theta(i);
        TT_att{br} = {T};

    end
       
end

%%       ********************OBJECT POSITIONS
%% manually setting object positions

% TauN=[100  100  100  65 65 65  90  90  90  60  60  60 120 120 120 
%       40   40    40  25 25 25  45  45  45  30  30  30  30  30  30 
%        1    1     1  18 18 18  10  10  10  35  35  35  15  15  15 ];
   
%% reading the object positions from file
 
TauNS=[];
for ii=1:nbDemo 

 TauNS{ii} = dlmread([demonstracije,'TauN',num2str(ii),'.txt'])'   

   for i=1:size(TauNS{ii},2)
    
       theta = TauNS{ii}(3,i)
  
       T=[cos(theta) sin(theta) TauNS{ii}(1,i)*1000
          -sin(theta) cos(theta)   TauNS{ii}(2,i)*1000
               0    0   1];
       figure(1)
       %trplot2(T,'color', 'r','length', 20)

    end
 
end


br_predmeta = size(TauNS{1},2);  % number of objects in scene

TauN=[];
for jj=1:br_predmeta
    
    for i=1:nbDemo
        
        TauN = [TauN, TauNS{1,i}(:,jj)]
    
    end
end

TauN(3,:) = TauN(3,:).*180/pi;

nbTau = size(TauN,2);

TauNH={};
for i=1:nbTau
    
   xTau = TauN(1,i);
   yTau = TauN(2,i);
   theta = TauN(3,i)*pi/180;
    
   T=[cos(theta) sin(theta) xTau*1000
        -sin(theta) cos(theta)   yTau*1000
           0    0   1];
       
   TauNH{i} = T; 
   %**
   %figure(ii)
   %trplot2(T,'color', 'r','length', 20)
   
end

%% adding positions of interest manually to object locations

%translacija po +x-osi
DFx60 = eye(3);
DFx60(1,3)= 60;

%translacija po -x-osi
DFx60n = eye(3);
DFx60n(1,3)= -60;

%translacija po +y-osi
DFy60 = eye(3);
DFy60(2,3)= 60;

%translacija po -y-osi
DFy60n = eye(3);
DFy60n(2,3)= -60;


%translacija po +x-osi
DFx50 = eye(3);
DFx50(1,3)= 50;

%translacija po -x-osi
DFx50n = eye(3);
DFx50n(1,3)= -50;

%translacija po +y-osi
DFy50 = eye(3);
DFy50(2,3)= 50;

%translacija po -y-osi
DFy50n = eye(3);
DFy50n(2,3)= -50;

%% zakomentirati ako nema dodavanja tocaka
% for i=1:br_predmeta*nbDemo
%     
% 
%     TauNH = [TauNH, TauNH{i}*DFx]
% 
% 
% end
% 
% nbTau = size(TauNH,2);


%% dodavanje tocaka za predmet 1

if (DO1x == true)
    
    for i=1:nbDemo
        TauNH = [TauNH, TauNH{i}*DFx60]
    end
end

if (DO1y == true)
    
    for i=1:nbDemo
        TauNH = [TauNH, TauNH{i}*DFy60]
    end
end

 %% dodavanje tocaka za predmet 2
 
if (DO2x == true)
    
    for i=(nbDemo+1):2*nbDemo
        TauNH = [TauNH, TauNH{i}*DFx60]
    end
end

if (DO2y == true)
    
    for i=(nbDemo+1):2*nbDemo
        TauNH = [TauNH, TauNH{i}*DFy60]
    end
end

%% dodavanje tocaka za predmet 3

if (DO3x == true)
    for i=(2*nbDemo+1):(3*nbDemo)

        TauNH = [TauNH, TauNH{i}*DFx50]

    end
end

if (DO3_x == true)
    for i=(2*nbDemo+1):(3*nbDemo)

        TauNH = [TauNH, TauNH{i}*DFy50]
        
    end
end

if (DO3y == true)
    for i=(2*nbDemo+1):(3*nbDemo)

        TauNH = [TauNH, TauNH{i}*DFy50n]

    end
end

if (DO3_y == true)
    for i=(2*nbDemo+1):(3*nbDemo)

        TauNH = [TauNH, TauNH{i}*DFx50n]

    end
end

nbTau = size(TauNH,2);

%% **NOVO uspis svih frameova za svaku demonstraciju 

for j=1:nbDemo
     for i=1:size(TauNH,2)/nbDemo
         
          ii=((i-1)*nbDemo)+j

           T= TauNH{ii}
           
           T = [T(1:2,:); [0 0 0]; T(3,:)]
           T = [T(:,1:2) [0;0;1;0] T(:,3)]

           figure(j)
           %trplot2(T,'color', 'r','length', 20)
           %trplot2(T,'color', 'r','length',24, 'LineWeight', 'bold', 'text_opts', {'FontSize', 8, 'FontWeight', 'bold'})
           
           %trplot(T,'length', 40,'thick', 4,'color','r','notext')

           trplot(T,'length', 35,'thick', 3,'color','r','notext')
     end
end

%% transforming the trajectory frames into the object frames to achieve relative relation
%  transformacija tocaka u TauN frameove

for ii=1:size(TauNH,2)/nbDemo
    
    for i=1:nbDemo

        for j=1:nbStanja

            k = nbStanja*(i-1)+j;
            Ttransf{i,j} = inv(TauNH{(ii-1)*nbDemo+i})*TT_att{k}{:}; 
            TtranfSve{ii,1}=Ttransf;
        end

    end
    
end


%% **************************VARIABILITY ANALYSIS FOR EVERY FEATURE FRAME IN TauN
%% analiza varijabilnosti relativno

xxF=[];
iN=[];
for i=1:1000
       
    xF = 0.8^i;
    xxF(i)=xF;
    iN(i)=i;
    
end

Klaster={};
variability_disc=[];
for i = 1:size(TtranfSve, 1)
    
    br=0;
    Klaster{i,1}={};
    for j = 1:nbDemo
        
        dValue=[];
        sigma=[];
        for k = 1:nbStanja
            
            x = TtranfSve{i}{j,k}(1,3);
            y = TtranfSve{i}{j,k}(2,3);
            
            sigma(k) = sqrt(x^2 + y^2)/10; %Distances
            
            if(sigma(k)<1)
                dValue(k)=0.9
            else
                dValue(k) = interp1(iN,xxF,sigma(k));
            end
            
        end
        
        [vald, nbVald] = max(dValue);
        
        Klaster{i,1}{j} = TtranfSve{i}{j,nbVald};
        
        var_ind_demo(j) = sum(dValue); %variation in individual demonstration
    
    end
   
   posVariability(i) =  sum(var_ind_demo); %complete variability for task point
   
   
   if(posVariability(i)<1)
        variability_disc(i) = -1 %varijabla za klasifikaciju
    end
%     if (posVariability(i)> 0.8 && posVariability(i)< 1)
%          variability_disc(i) = 0          
%     end
    if (posVariability(i)> 1)
         variability_disc(i) = 1 
    end
    
end

%% odredivanje varijabilnosti orijentacije klastera preko matrice rotacija

xxF=[];
iN=[];
for i=1:50
    xF = 0.8^i;
    xxF(i)=xF;
    iN(i)=i; 
end

    
dValue = interp1(iN,xxF,15)*4

oriVariability_raw=[];
oriVariability =[];

nr=0;
for i=1:size(Klaster,1)
    
    if(variability_disc(i)==1) %uvjet da se oriVariability racuna samo za attractor pointe
        nr=nr+1;
        
        E_val=[];
        for j=1:size(Klaster{i},2)

            T1 = Klaster{i}{j};
            T1 = T1(1:2,1:2);

            br=0;
            for jj=1:size(Klaster{i},2)

                if (jj~=j)

                    br=br+1;
                    T2 = Klaster{i}{jj}
                    T2 = T2(1:2,1:2);

                    E(br) = norm(T1-T2)

                end
            end
            E_val(j)=sum(E);

        end

        oriVariability_raw(nr) = sum(E_val); %vrijednost veca sto su razlicitije 
        
        if(oriVariability_raw(nr)<1)
                oriVariability(nr)=0.9*4
            else
                oriVariability(nr) = interp1(iN,xxF,oriVariability_raw(nr))*4; %skaliranje
        end

        %vadenje srednje orijentacije - orijentacija koja najmanje odstupa od
        %svih ostalih
        [minValue, minNum] = min(E_val);

        meanOrientation{nr,1} = Klaster{i}{minNum};

        meanOrientation{nr,1} = meanOrientation{nr,1}(1:2,1:2)
        
    end
end

Tori=meanOrientation;

%%  goal classification - via distance
% klasifikacija goala - po udaljenosti

sigma=[];
goal_dist=[];
for i = 1:size(TtranfSve, 1)
    
    for j = 1:nbDemo
        
        x = TtranfSve{i}{j,nbStanja}(1,3);
        y = TtranfSve{i}{j,nbStanja}(2,3);
            
        sigma(j) = sqrt(x^2 + y^2); %Distances  
    end
    goal_dist(i) = sum(sigma);
end

[minGoalDist,nbGoal] = min(goal_dist);

%% goal classification - via orientation
% klasifikacija goala - po orijentaciji

for i=1:nbDemo
    
    T1 = TtranfSve{nbGoal}{i,nbStanja}
    T1 = T1(1:2,1:2);
    
    for ii=1:nbDemo
        
        br=0;
        if (ii~=i)
            br=br+1;
            
            T2 = TtranfSve{nbGoal}{ii,nbStanja}
            T2 = T2(1:2,1:2);
            E(br) = norm(T1-T2)
        end
    end
    E_orientation(i) = sum(E);
end

goal_ori_variation = sum(E_orientation);

%% *****************************READING THE NEW SITUATION FOR EXECUTION
%% citanje tocaka iz nove situacije

TauNew=[];
         
TauNew =dlmread([demonstracije,situacija,'TauNX.txt'])'

TauNew(1:2,:) = TauNew(1:2,:)*1000

%% dodatne tocke - nova situacija

TauNH_new={};
for i=1:size(TauNew,2)
    
   xTau = TauNew(1,i);
   yTau = TauNew(2,i);
   theta = TauNew(3,i);%*pi/180;
    
   T=[cos(theta) sin(theta) xTau
        -sin(theta) cos(theta)   yTau
           0    0   1];
       
   TauNH_new{i} = T; 
   
   %**
   %figure(ii)
   %trplot2(T,'color', 'r','length', 20)
   
end


%% zakomentirati ako nema dodavanja tocaka
% for i=1:size(TauNew,2)
%     
%     TauNH_new = [TauNH_new, TauNH_new{i}*DFx]
%     
% end

%% adding frames to new configuration

if (DO1x == true)
    TauNH_new = [TauNH_new, TauNH_new{1}*DFx60]
end
if (DO1y == true)
    TauNH_new = [TauNH_new, TauNH_new{1}*DFy60]
end


if (DO2x == true)
    TauNH_new = [TauNH_new, TauNH_new{2}*DFx60]
end

if (DO2y == true)
    TauNH_new = [TauNH_new, TauNH_new{2}*DFy60]
end


if (DO3x == true)
    TauNH_new = [TauNH_new, TauNH_new{3}*DFx50]
end

if (DO3y == true)
    TauNH_new = [TauNH_new, TauNH_new{3}*DFy50]
end

if (DO3_y == true)
    TauNH_new = [TauNH_new, TauNH_new{3}*DFy50n]
end

if (DO3_x == true)
    TauNH_new = [TauNH_new, TauNH_new{3}*DFx50n]
end

%% ***********************CLASSIFICATION OF FEATURE FRAMES BASED ON VARIABILITY
%% grupiranje tocaka u attract i reject

TauAttract={};
TauReject={};
posW=[];
 for i=1:nbTau/nbDemo 
    
    if variability_disc(i)==1
        
        posW = [posW posVariability(i)];
       %TauAttract =  [TauAttract TauNew(:,i)];
        TauAttract = [TauAttract, TauNH_new{i}];
    end
    if variability_disc(i)==-1
    
        %TauReject = [TauReject TauNew(:,i)];
        TauReject = [TauReject, TauNH_new{i}];
    end
    
 end
  
 
 %% plot TauNew transformacija
% figure(10)
% 
% xlabel('x1','FontSize',12)
% ylabel('x2','FontSize',12)
% 
% set(gca,'FontSize',13)
% 
% hold on
% 
% for i=1:size(TauNH_new,2)
%     
%     trplot2(TauNH_new{i},'color', 'b','length', 65)
%     
% end

%% variability plot

figure(30)
hold on

% for i=1:size(posVariability,2)
%     
%     bar(i,posVariability(i), 0.4,'FaceColor',[0, 0.4470, 0.7410])
%     %bar(2,posVariability(2), 0.4,'g')
%     %bar(3,posVariability(3), 0.4,'b')
%     
% end

xbar = linspace(1, size(posVariability,2), size(posVariability,2))

bar(xbar,posVariability, 0.4,'FaceColor',[0, 0.4470, 0.7410])


%% ****************************CONSTRUCTING INITIAL TRAJECTORY
%% generiranje pocetne trajektorije

start_=dlmread([demonstracije,situacija,'scriptX.txt'], ',');  % reading initial position of the robot

theta=[];
ex=[];
ey=[];
ez=[];
rot_robot_sve=[];
Robot_pos=[];
Start_XYZ_eul=[];

theta=sqrt(start_(4)^2 + start_(5)^2 + start_(6)^2);

ex = start_(4)/theta;
ey = start_(5)/theta;
ez = start_(6)/theta;

rot_start = [ex, ey, ez, theta];

start_ori=axang2rotm(rot_start);

Robot_pos = [start_ori; 0 0 0]; 
Robot_pos = [Robot_pos, [start_(1:3), 1]'];
Start_euler = tform2eul(Robot_pos);
Start_XYZ_eul = [start_(1:3), Start_euler];

start = [Start_XYZ_eul(1)*1000; Start_XYZ_eul(2)*1000; Start_XYZ_eul(4)*180/pi]

goal = [TauNH_new{nbGoal}(1,3); TauNH_new{nbGoal}(2,3)];

init_state_x = linspace(start(1),goal(1),nbStanja)
init_state_y = linspace(start(2),goal(2),nbStanja)
%angleNull = zeros(1,nbStanja);
angleNull = ones(1,nbStanja)*Start_XYZ_eul(4)*180/pi*-1;

initStates=[init_state_x; init_state_y; angleNull];

figure(10)
hold on

xlabel('X[mm]','FontSize',12)
ylabel('Y[mm]','FontSize',12)

set(gca,'FontSize',13)

scatter(init_state_x, init_state_y)

%% ploting initial trajectory

for i=1:nbStanja
    
    theta=angleNull(i);
    T = [cos(theta) sin(theta) init_state_x(i)
        -sin(theta) cos(theta) init_state_y(i)
         0             0          1];
     
    theta=angleNull(i);
    T3D = [cos(theta) sin(theta)  0    init_state_x(i)
        -sin(theta) cos(theta)  0    init_state_y(i)
            0           0       1           0
            0           0       0           1];
     
     figure(10)
     %trplot2(T,'color', 'm','length', 3)
     %trplot2(T,'color', 'black','length',10, 'LineWeight', 'bold', 'text_opts', {'FontSize', 8, 'FontWeight', 'bold'})
     L='XY '
     %trplot(T3D,'length', 20,'thick', 1.1,'color','black','labels', L,'text_opts', {'FontSize',10})
     trplot(T3D,'length', 22,'thick', 1.1,'rgb','notext')
     hold on
end

%%

xNull=initStates;

%% for data logging during optimization

nbIter=0;

global States reward;
global reward1 reward2 reward3 reward4 reward5 reward6 reward7 reward8;
global reward1_raw reward2_raw reward3_raw reward4_raw reward5_raw reward6_raw reward7_raw reward8_raw;
global pocetna;

pocetna = sqrt( (xNull(1,1) - xNull(1,size(xNull,2)))^2 + (xNull(2,1) - xNull(2,size(xNull,2)))^2);

States =[];
reward =[];

reward1=[];
reward2=[];
reward3=[];
reward4=[];
reward5=[];
reward6=[];
reward7=[];
reward8=[];

reward1_raw=[];
reward2_raw=[];
reward3_raw=[];
reward4_raw=[];
reward5_raw=[];
reward6_raw=[];
reward7_raw=[];
reward8_raw=[];



%% ***********************OPTIMIZATION
%cmaes_josip
%% poziv optimizacije
a=1; %breakpoint

%options = psoptimset('MaxFunEvals',10000,'TolMesh',0.01,'InitialMeshSize',10,'MeshExpansion',2,...
%                     'MeshContraction',0.5,'ScaleMesh','on');
options = psoptimset('MaxFunEvals',5000);
%options = optimoptions('patternsearch','FunctionTolerance',0.25,'StepTolerance',0.25,'Display','iter','PlotFcn',@psplotbestf);
A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
nonlcon = [];

xNullVec = reshape(xNull, [1,size(xNull,1)*size(xNull,2)])

OPTS = cmaes_v2;
OPTS.MaxFunEvals=numIter;
OPTS.PopSize = '(100 + floor(3*log(N)))'
[xmin,fmin,counteval,stopflag, out, bestever] = cmaes_v2('cost_trajektorija_igra',xNullVec,iniSTD, OPTS);

%[xmin,fval]=patternsearch(@cost_trajektorija_igra,xNull,A,b,Aeq,beq,lb,ub,nonlcon,options); %lose

%[xFin,fval]=fminsearch(@cost_trajektorija_igra,xNull) %lose

%[xFin,fval]=fminunc(@cost_trajektorija_igra,xNull)




%% plot functions


% plotting trajectory points
figure(10)

x_plotaj=xmin;
x_plotaj = reshape(x_plotaj, 3, nbStanja);

scatter(x_plotaj(1,:), x_plotaj(2,:))
plot(x_plotaj(1,:), x_plotaj(2,:))
%scatter(Tau1(1), Tau1(2));

for i=1:size(x_plotaj,2)
    
    theta=x_plotaj(3,i)*pi/180;
    
    T = [cos(theta) sin(theta)  0   x_plotaj(1,i)
        -sin(theta) cos(theta)  0   x_plotaj(2,i)
         0            0         1       0
         0            0         0       1];
     
     figure(10)
     hold on
     %trplot2(T,'color', 'g','length', 2.5)
     %trplot(T, 'rgb','length', 10)
     trplot(T,'length', 32,'thick', 2.5,'rgb','notext')
end

% plotting attractor frames

for i=1:size(TauAttract,2)
    
%trplot2(TauAttract{i},'color', 'b','length', 20)

%trplot2(TauAttract{i},'color', 'b','length',60, 'LineWeight', 'bold', 'text_opts', {'FontSize', 10, 'FontWeight', 'bold'})

T3D = TauAttract{i};
T3D = [T3D(1:2,:); [0 0 0]; T3D(3,:)]
T3D = [T3D(:,1:2) [0;0;1;0] T3D(:,3)]
L = 'XY ' 
trplot(T3D,'length', 55,'thick', 2.5,'color','b','labels', L,'text_opts', {'FontSize',14, 'FontWeight', 'bold'})

end


% plotting obstacle frames

for i=1:size(TauReject,2)
    
%trplot2(TauReject{i},'color', 'r','length', 20)

trplot2(TauReject{i},'color', 'r','length',24, 'LineWeight', 'bold', 'text_opts', {'FontSize', 8, 'FontWeight', 'bold'})

end

%% pisanje u file

dlmwrite('x_plotaj.txt', x_plotaj)

%dlmread('x_plotaj.txt')

%% plot obstacle

if size(TauReject,2)>0
    scatter(TauReject{1}(1,3), TauReject{1}(2,3),2000,'LineWidth',2)
end

%% generiranje dmp-a
%attractor_point_dmp


%% analiza 

figure(41)
plot(reward1)
% scatter([1:1:size(reward1,2)],reward1)

figure(42)
plot(reward2)
% scatter([1:1:size(reward2,2)],reward2)

figure(43)
plot(reward3)
% scatter([1:1:size(reward3,2)],reward3)

figure(44)
plot(reward4)
% scatter([1:1:size(reward4,2)],reward4)

figure(45)
plot(reward5)
% scatter([1:1:size(reward5,2)],reward5)

figure(46)
plot(reward6)
% scatter([1:1:size(reward5,2)],reward5)

figure(47)
plot(reward7)
% scatter([1:1:size(reward5,2)],reward5)

figure(48)
plot(reward8_raw)
% scatter([1:1:size(reward5,2)],reward5)


figure(50)

 xlabel('Iteration','FontSize',13)
 ylabel('Cost','FontSize',13)
 set(gca,'FontSize',12)

plot(reward, 'Color', [0.4941    0.4941    0.4941])
%plot(reward, 'Color', [0.8471    0.8471    0.8471])
%plot(reward, 'Color', [ 0   0   0   0.02],'LineWidth',5)
hold on
% scatter([1:1:size(reward5,2)],reward5)


x1=[1:1:size(reward,1)]'
p1 = polyfit(x1,reward,10)
y1 = polyval(p1,x1)
plot(x1,y1,'LineWidth',3,'Color','k')

rewardi=[reward1 reward2 reward3 reward4 reward5 reward6 reward7 reward8]

%% evolucija
figure(212)

epizoda=States{numIter};

plot(epizoda(1,:), epizoda(2,:))

%%

figure(11)

xlabel('X[mm]','FontSize',12)
ylabel('Y[mm]','FontSize',12)

set(gca,'FontSize',13)

hold on 

if size(TauReject,2)>0
    scatter(TauReject{1}(1,3), TauReject{1}(2,3),2000,'LineWidth',2)
end


% plotting attractor frames

for i=1:size(TauAttract,2)
    
%trplot2(TauAttract{i},'color', 'b','length', 20)

%trplot2(TauAttract{i},'color', 'b','length',60, 'LineWeight', 'bold', 'text_opts', {'FontSize', 10, 'FontWeight', 'bold'})

T3D = TauAttract{i};
T3D = [T3D(1:2,:); [0 0 0]; T3D(3,:)]
T3D = [T3D(:,1:2) [0;0;1;0] T3D(:,3)]
L = 'XY ' 
trplot(T3D,'length', 55,'thick', 2.5,'color','b','labels', L,'text_opts', {'FontSize',14, 'FontWeight', 'bold'})

end


% plotting obstacle frames

for i=1:size(TauReject,2)
    
%trplot2(TauReject{i},'color', 'r','length', 20)

trplot2(TauReject{i},'color', 'r','length',24, 'LineWeight', 'bold', 'text_opts', {'FontSize', 8, 'FontWeight', 'bold'})

end


scatter(x_plotaj(1,:), x_plotaj(2,:))
plot(x_plotaj(1,:), x_plotaj(2,:))









