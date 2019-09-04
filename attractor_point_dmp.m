clear all

x_plotaj = dlmread('x_plotaj.txt')

%% orientation normalization

x_plotaj_new = [];

x_plotaj_new = rem(x_plotaj(3,:),360)

for i=1:size(x_plotaj,2)
    
    if (sign(x_plotaj_new(i))==-1)  %&& (abs(x_plotaj_new(i))>360)
        
       x_plotaj_new(i) = x_plotaj_new(i)+360
    
    elseif sign(x_plotaj_new(i))==1 && x_plotaj_new(i)<=180
       
        x_plotaj_new(i) = x_plotaj_new(i)+360
        
    elseif sign(x_plotaj_new(i))==1 && x_plotaj_new(i)<=180
       
        x_plotaj_new(i) = x_plotaj_new(i)+360
        
        
    end     
       
end


x_plotaj(3,:) = x_plotaj_new;

%%

%dodavanje tocaka kako bi se kretnja izvela do kraja
x_plotaj = [x_plotaj, x_plotaj(:,size(x_plotaj,2)), x_plotaj(:,size(x_plotaj,2))]

%figure(10)
%hold on
%scatter(x_plotaj(1,:), x_plotaj(2,:))

n_dmps=3;

run_time=1
dt=0.02
timesteps=run_time/dt
path=zeros(n_dmps, timesteps)

nbStates = size(x_plotaj,2);
nbData = timesteps;

%Gaussians equally distributed in time
Mu_t = fliplr(linspace(0,nbData*dt,nbStates));
Sigma_t = (nbData*dt/nbStates)*8E-2;


Mu_x = x_plotaj;


%% inicijalni uvjeti

%inicijalni uvjeti
ax=7; x=1; y=Mu_x(:,1)'; dy=[0 0 0]; ddy=[0 0 0]
ay=16; by=76; 

%% povecanje ay, by - smanjuje se prebacaj

%% 
x=1

for i=1:timesteps
   
    %x=x+(-ax*x*dt); %normalna decay fja

    x=x-dt;  %linearna decay fja
    
    %x = 3/(2+exp(i/8))  %sigmoidalna d.f.
    
    %figure(3)
    %hold on; scatter(i,x);
    
    
    for j=1:nbStates
        h2(:,j) = gaussPDF(x,Mu_t(j),Sigma_t); %Probability to be in a given state
    end
    h2 = h2./sum(h2); %Normalization
    
    currTar=0;
    for ii=1:nbStates
        currTar = currTar + Mu_x(:,ii).* h2(ii);
    end

    ddy=ay*(by*(currTar'-y)-dy);
    dy=dy+ddy*dt
    y=y+dy*dt
       
    y_track(i,:)=y
    dy_track(i,:)=dy
    ddy_track(i,:)=ddy
end

figure(11)
hold on
plot(y_track(:,1),y_track(:,2),'linewidth',2)

% figure(6)
% plot(dy_track(:,1))
% 
% figure(7)
% plot(dy_track(:,2))


%% plotting the DMP
figure(11)
hold on
for i=1:size(y_track,1)
    
    theta=y_track(i,3)*pi/180;
    
    T = [cos(theta) sin(theta)  0   y_track(i,1)
        -sin(theta) cos(theta)  0   y_track(i,2)
         0            0         1       0
         0            0         0       1];

     %trplot2(T,'color', 'g','length', 2.5)
     %trplot2(T,'color', 'g','length',16, 'LineWeight', 'bold', 'text_opts', {'FontSize', 8, 'FontWeight', 'bold'})
     
     trplot(T,'length', 20,'thick', 2,'rgb','notext')
     hold on
     
end














