% create random field with autocorrelation

clear all;
close all;

tic;


load Dataset_Abyss_0_2.mat;
load predicted_nodule_density.mat;

Dataset_new = Dataset_Abyss_0_2(1:end-2,:);
[X,Y] = meshgrid(-118:0.01:-115,11.5:0.01:13);

x = Dataset_new(:,1);
y = Dataset_new(:,2);
z = Dataset_new(:,3);

Dataset_new = [];

[Data_X,Data_Y] = meshgrid(Lon_vals,Lat_vals);
output_map_nodule_den = output_map_nodule_den';
Dataset_new = [Data_X(:),Data_Y(:),output_map_nodule_den(:)];
Dataset_new(30352:end,:) = [];
subsample = randi(size(Dataset_new,1),15,1);
x1 = Dataset_new(subsample,1);
y1 = Dataset_new(subsample,2);
z1 = Dataset_new(subsample,3);

ind = find(isnan(z1));
x1(ind) = [];
y1(ind) = [];
z1(ind) = [];



scale = std(z);
drift = mean(z);
z = (z - mean(z))./std(z);

scale1 = std(z1);
drift1 = mean(z1);
z1 = (z1 - mean(z1))./std(z1);

v12 = crossvariogram([x y],z,[x1 y1],z1,'plotit',false,'maxdist',0.9);
% and fit a spherical variogram

v1 = variogram([x y],z,'plotit',false,'maxdist',0.9);
v2 = variogram([x1 y1],z1,'plotit',false,'maxdist',0.9);


figure(1)
[dum,dum,dum,vstruct1] = variogramfit(v1.distance,v1.val,[],[],[],'model','spherical');%,'nugget',1.0);

figure(2)
[dum,dum,dum,vstruct2] = variogramfit(v2.distance,v2.val,[],[],[],'model','spherical');%,'nugget',1.0);

figure(3)
[dum,dum,dum,vstruct12] = variogramfit(v12.distance,v12.val,[],[],[],'model','spherical');%,'nugget',1.0);
title('variogram','FontSize',20)

% now use the sampled locations in a kriging
[Zhat,Zvar] = Cokriging(vstruct1,vstruct2,vstruct12,x,y,z,x1,y1,z1,X,Y,2000);
Zhat = scale.*Zhat + drift;
figure(4)
imagesc(X(1,:),Y(:,1),Zhat); axis image;
axis xy;
hold on;
plot([x;x1],[y;y1],'k*');
title('Co-Kriging predictions of nodule abundance,kg/m^2','FontSize',20)
xlabel('Easting (degrees)','FontSize',20)
ylabel('Northing (degrees)','FontSize',20)
colorbar
colormap(jet)

figure(5)
imagesc(X(1,:),Y(:,1),Zvar); axis image;axis xy
title('kriging variance','FontSize',20)
xlabel('Easting (degrees)','FontSize',20)
ylabel('Northing (degrees)','FontSize',20)
colorbar
colormap(jet)


[Zhat2,Zvar2] = kriging(vstruct2,x1,y1,z1,X,Y,2000);
Zhat2= scale1.*Zhat2 + drift1;
figure(7)
imagesc(X(1,:),Y(:,1),Zhat2); axis image;
axis xy;
hold on;
plot(x1,y1,'k*');
title('Kriging predictions of nodule abundance using "cheap" data,kg/m^2','FontSize',20)
xlabel('Easting (degrees)','FontSize',20)
ylabel('Northing (degrees)','FontSize',20)
colorbar
colormap(jet)

[Zhat1,Zvar1] = kriging(vstruct1,x,y,z,X,Y,2000);
Zhat1 = scale.*Zhat1 + drift;
figure(6)
imagesc(X(1,:),Y(:,1),Zhat1); axis image;
axis xy;
hold on;
plot(x,y,'k*');
title('Kriging predictions of nodule abundance using "expensive" data,kg/m^2','FontSize',20)
xlabel('Easting (degrees)','FontSize',20)
ylabel('Northing (degrees)','FontSize',20)
colorbar
colormap(jet)

toc;
% autoArrangeFigures();




