% create random field with autocorrelation
tic;
load Dataset.mat;
load Dataset_2.mat;
load Dataset_3.mat;
load Dataset_Abyss_0_2;


Dataset_new = [Dataset;Dataset_2;Dataset_3;Dataset_Abyss_0_2];

[X,Y] = meshgrid(-180:0.1:-115,0:0.1:20);

x = Dataset_new(:,1);
y = Dataset_new(:,2);
z = Dataset_new(:,3);



% calculate the sample variogram

v = variogram([x y],z,'plotit',false,'maxdist',40);
% and fit a spherical variogram
figure(1)
hold on;
[dum,dum,dum,vstruct] = variogramfit(v.distance,v.val,[],[],[],'model','spherical');
title('variogram','FontSize',20)

% now use the sampled locations in a kriging
[Zhat,Zvar] = kriging(vstruct,x,y,z,X,Y);
figure(2)
imagesc(X(1,:),Y(:,1),Zhat); axis image;
axis xy;
title('Kriging predictions of nodule abundance,kg/m^2','FontSize',20)
xlabel('Easting (degrees)','FontSize',20)
ylabel('Northing (degrees)','FontSize',20)

colorbar

colormap(jet)
figure(3)
contour(X,Y,Zvar); axis image
title('kriging variance','FontSize',20)
xlabel('Easting (degrees)','FontSize',20)
ylabel('Northing (degrees)','FontSize',20)
toc;

colormap(jet)


