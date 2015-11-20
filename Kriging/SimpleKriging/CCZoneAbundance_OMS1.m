    % create random field with autocorrelation
    
    clear all;
    close all;
    
    tic;
    
    load Dataset.mat;
    load Dataset_2.mat;
    load Dataset_3.mat;
    load Dataset_Abyss_0_2.mat;
    load Dataset_Abyss_All.mat

    Dataset_new = Dataset_Abyss_0_2(1:end-2,:);
    [X,Y] = meshgrid(-118:0.01:-115,11.5:0.01:13);

    x = Dataset_new(:,1);
    y = Dataset_new(:,2);
    z = Dataset_new(:,3);

    

    % calculate the sample variogram

    v = variogram([x y],z,'plotit',false,'maxdist',0.5);
    % and fit a spherical variogram
    figure(1)

    inx = find(isnan(v.val));
    
     
    [dum,dum,dum,vstruct] = variogramfit(v.distance,v.val,[],[],[],'model','spherical','nugget',1.0);
    title('variogram','FontSize',20)
    
    
    
    % now use the sampled locations in a kriging
    [Zhat,Zvar] = kriging(vstruct,x,y,z,X,Y,1000);
    figure(4)
    imagesc(X(1,:),Y(:,1),Zhat); axis image;
    axis xy;
    hold on;
    plot(x,y,'k*');
    title('Kriging predictions of nodule abundance,kg/m^2','FontSize',20)
    xlabel('Easting (degrees)','FontSize',20)
    ylabel('Northing (degrees)','FontSize',20)
     
     colorbar

     colormap(jet)
    figure(3)
    imagesc(X(1,:),Y(:,1),Zvar); axis image;axis xy
    title('kriging variance','FontSize',20)
    xlabel('Easting (degrees)','FontSize',20)
    ylabel('Northing (degrees)','FontSize',20)
    colorbar
    toc;
    
    colormap(jet)
    autoArrangeFigures();
    