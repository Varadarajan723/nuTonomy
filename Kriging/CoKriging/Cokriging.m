function [zi,s2zi] = kriging(vstruct1,vstruct2,vstruct12,x1,y1,z1,x2,y2,z2,xi,yi,chunksize)

% interpolation with ordinary kriging in two dimensions
%
% Syntax:
%
%     [zi,zivar] = kriging(vstruct,x,y,z,xi,yi)
%     [zi,zivar] = kriging(vstruct,x,y,z,xi,yi,chunksize)
%
% Description:
%
%     kriging uses ordinary kriging to interpolate a variable z measured at
%     locations with the coordinates x and y at unsampled locations xi, yi.
%     The function requires the variable vstruct that contains all
%     necessary information on the variogram. vstruct is the forth output
%     argument of the function variogramfit.
%
%     This is a rudimentary, but easy to use function to perform a simple
%     kriging interpolation. I call it rudimentary since it always includes
%     ALL observations to estimate values at unsampled locations. This may
%     not be necessary when sample locations are not within the
%     autocorrelation range but would require something like a k nearest
%     neighbor search algorithm or something similar. Thus, the algorithms
%     works best for relatively small numbers of observations (100-500).
%     For larger numbers of observations I recommend the use of GSTAT.
%
%     Note that kriging fails if there are two or more observations at one
%     location or very, very close to each other. This may cause that the 
%     system of equation is badly conditioned. Currently, I use the
%     pseudo-inverse (pinv) to come around this problem. If you have better
%     ideas, please let me know.
%
% Input arguments:
%
%     vstruct   structure array with variogram information as returned
%               variogramfit (forth output argument)
%     x,y       coordinates of observations
%     z         values of observations
%     xi,yi     coordinates of locations for predictions 
%     chunksize nr of elements in zi that are processed at one time.
%               The default is 100, but this depends largely on your 
%               available main memory and numel(x).
%
% Output arguments:
%
%     zi        kriging predictions
%     zivar     kriging variance
%
% Example:
%
%     % create random field with autocorrelation
%     [X,Y] = meshgrid(0:500);
%     Z = randn(size(X));
%     Z = imfilter(Z,fspecial('gaussian',[40 40],8));
%
%     % sample the field
%     n = 500;
%     x = rand(n,1)*500;
%     y = rand(n,1)*500;
%     z = interp2(X,Y,Z,x,y);
%
%     % plot the random field
%     subplot(2,2,1)
%     imagesc(X(1,:),Y(:,1),Z); axis image; axis xy
%     hold on
%     plot(x,y,'.k')
%     title('random field with sampling locations')
%
%     % calculate the sample variogram
%     v = variogram([x y],z,'plotit',false,'maxdist',100);
%     % and fit a spherical variogram
%     subplot(2,2,2)
%     [dum,dum,dum,vstruct] = variogramfit(v.distance,v.val,[],[],[],'model','stable');
%     title('variogram')
%
%     % now use the sampled locations in a kriging
%     [Zhat,Zvar] = kriging(vstruct,x,y,z,X,Y);
%     subplot(2,2,3)
%     imagesc(X(1,:),Y(:,1),Zhat); axis image; axis xy
%     title('kriging predictions')
%     subplot(2,2,4)
%     contour(X,Y,Zvar); axis image
%     title('kriging variance')
%
%
% see also: variogram, variogramfit, consolidator, pinv
%


% size of input arguments
sizest = size(xi);
numest = numel(xi);
numobs1 = numel(x1);
numobs2 = numel(x2);
% force column vectors
xi = xi(:);
yi = yi(:);
x1  = x1(:);
y1  = y1(:);
z1  = z1(:);

x2  = x2(:);
y2  = y2(:);
z2  = z2(:); 

% if nargin == 6;
%     chunksize = 100;
% elseif nargin == 7;
% else
%     error('wrong number of input arguments')
% end

% check if the latest version of variogramfit is used
if ~isfield(vstruct1, 'func')
    error('please download the latest version of variogramfit from the FEX')
end

if ~isfield(vstruct2, 'func')
    error('please download the latest version of variogramfit from the FEX')
end

if ~isfield(vstruct12, 'func')
    error('please download the latest version of variogramfit from the FEX')
end

% variogram function definitions
switch lower(vstruct1.model)    
    case {'whittle' 'matern'}
        error('whittle and matern are not supported yet');
    case 'stable'
        stablealpha = vstruct.stablealpha; %#ok<NASGU> % will be used in an anonymous function
end

switch lower(vstruct2.model)    
    case {'whittle' 'matern'}
        error('whittle and matern are not supported yet');
    case 'stable'
        stablealpha = vstruct.stablealpha; %#ok<NASGU> % will be used in an anonymous function
end

switch lower(vstruct12.model)    
    case {'whittle' 'matern'}
        error('whittle and matern are not supported yet');
    case 'stable'
        stablealpha = vstruct.stablealpha; %#ok<NASGU> % will be used in an anonymous function
end


% distance matrix of locations with known values
Dx1 = hypot(bsxfun(@minus,x1,x1'),bsxfun(@minus,y1,y1'));
Dx2 = hypot(bsxfun(@minus,x2,x2'),bsxfun(@minus,y2,y2'));

for ii = 1:numobs1
    for jj = 1:numobs2
        Dx12(ii,jj) = hypot(x1(ii) - x2(jj),y1(ii) - y2(jj));
    end
end



% if we have a bounded variogram model, it is convenient to set distances
% that are longer than the range to the range since from here on the
% variogram value remains the same and we dont need composite functions.
switch vstruct1.type;
    case 'bounded'
        Dx1 = min(Dx1,vstruct1.range);
        Dx2 = min(Dx2,vstruct2.range);
        Dx12 = min(Dx12,vstruct12.range);
    otherwise
end

% now calculate the matrix with variogram values 
A1 = vstruct1.func([vstruct1.range vstruct1.sill],Dx1);
if ~isempty(vstruct1.nugget)
    A1 = A1+vstruct1.nugget;
end

A2 = vstruct2.func([vstruct2.range vstruct2.sill],Dx2);
if ~isempty(vstruct2.nugget)
    A2 = A2+vstruct2.nugget;
end

A12 = vstruct12.func([vstruct12.range vstruct12.sill],Dx12);
if ~isempty(vstruct12.nugget)
    A12 = A12+vstruct12.nugget;
end


% the matrix must be expanded by one line and one row to account for
% condition, that all weights must sum to one (lagrange multiplier)
A = [A1 A12 ones(numobs1,1) zeros(numobs1,1);A12' A2 zeros(numobs2,1) ones(numobs2,1);ones(1,numobs1) zeros(1,numobs2) 0 0;zeros(1,numobs1) ones(1,numobs2) 0 0];


% A is often very badly conditioned. Hence we use the Pseudo-Inverse for
% solving the equations
A = pinv(A);


% we also need to expand z
z  = [z1;z2;0;0];

% allocate the output zi
zi = nan(numest,1);

if nargout == 2;
    s2zi = nan(numest,1);
    krigvariance = true;
else
    krigvariance = false;
end

% parametrize engine
nrloops   = ceil(numest/chunksize);

% initialize the waitbar
% h  = waitbar(0,'Kr...kr...kriging');

% now loop 
for r = 1:nrloops;
    % waitbar 
%     waitbar(r / nrloops,h);
    
    % built chunks
    if r<nrloops
        IX = (r-1)*chunksize +1 : r*chunksize;
    else
        IX = (r-1)*chunksize +1 : numest;
        chunksize = numel(IX);
    end
    
    % build b
    b1 = hypot(bsxfun(@minus,x1,xi(IX)'),bsxfun(@minus,y1,yi(IX)'));
    b2 = hypot(bsxfun(@minus,x2,xi(IX)'),bsxfun(@minus,y2,yi(IX)'));
    % again set maximum distances to the range
    switch vstruct1.type
        case 'bounded'
            b1 = min(vstruct1.range,b1);
            b2 = min(vstruct12.range,b2);
    end
    
    % expand b with ones
    b = [vstruct1.func([vstruct1.range vstruct1.sill],b1);vstruct12.func([vstruct12.range vstruct12.sill],b2);ones(1,chunksize);zeros(1,chunksize)];
    % need to correct for nugget case
    if ~isempty(vstruct1.nugget)
        b = b+vstruct1.nugget;
        b(end-1,:) = ones(1,size(b,2));
         b(end,:) = zeros(1,size(b,2));
    end
    
    
%     if r == nrloops
%         save('b','b');
%     end
        
    % solve system
    lambda = A*b;
    
    if r == 1
        save('lambda','lambda');
%         lambda(end,1:10)
    end
    
    % estimate zi
    zi(IX)  = lambda'*z;
%     if r == 1
%         display(size(z));
%     end
    % calculate kriging variance
    if krigvariance
        s2zi(IX) = sum(b.*lambda,1);
    end
    
end

% close waitbar
% close(h)

% reshape zi
zi = reshape(zi,sizest);

if krigvariance
    s2zi = reshape(s2zi,sizest);
end
