
function [ T, bestHits] = chamferMatch( T, scan, ogrid, Dmap, varargin)
%CHAMFERMATCH Chamfer distance based scan-matching
%   Detailed explanation goes here

    % Read optional parameters
    p = inputParser;  
    p.addParameter('SearchRot' , deg2rad( 0.25  ), @(x)isnumeric(x));    
    p.addParameter('SearchLin' ,          0.5    , @(x)isnumeric(x));    
    p.addParameter('MaxItterations' ,    30      , @(x)isnumeric(x));    
    p.addParameter('MaxDepth' ,           3      , @(x)isnumeric(x));  
    p.parse(varargin{:})

    SearchRot   = p.Results.SearchRot;
    SearchLin   = p.Results.SearchLin;
    MaxItterations = p.Results.MaxItterations;
    MaxDepth = p.Results.MaxDepth;
    
    
    % Search Results
    bestScore = Inf;
    Tbest     = T;
       
    
    % Decent walk
    t = SearchLin;
    r = SearchRot;

    siz = size(ogrid.grid);
    tmpX = ((ogrid.maxX - ogrid.minX + ogrid.pixelSize) / siz(1));
    tmpY = ((ogrid.maxY - ogrid.minY + ogrid.pixelSize) / siz(2));


    dmax = 0;
    imax = 0;
    while imax < MaxItterations

        for theta = ([-r 0 r]) + T(1,3)

            % Rotate scan
            m = [cos(theta) -sin(theta);
                 sin(theta)  cos(theta)] ;
            S = (m * scan')';


            for x = ([-t 0 t]) + T(1,1)
                for y = ([-t 0 t]) + T(1,2)

                % Translate points and convert to pixel coords                    
                Sx1 = round((S(:,1) + (x - ogrid.minX)) / tmpX );  
                Sy1 = round((S(:,2) + (y - ogrid.minY)) / tmpY );


                    % Bounds Checking
                    I = (Sx1 > 1) & ...
                        (Sy1 > 1) & ...
                        (Sx1 < siz(1)) & ...
                        (Sy1 < siz(2)) ;
                    Sx2 = Sx1(I);
                    Sy2 = Sy1(I);

                    % Fitness
                    ind = Sx2 + (Sy2 - 1).*siz(1);                    

                    hits  = Dmap(ind);
                    score = sum(hits);


                    % Keep best score
                    if score < bestScore
                        Tbest     = [x y theta];
                        bestScore = score;  
                        bestHits  = hits;                          
                    end       
                end 
            end
        end

        % No better match was found, increase resolution
        if T == Tbest
            r = r/2;
            t = t/2;

            dmax = dmax + 1;
            if dmax > MaxDepth
                break
            end
        end

        % Init Next Iteration
        T = Tbest;
        imax = imax + 1;
    end
   
    
    bestHits = (bestHits == 0);
end

