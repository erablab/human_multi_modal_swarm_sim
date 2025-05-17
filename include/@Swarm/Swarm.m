classdef Swarm < handle

    properties
        N
        robots
        L
        environment
        phi
        hG
    end

    methods
        function obj = Swarm(varargin)
            ip = inputParser;
            addParameter(ip, 'robots', [])
            addParameter(ip, 'L', [])
            addParameter(ip, 'environment', [])
            addParameter(ip, 'densityFunction', 'uniform')
            parse(ip,varargin{:})

            obj.N = length(ip.Results.robots);
            obj.robots = ip.Results.robots;
            if obj.N == 1
                obj.L = 0;
            else
                obj.L = ip.Results.L;
                if isempty(obj.L)
                    obj.L = zeros(obj.N);
                elseif ~isnumeric(obj.L)
                    if obj.N < 3 && strcmp(obj.L,'cycle')
                        obj.L = 'line';
                    end
                    switch obj.L
                        case 'line'
                            obj.L = full(gallery('tridiag',obj.N,-1,2,-1));
                            obj.L(1,1) = 1;
                            obj.L(obj.N,obj.N) = 1;
                        case 'cycle'
                            obj.L = toeplitz([2;-1;zeros(obj.N-3,1);-1]);
                        case 'complete'
                            obj.L = obj.N*eye(obj.N)-ones(obj.N);
                    end
                else
                    assert(sqrt(numel(obj.L))==obj.N, 'Laplacian is not a square matrix whose dimension is the number of robots.')
                end
            end
            obj.environment = ip.Results.environment;
            if ~isempty(obj.environment)
                if norm(obj.environment(:,end)-obj.environment(:,1)) > 1e-3
                    obj.environment = [obj.environment obj.environment(:,1)];
                end
            end
            obj.phi = ip.Results.densityFunction;
            obj.hG = struct('figure',[],'graph',[],'env',[],'density',[],'voronoiCells',[],'voronoiCentroids',[],'robots',[]);
        end

        function q = getPoses(obj)
            d = size(obj.robots{1}.getPose(),1);
            q = NaN(d,obj.N);
            for i = 1 : obj.N
                q(:,i) = obj.robots{i}.getPose();
            end
            if d == 2
                q = [q; zeros(1,obj.N)];
            end
        end

        function neighbors = getNeighbors(obj, idx)
            neighbors = find(obj.L(idx, :) ~= 0);
            neighbors = neighbors(neighbors~=idx);
        end

        function setPoses(obj, q)
            for i = 1 : obj.N
                obj.robots{i}.setPose(q(:,i));
            end
        end

        function moveSingleIntegrators(obj, v)
            for i = 1 : obj.N
                obj.robots{i}.moveSingleIntegrator(v(:,i))
            end
        end

        function moveUnicycles(obj, v)
            for i = 1 : obj.N
                obj.robots{i}.moveUnicycle(v(:,i))
            end
        end

        function goToPoints(obj, p, varargin)
            for i = 1 : obj.N
                obj.robots{i}.goToPoint(p(:,i), varargin{:})
            end
        end

        function [G,A,VC] = coverageControl(obj,varargin)
            if isempty(varargin)
                p = eye(2,3)*obj.getPoses();
                Np = obj.N;
            else
                p = varargin{1};
                Np = size(p,2);
            end
            P = [p obj.mirrorRobotsAboutEnvironmentBoundary(p)];
            [V,C] = voronoin(P');
            V(V==Inf) = 1e3*max(abs(obj.environment(:)));
            G = nan(2,Np);
            A = nan(1,Np);
            VC = cell(1,Np);
            for i = 1 : Np
                VC{i} = [V(C{i},1) V(C{i},2)]';
                [Gi, Ai] = obj.centroid(VC{i});
                G(:,i) = Gi;
                A(i) = abs(Ai);
            end
        end

        function c = evaluateCoverageCost(obj, q, VC, varargin)
            p = eye(2,3)*q;
            c = 0;
            if ~isempty(varargin)
                idx = varargin{1};
                VC = {VC{idx}};
            end
            for i = 1 : length(VC)
                if isempty(varargin)
                    idx = i;
                end
                P = VC{i};
                xP = P(1,:);
                yP = P(2,:);
                if strcmp(obj.phi, 'uniform')
                    f = @(x,y) norm(p(:,idx)-[x;y])^2;
                else
                    f = @(x,y) norm(p(:,idx)-[x;y])^2 * obj.phi(x,y);
                end
                trngltn = delaunay(xP, yP);
                ci = 0;
                for n = 1 : size(trngltn, 1)
                    ci = ci + Swarm.intOfFOverT(f, 8, P(:,trngltn(n,:)));
                end
                c = c + ci;
            end
        end

        function f = evaluateHapticFeedback(obj, q, VC, varargin)
            global SIGMA_11 SIGMA_22 mu
            p = eye(2,3)*q;
            f1 = 0;
            f2 = 0;
            if ~isempty(varargin)
                idx = varargin{1};
                VC = {VC{idx}};
            end
            for i = 1 : length(VC)
                if isempty(varargin)
                    idx = i;
                end
                P = VC{i};
                xP = P(1,:);
                yP = P(2,:);
                if strcmp(obj.phi, 'uniform')
                    
                else
                    phi_hat_1 = @(x,y) norm(p(:,idx)-[x;y])^2 * obj.phi(x,y) * SIGMA_11 * (x - mu(1));
                    phi_hat_2 = @(x,y) norm(p(:,idx)-[x;y])^2 * obj.phi(x,y) * SIGMA_22 * (y - mu(2));
                end
                trngltn = delaunay(xP, yP);
                fi1 = 0;
                fi2 = 0;
                for n = 1 : size(trngltn, 1)
                    fi1 = fi1 + Swarm.intOfFOverT(phi_hat_1, 8, P(:,trngltn(n,:)));
                    fi2 = fi2 + Swarm.intOfFOverT(phi_hat_2, 8, P(:,trngltn(n,:)));
                end
                f1 = f1 + fi1;
                f2 = f2 + fi2;
            end
            f = [f1; f2];
        end

        function plotFigure(obj)
            obj.hG.figure = figure('units','normalized','position',[0 0 1 1],'MenuBar','none','ToolBar','none','NumberTitle','off');
            hold on, axis equal
            if ~isempty(obj.environment)
                axis([min(obj.environment(1,:))-1 max(obj.environment(1,:))+1 min(obj.environment(2,:))-1 max(obj.environment(2,:))+1])
            else
                axis([-1 1 -1 1])
            end
            set(gca,'Visible','off')
            obj.hG.figure.CurrentAxes.Clipping = 'off';
        end

        function plotRobots(obj, varargin)
            for i = 1 : obj.N
                obj.robots{i}.plotRobot(varargin{:})
            end
        end

        function plotRobotsFast(obj, q, varargin)
            if isempty(obj.hG.robots)
                obj.hG.robots = scatter(q(1,:), q(2,:), varargin{:});
            else
                set(obj.hG.robots, 'XData', q(1,:), 'YData', q(2,:));
            end
        end

        function plotGraph(obj, varargin)
            if ~isempty(varargin)
                args = varargin;
            else
                args = {'Color', [0 0 0], 'LineWidth', 2};
            end
            q = obj.getPoses();
            if isempty(obj.hG.graph)
                for i = 1 : obj.N
                    for j = obj.getNeighbors(i)
                        obj.hG.graph(end+1) = plot(zeros(1,2),zeros(1,2),args{:});
                    end
                end
            else
                edge_counter = 0;
                for i = 1 : obj.N
                    for j = obj.getNeighbors(i)
                        edge_counter = edge_counter + 1;
                        set(obj.hG.graph(edge_counter), 'XData', q(1,[i,j]), 'YData', q(2,[i,j]));
                    end
                end
            end
        end

        function plotEnvironment(obj, varargin)
            if ~isempty(varargin)
                args = varargin;
            else
                args = {'LineWidth', 5, 'Color', [0 0 0]};
            end
            obj.hG.env = plot(obj.environment(1,:), obj.environment(2,:), args{:});
        end

        function plotDensity(obj, varargin)
            if ~strcmp(obj.phi, 'uniform')
                x = min(obj.environment(1,:)):0.01:max(obj.environment(1,:));
                y = min(obj.environment(2,:)):0.01:max(obj.environment(2,:));
                [X ,Y] = meshgrid(x, y);
                Z = obj.phi(X, Y);
                caxis([min(Z(:)),max(Z(:))])
                if ~isempty(varargin)
                    levels = varargin{1};
                else
                    levels = linspace(min(Z(:)),max(Z(:)),10);
                end
                M = contourc(x, y, Z, levels);
                [xC, yC, zC] = Swarm.C2xyz(M);
                if isempty(obj.hG.density)
                    cmap = colormap(parula);
                    subsample = floor(size(cmap, 1) / numel(levels));
                    colors = fliplr(cmap(end : -subsample: 1, :)')';
                    obj.hG.density = cell(numel(xC), 1);
                    for i = 1 : numel(xC)
                        obj.hG.density{i} = plot(xC{i}, yC{i}, 'LineWidth', 2, 'Color', colors(i, :));
                    end
                else
                    for i = 1 : min(numel(xC), numel(obj.hG.density))
                        set(obj.hG.density{i}, 'XData', xC{i}, 'YData', yC{i})
                    end
                end
                obj.fillout(obj.environment(1,:),obj.environment(2,:),[min(obj.environment(1,:))-1 max(obj.environment(1,:))+1 min(obj.environment(2,:))-1 max(obj.environment(2,:))+1],0.94*[1 1 1]);
            end
        end

        function plotVoronoiCells(obj, VC, varargin)
            if ~isempty(varargin)
                args = varargin;
            else
                args = {'Color', [0 0 0], 'LineWidth', 2};
            end
            vcite = zeros(size(cell2mat(VC))+[0 length(VC)-1]);
            idx = 0;
            for i = 1 : length(VC)
                l_vcite = size(VC{i},2) + 1;
                if i == 1
                    vcite(:,1:l_vcite) = VC{i}(:,[1:end,1]);
                    idx = idx + l_vcite;
                else
                    vcite(:,idx+1:idx+l_vcite+1) = [NaN(2,1) VC{i}(:,[1:end,1])];
                    idx = idx + l_vcite + 1;
                end
            end
            if isempty(obj.hG.voronoiCells)
                obj.hG.voronoiCells = plot(vcite(1,:), vcite(2,:), args{:});
            else
                set(obj.hG.voronoiCells, 'XData', vcite(1,:), 'YData', vcite(2,:))
            end
        end

        function plotCentroids(obj, G, varargin)
            if ~isempty(varargin)
                args = varargin;
            else
                args = {'.', 'Color', [0 0 0], 'MarkerSize', 10};
            end
            if isempty(obj.hG.voronoiCentroids)
                obj.hG.voronoiCentroids = plot(G(1,:), G(2,:), args{:});
            else
                set(obj.hG.voronoiCentroids, 'XData', G(1,:), 'YData', G(2,:))
            end
        end
    end

    methods (Access = private)
        function mirroredRobots = mirrorRobotsAboutEnvironmentBoundary(obj, p)
            mirroredRobots = nan(2,size(p,2)*(size(obj.environment,2)-1));
            for i = 1 : size(p,2)
                point = p(:,i);
                for j = 1 : size(obj.environment,2)-1
                    pointWrtSide = (point - obj.environment(:,j));
                    side = obj.environment(:,j+1) - obj.environment(:,j);
                    lengthOfPProjectedOntoL = pointWrtSide' * side / norm(side)^2;
                    projectedVector = obj.environment(:,j) + lengthOfPProjectedOntoL * side;
                    mirroredRobots(:,(i-1)*(size(obj.environment,2)-1)+j) = point - 2 * (point - projectedVector);
                end
            end
        end

        function [G, A] = centroid(obj, P)
            if strcmp(obj.phi, 'uniform')
                n = length(P);
                M = [0 1;-1 0];
                A = 0;
                S = 0;
                for i = 1 : n
                    ri = P(:,i);
                    if i < n
                        j = i + 1;
                    else
                        j = 1;
                    end
                    rj = P(:,j);
                    rjo = M * rj;
                    A = A + ri'*rjo;
                    S = S + (ri' * rjo * (ri + rj));
                end
                A = A / 2;
                S = S / 6;
                G = S / A;
            else
                xP = P(1,:);
                yP = P(2,:);
                phiA = @(x,y) max(eps,obj.phi(x,y));
                phiSx = @(x,y) x.*max(eps,obj.phi(x,y));
                phiSy = @(x,y) y.*max(eps,obj.phi(x,y));
                trngltn = delaunay(xP, yP);
                A = 0;
                S = 0;
                for i = 1 : size(trngltn, 1)
                    A = A + Swarm.intOfFOverT(phiA, 8, P(:,trngltn(i,:)));
                    S = S + [Swarm.intOfFOverT(phiSx, 8, P(:,trngltn(i,:)));
                        Swarm.intOfFOverT(phiSy, 8, P(:,trngltn(i,:)))];
                end
                G = S / A;
            end
        end
    end

    methods (Static)
        function I = intOfFOverT(f, N, T)
            x1 = T(1,1);
            x2 = T(1,2);
            x3 = T(1,3);
            y1 = T(2,1);
            y2 = T(2,2);
            y3 = T(2,3);
            xyw = Swarm.TriGaussPoints(N);
            A = abs(x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2))/2;
            NP = size(xyw(:,1), 1);
            I = 0;
            for j = 1 : NP
                x = x1*(1-xyw(j,1)-xyw(j,2))+x2*xyw(j,1)+x3*xyw(j,2);
                y = y1*(1-xyw(j,1)-xyw(j,2))+y2*xyw(j,1)+y3*xyw(j,2);
                I = I + f(x,y)*xyw(j,3);
            end
            I = A*I;
        end

        function xw = TriGaussPoints(n)
            % https://github.com/FMenhorn/BGCEGit/blob/master/Prototypes/MATLAB/Sandbox/BennisChaos/marchingCubes/L2Projection/TriGaussPoints.m
            xw = zeros(n,3);
            if n==1
                xw = [0.33333333333333 0.33333333333333 1.00000000000000];
            elseif n==2
                xw = [0.16666666666667 0.16666666666667 0.33333333333333
                    0.16666666666667 0.66666666666667 0.33333333333333
                    0.66666666666667 0.16666666666667 0.33333333333333];
            elseif n==3
                xw = [0.33333333333333 0.33333333333333 -0.56250000000000
                    0.20000000000000 0.20000000000000 0.52083333333333
                    0.20000000000000 0.60000000000000 0.52083333333333
                    0.60000000000000 0.20000000000000 0.52083333333333];
            elseif n==4
                xw = [0.44594849091597 0.44594849091597 0.22338158967801
                    0.44594849091597 0.10810301816807 0.22338158967801
                    0.10810301816807 0.44594849091597 0.22338158967801
                    0.09157621350977 0.09157621350977 0.10995174365532
                    0.09157621350977 0.81684757298046 0.10995174365532
                    0.81684757298046 0.09157621350977 0.10995174365532];
            elseif n==5
                xw = [0.33333333333333 0.33333333333333 0.22500000000000
                    0.47014206410511 0.47014206410511 0.13239415278851
                    0.47014206410511 0.05971587178977 0.13239415278851
                    0.05971587178977 0.47014206410511 0.13239415278851
                    0.10128650732346 0.10128650732346 0.12593918054483
                    0.10128650732346 0.79742698535309 0.12593918054483
                    0.79742698535309 0.10128650732346 0.12593918054483];
            elseif n==6
                xw = [0.24928674517091 0.24928674517091 0.11678627572638
                    0.24928674517091 0.50142650965818 0.11678627572638
                    0.50142650965818 0.24928674517091 0.11678627572638
                    0.06308901449150 0.06308901449150 0.05084490637021
                    0.06308901449150 0.87382197101700 0.05084490637021
                    0.87382197101700 0.06308901449150 0.05084490637021
                    0.31035245103378 0.63650249912140 0.08285107561837
                    0.63650249912140 0.05314504984482 0.08285107561837
                    0.05314504984482 0.31035245103378 0.08285107561837
                    0.63650249912140 0.31035245103378 0.08285107561837
                    0.31035245103378 0.05314504984482 0.08285107561837
                    0.05314504984482 0.63650249912140 0.08285107561837];
            elseif n==7
                xw = [0.33333333333333 0.33333333333333 -0.14957004446768
                    0.26034596607904 0.26034596607904 0.17561525743321
                    0.26034596607904 0.47930806784192 0.17561525743321
                    0.47930806784192 0.26034596607904 0.17561525743321
                    0.06513010290222 0.06513010290222 0.05334723560884
                    0.06513010290222 0.86973979419557 0.05334723560884
                    0.86973979419557 0.06513010290222 0.05334723560884
                    0.31286549600487 0.63844418856981 0.07711376089026
                    0.63844418856981 0.04869031542532 0.07711376089026
                    0.04869031542532 0.31286549600487 0.07711376089026
                    0.63844418856981 0.31286549600487 0.07711376089026
                    0.31286549600487 0.04869031542532 0.07711376089026
                    0.04869031542532 0.63844418856981 0.07711376089026];
            elseif n==8
                xw = [0.33333333333333 0.33333333333333 0.14431560767779
                    0.45929258829272 0.45929258829272 0.09509163426728
                    0.45929258829272 0.08141482341455 0.09509163426728
                    0.08141482341455 0.45929258829272 0.09509163426728
                    0.17056930775176 0.17056930775176 0.10321737053472
                    0.17056930775176 0.65886138449648 0.10321737053472
                    0.65886138449648 0.17056930775176 0.10321737053472
                    0.05054722831703 0.05054722831703 0.03245849762320
                    0.05054722831703 0.89890554336594 0.03245849762320
                    0.89890554336594 0.05054722831703 0.03245849762320
                    0.26311282963464 0.72849239295540 0.02723031417443
                    0.72849239295540 0.00839477740996 0.02723031417443
                    0.00839477740996 0.26311282963464 0.02723031417443
                    0.72849239295540 0.26311282963464 0.02723031417443
                    0.26311282963464 0.00839477740996 0.02723031417443
                    0.00839477740996 0.72849239295540 0.02723031417443];
            end
        end

        function h = fillout(x,y,lims,varargin)
            % MMA 23-3-2006, mma@odyle.net
            h = [];
            if nargin <2
                disp(['## ',mfilename,' : more input arguments required']);
                return
            end

            if numel(x) > length(x)
                x = Swarm.var_border(x);
            end
            if numel(y) > length(y)
                y = Swarm.var_border(y);
            end
            if length(x) ~= length(y)
                disp(['## ',mfilename,' : x and y must have the same size']);
                return
            end
            if nargin<4
                varargin={'g'};
            end
            if nargin<3
                lims=[min(x) max(x) min(y) max(y)];
            else
                if lims(1) > min(x), lims(1)=min(x); end
                if lims(2) < max(x), lims(2)=max(x); end
                if lims(3) > min(y), lims(3)=min(y); end
                if lims(4) < max(y), lims(4)=max(y); end
            end
            xi=lims(1); xe=lims(2);
            yi=lims(3); ye=lims(4);
            i=find(x==min(x)); i=i(1);
            x=x(:);
            y=y(:);
            x=[x(i:end)' x(1:i-1)' x(i)];
            y=[y(i:end)' y(1:i-1)' y(i)];
            x=[xi   xi xe xe xi xi   x(1) x];
            y=[y(1) ye ye yi yi y(1) y(1) y];
            h=fill(x,y,varargin{:});
            set(h,'edgecolor','none');
        end

        function [x,xc] = var_border(M)
            % MMA 18-8-2004, martinho@fis.ua.pt
            x  = [];
            xc = [];
            if nargin == 0
                return
            end
            xl = M(:,1);
            xt = M(end,:);  xt = xt';
            xr = M(:,end);  xr = flipud(xr);
            xb = M(1,:);    xb = flipud(xb');
            x =  [xl; xt; xr; xb];
            xc =  [xl(1) xl(end) xr(1) xr(end)];
        end

        function [x,y,z] = C2xyz(C)
            % C2XYZ returns the x and y coordinates of contours in a contour
            % matrix and their corresponding z values. C is the contour matrix given by
            % the contour function.
            %
            %
            %% Syntax
            %
            %  [x,y] = C2xyz(C)
            %  [x,y,z] = C2xyz(C)
            %
            %% Description
            %
            % [x,y] = C2xyz(C) returns x and y coordinates of contours in a contour
            % matrix C
            %
            % [x,y,z] = C2xyz(C) also returns corresponding z values.
            %
            %
            %% Example
            % Given a contour plot, you want to know the (x,y) coordinates of the contours,
            % as well as the z value corresponding to each contour line.
            %
            % C = contour(peaks);
            % [x,y,z] = C2xyz(C);
            %
            % This returns 1 x numberOfContourLines cells of x values and y values, and
            % their corresponding z values are given in a 1 x numberOfContourLines
            % array. If you'd like to plot a heavy black line along all of the z=0
            % contours and a dotted red line along the z = -2 contours, try this:
            %
            % hold on; % Allows plotting atop the preexisting peaks plot.
            % for n = find(z==0); % only loop through the z = 0 values.
            %     plot(x{n},y{n},'k','linewidth',2)
            % end
            %
            % for n = find(z==-2) % now loop through the z = -2 values.
            %     plot(x{n},y{n},'r:','linewidth',2)
            % end
            %
            % * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
            % Created by Chad Greene, August 2013.
            %
            % * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
            % See also contour, contourf, clabel, contour3, and C2xy.
            m(1)=1;
            n=1;
            try
                while n<length(C)
                    n=n+1;
                    m(n) = m(n-1)+C(2,m(n-1))+1;

                end
            end
            for nn = 1:n-2
                x{nn} = C(1,m(nn)+1:m(nn+1)-1);
                y{nn} = C(2,m(nn)+1:m(nn+1)-1);
                if nargout==3
                    z(nn) = C(1,m(nn));
                end
            end
        end
    end
end
