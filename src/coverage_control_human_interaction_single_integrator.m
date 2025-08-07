clc
clear
close all

global hd_pos
global SIGMA_11 SIGMA_22 mu
SIGMA_11 = 0.5;
SIGMA_22 = 1;

PLOT_VORONOI = true;
PLOT_CENTROID = false;
PLOT_DENSITY = true;
UNI_SI = 'si'; % 'uni' or 'si'
DENSITY = ''; % '' or 'uniform'

N = 50;
DT = 0.01;
T = 10;
robots = cell(1,N);
for i = 1 : N
    if strcmpi(UNI_SI,'uni')
        robots{i} = Unicycle('width',0.1,...
            'length',0.1,...
            'initialState',[-1;0.1;0]+0.001*rand(3,1),...
            'simulationTimeStep',DT);
    elseif strcmpi(UNI_SI,'si')
        robots{i} = SingleIntegrator('width',0.05,...
            'initialState',[-1;0.1;0]+0.01*rand(3,1),...
            'simulationTimeStep',DT);
    end
end
environment = 4 * [cos(linspace(0,2*pi,7)); sin(linspace(0,2*pi,7))];

mu = [0; 0];
phi = phi_handle_param(mu);

s = Swarm('robots',robots,...
    'environment',environment,...
    'densityFunction',phi);

s.plotFigure()
if PLOT_DENSITY
    s.plotDensity(linspace(0,1,12))
end
s.plotEnvironment('LineWidth', 5, 'Color', [0 0 0])

set(s.hG.figure, 'WindowButtonMotionFcn', @mouseMove);
hq = quiver(nan, nan, nan, nan, 'LineWidth', 3, 'MaxHeadSize', 10, 'Color', 'r');

for t = 0 : DT : T
    tic

    % mu_dot = [0.1; 0];
    if ~isempty(hd_pos)
        mu_dot = 4 * (hd_pos - mu);
    else
        mu_dot = [0; 0];
    end
    mu = mu + mu_dot * DT;
    s.phi = phi_handle_param(mu);
    if mod(t, 10 * DT) == 0
        s.plotDensity(linspace(0,1,12))
    end

    q = s.getPoses();

    [G,A,VC] = s.coverageControl();

    f = s.evaluateHapticFeedback(q, VC);

    s.goToPoints(G, 100)

    s.plotRobots([0.933,0.698,0.067],'EdgeColor','none')
    if PLOT_VORONOI
        s.plotVoronoiCells(VC,'Color',[0.25 0.25 0.25],'LineWidth',2)
    end
    if PLOT_CENTROID
        s.plotCentroids(G,'.','Color',[0.5 0.5 0.5],'MarkerSize',20)
    end

    hq.XData = hd_pos(1);
    hq.YData = hd_pos(2);
    hq.UData = -10 * f(1);
    hq.VData = -10 * f(2);

    drawnow limitrate
    pause(DT-toc)
end

function phi_handle = phi_handle_param(mu)
    global SIGMA_11 SIGMA_22
    function phi_value = phi_function(x,y)
        phi_value = exp( -( (x-mu(1)) .^ 2 / SIGMA_11 + (y-mu(2)) .^ 2 / SIGMA_22) );
    end
phi_handle = @phi_function;
end

function mouseMove(object, eventdata)
global hd_pos
C = get (gca, 'CurrentPoint');
hd_pos = [C(1,1); C(1,2)];
end