%% Create RoadRunner Scene Definition - IndiaNAV (SIH 2026)
% Programmatically generates single-lane unmarked Indian road geometry,
% static pole positions, surface pothole bounding zones, parked car slots,
% and environment plugin definitions (Rural Dirt Road, Urban Intersection).

function sceneSpec = create_roadrunner_scene(envType)
    if nargin < 1
        envType = 'UNMARKED_INDIAN_ROAD';
    end

    fprintf('====================================================\n');
    fprintf('  Generating Scene Definition: [%s]\n', envType);
    fprintf('====================================================\n');

    sceneSpec = struct();
    sceneSpec.EnvironmentType = envType;
    sceneSpec.RoadLength      = 200.0; % Total scenario track length (meters)
    sceneSpec.RoadWidth       = 3.5;   % Narrow single-lane width matching Indian streets (meters)
    sceneSpec.LaneMarkings    = false; % No painted lane lines
    sceneSpec.RoadCurvature   = 0.005; % Gentle curve radius ~200m

    %% Road Geometry Nodes (Centerline X-Y points)
    s = linspace(0, sceneSpec.RoadLength, 100);
    xCenter = s;
    yCenter = 1.5 * sin(s / 25.0); % Subtle organic road curvature
    sceneSpec.Centerline = [xCenter', yCenter'];

    % Road boundaries (Left & Right edges)
    halfW = sceneSpec.RoadWidth / 2.0;
    sceneSpec.LeftEdge  = [xCenter', (yCenter + halfW)'];
    sceneSpec.RightEdge = [xCenter', (yCenter - halfW)'];

    %% Environment-Specific Obstacles & Infrastructure
    switch upper(envType)
        case 'UNMARKED_INDIAN_ROAD'
            % Condition 1: Static Poles (Steer-around) & Potholes (Slow-down)
            % Condition 2: Parked Vehicles narrowing corridor
            sceneSpec.Poles = [
                struct('ID', 101, 'X', 45.0,  'Y',  1.0, 'Radius', 0.25, 'Height', 4.0, 'Type', 'POLE');
                struct('ID', 102, 'X', 110.0, 'Y', -0.8, 'Radius', 0.25, 'Height', 4.0, 'Type', 'POLE')
            ];
            
            sceneSpec.Potholes = [
                struct('ID', 201, 'X', 70.0,  'Y',  0.2, 'Length', 1.2, 'Width', 0.9, 'Depth', 0.12, 'Type', 'POTHOLE');
                struct('ID', 202, 'X', 145.0, 'Y', -0.3, 'Length', 1.0, 'Width', 0.8, 'Depth', 0.10, 'Type', 'POTHOLE')
            ];
            
            sceneSpec.ParkedCars = [
                struct('ID', 301, 'X', 85.0,  'Y', -1.0, 'Length', 4.2, 'Width', 1.7, 'Type', 'PARKED_CAR');
                struct('ID', 302, 'X', 130.0, 'Y',  1.1, 'Length', 4.0, 'Width', 1.7, 'Type', 'PARKED_CAR')
            ];

        case 'RURAL_UNPAVED_DIRT'
            % Plugin Environment: Dirt road, loose edges, mud patches, livestock
            sceneSpec.SurfaceType = 'Dirt_Gravel';
            sceneSpec.FrictionCoefficient = 0.55;
            sceneSpec.Poles = [
                struct('ID', 101, 'X', 50.0, 'Y', 1.2, 'Radius', 0.3, 'Height', 3.0, 'Type', 'WOODEN_POST')
            ];
            sceneSpec.Potholes = [
                struct('ID', 201, 'X', 65.0, 'Y', 0.0, 'Length', 2.0, 'Width', 1.5, 'Depth', 0.2, 'Type', 'MUD_RUT')
            ];
            sceneSpec.ParkedCars = [
                struct('ID', 301, 'X', 100.0, 'Y', -0.9, 'Length', 3.8, 'Width', 1.6, 'Type', 'TRACTOR')
            ];

        case 'URBAN_INTERSECTION'
            % Plugin Environment: T-Junction / 4-way intersection without lane lines
            sceneSpec.SurfaceType = 'Asphalt';
            sceneSpec.IntersectionType = 'Unsignalized_T_Junction';
            sceneSpec.IntersectionCenter = [100.0, 0.0];
            sceneSpec.Poles = [];
            sceneSpec.Potholes = [];
            sceneSpec.ParkedCars = [
                struct('ID', 301, 'X', 75.0, 'Y', 1.2, 'Length', 4.0, 'Width', 1.7, 'Type', 'PARKED_CAR')
            ];

        otherwise
            error('Unknown environment type: %s', envType);
    end

    %% Export RoadRunner Scene XML Descriptor Metadata
    xmlFilename = fullfile('scene', 'unmarked_indian_road.rrscene');
    fid = fopen(xmlFilename, 'w');
    if fid ~= -1
        fprintf(fid, '<?xml version="1.0" encoding="UTF-8"?>\n');
        fprintf(fid, '<RoadRunnerScene name="Unmarked_Indian_Single_Lane">\n');
        fprintf(fid, '  <Metadata environment="%s" width="%.2f" laneMarkings="false"/>\n', envType, sceneSpec.RoadWidth);
        fprintf(fid, '  <Centerline length="%.1f" points="%d"/>\n', sceneSpec.RoadLength, size(sceneSpec.Centerline, 1));
        fprintf(fid, '  <Obstacles poles="%d" potholes="%d" parkedCars="%d"/>\n', ...
            length(sceneSpec.Poles), length(sceneSpec.Potholes), length(sceneSpec.ParkedCars));
        fprintf(fid, '</RoadRunnerScene>\n');
        fclose(fid);
        fprintf('✓ Saved RoadRunner metadata file: %s\n', xmlFilename);
    end
end
