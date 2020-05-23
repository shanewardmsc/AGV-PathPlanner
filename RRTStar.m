%-----------------------------------------------------------------------------------------%
%          Project: RRT* Algorithm for AGV Path Planning Optimization using V-REP         %
%          Author:  Shane Ward                                                            %
%          Version: 0                                                                     %                                                                   
%-----------------------------------------------------------------------------------------%

% Initialise variables to zero
clearvars
close all

% Find the VREP library
vrep=remApi('remoteApi');

% Close all open connections just in case
vrep.simxFinish(-1);

% Create a new simualation and connect
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

% If a connection exists
if (clientID>-1)
    disp('Connected to V-REP...')
    
    % AGV Handle
    [~,AGV]=vrep.simxGetObjectHandle(clientID,'AGV',vrep.simx_opmode_blocking);
    
    % Packed Pallet Handle
    [~,PackedPallet]=vrep.simxGetObjectHandle(clientID,'PalletConfiguration',vrep.simx_opmode_blocking);
    
    % Read AGV position (X,Y)
    [~,AGV_Position]=vrep.simxGetObjectPosition(clientID,AGV,-1,vrep.simx_opmode_blocking);
    current_x_AGV = (round(AGV_Position(1),1));
    current_y_AGV = (round(AGV_Position(2),1));
    
    % Read Packed Pallet position (X,Y)
    [~,Pallet_Position]=vrep.simxGetObjectPosition(clientID,PackedPallet,-1,vrep.simx_opmode_blocking);
    current_x_pallet = (round(Pallet_Position(1),1));
    current_y_pallet = (round(Pallet_Position(2),1));
    
    % V-REP Scene Maximum X,Y Coordinates
    x_max = 6.5;
    y_max = 4.5;
    
    % V-REP Scene Obstacles
    obstacle_1 = [-0.1,3.525,3.6,0.2];
    obstacle_2 = [3.475, 1, 0.2, 5.025];
    obstacle_3 = [-0.1,6.025,3.8,0.2];
    obstacle_4 = [4.55,-0.1,0.2,3.475];
    obstacle_5 = [4.55,4.525,5.45,0.2];
    obstacle_6 = [4.55,6.025,5.45,0.2];
    
    % Obstacle Bounding Box Points
    BB_ob_1 = [obstacle_1(1) obstacle_1(2) obstacle_1(1)+obstacle_1(3) obstacle_1(2)+obstacle_1(4)];
    BB_ob_2 = [obstacle_2(1) obstacle_2(2) obstacle_2(1)+obstacle_2(3) obstacle_2(2)+obstacle_2(4)]; 
    BB_ob_3 = [obstacle_3(1) obstacle_3(2) obstacle_3(1)+obstacle_3(3) obstacle_3(2)+obstacle_3(4)]; 
    BB_ob_4 = [obstacle_4(1) obstacle_4(2) obstacle_4(1)+obstacle_4(3) obstacle_4(2)+obstacle_4(4)]; 
    BB_ob_5 = [obstacle_5(1) obstacle_5(2) obstacle_5(1)+obstacle_5(3) obstacle_5(2)+obstacle_5(4)]; 
    BB_ob_6 = [obstacle_6(1) obstacle_6(2) obstacle_6(1)+obstacle_6(3) obstacle_6(2)+obstacle_6(4)]; 
    
    % Obstacle 1 Corner Points
    obstacle1_Corner_1 = [BB_ob_1(1),BB_ob_1(2)];   obstacle1_Corner_2 = [BB_ob_1(1),BB_ob_1(4)];
    obstacle1_Corner_3 = [BB_ob_1(3),BB_ob_1(2)];   obstacle1_Corner_4 = [BB_ob_1(3),BB_ob_1(4)];
    
    % Obstacle 2 Corner Points
    obstacle2_Corner_1 = [BB_ob_2(1),BB_ob_2(2)];   obstacle2_Corner_2 = [BB_ob_2(1),BB_ob_2(4)];
    obstacle2_Corner_3 = [BB_ob_2(3),BB_ob_2(2)];   obstacle2_Corner_4 = [BB_ob_2(3),BB_ob_2(4)]; 
    
    % Obstacle 3 Corner Points
    obstacle3_Corner_1 = [BB_ob_3(1),BB_ob_3(2)];   obstacle3_Corner_2 = [BB_ob_3(1),BB_ob_3(4)];
    obstacle3_Corner_3 = [BB_ob_3(3),BB_ob_3(2)];   obstacle3_Corner_4 = [BB_ob_3(3),BB_ob_3(4)];
    
    % Obstacle 4 Corner Points
    obstacle4_Corner_1 = [BB_ob_4(1),BB_ob_4(2)];   obstacle4_Corner_2 = [BB_ob_4(1),BB_ob_4(4)];
    obstacle4_Corner_3 = [BB_ob_4(3),BB_ob_4(2)];   obstacle4_Corner_4 = [BB_ob_4(3),BB_ob_4(4)];
    
    % Obstacle 5 Corner Points
    obstacle5_Corner_1 = [BB_ob_5(1),BB_ob_5(2)];   obstacle5_Corner_2 = [BB_ob_5(1),BB_ob_5(4)];
    obstacle5_Corner_3 = [BB_ob_5(3),BB_ob_5(2)];   obstacle5_Corner_4 = [BB_ob_5(3),BB_ob_5(4)];
    
    % Obstacle 6 Corner Points
    obstacle6_Corner_1 = [BB_ob_6(1),BB_ob_6(2)];   obstacle6_Corner_2 = [BB_ob_6(1),BB_ob_6(4)];
    obstacle6_Corner_3 = [BB_ob_6(3),BB_ob_6(2)];   obstacle6_Corner_4 = [BB_ob_6(3),BB_ob_6(4)];
    
    % Adjustable Algorithm Variables
    step_Size = 5;
    num_States = 800;
    
    % Start Location (AGV Position)
    x_start.coord = [current_x_AGV current_y_AGV];
    x_start.cost = 0;
    x_start.parent = 0;
    
    % Target Location (AGV Position)
    x_target.coord = [current_x_pallet current_y_pallet];
    x_target.cost = 0;
    
    % Plot initial Graph with Obstacles 
    states(1) = x_start;
    figure('Name','RRT* Path Planner for VREP','NumberTitle','off');
    %axis([0 x_max 0 y_max])
    axis([0 10 0 10])
    rectangle('Position',obstacle_1,'FaceColor',[0 0.4470 0.7410])
    rectangle('Position',obstacle_2,'FaceColor',[0 0.4470 0.7410])
    rectangle('Position',obstacle_3,'FaceColor',[0 0.4470 0.7410])
    rectangle('Position',obstacle_4,'FaceColor',[0 0.4470 0.7410])
    rectangle('Position',obstacle_5,'FaceColor',[0 0.4470 0.7410])
    rectangle('Position',obstacle_6,'FaceColor',[0 0.4470 0.7410])
    hold on
    
    %Start Algorithm Timer
    tic;
    
    % Main Algorithm Loop
    for i = 1:1:num_States
        % Create & Plot Randomized Nodes
        x_random = [((rand(1)*x_max)) ((rand(1)*y_max))];
        plot(x_random(1), x_random(2), '.', 'Color',  [0.6350 0.0780 0.1840])
        
        % Break if goal node is already reached
        for j = 1:1:length(states)
            if states(j).coord == x_target.coord
                break
            end
        end
        
        % Pick the closest node from existing list to branch out from
        ndist = [];
        for j = 1:1:length(states)
            n = states(j);
            node_Distance = sqrt((n.coord(1)-x_random(1))^2 + (n.coord(2)-x_random(2))^2);
            tmp = node_Distance;
            ndist = [ndist tmp];
        end
        [val, idx] = min(ndist);
        x_near = states(idx);
        
        % Create Tree Branch from x_near to x_random using step_Size
        x_near_to_x_rand = [0 0];
   
        if val >= step_Size
            node_dist = sqrt((x_random(1)-x_near.coord(1))^2 + (x_random(2)-x_near.coord(2))^2);
            x_near_to_x_rand(1) = x_near.coord(1) + ((x_random(1)-x_near.coord(1))*step_Size)/node_dist;
            x_near_to_x_rand(2) = x_near.coord(2) + ((x_random(2)-x_near.coord(2))*step_Size)/node_dist;
        else
            x_near_to_x_rand(1) = x_random(1);
            x_near_to_x_rand(2) = x_random(2);
        end
        
        x_newNode.coord = [x_near_to_x_rand(1), x_near_to_x_rand(2)];
        
        % NO COLLISION CHECK
        x_A = [x_near.coord(1) x_near.coord(2)];
        x_B = [x_random(1) x_random(2)];
        
        wall1_check1 = intersect(x_A,obstacle1_Corner_1,obstacle1_Corner_2) ~=...
            intersect(x_B,obstacle1_Corner_1,obstacle1_Corner_2) && intersect(x_A,x_B,obstacle1_Corner_1) ~= intersect(x_A,x_B,obstacle1_Corner_2);
        wall1_check2 = intersect(x_A,obstacle1_Corner_1,obstacle1_Corner_3) ~=...
            intersect(x_B,obstacle1_Corner_1,obstacle1_Corner_3) && intersect(x_A,x_B,obstacle1_Corner_1) ~= intersect(x_A,x_B,obstacle1_Corner_3);
        wall1_check3 = intersect(x_A,obstacle1_Corner_4,obstacle1_Corner_3) ~=...
            intersect(x_B,obstacle1_Corner_4,obstacle1_Corner_3) && intersect(x_A,x_B,obstacle1_Corner_4) ~= intersect(x_A,x_B,obstacle1_Corner_3);
        wall1_check4 = intersect(x_A,obstacle1_Corner_4,obstacle1_Corner_2) ~=...
            intersect(x_B,obstacle1_Corner_4,obstacle1_Corner_2) && intersect(x_A,x_B,obstacle1_Corner_4) ~= intersect(x_A,x_B,obstacle1_Corner_2);
        
        wall2_check1 = intersect(x_A,obstacle2_Corner_1,obstacle2_Corner_2) ~=...
            intersect(x_B,obstacle2_Corner_1,obstacle2_Corner_2) && intersect(x_A,x_B,obstacle2_Corner_1) ~= intersect(x_A,x_B,obstacle2_Corner_2);
        wall2_check2 = intersect(x_A,obstacle2_Corner_1,obstacle2_Corner_3) ~=...
            intersect(x_B,obstacle2_Corner_1,obstacle2_Corner_3) && intersect(x_A,x_B,obstacle2_Corner_1) ~= intersect(x_A,x_B,obstacle2_Corner_3);
        wall2_check3 = intersect(x_A,obstacle2_Corner_4,obstacle2_Corner_3) ~=...
            intersect(x_B,obstacle2_Corner_4,obstacle2_Corner_3) && intersect(x_A,x_B,obstacle2_Corner_4) ~= intersect(x_A,x_B,obstacle2_Corner_3);
        wall2_check4 = intersect(x_A,obstacle2_Corner_4,obstacle2_Corner_2) ~=...
            intersect(x_B,obstacle2_Corner_4,obstacle2_Corner_2) && intersect(x_A,x_B,obstacle2_Corner_4) ~= intersect(x_A,x_B,obstacle2_Corner_2);
        
        wall3_check1 = intersect(x_A,obstacle3_Corner_1,obstacle3_Corner_2) ~=...
            intersect(x_B,obstacle3_Corner_1,obstacle3_Corner_2) && intersect(x_A,x_B,obstacle3_Corner_1) ~= intersect(x_A,x_B,obstacle3_Corner_2);
        wall3_check2 = intersect(x_A,obstacle3_Corner_1,obstacle3_Corner_3) ~=...
            intersect(x_B,obstacle3_Corner_1,obstacle3_Corner_3) && intersect(x_A,x_B,obstacle3_Corner_1) ~= intersect(x_A,x_B,obstacle3_Corner_3);
        wall3_check3 = intersect(x_A,obstacle3_Corner_4,obstacle3_Corner_3) ~=...
            intersect(x_B,obstacle3_Corner_4,obstacle3_Corner_3) && intersect(x_A,x_B,obstacle3_Corner_4) ~= intersect(x_A,x_B,obstacle3_Corner_3);
        wall3_check4 = intersect(x_A,obstacle3_Corner_4,obstacle3_Corner_2) ~=...
            intersect(x_B,obstacle3_Corner_4,obstacle3_Corner_2) && intersect(x_A,x_B,obstacle3_Corner_4) ~= intersect(x_A,x_B,obstacle3_Corner_2);
        
        wall4_check1 = intersect(x_A,obstacle4_Corner_1,obstacle4_Corner_2) ~=...
            intersect(x_B,obstacle4_Corner_1,obstacle4_Corner_2) && intersect(x_A,x_B,obstacle4_Corner_1) ~= intersect(x_A,x_B,obstacle4_Corner_2);
        wall4_check2 = intersect(x_A,obstacle4_Corner_1,obstacle4_Corner_3) ~=...
            intersect(x_B,obstacle4_Corner_1,obstacle4_Corner_3) && intersect(x_A,x_B,obstacle4_Corner_1) ~= intersect(x_A,x_B,obstacle4_Corner_3);
        wall4_check3 = intersect(x_A,obstacle4_Corner_4,obstacle4_Corner_3) ~=...
            intersect(x_B,obstacle4_Corner_4,obstacle4_Corner_3) && intersect(x_A,x_B,obstacle4_Corner_4) ~= intersect(x_A,x_B,obstacle4_Corner_3);
        wall4_check4 = intersect(x_A,obstacle4_Corner_4,obstacle4_Corner_2) ~=...
            intersect(x_B,obstacle4_Corner_4,obstacle4_Corner_2) && intersect(x_A,x_B,obstacle4_Corner_4) ~= intersect(x_A,x_B,obstacle4_Corner_2);
        
        wall5_check1 = intersect(x_A,obstacle5_Corner_1,obstacle5_Corner_2) ~=...
            intersect(x_B,obstacle5_Corner_1,obstacle5_Corner_2) && intersect(x_A,x_B,obstacle5_Corner_1) ~= intersect(x_A,x_B,obstacle5_Corner_2);
        wall5_check2 = intersect(x_A,obstacle5_Corner_1,obstacle5_Corner_3) ~=...
            intersect(x_B,obstacle5_Corner_1,obstacle5_Corner_3) && intersect(x_A,x_B,obstacle5_Corner_1) ~= intersect(x_A,x_B,obstacle5_Corner_3);
        wall5_check3 = intersect(x_A,obstacle5_Corner_4,obstacle5_Corner_3) ~=...
            intersect(x_B,obstacle5_Corner_4,obstacle5_Corner_3) && intersect(x_A,x_B,obstacle5_Corner_4) ~= intersect(x_A,x_B,obstacle5_Corner_3);
        wall5_check4 = intersect(x_A,obstacle5_Corner_4,obstacle5_Corner_2) ~=...
            intersect(x_B,obstacle5_Corner_4,obstacle5_Corner_2) && intersect(x_A,x_B,obstacle5_Corner_4) ~= intersect(x_A,x_B,obstacle5_Corner_2);
        
        wall6_check1 = intersect(x_A,obstacle4_Corner_1,obstacle6_Corner_2) ~=...
            intersect(x_B,obstacle6_Corner_1,obstacle6_Corner_2) && intersect(x_A,x_B,obstacle6_Corner_1) ~= intersect(x_A,x_B,obstacle6_Corner_2);
        wall6_check2 = intersect(x_A,obstacle4_Corner_1,obstacle6_Corner_3) ~=...
            intersect(x_B,obstacle6_Corner_1,obstacle6_Corner_3) && intersect(x_A,x_B,obstacle6_Corner_1) ~= intersect(x_A,x_B,obstacle6_Corner_3);
        wall6_check3 = intersect(x_A,obstacle4_Corner_4,obstacle6_Corner_3) ~=...
            intersect(x_B,obstacle6_Corner_4,obstacle6_Corner_3) && intersect(x_A,x_B,obstacle6_Corner_4) ~= intersect(x_A,x_B,obstacle6_Corner_3);
        wall6_check4 = intersect(x_A,obstacle4_Corner_4,obstacle6_Corner_2) ~=...
            intersect(x_B,obstacle6_Corner_4,obstacle6_Corner_2) && intersect(x_A,x_B,obstacle6_Corner_4) ~= intersect(x_A,x_B,obstacle6_Corner_2);
        
        if(wall1_check1==0 && wall1_check2==0 && wall1_check3==0 && wall1_check4==0 &&...
                wall2_check1==0 && wall2_check2==0 && wall2_check3==0 && wall2_check4==0 &&...
                wall3_check1==0 && wall3_check2==0 && wall3_check3==0 && wall3_check4==0 &&...
                wall4_check1==0 && wall4_check2==0 && wall4_check3==0 && wall4_check4==0 &&...
                wall5_check1==0 && wall5_check2==0 && wall5_check3==0 && wall5_check4==0 &&...
                wall6_check1==0 && wall6_check2==0 && wall6_check3==0 && wall6_check4==0)
            totalCollisionFree = 1;
        else
            totalCollisionFree = 0;
        end

        if(totalCollisionFree==1)
            line([x_near.coord(1), x_newNode.coord(1)], [x_near.coord(2), x_newNode.coord(2)], 'Color', 'k', 'LineWidth', 2);
            drawnow
            hold on
            node_dis = sqrt((x_newNode.coord(1)-x_near.coord(1))^2 + (x_newNode.coord(2)-x_near.coord(2))^2);
            x_newNode.cost = node_dis + x_near.cost;
            
            % Within a radius of r, find all existing states
            x_nearest = [];
            r = 5;
            neighbor_count = 1; 
            for j = 1:1:length(states)
                
                % NO COLLISION CHECK
                x_A = [x_newNode.coord(1) x_newNode.coord(2)];
                x_B = [states(j).coord(1) states(j).coord(2)];
                
                wall1_check1 = intersect(x_A,obstacle1_Corner_1,obstacle1_Corner_2) ~=...
                    intersect(x_B,obstacle1_Corner_1,obstacle1_Corner_2) && intersect(x_A,x_B,obstacle1_Corner_1) ~= intersect(x_A,x_B,obstacle1_Corner_2);
                wall1_check2 = intersect(x_A,obstacle1_Corner_1,obstacle1_Corner_3) ~=...
                    intersect(x_B,obstacle1_Corner_1,obstacle1_Corner_3) && intersect(x_A,x_B,obstacle1_Corner_1) ~= intersect(x_A,x_B,obstacle1_Corner_3);
                wall1_check3 = intersect(x_A,obstacle1_Corner_4,obstacle1_Corner_3) ~=...
                    intersect(x_B,obstacle1_Corner_4,obstacle1_Corner_3) && intersect(x_A,x_B,obstacle1_Corner_4) ~= intersect(x_A,x_B,obstacle1_Corner_3);
                wall1_check4 = intersect(x_A,obstacle1_Corner_4,obstacle1_Corner_2) ~=...
                    intersect(x_B,obstacle1_Corner_4,obstacle1_Corner_2) && intersect(x_A,x_B,obstacle1_Corner_4) ~= intersect(x_A,x_B,obstacle1_Corner_2);
                
                wall2_check1 = intersect(x_A,obstacle2_Corner_1,obstacle2_Corner_2) ~=...
                    intersect(x_B,obstacle2_Corner_1,obstacle2_Corner_2) && intersect(x_A,x_B,obstacle2_Corner_1) ~= intersect(x_A,x_B,obstacle2_Corner_2);
                wall2_check2 = intersect(x_A,obstacle2_Corner_1,obstacle2_Corner_3) ~=...
                    intersect(x_B,obstacle2_Corner_1,obstacle2_Corner_3) && intersect(x_A,x_B,obstacle2_Corner_1) ~= intersect(x_A,x_B,obstacle2_Corner_3);
                wall2_check3 = intersect(x_A,obstacle2_Corner_4,obstacle2_Corner_3) ~=...
                    intersect(x_B,obstacle2_Corner_4,obstacle2_Corner_3) && intersect(x_A,x_B,obstacle2_Corner_4) ~= intersect(x_A,x_B,obstacle2_Corner_3);
                wall2_check4 = intersect(x_A,obstacle2_Corner_4,obstacle2_Corner_2) ~=...
                    intersect(x_B,obstacle2_Corner_4,obstacle2_Corner_2) && intersect(x_A,x_B,obstacle2_Corner_4) ~= intersect(x_A,x_B,obstacle2_Corner_2);
                
                wall3_check1 = intersect(x_A,obstacle3_Corner_1,obstacle3_Corner_2) ~=...
                    intersect(x_B,obstacle3_Corner_1,obstacle3_Corner_2) && intersect(x_A,x_B,obstacle3_Corner_1) ~= intersect(x_A,x_B,obstacle3_Corner_2);
                wall3_check2 = intersect(x_A,obstacle3_Corner_1,obstacle3_Corner_3) ~=...
                    intersect(x_B,obstacle3_Corner_1,obstacle3_Corner_3) && intersect(x_A,x_B,obstacle3_Corner_1) ~= intersect(x_A,x_B,obstacle3_Corner_3);
                wall3_check3 = intersect(x_A,obstacle3_Corner_4,obstacle3_Corner_3) ~=...
                    intersect(x_B,obstacle3_Corner_4,obstacle3_Corner_3) && intersect(x_A,x_B,obstacle3_Corner_4) ~= intersect(x_A,x_B,obstacle3_Corner_3);
                wall3_check4 = intersect(x_A,obstacle3_Corner_4,obstacle3_Corner_2) ~=...
                    intersect(x_B,obstacle3_Corner_4,obstacle3_Corner_2) && intersect(x_A,x_B,obstacle3_Corner_4) ~= intersect(x_A,x_B,obstacle3_Corner_2);
                
                wall4_check1 = intersect(x_A,obstacle4_Corner_1,obstacle4_Corner_2) ~=...
                    intersect(x_B,obstacle4_Corner_1,obstacle4_Corner_2) && intersect(x_A,x_B,obstacle4_Corner_1) ~= intersect(x_A,x_B,obstacle4_Corner_2);
                wall4_check2 = intersect(x_A,obstacle4_Corner_1,obstacle4_Corner_3) ~=...
                    intersect(x_B,obstacle4_Corner_1,obstacle4_Corner_3) && intersect(x_A,x_B,obstacle4_Corner_1) ~= intersect(x_A,x_B,obstacle4_Corner_3);
                wall4_check3 = intersect(x_A,obstacle4_Corner_4,obstacle4_Corner_3) ~=...
                    intersect(x_B,obstacle4_Corner_4,obstacle4_Corner_3) && intersect(x_A,x_B,obstacle4_Corner_4) ~= intersect(x_A,x_B,obstacle4_Corner_3);
                wall4_check4 = intersect(x_A,obstacle4_Corner_4,obstacle4_Corner_2) ~=...
                    intersect(x_B,obstacle4_Corner_4,obstacle4_Corner_2) && intersect(x_A,x_B,obstacle4_Corner_4) ~= intersect(x_A,x_B,obstacle4_Corner_2);
                
                wall5_check1 = intersect(x_A,obstacle5_Corner_1,obstacle5_Corner_2) ~=...
                    intersect(x_B,obstacle5_Corner_1,obstacle5_Corner_2) && intersect(x_A,x_B,obstacle5_Corner_1) ~= intersect(x_A,x_B,obstacle5_Corner_2);
                wall5_check2 = intersect(x_A,obstacle5_Corner_1,obstacle5_Corner_3) ~=...
                    intersect(x_B,obstacle5_Corner_1,obstacle5_Corner_3) && intersect(x_A,x_B,obstacle5_Corner_1) ~= intersect(x_A,x_B,obstacle5_Corner_3);
                wall5_check3 = intersect(x_A,obstacle5_Corner_4,obstacle5_Corner_3) ~=...
                    intersect(x_B,obstacle5_Corner_4,obstacle5_Corner_3) && intersect(x_A,x_B,obstacle5_Corner_4) ~= intersect(x_A,x_B,obstacle5_Corner_3);
                wall5_check4 = intersect(x_A,obstacle5_Corner_4,obstacle5_Corner_2) ~=...
                    intersect(x_B,obstacle5_Corner_4,obstacle5_Corner_2) && intersect(x_A,x_B,obstacle5_Corner_4) ~= intersect(x_A,x_B,obstacle5_Corner_2);
                
                wall6_check1 = intersect(x_A,obstacle4_Corner_1,obstacle6_Corner_2) ~=...
                    intersect(x_B,obstacle6_Corner_1,obstacle6_Corner_2) && intersect(x_A,x_B,obstacle6_Corner_1) ~= intersect(x_A,x_B,obstacle6_Corner_2);
                wall6_check2 = intersect(x_A,obstacle4_Corner_1,obstacle6_Corner_3) ~=...
                    intersect(x_B,obstacle6_Corner_1,obstacle6_Corner_3) && intersect(x_A,x_B,obstacle6_Corner_1) ~= intersect(x_A,x_B,obstacle6_Corner_3);
                wall6_check3 = intersect(x_A,obstacle4_Corner_4,obstacle6_Corner_3) ~=...
                    intersect(x_B,obstacle6_Corner_4,obstacle6_Corner_3) && intersect(x_A,x_B,obstacle6_Corner_4) ~= intersect(x_A,x_B,obstacle6_Corner_3);
                wall6_check4 = intersect(x_A,obstacle4_Corner_4,obstacle6_Corner_2) ~=...
                    intersect(x_B,obstacle6_Corner_4,obstacle6_Corner_2) && intersect(x_A,x_B,obstacle6_Corner_4) ~= intersect(x_A,x_B,obstacle6_Corner_2);
                
                if(wall1_check1==0 && wall1_check2==0 && wall1_check3==0 && wall1_check4==0 &&...
                        wall2_check1==0 && wall2_check2==0 && wall2_check3==0 && wall2_check4==0 &&...
                        wall3_check1==0 && wall3_check2==0 && wall3_check3==0 && wall3_check4==0 &&...
                        wall4_check1==0 && wall4_check2==0 && wall4_check3==0 && wall4_check4==0 &&...
                        wall5_check1==0 && wall5_check2==0 && wall5_check3==0 && wall5_check4==0 &&...
                        wall6_check1==0 && wall6_check2==0 && wall6_check3==0 && wall6_check4==0)
                    totalCollisionFree = 1;
                else
                    totalCollisionFree = 0;
                end
                
                node_distan = sqrt((states(j).coord(1)-x_newNode.coord(1))^2 + (states(j).coord(2)-x_newNode.coord(2))^2);
                
                if (totalCollisionFree==1) &&...
                        node_distan <= r
                    x_nearest(neighbor_count).coord = states(j).coord;
                    x_nearest(neighbor_count).cost = states(j).cost;
                    neighbor_count = neighbor_count+1;
                end
            end
            
            % Initialize cost to currently known value
            x_min = x_near;
            C_min = x_newNode.cost;
            
            % Iterate through all nearest neighbors to find alternate lower
            % cost paths
            
            for k = 1:1:length(x_nearest)
                
                % NO COLLISION CHECK
                x_A = [x_newNode.coord(1) x_newNode.coord(2)];
                x_B = [x_nearest(k).coord(1) x_nearest(k).coord(2)];
                
                wall1_check1 = intersect(x_A,obstacle1_Corner_1,obstacle1_Corner_2) ~=...
                    intersect(x_B,obstacle1_Corner_1,obstacle1_Corner_2) && intersect(x_A,x_B,obstacle1_Corner_1) ~= intersect(x_A,x_B,obstacle1_Corner_2);
                wall1_check2 = intersect(x_A,obstacle1_Corner_1,obstacle1_Corner_3) ~=...
                    intersect(x_B,obstacle1_Corner_1,obstacle1_Corner_3) && intersect(x_A,x_B,obstacle1_Corner_1) ~= intersect(x_A,x_B,obstacle1_Corner_3);
                wall1_check3 = intersect(x_A,obstacle1_Corner_4,obstacle1_Corner_3) ~=...
                    intersect(x_B,obstacle1_Corner_4,obstacle1_Corner_3) && intersect(x_A,x_B,obstacle1_Corner_4) ~= intersect(x_A,x_B,obstacle1_Corner_3);
                wall1_check4 = intersect(x_A,obstacle1_Corner_4,obstacle1_Corner_2) ~=...
                    intersect(x_B,obstacle1_Corner_4,obstacle1_Corner_2) && intersect(x_A,x_B,obstacle1_Corner_4) ~= intersect(x_A,x_B,obstacle1_Corner_2);
                
                wall2_check1 = intersect(x_A,obstacle2_Corner_1,obstacle2_Corner_2) ~=...
                    intersect(x_B,obstacle2_Corner_1,obstacle2_Corner_2) && intersect(x_A,x_B,obstacle2_Corner_1) ~= intersect(x_A,x_B,obstacle2_Corner_2);
                wall2_check2 = intersect(x_A,obstacle2_Corner_1,obstacle2_Corner_3) ~=...
                    intersect(x_B,obstacle2_Corner_1,obstacle2_Corner_3) && intersect(x_A,x_B,obstacle2_Corner_1) ~= intersect(x_A,x_B,obstacle2_Corner_3);
                wall2_check3 = intersect(x_A,obstacle2_Corner_4,obstacle2_Corner_3) ~=...
                    intersect(x_B,obstacle2_Corner_4,obstacle2_Corner_3) && intersect(x_A,x_B,obstacle2_Corner_4) ~= intersect(x_A,x_B,obstacle2_Corner_3);
                wall2_check4 = intersect(x_A,obstacle2_Corner_4,obstacle2_Corner_2) ~=...
                    intersect(x_B,obstacle2_Corner_4,obstacle2_Corner_2) && intersect(x_A,x_B,obstacle2_Corner_4) ~= intersect(x_A,x_B,obstacle2_Corner_2);
                
                wall3_check1 = intersect(x_A,obstacle3_Corner_1,obstacle3_Corner_2) ~=...
                    intersect(x_B,obstacle3_Corner_1,obstacle3_Corner_2) && intersect(x_A,x_B,obstacle3_Corner_1) ~= intersect(x_A,x_B,obstacle3_Corner_2);
                wall3_check2 = intersect(x_A,obstacle3_Corner_1,obstacle3_Corner_3) ~=...
                    intersect(x_B,obstacle3_Corner_1,obstacle3_Corner_3) && intersect(x_A,x_B,obstacle3_Corner_1) ~= intersect(x_A,x_B,obstacle3_Corner_3);
                wall3_check3 = intersect(x_A,obstacle3_Corner_4,obstacle3_Corner_3) ~=...
                    intersect(x_B,obstacle3_Corner_4,obstacle3_Corner_3) && intersect(x_A,x_B,obstacle3_Corner_4) ~= intersect(x_A,x_B,obstacle3_Corner_3);
                wall3_check4 = intersect(x_A,obstacle3_Corner_4,obstacle3_Corner_2) ~=...
                    intersect(x_B,obstacle3_Corner_4,obstacle3_Corner_2) && intersect(x_A,x_B,obstacle3_Corner_4) ~= intersect(x_A,x_B,obstacle3_Corner_2);
                
                wall4_check1 = intersect(x_A,obstacle4_Corner_1,obstacle4_Corner_2) ~=...
                    intersect(x_B,obstacle4_Corner_1,obstacle4_Corner_2) && intersect(x_A,x_B,obstacle4_Corner_1) ~= intersect(x_A,x_B,obstacle4_Corner_2);
                wall4_check2 = intersect(x_A,obstacle4_Corner_1,obstacle4_Corner_3) ~=...
                    intersect(x_B,obstacle4_Corner_1,obstacle4_Corner_3) && intersect(x_A,x_B,obstacle4_Corner_1) ~= intersect(x_A,x_B,obstacle4_Corner_3);
                wall4_check3 = intersect(x_A,obstacle4_Corner_4,obstacle4_Corner_3) ~=...
                    intersect(x_B,obstacle4_Corner_4,obstacle4_Corner_3) && intersect(x_A,x_B,obstacle4_Corner_4) ~= intersect(x_A,x_B,obstacle4_Corner_3);
                wall4_check4 = intersect(x_A,obstacle4_Corner_4,obstacle4_Corner_2) ~=...
                    intersect(x_B,obstacle4_Corner_4,obstacle4_Corner_2) && intersect(x_A,x_B,obstacle4_Corner_4) ~= intersect(x_A,x_B,obstacle4_Corner_2);
                
                wall5_check1 = intersect(x_A,obstacle5_Corner_1,obstacle5_Corner_2) ~=...
                    intersect(x_B,obstacle5_Corner_1,obstacle5_Corner_2) && intersect(x_A,x_B,obstacle5_Corner_1) ~= intersect(x_A,x_B,obstacle5_Corner_2);
                wall5_check2 = intersect(x_A,obstacle5_Corner_1,obstacle5_Corner_3) ~=...
                    intersect(x_B,obstacle5_Corner_1,obstacle5_Corner_3) && intersect(x_A,x_B,obstacle5_Corner_1) ~= intersect(x_A,x_B,obstacle5_Corner_3);
                wall5_check3 = intersect(x_A,obstacle5_Corner_4,obstacle5_Corner_3) ~=...
                    intersect(x_B,obstacle5_Corner_4,obstacle5_Corner_3) && intersect(x_A,x_B,obstacle5_Corner_4) ~= intersect(x_A,x_B,obstacle5_Corner_3);
                wall5_check4 = intersect(x_A,obstacle5_Corner_4,obstacle5_Corner_2) ~=...
                    intersect(x_B,obstacle5_Corner_4,obstacle5_Corner_2) && intersect(x_A,x_B,obstacle5_Corner_4) ~= intersect(x_A,x_B,obstacle5_Corner_2);
                
                wall6_check1 = intersect(x_A,obstacle4_Corner_1,obstacle6_Corner_2) ~=...
                    intersect(x_B,obstacle6_Corner_1,obstacle6_Corner_2) && intersect(x_A,x_B,obstacle6_Corner_1) ~= intersect(x_A,x_B,obstacle6_Corner_2);
                wall6_check2 = intersect(x_A,obstacle4_Corner_1,obstacle6_Corner_3) ~=...
                    intersect(x_B,obstacle6_Corner_1,obstacle6_Corner_3) && intersect(x_A,x_B,obstacle6_Corner_1) ~= intersect(x_A,x_B,obstacle6_Corner_3);
                wall6_check3 = intersect(x_A,obstacle4_Corner_4,obstacle6_Corner_3) ~=...
                    intersect(x_B,obstacle6_Corner_4,obstacle6_Corner_3) && intersect(x_A,x_B,obstacle6_Corner_4) ~= intersect(x_A,x_B,obstacle6_Corner_3);
                wall6_check4 = intersect(x_A,obstacle4_Corner_4,obstacle6_Corner_2) ~=...
                    intersect(x_B,obstacle6_Corner_4,obstacle6_Corner_2) && intersect(x_A,x_B,obstacle6_Corner_4) ~= intersect(x_A,x_B,obstacle6_Corner_2);
                
                if(wall1_check1==0 && wall1_check2==0 && wall1_check3==0 && wall1_check4==0 &&...
                        wall2_check1==0 && wall2_check2==0 && wall2_check3==0 && wall2_check4==0 &&...
                        wall3_check1==0 && wall3_check2==0 && wall3_check3==0 && wall3_check4==0 &&...
                        wall4_check1==0 && wall4_check2==0 && wall4_check3==0 && wall4_check4==0 &&...
                        wall5_check1==0 && wall5_check2==0 && wall5_check3==0 && wall5_check4==0 &&...
                        wall6_check1==0 && wall6_check2==0 && wall6_check3==0 && wall6_check4==0)
                    totalCollisionFree = 1;
                else
                    totalCollisionFree = 0;
                end

                node_distanc = sqrt((x_nearest(k).coord(1)-x_newNode.coord(1))^2 + (x_nearest(k).coord(2)-x_newNode.coord(2))^2);
                
                if (totalCollisionFree==1) &&...
                        x_nearest(k).cost + node_distanc < C_min
                    x_min = x_nearest(k);
                    C_min = x_nearest(k).cost + node_distanc;
                    %line([x_min.coord(1), x_newNode.coord(1)], [x_min.coord(2), x_newNode.coord(2)], 'Color', 'g', 'LineWidth', 0.25);
                    hold on
                end
            end
            
            % Update parent to least cost-from node
            for j = 1:1:length(states)
                if states(j).coord == x_min.coord
                    x_newNode.parent = j;
                end
            end
            
            % Append to states
            states = [states x_newNode];
        end
    end
    
    D = [];
    for j = 1:1:length(states)
        tmpdist = sqrt((states(j).coord(1)-x_target.coord(1))^2 + (states(j).coord(2)-x_target.coord(2))^2);
        D = [D tmpdist];
    end
    
    % Search backwards from goal to start to find the optimal least cost path
    [val, idx] = min(D);
    x_final = states(idx);
    x_target.parent = idx;
    x_end = x_target;
    states = [states x_target];
    test_Path = [];
    
    while x_end.parent ~= 0
        start = x_end.parent;
        line([x_end.coord(1), states(start).coord(1)], [x_end.coord(2), states(start).coord(2)], 'Color', 'g', 'LineWidth', 3);

        tmpStart = [states(start).coord(1), states(start).coord(2)];
        tmpFinish = [x_end.coord(1), x_end.coord(2)];
        tmpAdd = tmpStart;
        
        test_Path = [test_Path ; tmpAdd];
        
        hold on
        x_end = states(start);
    end
    
    %End Algorithm Timer
    toc;
    
    %% Add Path to V-REP Scene
    
    % Defines the array of Path Points to be sent to V-REP
    Path_sort = flip(test_Path);
    Path_Coordinates = [Path_sort ; Pallet_Position(1),Pallet_Position(2)];
    nodes_Size = length(Path_Coordinates);
    tmp_Length = [];
    
    % Calculate the Path Length
    for i=1:nodes_Size
        
        if(i > 1)
            currentPath_Point = [Path_Coordinates(i,1) Path_Coordinates(i,2)];
            PreviousPath_Point = [Path_Coordinates(i-1,1), Path_Coordinates(i-1,2)];
            currentDistance = sqrt((currentPath_Point(1)-PreviousPath_Point(1))^2 + (currentPath_Point(2)-PreviousPath_Point(2))^2);
            tmp_Length = [tmp_Length ; currentDistance];
            Path_Length = sum(tmp_Length);
        end
    end
    
    % Plots Optimized Path within V-REP
    [res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'PlotPath',vrep.sim_scripttype_childscript,'plotPath_function',...
        nodes_Size,Path_Coordinates,[],[],vrep.simx_opmode_blocking);
    
    if (res==vrep.simx_return_ok)
        fprintf('Path Coordinates: %.4d\n',retFloats);
        fprintf('Path Length: %.4d\n', Path_Length);
    else
        fprintf('Remote function call failed\n');
    end
    
    % Move the AGV about the Path within V-REP
    %[res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'FollowPath',vrep.sim_scripttype_childscript,'followPath_function',...
    %    nodes_Size,Path_Coordinates,[],[],vrep.simx_opmode_blocking);
    
    %if (res==vrep.simx_return_ok)
    %    fprintf('Dummy handle: %.2d\n',retFloats);
    %    fprintf('Success');
    %else
    %    fprintf('Remote function call failed\n');
    %end
    
    % Stop the simulation
    vrep.simxFinish(-1);
else
    disp('Failed connecting to remote API server');
end
% Call the destructor
vrep.delete();

