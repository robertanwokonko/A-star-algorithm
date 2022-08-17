function [optimalPath, totalCost] = aStar()

    % To run this code use: [op, tc] = aStar();
    % This should return the optimal path found (op) if any and the total cost (tc)
    % To see optimal path use: op
    % To see total cost use: tc
    % aStar() would also work but shows only whether a path was found or not.
  
    % Test grid
    % grid = [
    %     0   0   0   0   0   0   0   0   0   0   0
    %     0   0   0   0   0   0   1   1   1   0   0
    %     0   0   1   1   0   0   0   1   1   0   0
    %     0   0   1   0   0   0   0   0   1   0   0
    %     0   0   0   0   0   0   1   0   0   0   0
    %     0   0   0   0   0   0   0   0   0   0   0
    %     0   0   1   1   1   1   1   1   1   0   0
    %     0   0   0   0   0   0   0   0   0   0   0
    % ];

    % We can generate a randomized grid of ones and zeros. For example: a 5 by 5 per the assignment
    % Note: The grid starts at [1,1] and ends at [4, 4]

    grid = generateGrid(5, 5);
    
    % ensure start and goal cells are reachable
    grid(4,4) = 0;
    grid(1,1) = 0;

    optimalPath = {}; % the optimal path to the goal
    totalCost = 0;
    openCells = struct([]); % list containing the cells that may fall on the optimal path we want
    visitedCells = struct([]); % list of visited cells

    start.pos = [4,4];
    start.parent = [];
    start.distance = 0;
    
    goal.pos = [1,1];
    goal.parent = [];
    goal.distance = 0;
    
    openCells = [openCells; start]; % Add start to the list. openCells.append(start)

    while ~isempty(openCells)
        current = openCells(1);
        openCells = openCells(2:end);

        if isVisited(current)
            continue;
        end

        if isGoal(current)
            optimalPath = flip(path(current));
            totalCost = length(visitedCells);
            
            disp('Optimal path found');
            
            return;
        end

        visitedCells = [visitedCells; current]; % add to visited list

        x = current.pos(1);
        y = current.pos(2);

        adjCells = {[x+1, y], [x-1, y], [x, y+1], [x, y-1]};
        
        for i = 1:length(adjCells)
            cell = adjCells{i};

            if isValidSuccessor(cell)
                distance = calculateDistance(start.pos, cell, goal.pos);
                addOpenCell(cell, current, distance);
            end
        end

        openCells = sortCellsByDistance(openCells);
    end

    % Path not found if goal cell is not visited
    if ~isVisited(goal)
        disp('Path not found');
    end
    
    % Utility functions
    function successor = isValidSuccessor(pos)
        successor = isValidCell(pos) && isReachable(pos) && ~(posMemberOf(pos, openCells)) && ~(posMemberOf(pos, visitedCells));
    end

    function valid = isValidCell(pos)
        gridSize = size(grid);
        gridHeight = gridSize(1);
        gridWidth = gridSize(2);
        
        valid = (pos(1) >= 1) && (pos(1) <= gridHeight) && (pos(2) >= 1) && (pos(2) <= gridWidth);
    end

    function reachable = isReachable(pos)
        reachable = (grid(pos(1), pos(2)) == 0);
    end

    % returns the path by traversing backwards to the goal node
    function path = path(cell)
        path = {};
        path{end+1} = cell.pos;
        
        while ~isempty(cell.parent)
            parent = cell.parent;
            path{end+1} = parent.pos;
            cell = parent;
        end
    end

    function isGoal = isGoal(cell)
        isGoal = 0;

        if (cell.pos(1) == goal.pos(1)) && (cell.pos(2) == goal.pos(2))
            isGoal = 1;
        end
    end

    function visited = isVisited(cell)
        visited = posMemberOf(cell.pos, visitedCells);
    end

    function member = posMemberOf(pos, list)
        member = 0;

        for i = 1:length(list)
            x = list(i).pos(1);
            y = list(i).pos(2);
            
            if (x == pos(1)) && (y == pos(2))
                member = 1;
                return;
            end      
        end
    end

    function addOpenCell(pos, cell, distance)
        openCells(end+1) = struct('pos', pos, 'parent', cell, 'distance', distance );
    end
    
    function distance = calculateDistance(startPos, currentPos, goalPos)
        h_x = sqrt( (currentPos(1) - goalPos(1))^2 + (currentPos(2) - goalPos(2))^2 );
        g_x = abs(startPos(1) - currentPos(1)) + abs(startPos(2) - currentPos(2));
        
        distance = g_x + h_x;
    end

    function grid = generateGrid(r, c)
        grid = randi([0, 1], r, c);
    end

    function sortedList = sortCellsByDistance(list)
        table = struct2table(list); % convert the struct array to a table
        sortedTable = sortrows(table, 'distance'); % sort the table by 'distance'
        sortedList = table2struct(sortedTable);
    end

end
