package algorithms;

import map.*;
import map.Map;
import robot.Robot;
import robot.RobotConstants;
import robot.RobotConstants.DIRECTION;
import robot.RobotConstants.MOVEMENT;
import utils.CommMgr;
import map.MapConstants;

import java.lang.reflect.Array;
import java.util.*;

/**
 * Exploration algorithm for the robot.
 *
 * @author Priyanshu Singh
 * @author Suyash Lakhotia
 */

public class ExplorationAlgo {
    private final Map exploredMap;
    private final Map realMap;
    private final Robot bot;
    private final int coverageLimit;
    private final int timeLimit;
    private int areaExplored;
    private long startTime;
    private long endTime;
    private int lastCalibrate;
    private boolean calibrationMode;
    private ArrayList<Cell> unExploredCells;
    private boolean imageRun = true;
    HashMap<String, ObsSurface> notYetTaken;
    List<ObsSurface> notAccessibleSurface;

    public ExplorationAlgo(Map exploredMap, Map realMap, Robot bot, int coverageLimit, int timeLimit) {
        this.exploredMap = exploredMap;
        this.realMap = realMap;
        this.bot = bot;
        this.coverageLimit = coverageLimit;
        this.timeLimit = timeLimit;
    }

    /**
     * Main method that is called to start the exploration.
     */
    public void runExploration() {
        if (bot.getRealBot()) {
            System.out.println("Starting calibration...");

            CommMgr.getCommMgr().recvMsg();
            if (bot.getRealBot()) {
                bot.move(MOVEMENT.LEFT, false);
                CommMgr.getCommMgr().recvMsg();
                bot.move(MOVEMENT.CALIBRATE, false);
                CommMgr.getCommMgr().recvMsg();
                bot.move(MOVEMENT.LEFT, false);
                CommMgr.getCommMgr().recvMsg();
                bot.move(MOVEMENT.CALIBRATE, false);
                CommMgr.getCommMgr().recvMsg();
                bot.move(MOVEMENT.RIGHT, false);
                CommMgr.getCommMgr().recvMsg();
                bot.move(MOVEMENT.CALIBRATE, false);
                CommMgr.getCommMgr().recvMsg();
                bot.move(MOVEMENT.RIGHT, false);
            }

            while (true) {
                System.out.println("Waiting for EX_START...");
                String msg = CommMgr.getCommMgr().recvMsg();
                String[] msgArr = msg.split(";");
                if (msgArr[0].equals(CommMgr.EX_START)) break;
            }
        }

        System.out.println("Starting exploration...");

        startTime = System.currentTimeMillis();
        endTime = startTime + (timeLimit * 1000);

        if (bot.getRealBot()) {
            CommMgr.getCommMgr().sendMsg(null, CommMgr.BOT_START);
        }
        senseAndRepaint();

        areaExplored = calculateAreaExplored();
        System.out.println("Explored Area: " + areaExplored);

        explorationLoop(bot.getRobotPosRow(), bot.getRobotPosCol());
    }

    /**
     * Loops through robot movements until one (or more) of the following conditions is met:
     * 1. Robot is back at (r, c)
     * 2. areaExplored > coverageLimit
     * 3. System.currentTimeMillis() > endTime
     */
    private void explorationLoop(int r, int c) {
        boolean hugCompleted = false;
        do {
            if (!hugCompleted){
                nextMove();
            }
            else {
                Cell nearestCell = calcNearest();
                ArrayList<Cell> path = findPath(nearestCell);
                if (path == null) {
                    boolean foundCandidate = false;
                    Cell nextcandidateCell = null;
                    for (int i = -2; i <= 2; i+=4) {
                        Cell candidateCell = exploredMap.getCell(nearestCell.getRow() + i, nearestCell.getCol());
                        if (candidateCell.getIsExplored() && !(candidateCell.getIsVirtualWall() || candidateCell.getIsObstacle())) {
                            foundCandidate = true;
                            nextcandidateCell = candidateCell;
                            break;
                        }
                    }
                    if (!foundCandidate) {
                        for (int i = -2; i <= 2; i+=4) {
                            Cell candidateCell = exploredMap.getCell(nearestCell.getRow(), nearestCell.getCol() + i);
                            if (!(candidateCell.getIsExplored() && (candidateCell.getIsVirtualWall() || candidateCell.getIsObstacle()))) {
                                foundCandidate = true;
                                nextcandidateCell = candidateCell;
                                break;
                            }
                        }
                    }
                    System.out.println("Switch to candidate: " + nextcandidateCell.getRow() + " " + nextcandidateCell.getCol());
                    ArrayList<Cell> newpath = findPath(nextcandidateCell);
                    goToNearest(nextcandidateCell, newpath, true);
                }
                else {
                    goToNearest(nearestCell, path, false);
                }
            }

            areaExplored = calculateAreaExplored();
            System.out.println("Area explored: " + areaExplored);

//            if (bot.getRobotPosRow() == r && bot.getRobotPosCol() == c) {
//                if (areaExplored >= 100) {
//                    break;
//                }
//            }
            if (bot.getRobotPosRow() == r && bot.getRobotPosCol() == c) {
                hugCompleted = true;
            }
        } while (areaExplored < coverageLimit && System.currentTimeMillis() <= endTime);
        System.out.println("AreaExplored: " + areaExplored);
        System.out.println("coverageLimit: " + coverageLimit);

        goHome2();

        if (imageRun) {
            System.out.println("image run");
            HashMap<String, ObsSurface> coverage = this.getAllObsSurfaces();
            List<ObsSurface> allPossibleSurfaceCoverage = new ArrayList<ObsSurface>();
            for (String key : coverage.keySet()) {
                allPossibleSurfaceCoverage.add(coverage.get(key));
            }

            notYetTaken = getUntakenSurfaces();
            if (notYetTaken.size() == 0) {
                return;
            }
            List<ObsSurface> notYetTakenList = new ArrayList<ObsSurface>();
            for (String key : notYetTaken.keySet()) {
                notYetTakenList.add(notYetTaken.get(key));
            }
            exploredMap.setNotYetTakenList(notYetTakenList);
            System.out.println("check");
            System.out.println(notYetTakenList);
            exploredMap.repaint();

//            ObsSurface nearestObstacle;
//            Cell nearestCell;
//            nearestObstacle = exploredMap.nearestObsSurface(bot.getPos(), notYetTaken);
//            nearestCell = exploredMap.nearestMovable(nearestObstacle);
//            System.out.println(nearestObstacle);
//            System.out.println(nearestCell.getPos());
//            exploredMap.repaint();

            while (notYetTaken.size() > 0) {
                System.out.println("image loop");
                imageLoop();
                exploredMap.repaint();
                // TODO
            }
            System.out.println("Finished, now printing unaccessible ones");
            exploredMap.setNotYetTakenList(notAccessibleSurface);
            exploredMap.repaint();

            FastestPathAlgo returnToStart = new FastestPathAlgo(exploredMap, bot);
            returnToStart.runFastestPath(RobotConstants.START_ROW, RobotConstants.START_COL);

            exploredMap.repaint();
        }


    }

    /**
     * Determines the next move for the robot and executes it accordingly.
     */
    private void nextMove() {
        if (lookLeft()) {
            moveBot(MOVEMENT.LEFT);
            if (lookForward()) moveBot(MOVEMENT.FORWARD);
        } else if (lookForward()) {
            moveBot(MOVEMENT.FORWARD);
        } else if (lookRight()) {
            moveBot(MOVEMENT.RIGHT);
            if (lookForward()) moveBot(MOVEMENT.FORWARD);
        } else {
            moveBot(MOVEMENT.LEFT);
            moveBot(MOVEMENT.LEFT);
        }
    }

    /**
     * If full coverage not reached after one round of hugging, find the nearest cell by manhattan distance
     */
    private Cell calcNearest() {
        unExploredCells = new ArrayList<>();
        int minDist = 50;
        Cell nearestCell = null;
        for (int i = 0; i < MapConstants.MAP_ROWS; i++) {
            for (int j = 0; j < MapConstants.MAP_COLS; j++) {
                Cell currentCell = exploredMap.getCell(i, j);
                if (!currentCell.getIsExplored()) {
                    unExploredCells.add(currentCell);
                    int dist = Math.abs(bot.getRobotPosRow() - i) + Math.abs(bot.getRobotPosCol() - j);
                    if (dist < minDist) {
                        minDist = dist;
                        nearestCell = currentCell;
                    }
                }
            }
        }
        return nearestCell;
    }

    /**
     *  Return the shortest path from current path to cell c
     */
    private ArrayList<Cell> findPath(Cell c) {
        int[] rowmov = {1, 0, -1, 0};
        int[] colmov = {0, 1, 0, -1};
        int[][] dist = new int[MapConstants.MAP_ROWS][MapConstants.MAP_COLS];
        for (int i = 0; i < MapConstants.MAP_ROWS; i++) {
            for (int j = 0; j < MapConstants.MAP_COLS; j++) {
                dist[i][j] = -1;
            }
        }
        ArrayDeque<Cell> nextExplore = new ArrayDeque<>();
        nextExplore.add(exploredMap.getCell(bot.getRobotPosRow(), bot.getRobotPosCol()));
        dist[bot.getRobotPosRow()][bot.getRobotPosCol()] = 0;
        while (!nextExplore.isEmpty()) {
            Cell current = nextExplore.removeFirst();
            if (current.getCol() == c.getCol() && current.getRow() == c.getRow()) {
                break;
            }
            for (int i = 0; i < 4; i++){
                int nextrow = current.getRow() + rowmov[i];
                int nextcol = current.getCol() + colmov[i];
                if (nextcol >= 0 && nextrow >= 0 && nextcol < MapConstants.MAP_COLS && nextrow < MapConstants.MAP_ROWS) {
                    if (dist[nextrow][nextcol] == -1) {
                        Cell nextCell = exploredMap.getCell(nextrow, nextcol);
                        if (nextCell.getIsExplored() && (nextCell.getIsVirtualWall() || nextCell.getIsObstacle())) {
                            dist[nextrow][nextcol] = 10000;
                        }
                        else {
                            dist[nextrow][nextcol] = dist[current.getRow()][current.getCol()] + 1;
                            nextExplore.addLast(nextCell);
                        }
                    }
                }
            }
        }
        System.out.println(dist[c.getRow()][c.getCol()]);
        if (dist[c.getRow()][c.getCol()] == -1)
            return null;
        ArrayList<Cell> path = new ArrayList<>();
        Cell current = c;
        path.add(c);
        while(!(current.getRow() == bot.getRobotPosRow() & current.getCol() == bot.getRobotPosCol())) {
            for (int i = 0; i < 4; i++) {
                int nextrow = current.getRow() + rowmov[i];
                int nextcol = current.getCol() + colmov[i];
                if (nextcol >= 0 && nextrow >= 0 && nextcol <= MapConstants.MAP_COLS && nextrow <= MapConstants.MAP_ROWS) {
                    if (dist[nextrow][nextcol] == dist[current.getRow()][current.getCol()] - 1) {
                        current = exploredMap.getCell(nextrow, nextcol);
                        path.add(current);
                        break;
                    }
                }
            }
        }

        System.out.println("Shortest path from " + bot.getRobotPosRow() + " " + bot.getRobotPosCol() + " to " + c.getRow() + " " + c.getCol());
        for (int i = path.size() - 1; i >= 0; i--) {
            System.out.print(path.get(i).getRow() + " " + path.get(i).getCol() + " -> ");
        }
        System.out.println();

        return path;

    }

    /**
     * If full coverage not reached after one round of hugging
     */
    private void goToNearest(Cell nearestCell, ArrayList<Cell> path, boolean candidate) {
        for (int i = 0; i < path.size(); i++) {
            Cell nextCell = path.get(path.size() - i - 1);
            System.out.println(nextCell.getPos());
            System.out.println(bot.getPos());
            while ((bot.getRobotPosRow() != nextCell.getRow()) || (bot.getRobotPosCol() != nextCell.getCol())) {
                System.out.println(nextCell.getRow() + " " + nextCell.getCol());

                DIRECTION targetDir = getTargetDir(bot.getRobotPosRow(), bot.getRobotPosCol(), bot.getRobotCurDir(), nextCell);

                System.out.println(targetDir);
                if (bot.getRobotCurDir() != targetDir) {
                    MOVEMENT m = getTargetMove(bot.getRobotCurDir(), targetDir);
                    System.out.println(m);
                    moveBot(m);
                }
                else {
                    moveBot(MOVEMENT.FORWARD);
                }
            }
            if (!candidate && exploredMap.getCell(nearestCell.getRow(), nearestCell.getCol()).getIsExplored()) {
                break;
            }

        }
        System.out.println("finished");
    }

    /**
     * Returns the target direction of the bot from [botR, botC] to target Cell.
     */
    private DIRECTION getTargetDir(int botR, int botC, DIRECTION botDir, Cell target) {
        if (botC - target.getCol() > 0) {
            return DIRECTION.WEST;
        } else if (target.getCol() - botC > 0) {
            return DIRECTION.EAST;
        } else {
            if (botR - target.getRow() > 0) {
                return DIRECTION.SOUTH;
            } else if (target.getRow() - botR > 0) {
                return DIRECTION.NORTH;
            } else {
                return botDir;
            }
        }
    }

    /**
     * Returns the movement to execute to get from one direction to another.
     */
    private MOVEMENT getTargetMove(DIRECTION a, DIRECTION b) {
        switch (a) {
            case NORTH:
                switch (b) {
                    case NORTH:
                        return MOVEMENT.ERROR;
                    case SOUTH:
                        return MOVEMENT.LEFT;
                    case WEST:
                        return MOVEMENT.LEFT;
                    case EAST:
                        return MOVEMENT.RIGHT;
                }
                break;
            case SOUTH:
                switch (b) {
                    case NORTH:
                        return MOVEMENT.LEFT;
                    case SOUTH:
                        return MOVEMENT.ERROR;
                    case WEST:
                        return MOVEMENT.RIGHT;
                    case EAST:
                        return MOVEMENT.LEFT;
                }
                break;
            case WEST:
                switch (b) {
                    case NORTH:
                        return MOVEMENT.RIGHT;
                    case SOUTH:
                        return MOVEMENT.LEFT;
                    case WEST:
                        return MOVEMENT.ERROR;
                    case EAST:
                        return MOVEMENT.LEFT;
                }
                break;
            case EAST:
                switch (b) {
                    case NORTH:
                        return MOVEMENT.LEFT;
                    case SOUTH:
                        return MOVEMENT.RIGHT;
                    case WEST:
                        return MOVEMENT.LEFT;
                    case EAST:
                        return MOVEMENT.ERROR;
                }
        }
        return MOVEMENT.ERROR;
    }

    /**
     * Returns true if the right side of the robot is free to move into.
     */
    private boolean lookRight() {
        switch (bot.getRobotCurDir()) {
            case NORTH:
                return eastFree();
            case EAST:
                return southFree();
            case SOUTH:
                return westFree();
            case WEST:
                return northFree();
        }
        return false;
    }

    /**
     * Returns true if the robot is free to move forward.
     */
    private boolean lookForward() {
        switch (bot.getRobotCurDir()) {
            case NORTH:
                return northFree();
            case EAST:
                return eastFree();
            case SOUTH:
                return southFree();
            case WEST:
                return westFree();
        }
        return false;
    }

    /**
     * * Returns true if the left side of the robot is free to move into.
     */
    private boolean lookLeft() {
        switch (bot.getRobotCurDir()) {
            case NORTH:
                return westFree();
            case EAST:
                return northFree();
            case SOUTH:
                return eastFree();
            case WEST:
                return southFree();
        }
        return false;
    }

    /**
     * Returns true if the robot can move to the north cell.
     */
    private boolean northFree() {
        int botRow = bot.getRobotPosRow();
        int botCol = bot.getRobotPosCol();
        return (isExploredNotObstacle(botRow + 1, botCol - 1) && isExploredAndFree(botRow + 1, botCol) && isExploredNotObstacle(botRow + 1, botCol + 1));
    }

    /**
     * Returns true if the robot can move to the east cell.
     */
    private boolean eastFree() {
        int botRow = bot.getRobotPosRow();
        int botCol = bot.getRobotPosCol();
        return (isExploredNotObstacle(botRow - 1, botCol + 1) && isExploredAndFree(botRow, botCol + 1) && isExploredNotObstacle(botRow + 1, botCol + 1));
    }

    /**
     * Returns true if the robot can move to the south cell.
     */
    private boolean southFree() {
        int botRow = bot.getRobotPosRow();
        int botCol = bot.getRobotPosCol();
        return (isExploredNotObstacle(botRow - 1, botCol - 1) && isExploredAndFree(botRow - 1, botCol) && isExploredNotObstacle(botRow - 1, botCol + 1));
    }

    /**
     * Returns true if the robot can move to the west cell.
     */
    private boolean westFree() {
        int botRow = bot.getRobotPosRow();
        int botCol = bot.getRobotPosCol();
        return (isExploredNotObstacle(botRow - 1, botCol - 1) && isExploredAndFree(botRow, botCol - 1) && isExploredNotObstacle(botRow + 1, botCol - 1));
    }

    /**
     * Returns the robot to START after exploration and points the bot northwards.
     */
    private void goHome() {
        System.out.println("goHome now");
        if (!bot.getTouchedGoal() && coverageLimit == 300 && timeLimit == 3600) {
            FastestPathAlgo goToGoal = new FastestPathAlgo(exploredMap, bot, realMap);
            goToGoal.runFastestPath(RobotConstants.GOAL_ROW, RobotConstants.GOAL_COL);
        }

        FastestPathAlgo returnToStart = new FastestPathAlgo(exploredMap, bot, realMap);
        returnToStart.runFastestPath(RobotConstants.START_ROW, RobotConstants.START_COL);

        System.out.println("Exploration complete!");
        areaExplored = calculateAreaExplored();
        System.out.printf("%.2f%% Coverage", (areaExplored / 300.0) * 100.0);
        System.out.println(", " + areaExplored + " Cells");
        System.out.println((System.currentTimeMillis() - startTime) / 1000 + " Seconds");

        if (bot.getRealBot()) {
            turnBotDirection(DIRECTION.WEST);
            moveBot(MOVEMENT.CALIBRATE);
            turnBotDirection(DIRECTION.SOUTH);
            moveBot(MOVEMENT.CALIBRATE);
            turnBotDirection(DIRECTION.WEST);
            moveBot(MOVEMENT.CALIBRATE);
        }
        turnBotDirection(DIRECTION.NORTH);
    }

    // correct implementation of go home
    private void goHome2() {
        if (!bot.getTouchedGoal() && coverageLimit == 300 && timeLimit == 3600) {
            FastestPathAlgo goToGoal = new FastestPathAlgo(exploredMap, bot, realMap);
            goToGoal.runFastestPath(RobotConstants.GOAL_ROW, RobotConstants.GOAL_COL);
        }

        FastestPathAlgo returnToStart = new FastestPathAlgo(exploredMap, bot);
        returnToStart.runFastestPath(RobotConstants.START_ROW, RobotConstants.START_COL);

        System.out.println("Exploration complete!");
        areaExplored = calculateAreaExplored();
        System.out.printf("%.2f%% Coverage", (areaExplored / 300.0) * 100.0);
        System.out.println(", " + areaExplored + " Cells");
        System.out.println((System.currentTimeMillis() - startTime) / 1000 + " Seconds");

        if (bot.getRealBot()) {
            turnBotDirection(DIRECTION.WEST);
            moveBot(MOVEMENT.CALIBRATE);
            turnBotDirection(DIRECTION.SOUTH);
            moveBot(MOVEMENT.CALIBRATE);
            turnBotDirection(DIRECTION.WEST);
            moveBot(MOVEMENT.CALIBRATE);
        }
        turnBotDirection(DIRECTION.NORTH);
        System.out.println(bot.getPos());
    }

    /**
     * Returns true for cells that are explored and not obstacles.
     */
    private boolean isExploredNotObstacle(int r, int c) {
        if (exploredMap.checkValidCoordinates(r, c)) {
            Cell tmp = exploredMap.getCell(r, c);
            return (tmp.getIsExplored() && !tmp.getIsObstacle());
        }
        return false;
    }

    /**
     * Returns true for cells that are explored, not virtual walls and not obstacles.
     */
    private boolean isExploredAndFree(int r, int c) {
        if (exploredMap.checkValidCoordinates(r, c)) {
            Cell b = exploredMap.getCell(r, c);
            return (b.getIsExplored() && !b.getIsVirtualWall() && !b.getIsObstacle());
        }
        return false;
    }

    /**
     * Returns the number of cells explored in the grid.
     */
    private int calculateAreaExplored() {
        int result = 0;
        for (int r = 0; r < MapConstants.MAP_ROWS; r++) {
            for (int c = 0; c < MapConstants.MAP_COLS; c++) {
                if (exploredMap.getCell(r, c).getIsExplored()) {
                    result++;
                }
            }
        }
        return result;
    }

    /**
     * Moves the bot, repaints the map and calls senseAndRepaint().
     * Calibration: can
     */
    private void moveBot(MOVEMENT m) {
        bot.move(m);
        exploredMap.repaint();
        if (m != MOVEMENT.CALIBRATE) {
            senseAndRepaint();
        } else {
            CommMgr commMgr = CommMgr.getCommMgr();
            commMgr.recvMsg();
        }

        if (bot.getRealBot()&& !calibrationMode){
            calibrationMode = true;
            if (atMazeCorner()){
                lastCalibrate = 0;
                moveBot(MOVEMENT.CORNER_CALIBRATE);
            }
            else{
                if (lastCalibrate >= 5){
                    MOVEMENT movement;
                    movement = canCalibrateOnTheSpot(bot.getRobotCurDir());
                    if (movement!=null){
                        lastCalibrate = 0;
                        moveBot(movement);
                    }
                }
                else
                    lastCalibrate++;
            }

            calibrationMode = true;
        }
//        if (bot.getRealBot() && !calibrationMode) {
//            calibrationMode = true;
//
//            if (canCalibrateOnTheSpot(bot.getRobotCurDir())) {
//                lastCalibrate = 0;
//                moveBot(MOVEMENT.CALIBRATE);
//            } else {
//                lastCalibrate++;
//                if (lastCalibrate >= 5) {
//                    DIRECTION targetDir = getCalibrationDirection();
//                    if (targetDir != null) {
//                        lastCalibrate = 0;
//                        calibrateBot(targetDir);
//                    }
//                }
//            }
//
//            calibrationMode = false;
//        }
    }

    /**
     * Sets the bot's sensors, processes the sensor data and repaints the map.
     */
    private void senseAndRepaint() {
        bot.setSensors();
        bot.sense(exploredMap, realMap);
        exploredMap.repaint();
    }

    /**
     * Checks if the robot can calibrate at its current position given a direction.
     */
    private MOVEMENT canCalibrateOnTheSpot(DIRECTION botDir) {
        int row = bot.getRobotPosRow();
        int col = bot.getRobotPosCol();

        switch (botDir) {
            case NORTH: {
                if(exploredMap.getIsObstacleOrWall(row + 1, col - 2) && exploredMap.getIsObstacleOrWall(row - 1, col - 2))
                    return MOVEMENT.LEFT_CALIBRATE;
                if(exploredMap.getIsObstacleOrWall(row + 2, col - 1) && exploredMap.getIsObstacleOrWall(row + 2, col + 1))
                    return MOVEMENT.FRONT_CALIBRATE;
            }
            case EAST:
                if(exploredMap.getIsObstacleOrWall(row + 2, col + 1) && exploredMap.getIsObstacleOrWall(row + 2, col - 1))
                    return MOVEMENT.LEFT_CALIBRATE;
                if(exploredMap.getIsObstacleOrWall(row + 1, col + 2) && exploredMap.getIsObstacleOrWall(row - 1, col + 2))
                    return MOVEMENT.FRONT_CALIBRATE;

            case SOUTH:
                if(exploredMap.getIsObstacleOrWall(row + 1, col + 2) && exploredMap.getIsObstacleOrWall(row - 1, col + 2))
                    return MOVEMENT.LEFT_CALIBRATE;
                if(exploredMap.getIsObstacleOrWall(row - 2, col - 1) && exploredMap.getIsObstacleOrWall(row - 2, col + 1))
                    return MOVEMENT.FRONT_CALIBRATE;
            case WEST:
                if(exploredMap.getIsObstacleOrWall(row - 2, col + 1) && exploredMap.getIsObstacleOrWall(row - 2, col - 1))
                    return MOVEMENT.LEFT_CALIBRATE;
                if(exploredMap.getIsObstacleOrWall(row + 1, col - 2) && exploredMap.getIsObstacleOrWall(row - 1, col - 2))
                    return MOVEMENT.FRONT_CALIBRATE;
        }

        return null;
    }

//    /**
//     * Returns a possible direction for robot calibration or null, otherwise.
//     */
//    private DIRECTION getCalibrationDirection() {
//        DIRECTION origDir = bot.getRobotCurDir();
//        DIRECTION dirToCheck;
//
//        dirToCheck = DIRECTION.getNext(origDir);                    // right turn
//        if (canCalibrateOnTheSpot(dirToCheck)) return dirToCheck;
//
//        dirToCheck = DIRECTION.getPrevious(origDir);                // left turn
//        if (canCalibrateOnTheSpot(dirToCheck)) return dirToCheck;
//
//        dirToCheck = DIRECTION.getPrevious(dirToCheck);             // u turn
//        if (canCalibrateOnTheSpot(dirToCheck)) return dirToCheck;
//
//        return null;
//    }

    /**
     * Turns the bot in the needed direction and sends the CALIBRATE movement. Once calibrated, the bot is turned back
     * to its original direction.
     */
    private void calibrateBot(DIRECTION targetDir) {
        DIRECTION origDir = bot.getRobotCurDir();

        turnBotDirection(targetDir);
        moveBot(MOVEMENT.CALIBRATE);
        turnBotDirection(origDir);
    }

    /**
     * Turns the robot to the required direction.
     */
    private void turnBotDirection(DIRECTION targetDir) {
        int numOfTurn = Math.abs(bot.getRobotCurDir().ordinal() - targetDir.ordinal());
        if (numOfTurn > 2) numOfTurn = numOfTurn % 2;

        if (numOfTurn == 1) {
            if (DIRECTION.getNext(bot.getRobotCurDir()) == targetDir) {
                moveBot(MOVEMENT.RIGHT);
            } else {
                moveBot(MOVEMENT.LEFT);
            }
        } else if (numOfTurn == 2) {
            moveBot(MOVEMENT.RIGHT);
            moveBot(MOVEMENT.RIGHT);
        }
    }

    private boolean atMazeCorner(){
        int row = bot.getRobotPosRow();
        int col = bot.getRobotPosCol();
        if (((row == 2)&&(col == 2)) || ((row == MapConstants.MAP_ROWS -1 )&&(col == 2)) || ((row == MapConstants.MAP_ROWS -1 )&&(col == MapConstants.MAP_COLS -1)) || ((row == 2 )&&(col == MapConstants.MAP_COLS -1)) )
            return true;
        return false;
    }

    public HashMap<String, ObsSurface> getAllObsSurfaces() {
        // TODO
        Cell tempCell;
        Cell temp;
        ObsSurface tempObsSurface;
        HashMap<Direction, Cell> tempNeighbours;
        HashMap<String, ObsSurface> allPossibleSurfaces = new HashMap<String, ObsSurface>();
        for (int row = 0; row < MapConstants.MAP_ROWS; row++) {
            for (int col = 0; col < MapConstants.MAP_COLS; col++) {
                tempCell = exploredMap.getCell(row, col);

                if (tempCell.getIsObstacle()) {
                    // check neighbouring
                    tempNeighbours = exploredMap.getNeighboursMap(tempCell);

                    for (Direction neighbourDir: tempNeighbours.keySet()) {
                        temp = tempNeighbours.get(neighbourDir);

                        if (!temp.getIsObstacle()) {
                            tempObsSurface = new ObsSurface(tempCell.getRow(),tempCell.getCol(), neighbourDir);
                            allPossibleSurfaces.put(tempObsSurface.toString(), tempObsSurface);
                        }
                    }
                }

            }
        }
        return allPossibleSurfaces;
    }

    private HashMap<String, ObsSurface> getUntakenSurfaces() {
        this.notYetTaken = getAllObsSurfaces();
        List<ObsSurface> coverage = exploredMap.getSurfaceCoverage();

        if(coverage!=null) {
            System.out.println(coverage.size());
            int count = 0;

            for (ObsSurface tempObsSurfaceStr : coverage) {
                count=+1;
                System.out.println(count);
                System.out.println("entered getUntakenSurfaces");
                removeFromNotYetTaken(tempObsSurfaceStr);
                System.out.println(coverage.size());
            }
            System.out.println("finished getUntakenSurfaces");
        }
        return this.notYetTaken;
    }

    private void imageLoop(){
        ObsSurface nearestObstacle;
        Cell nearestCell;
        nearestObstacle = exploredMap.nearestObsSurface(bot.getPos(), notYetTaken);
        System.out.println("Nearest Surface");
        System.out.println(nearestObstacle);
        nearestCell = exploredMap.nearestMovable(nearestObstacle);
        System.out.println("Nearest Valid Cell");
        if(nearestCell==null) System.out.println("No valid cell");
        else System.out.println(nearestCell.getPos());
        System.out.println("size of not taken:");
        System.out.println(notYetTaken.size());
        if (nearestCell != null) {
            // go to nearest cell
            ArrayList<Cell> path = findPath_image(nearestCell);
            System.out.println(path);
            Cell nearestCell_modified = new Cell(nearestCell.getRow()+1,nearestCell.getCol()+1);
            goToNearestSurface(nearestCell_modified, path, nearestObstacle);
            System.out.println("Bot Pos after goToNearestSurface");
            System.out.println(bot.getPos());
            DIRECTION curDir = bot.getRobotCurDir();
            DIRECTION dirForImage = getDirForTakingImage(nearestObstacle);
            while(curDir!=dirForImage){
                MOVEMENT mov = getTargetMove(curDir,dirForImage);
                System.out.println("imageloop loop for direction");
                System.out.println(mov);
                bot.move(mov);
                curDir = bot.getRobotCurDir();
            }
            System.out.println("surface Taken");
            List<ObsSurface> surfTaken = exploredMap.getSurfaceCoverage();
            System.out.println(surfTaken);
            if (surfTaken!=null)
            {
                removeFromNotYetTaken(nearestObstacle);
                exploredMap.appendSurfaceCoverage(nearestObstacle);
//                System.out.println("ever entered this loop?");
            }
            //todo, send command to take image
            //updateNotYetTaken(nearestObstacle);

        }
        else{
            System.out.println("Unaccessible surface:");
            System.out.println(nearestObstacle.getPos());
            removeFromNotYetTaken(nearestObstacle);
            exploredMap.appendSurfaceCoverage(nearestObstacle);
        }


        System.out.println("size of not taken:");
        System.out.println(notYetTaken.size());
        System.out.println("Surface removed:");
        System.out.println(nearestObstacle.toString());
    }

    private void removeFromNotYetTaken(ObsSurface obsSurface) {
        System.out.println("Testing removeFromNotYetTaken");
        System.out.println(notYetTaken);
        System.out.println(obsSurface.toString());
        System.out.println(notYetTaken.containsKey(obsSurface.toString()));

        notYetTaken.remove(obsSurface.toString());

        System.out.println(notYetTaken);
    }

    private void updateNotYetTaken(List<ObsSurface> surfTaken) {
        for (ObsSurface obsSurface : surfTaken) {
            System.out.println("Testting updateNotYetTaken");
            System.out.println(notYetTaken);
            System.out.println(obsSurface.toString());
            System.out.println(notYetTaken.containsKey(obsSurface.toString()));
            if (notYetTaken.containsKey(obsSurface.toString())) {
                notYetTaken.remove(obsSurface.toString());
                System.out.println(notYetTaken);
                System.out.println(obsSurface.toString());
            }
        }
    }
    /**
     * Get direction for taking image.
     */
    public DIRECTION getDirForTakingImage(ObsSurface obs){
        switch (obs.getSurface()) {
            case UP:{
                return DIRECTION.WEST;
            }
            case DOWN: {
                return DIRECTION.EAST;
            }
            case LEFT:{
                return DIRECTION.SOUTH;
            }
            case RIGHT: {
                return DIRECTION.NORTH;
            }
        }
        return DIRECTION.WEST;
    }
    /**
     * If full coverage not reached after one round of hugging
     */
    private void goToNearestSurface(Cell nearestCell, ArrayList<Cell> path, ObsSurface surface) {
        for (int i = 0; i < path.size(); i++) {
            Cell nextCell = path.get(path.size() - i - 1);
            System.out.println(nextCell.getPos());
            System.out.println(bot.getPos());
            while ((bot.getRobotPosRow() != nextCell.getRow()) || (bot.getRobotPosCol() != nextCell.getCol())) {
                System.out.println(nextCell.getRow() + " " + nextCell.getCol());
                DIRECTION targetDir = getTargetDir(bot.getRobotPosRow(), bot.getRobotPosCol(), bot.getRobotCurDir(), nextCell);

                System.out.println(targetDir);
                if (bot.getRobotCurDir() != targetDir) {
                    MOVEMENT m = getTargetMove(bot.getRobotCurDir(), targetDir);
                    System.out.println(m);
                    moveBot(m);
                }
                else {
                    System.out.println("gotonearestsurface loop");
                    moveBot(MOVEMENT.FORWARD);
                }
            }
            notYetTaken = getUntakenSurfaces();
            if (notYetTaken.size() == 0) {
                return;
            }
            System.out.println("1");
            List<ObsSurface> notYetTakenList = new ArrayList<ObsSurface>();
            for (String key : notYetTaken.keySet()) {
                System.out.println("2");
                notYetTakenList.add(notYetTaken.get(key));
            }
            System.out.println("3");
            exploredMap.setNotYetTakenList(notYetTakenList);
            exploredMap.repaint();
            System.out.println("4");
        }
        System.out.println("finished one surface");
    }



    /**
     *  Return the shortest path from current path to cell c
     */
    private ArrayList<Cell> findPath_image(Cell c) {
        int[] rowmov = {1, 0, -1, 0};
        int[] colmov = {0, 1, 0, -1};
        int[][] dist = new int[MapConstants.MAP_ROWS][MapConstants.MAP_COLS];
        for (int i = 0; i < MapConstants.MAP_ROWS; i++) {
            for (int j = 0; j < MapConstants.MAP_COLS; j++) {
                dist[i][j] = -1;
            }
        }
        ArrayDeque<Cell> nextExplore = new ArrayDeque<>();
        nextExplore.add(exploredMap.getCell(bot.getRobotPosRow(), bot.getRobotPosCol()));
        dist[bot.getRobotPosRow()][bot.getRobotPosCol()] = 0;
        while (!nextExplore.isEmpty()) {
            Cell current = nextExplore.removeFirst();
            if (current.getCol() == c.getCol() && current.getRow() == c.getRow()) {
                break;
            }
            for (int i = 0; i < 4; i++){
                int nextrow = current.getRow() + rowmov[i];
                int nextcol = current.getCol() + colmov[i];
                if (nextcol >= 0 && nextrow >= 0 && nextcol < MapConstants.MAP_COLS && nextrow < MapConstants.MAP_ROWS) {
                    if (dist[nextrow][nextcol] == -1) {
                        Cell nextCell = exploredMap.getCell(nextrow, nextcol);
                        if ((nextCell.getIsVirtualWall() || nextCell.getIsObstacle())) {
                            dist[nextrow][nextcol] = 10000;
                        }
                        else {
                            dist[nextrow][nextcol] = dist[current.getRow()][current.getCol()] + 1;
                            nextExplore.addLast(nextCell);
                        }
                    }
                }
            }
        }
        System.out.println(dist[c.getRow()][c.getCol()]);
        if (dist[c.getRow()][c.getCol()] == -1){
            return null;
        }
        ArrayList<Cell> path = new ArrayList<>();
        Cell current = c;
        path.add(c);

        System.out.println("enter while");
        while(!(current.getRow() == bot.getRobotPosRow() & current.getCol() == bot.getRobotPosCol())) {

            for (int i = 0; i < 4; i++) {
                int nextrow = current.getRow() + rowmov[i];
                int nextcol = current.getCol() + colmov[i];
                if (nextcol >= 0 && nextrow >= 0 && nextcol <= MapConstants.MAP_COLS && nextrow <= MapConstants.MAP_ROWS) {
                    if (dist[nextrow][nextcol] == dist[current.getRow()][current.getCol()] - 1) {
                        current = exploredMap.getCell(nextrow, nextcol);
                        path.add(current);
                        break;
                    }
                }
            }
        }

        System.out.println("Shortest path from " + bot.getRobotPosRow() + " " + bot.getRobotPosCol() + " to " + c.getRow() + " " + c.getCol());
        for (int i = path.size() - 1; i >= 0; i--) {
            System.out.print(path.get(i).getRow() + " " + path.get(i).getCol() + " -> ");
        }
        System.out.println();

        return path;

    }
}
