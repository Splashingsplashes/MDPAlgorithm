package map;
import java.util.*;

import robot.Robot;
import robot.RobotConstants;
import sun.java2d.Surface;
import sun.lwawt.macosx.CSystemTray;

import java.lang.*;
import javax.swing.*;
import java.awt.*;
import java.lang.reflect.Array;
import java.util.List;

import static robot.RobotConstants.DIRECTION.NORTH;

/**
 * Represents the entire map grid for the arena.
 *
 * @author Suyash Lakhotia
 */

public class Map extends JPanel {
    private final Cell[][] grid;
    private final Robot bot;
    private static List<List<Integer>> imageCoverage = new ArrayList<List<Integer>>();
    private static List<List<Integer>> cameraPos = new ArrayList<List<Integer>>();
    private List<List<Integer>> imagePos = new ArrayList<List<Integer>>();
    private static boolean takeImage = false;
    private static int stepCount = 2;
    private static RobotConstants.DIRECTION prev_d = NORTH;
    private static List<ObsSurface> surfaceCoverage = new ArrayList<ObsSurface>();
    public static HashMap<String, ObsSurface> surfaceTaken = new HashMap<String, ObsSurface>();
    private static List<ObsSurface> notYetTakenList = new ArrayList<ObsSurface>();
    private static List<ObsSurface> notAccessibleSurface = new ArrayList<ObsSurface>();
    private boolean makeUpImageRun = false;
    private boolean arriveAtImagePos = false;

    /**
     * Initialises a Map object with a grid of Cell objects.
     */
    public Map(Robot bot) {
        this.bot = bot;

        grid = new Cell[MapConstants.MAP_ROWS][MapConstants.MAP_COLS];
        for (int row = 0; row < grid.length; row++) {
            for (int col = 0; col < grid[0].length; col++) {
                grid[row][col] = new Cell(row, col);

                // Set the virtual walls of the arena
                if (row == 0 || col == 0 || row == MapConstants.MAP_ROWS - 1 || col == MapConstants.MAP_COLS - 1) {
                    grid[row][col].setVirtualWall(true);
                }
            }
        }
    }

    public boolean checkValidCell(int row, int col) {
        boolean res = row >= 0 && col >= 0 && row < MapConstants.MAP_ROWS && col < MapConstants.MAP_COLS;
        return res;
    }
    public Cell getCell(Point pos) {
        return grid[pos.y][pos.x];
    }
    public HashMap<Direction, Cell> getNeighboursMap(Cell c) {

        HashMap<Direction, Cell> neighbours = new HashMap<Direction, Cell>();
        Point up = new Point(c.getCol() , c.getRow() + 1);
        Point down = new Point(c.getCol(), c.getRow() - 1);
        Point left = new Point(c.getCol() - 1 , c.getRow());
        Point right = new Point(c.getCol() + 1 , c.getRow() );

        // UP
        if (checkValidCell(up.y, up.x)){
            neighbours.put(Direction.UP, getCell(up));
        }

        // DOWN
        if (checkValidCell(down.y, down.x)) {
            neighbours.put(Direction.DOWN, getCell(down));
        }

        // LEFT
        if (checkValidCell(left.y, left.x)){
            neighbours.put(Direction.LEFT, getCell(left));
        }

        // RIGHT
        if (checkValidCell(right.y, right.x)){
            neighbours.put(Direction.RIGHT, getCell(right));
        }

        return neighbours;
    }

    /**
     * Returns true if the row and column values are valid.
     */
    public boolean checkValidCoordinates(int row, int col) {
        return row >= 0 && col >= 0 && row < MapConstants.MAP_ROWS && col < MapConstants.MAP_COLS;
    }

    /**
     * Returns true if the row and column values are in the start zone.
     */
    private boolean inStartZone(int row, int col) {
        return row >= 0 && row <= 2 && col >= 0 && col <= 2;
    }

    /**
     * Returns true if the row and column values are in the goal zone.
     */
    private boolean inGoalZone(int row, int col) {
        return (row <= MapConstants.GOAL_ROW + 1 && row >= MapConstants.GOAL_ROW - 1 && col <= MapConstants.GOAL_COL + 1 && col >= MapConstants.GOAL_COL - 1);
    }

    /**
     * Returns a particular cell in the grid.
     */
    public Cell getCell(int row, int col) {
        return grid[row][col];
    }

    /**
     * Returns true if a cell is an obstacle.
     */
    public boolean isObstacleCell(int row, int col) {
        return grid[row][col].getIsObstacle();
    }

    /**
     * Returns true if a cell is a virtual wall.
     */
    public boolean isVirtualWallCell(int row, int col) {
        return grid[row][col].getIsVirtualWall();
    }

    /**
     * Sets all cells in the grid to an explored state.
     */
    public void setAllExplored() {
        for (int row = 0; row < grid.length; row++) {
            for (int col = 0; col < grid[0].length; col++) {
                grid[row][col].setIsExplored(true);
            }
        }
    }

    /**
     * Sets all cells in the grid to an unexplored state except for the START & GOAL zone.
     */
    public void setAllUnexplored() {
        for (int row = 0; row < grid.length; row++) {
            for (int col = 0; col < grid[0].length; col++) {
                if (inStartZone(row, col) || inGoalZone(row, col)) {
                    grid[row][col].setIsExplored(true);
                } else {
                    grid[row][col].setIsExplored(false);
                }
            }
        }
    }

    /**
     * Sets a cell as an obstacle and the surrounding cells as virtual walls or resets the cell and surrounding
     * virtual walls.
     */
    public void setObstacleCell(int row, int col, boolean obstacle) {
        if (obstacle && (inStartZone(row, col) || inGoalZone(row, col)))
            return;

        grid[row][col].setIsObstacle(obstacle);

        if (row >= 1) {
            grid[row - 1][col].setVirtualWall(obstacle);            // bottom cell

            if (col < MapConstants.MAP_COLS - 1) {
                grid[row - 1][col + 1].setVirtualWall(obstacle);    // bottom-right cell
            }

            if (col >= 1) {
                grid[row - 1][col - 1].setVirtualWall(obstacle);    // bottom-left cell
            }
        }

        if (row < MapConstants.MAP_ROWS - 1) {
            grid[row + 1][col].setVirtualWall(obstacle);            // top cell

            if (col < MapConstants.MAP_COLS - 1) {
                grid[row + 1][col + 1].setVirtualWall(obstacle);    // top-right cell
            }

            if (col >= 1) {
                grid[row + 1][col - 1].setVirtualWall(obstacle);    // top-left cell
            }
        }

        if (col >= 1) {
            grid[row][col - 1].setVirtualWall(obstacle);            // left cell
        }

        if (col < MapConstants.MAP_COLS - 1) {
            grid[row][col + 1].setVirtualWall(obstacle);            // right cell
        }
    }

    /**
     * Returns true if the given cell is out of bounds or an obstacle.
     */
    public boolean getIsObstacleOrWall(int row, int col) {
        return !checkValidCoordinates(row, col) || getCell(row, col).getIsObstacle();
    }

    public boolean checkObstacleOTW(RobotConstants.DIRECTION d, int cam_r, int cam_c, int image_r, int image_c){
        int r;
        int c;
        boolean res = false;
        switch (d) {
            case NORTH:
                if (cam_r > image_r){
                    c = image_c + 1;
                    for(r = image_r+1;r < cam_r;r++){
                        if (isObstacleCell(r,c))
                            res = true;
                        c += 1;
                    }
                }
                if (cam_r < image_r){
                    c = image_c + 1;
                    for(r = image_r-1;r > cam_r;r--){
                        if (isObstacleCell(r,c))
                            res = true;

                        c += 1;
                    }
                }
                break;
            case EAST:
                if (image_c < cam_c){
                    r = image_r - 1;
                    for(c = image_c + 1;c < cam_c;c++){
                        if (isObstacleCell(r,c))
                            res = true;
                        r -= 1;
                    }
                }
                if (image_c > cam_c){
                    r = image_r - 1;
                    for(c = image_c - 1;c > cam_c;c--) {
                        if (isObstacleCell(r, c))
                            res = true;
                        r -= 1;
                    }
                }
                break;
            case SOUTH:
                if (image_r < cam_r){
                    c = image_c - 1;
                    for(r = image_r+1;r < cam_r;r++){
                        if (isObstacleCell(r,c))
                            res = true;
                        c -= 1;
                    }
                }
                if (cam_r < image_r){
                    c = image_c - 1;
                    for(r = image_r-1;r > cam_r;r--){
                        if (isObstacleCell(r,c))
                            res = true;

                        c -= 1;
                    }
                }
                break;
            case WEST:
                if (image_c < cam_c){
                    r = image_r + 1;
                    for(c = image_c + 1;c < cam_c;c++){
                        if (isObstacleCell(r,c))
                            res = true;
                        r += 1;
                    }
                }
                if (image_c > cam_c){
                    r = image_r + 1;
                    for(c = image_c - 1;c > cam_c;c--) {
                        if (isObstacleCell(r, c))
                            res = true;
                        r += 1;
                    }
                }
                break;
        }

        return res;

    }

    public boolean computeImageCoverage(Graphics g,RobotConstants.DIRECTION d,int r,int c){
        boolean res_flag = false;
        if((takeImage&&!makeUpImageRun) || (makeUpImageRun&&arriveAtImagePos)) {
            res_flag = true;
            takeImage = false;
            stepCount = 4;
            arriveAtImagePos = false;
            List<Integer> res = new ArrayList<Integer>();

            int x_offset = 120;
            int y_offset = 30;
            int size = 30;
            int depthLimit = 10;
            System.out.println(d);
            List<Integer> flag = new ArrayList<Integer>();
            List<Integer> leftFlag = new ArrayList<Integer>();
            List<Integer> rightFlag = new ArrayList<Integer>();

            ObsSurface surface;

            switch (d) {
                case NORTH:
//                    c-=2;
                    for (int depth = 0; depth < depthLimit; depth++){
                        for (int spreadth = 0; spreadth <= depth; spreadth++){
//                            if (flag.contains(spreadth))
//                                continue;
//                            if (rightFlag.isEmpty()||(spreadth>   Collections.max(rightFlag))){
                            if (((r+spreadth < 20 ) && (c-depth >= 0)&& (grid[r + spreadth][c - depth].getIsObstacle())&&(!grid[r+spreadth][c-depth+1].getIsObstacle()))){
                                if(!checkObstacleOTW(d,r,c,r+spreadth,c-depth)) {
                                    res = Arrays.asList(x_offset + (c-depth) * size + size, y_offset + (MapConstants.MAP_ROWS - (r+spreadth)) * size, x_offset + (c - depth) * size + size, y_offset + (MapConstants.MAP_ROWS - (r+spreadth) - 1) * size);

                                    flag.add(spreadth);
                                    rightFlag.add(spreadth);
                                    imageCoverage.add(res);

                                    surface = new ObsSurface(r+spreadth,c-depth,Direction.RIGHT);
                                    surfaceTaken.put(surface.toString(), surface);
                                    surfaceCoverage.add(surface);
                                }
                            }
                            if (((r - spreadth >= 0) && (c - depth >= 0) && (grid[r - spreadth][c - depth].getIsObstacle()) && (!grid[r - spreadth][c - depth + 1].getIsObstacle()))) {
                                if(!checkObstacleOTW(d,r,c,r-spreadth,c-depth)) {
                                    res = Arrays.asList(x_offset + (c - depth) * size + size, y_offset + (MapConstants.MAP_ROWS - (r - spreadth)) * size, x_offset + (c - depth) * size + size, y_offset + (MapConstants.MAP_ROWS - (r - spreadth) - 1) * size);
                                    flag.add(spreadth);
                                    leftFlag.add(spreadth);
                                    imageCoverage.add(res);

                                    surface = new ObsSurface(r-spreadth,c-depth,Direction.RIGHT);
                                    surfaceTaken.put(surface.toString(), surface);
                                    surfaceCoverage.add(surface);
                                }
                            }
                        }
                    }

                    break;
                case EAST:
//                    r+=2;
                    for (int depth = 0; depth < depthLimit; depth++){
                        for (int spreadth = 0; spreadth <= depth; spreadth++){
//                            if (flag.contains(spreadth))
//                                continue;
//                            if (((c+spreadth < 15 )&& (r+depth < 20) &&(grid[r+depth][c-spreadth].getIsObstacle()) &&(!grid[r+depth-1][c+spreadth].getIsObstacle()))){
//                                    res = Arrays.asList(x_offset + (c + spreadth) * size, y_offset + (MapConstants.MAP_ROWS - (r + depth)) * size, x_offset + (c + spreadth) * size + size, y_offset + (MapConstants.MAP_ROWS - (r + depth)) * size);
//                                    flag.add(spreadth);
//                                    rightFlag.add(spreadth);
//                                    imageCoverage.add(res);
//                                }
//                            if (((c-spreadth >= 0 )&& (r+depth < 20)&&(grid[r+depth][c-spreadth].getIsObstacle()) &&(!grid[r+depth-1][c-spreadth].getIsObstacle()))) {
//                                    res = Arrays.asList(x_offset + (c - spreadth) * size, y_offset + (MapConstants.MAP_ROWS - (r + depth)) * size, x_offset + (c - spreadth) * size + size, y_offset + (MapConstants.MAP_ROWS - (r + depth)) * size);
//                                    flag.add(spreadth);
//                                    leftFlag.add(spreadth);
//                                    imageCoverage.add(res);
//                                }
//                            if (rightFlag.isEmpty()||(spreadth> Collections.max(rightFlag))){
                            if (((c+spreadth < 15 )&& (r+depth < 20)&&(grid[r+depth][c+spreadth].getIsObstacle())&&(!grid[r+depth-1][c+spreadth].getIsObstacle()))){
                                if(!checkObstacleOTW(d,r,c,r+depth,c+spreadth)) {
                                    res = Arrays.asList(x_offset + (c + spreadth) * size, y_offset + (MapConstants.MAP_ROWS - (r + depth)) * size, x_offset + (c + spreadth) * size + size, y_offset + (MapConstants.MAP_ROWS - (r + depth)) * size);
                                    flag.add(spreadth);
                                    rightFlag.add(spreadth);
                                    imageCoverage.add(res);
                                    surface = new ObsSurface(r+depth,c + spreadth,Direction.DOWN);
                                    surfaceTaken.put(surface.toString(), surface);
                                    surfaceCoverage.add(surface);
                                }
                            }
//                            if (leftFlag.isEmpty()||(spreadth>  Collections.max(leftFlag))) {
                            if (((c-spreadth >= 0 )&& (r+depth < 20)&&(grid[r+depth][c-spreadth].getIsObstacle()) &&(!grid[r+depth-1][c-spreadth].getIsObstacle()))) {
                                if(!checkObstacleOTW(d,r,c,r+depth,c-spreadth)) {
                                    res = Arrays.asList(x_offset + (c - spreadth) * size, y_offset + (MapConstants.MAP_ROWS - (r + depth)) * size, x_offset + (c - spreadth) * size + size, y_offset + (MapConstants.MAP_ROWS - (r + depth)) * size);
                                    flag.add(spreadth);
                                    leftFlag.add(spreadth);
                                    imageCoverage.add(res);

                                    surface = new ObsSurface(r+depth,c - spreadth,Direction.DOWN);
                                    surfaceTaken.put(surface.toString(), surface);
                                    surfaceCoverage.add(surface);
                                }
                            }
                        }
                    }
                    break;
                case SOUTH:
//                    c+=2;
                    for (int depth = 0; depth < depthLimit; depth++){
                        for (int spreadth = 0; spreadth <= depth; spreadth++) {
//                            if (flag.contains(spreadth))
//                                continue;
//                            if (((r + spreadth < 20) && (c + depth < 15) && (grid[r + spreadth][c + depth].getIsObstacle()) && (!grid[r + spreadth][c + depth -1].getIsObstacle()) ) && (rightFlag.isEmpty()||(spreadth> depth + Collections.max(rightFlag)))) {
//                            if (rightFlag.isEmpty()||(spreadth> Collections.max(rightFlag))){
                            if (((r + spreadth < 20) && (c + depth < 15)  && (grid[r + spreadth][c + depth].getIsObstacle()) && (!grid[r + spreadth][c + depth -1].getIsObstacle()))) {
                                if(!checkObstacleOTW(d,r,c,r+spreadth,c+depth)) {
                                    res = Arrays.asList(x_offset + (c + depth) * size, y_offset + (MapConstants.MAP_ROWS - (r + spreadth)) * size, x_offset + (c + depth) * size, y_offset + (MapConstants.MAP_ROWS - (r + spreadth) - 1) * size);

                                    flag.add(spreadth);
                                    rightFlag.add(spreadth);
                                    imageCoverage.add(res);

                                    surface = new ObsSurface(r+spreadth,c + depth,Direction.LEFT);
                                    surfaceTaken.put(surface.toString(), surface);
                                    surfaceCoverage.add(surface);
                                }
                            }
//                            if (((r - spreadth >= 0) && (c + depth < 15) && (grid[r - spreadth][c + depth].getIsObstacle()) && (!grid[r - spreadth][c + depth -1].getIsObstacle()) ) && (leftFlag.isEmpty()||(spreadth> depth + Collections.max(rightFlag)))) {
//                            if (((r - spreadth >= 0) && (c + depth < 15) && (grid[r - spreadth][c + depth].getIsObstacle()) && (!grid[r - spreadth][c + depth -1].getIsObstacle()))) {
//                                res = Arrays.asList(x_offset + (c + depth) * size, y_offset + (MapConstants.MAP_ROWS - ((r - spreadth))) * size, x_offset + (c + depth) * size, y_offset + (MapConstants.MAP_ROWS - (r - spreadth) - 1) * size);
//                                flag.add(spreadth);
//                                leftFlag.add(spreadth);
//                                imageCoverage.add(res);
//                            }
//                            if (leftFlag.isEmpty()||(spreadth>  Collections.max(leftFlag))) {
                            if (((r - spreadth >= 0) && (c + depth < 15) && (grid[r - spreadth][c + depth].getIsObstacle()) && (!grid[r - spreadth][c + depth -1].getIsObstacle()))) {
                                if(!checkObstacleOTW(d,r,c,r-spreadth,c+depth)) {
                                    res = Arrays.asList(x_offset + (c + depth) * size, y_offset + (MapConstants.MAP_ROWS - ((r - spreadth))) * size, x_offset + (c + depth) * size, y_offset + (MapConstants.MAP_ROWS - (r - spreadth) - 1) * size);
                                    flag.add(spreadth);
                                    leftFlag.add(spreadth);
                                    imageCoverage.add(res);

                                    surface = new ObsSurface(r-spreadth,c + depth,Direction.LEFT);
                                    surfaceTaken.put(surface.toString(), surface);
                                    surfaceCoverage.add(surface);
                                }
                            }
                        }
                    }

                    break;
                case WEST:
//                    r-=2;
                    for (int depth = 0; depth < depthLimit; depth++){
                        for (int spreadth = 0; spreadth <= depth; spreadth++){
//                            if (flag.contains(spreadth))
//                                continue;
//                            if (rightFlag.isEmpty()||(spreadth> Collections.max(rightFlag))){
                            if (((c+spreadth < 15 )&& (r-depth >= 0) &&(grid[r-depth][c+spreadth].getIsObstacle()) && (!grid[r-depth+1][c+spreadth].getIsObstacle()))){
                                if(!checkObstacleOTW(d,r,c,r-depth,c+spreadth)) {
                                    res = Arrays.asList(x_offset+(c+spreadth)*size,y_offset+(MapConstants.MAP_ROWS-(r-depth)-1)*size,x_offset+(c+spreadth)*size+size,y_offset+(MapConstants.MAP_ROWS-(r-depth)-1)*size);
                                    imageCoverage.add(res);
                                    rightFlag.add(spreadth);
                                    flag.add(spreadth);

                                    surface = new ObsSurface(r-depth,c + spreadth,Direction.UP);
                                    surfaceTaken.put(surface.toString(), surface);
                                    surfaceCoverage.add(surface);
                                }
                            }
//                            if (leftFlag.isEmpty()||(spreadth> Collections.max(leftFlag))){
                            if (((c-spreadth >= 0 )&& (r-depth >= 0)&&(grid[r-depth][c-spreadth].getIsObstacle()) && (!grid[r-depth+1][c-spreadth].getIsObstacle()))){
                                if(!checkObstacleOTW(d,r,c,r-depth,c-spreadth)) {
                                    res = Arrays.asList(x_offset+(c-spreadth)*size,y_offset+(MapConstants.MAP_ROWS-(r-depth)-1)*size,x_offset+(c-spreadth)*size+size,y_offset+(MapConstants.MAP_ROWS-(r-depth)-1)*size);
                                    imageCoverage.add(res);
                                    leftFlag.add(spreadth);
                                    flag.add(spreadth);

                                    surface = new ObsSurface(r-depth,c - spreadth,Direction.UP);
                                    surfaceTaken.put(surface.toString(), surface);
                                    surfaceCoverage.add(surface);
                                }
                            }
                        }
                    }
                    break;
            }

        }
        else{
            stepCount-=1;
        }
        return res_flag;
    }

    /**
     * Overrides JComponent's paintComponent() method. It creates a two-dimensional array of _DisplayCell objects
     * to store the current map state. Then, it paints square cells for the grid with the appropriate colors as
     * well as the robot on-screen.
     */
    public void paintComponent(Graphics g) {
        // Create a two-dimensional array of _DisplayCell objects for rendering.
        _DisplayCell[][] _mapCells = new _DisplayCell[MapConstants.MAP_ROWS][MapConstants.MAP_COLS];
        for (int mapRow = 0; mapRow < MapConstants.MAP_ROWS; mapRow++) {
            for (int mapCol = 0; mapCol < MapConstants.MAP_COLS; mapCol++) {
                _mapCells[mapRow][mapCol] = new _DisplayCell(mapCol * GraphicsConstants.CELL_SIZE, mapRow * GraphicsConstants.CELL_SIZE, GraphicsConstants.CELL_SIZE);
            }
        }


        // Paint the cells with the appropriate colors.
        for (int mapRow = 0; mapRow < MapConstants.MAP_ROWS; mapRow++) {
            for (int mapCol = 0; mapCol < MapConstants.MAP_COLS; mapCol++) {
                Color cellColor;

                if (inStartZone(mapRow, mapCol))
                    cellColor = GraphicsConstants.C_START;
                else if (inGoalZone(mapRow, mapCol))
                    cellColor = GraphicsConstants.C_GOAL;
                else {
                    if (!grid[mapRow][mapCol].getIsExplored()) {
                        cellColor = GraphicsConstants.C_UNEXPLORED;
                    }
                    else if (grid[mapRow][mapCol].getIsObstacle()) {
                        cellColor = GraphicsConstants.C_OBSTACLE;
                    }
//                    else if (grid[mapRow][mapCol].getIsVirtualWall()) {
//                        cellColor = Color.orange;
//                    }
                    else
                        cellColor = GraphicsConstants.C_FREE;
                }

                g.setColor(cellColor);
                g.fillRect(_mapCells[mapRow][mapCol].cellX + GraphicsConstants.MAP_X_OFFSET, _mapCells[mapRow][mapCol].cellY, _mapCells[mapRow][mapCol].cellSize, _mapCells[mapRow][mapCol].cellSize);

            }
        }

        RobotConstants.DIRECTION d = bot.getRobotCurDir();
        int r = bot.getRobotPosRow();
        int c = bot.getRobotPosCol();

        if(d != prev_d)
            takeImage = true;
        prev_d = d;

        // Paint the robot on-screen.
        g.setColor(GraphicsConstants.C_ROBOT);
        g.fillOval((c - 1) * GraphicsConstants.CELL_SIZE + GraphicsConstants.ROBOT_X_OFFSET + GraphicsConstants.MAP_X_OFFSET, GraphicsConstants.MAP_H - (r * GraphicsConstants.CELL_SIZE + GraphicsConstants.ROBOT_Y_OFFSET), GraphicsConstants.ROBOT_W, GraphicsConstants.ROBOT_H);

        // Paint the robot's direction indicator on-screen.
        g.setColor(GraphicsConstants.C_ROBOT_DIR);
        switch (d) {
            case NORTH:
                g.fillOval(c * GraphicsConstants.CELL_SIZE + 10 + GraphicsConstants.MAP_X_OFFSET, GraphicsConstants.MAP_H - r * GraphicsConstants.CELL_SIZE - 15, GraphicsConstants.ROBOT_DIR_W, GraphicsConstants.ROBOT_DIR_H);
                break;
            case EAST:
                g.fillOval(c * GraphicsConstants.CELL_SIZE + 35 + GraphicsConstants.MAP_X_OFFSET, GraphicsConstants.MAP_H - r * GraphicsConstants.CELL_SIZE + 10, GraphicsConstants.ROBOT_DIR_W, GraphicsConstants.ROBOT_DIR_H);
                break;
            case SOUTH:
                g.fillOval(c * GraphicsConstants.CELL_SIZE + 10 + GraphicsConstants.MAP_X_OFFSET, GraphicsConstants.MAP_H - r * GraphicsConstants.CELL_SIZE + 35, GraphicsConstants.ROBOT_DIR_W, GraphicsConstants.ROBOT_DIR_H);
                break;
            case WEST:
                g.fillOval(c * GraphicsConstants.CELL_SIZE - 15 + GraphicsConstants.MAP_X_OFFSET, GraphicsConstants.MAP_H - r * GraphicsConstants.CELL_SIZE + 10, GraphicsConstants.ROBOT_DIR_W, GraphicsConstants.ROBOT_DIR_H);
                break;
        }

        if(computeImageCoverage(g,d,r,c)) {
            List<Integer> pos = new ArrayList<>();
            List<Integer> cPos = new ArrayList<>();
            switch (d) {
                case NORTH:
                    c+=1;
                    pos = Arrays.asList(r,c);
                    cPos = Arrays.asList(c * GraphicsConstants.CELL_SIZE  + GraphicsConstants.MAP_X_OFFSET, GraphicsConstants.MAP_H - r * GraphicsConstants.CELL_SIZE + 15);
                    break;
                case EAST:
                    r-=1;
                    pos = Arrays.asList(r,c);
                    cPos = Arrays.asList(c * GraphicsConstants.CELL_SIZE + 12 + GraphicsConstants.MAP_X_OFFSET, GraphicsConstants.MAP_H - r * GraphicsConstants.CELL_SIZE - 3);
                    break;
                case SOUTH:
                    c-=1;
                    pos = Arrays.asList(r,c);
                    cPos = Arrays.asList(c * GraphicsConstants.CELL_SIZE + 26 + GraphicsConstants.MAP_X_OFFSET, GraphicsConstants.MAP_H - r * GraphicsConstants.CELL_SIZE + 12);
                    break;
                case WEST:
                    r+=1;
                    pos = Arrays.asList(r,c);
                    cPos = Arrays.asList(c * GraphicsConstants.CELL_SIZE + 12 + GraphicsConstants.MAP_X_OFFSET, GraphicsConstants.MAP_H - r * GraphicsConstants.CELL_SIZE + 23);
                    break;
            }

            imagePos.add(pos);
            cameraPos.add(cPos);
        }
        for (List<Integer> pos : imagePos){
            g.setColor(Color.magenta);
            g.fillRect(_mapCells[pos.get(0)][pos.get(1)].cellX + GraphicsConstants.MAP_X_OFFSET, _mapCells[pos.get(0)][pos.get(1)].cellY, _mapCells[pos.get(0)][pos.get(1)].cellSize, _mapCells[pos.get(0)][pos.get(1)].cellSize);
        }

        for (List<Integer> pos : cameraPos){
            g.setColor(Color.cyan);
            g.fillOval(pos.get(0),pos.get(1),6,6);
        }
//
//        paintSurface(surfaceCoverage,g);
        if(notYetTakenList!=null && notYetTakenList.size()!=0){
            paintSurface(notYetTakenList,g,Color.cyan);
        }
        if(notAccessibleSurface!=null && notAccessibleSurface.size()!=0){
            paintSurface(notAccessibleSurface,g,Color.RED);
        }
//            System.out.println("painted surfaces");
//            System.out.println(notYetTakenList);


    }
    public void paintSurface(List<ObsSurface>coverage,Graphics g, Color color){

        List<Integer> res = new ArrayList<Integer>();

        Graphics2D g2 = (Graphics2D) g;
        g2.setColor(color);
        g2.setStroke(new BasicStroke(3));
        int x_offset = GraphicsConstants.MAP_X_OFFSET;
        int y_offset = GraphicsConstants.MAP_Y_OFFSET;
        int size = GraphicsConstants.CELL_SIZE;

        for (ObsSurface surface : coverage){
            switch(surface.getSurface()){
                case UP:
                {
                    res = Arrays.asList(x_offset+ surface.getCol()*size,y_offset+(MapConstants.MAP_ROWS-surface.getRow()-1)*size,x_offset+ surface.getCol()*size+size,y_offset+(MapConstants.MAP_ROWS-surface.getRow()-1)*size);
                    g2.drawLine(res.get(0),res.get(1),res.get(2),res.get(3));
                    break;
                }
                case DOWN:
                {
                    res = Arrays.asList(x_offset + surface.getCol() * size, y_offset + (MapConstants.MAP_ROWS - surface.getRow()) * size, x_offset +  surface.getCol() * size + size, y_offset + (MapConstants.MAP_ROWS - surface.getRow()) * size);
                    g2.drawLine(res.get(0),res.get(1),res.get(2),res.get(3));
                    break;
                }
                case LEFT:
                {
                    res = Arrays.asList(x_offset + surface.getCol() * size, y_offset + (MapConstants.MAP_ROWS - (surface.getRow())) * size, x_offset + surface.getCol() * size, y_offset + (MapConstants.MAP_ROWS - surface.getRow() - 1) * size);
                    g2.drawLine(res.get(0),res.get(1),res.get(2),res.get(3));
                    break;
                }
                case RIGHT:
                {
                    res = Arrays.asList(x_offset + surface.getCol() * size + size, y_offset + (MapConstants.MAP_ROWS - surface.getRow()) * size, x_offset + surface.getCol() * size + size, y_offset + (MapConstants.MAP_ROWS - surface.getRow() - 1) * size);
                    g2.drawLine(res.get(0),res.get(1),res.get(2),res.get(3));
                    break;
                }
            }
        }
        surfaceTaken.clear();
    }

    private class _DisplayCell {
        public final int cellX;
        public final int cellY;
        public final int cellSize;

        public _DisplayCell(int borderX, int borderY, int borderSize) {
            this.cellX = borderX + GraphicsConstants.CELL_LINE_WEIGHT;
            this.cellY = GraphicsConstants.MAP_H - (borderY - GraphicsConstants.CELL_LINE_WEIGHT);
            this.cellSize = borderSize - (GraphicsConstants.CELL_LINE_WEIGHT * 2);
        }
    }
    /**
     * Sets all cells in the grid to an unexplored state except for the START & GOAL zone.
     */
    public void clearAll() {
        for (int row = 0; row < grid.length; row++) {
            for (int col = 0; col < grid[0].length; col++) {
                grid[row][col] = new Cell(row, col);

                // Set the virtual walls of the arena
                if (row == 0 || col == 0 || row == MapConstants.MAP_ROWS - 1 || col == MapConstants.MAP_COLS - 1) {
                    grid[row][col].setVirtualWall(true);
                }
            }
        }
        surfaceCoverage.clear();
        imagePos.clear();
        cameraPos.clear();
        notYetTakenList.clear();
        surfaceTaken.clear();
    }

    public List<ObsSurface> getSurfaceCoverage(){
        if(surfaceCoverage!=null) {
            System.out.println("coverage not null");
            List<ObsSurface> res = new ArrayList<ObsSurface>(new HashSet<>(surfaceCoverage));
            return res;
        }
        else return null;
    }
    public List<ObsSurface> getSurfaceTaken(){
        if(surfaceTaken!=null) {
            System.out.println("current coverage not null");
            List<ObsSurface> res = new ArrayList<ObsSurface>();
            for(String surface:surfaceTaken.keySet()){
                res.add(surfaceTaken.get(surface));
            }
            return res;
        }
        else return null;
    }

    public void setNotYetTakenList(List<ObsSurface> surfaces){
        notYetTakenList = surfaces;
    }
    public void addNotAccessibleSurface(ObsSurface surfaces){
        notAccessibleSurface.add(surfaces);
    }

    public ObsSurface nearestObsSurface(Point loc, HashMap<String, ObsSurface> notYetTaken) {
        double dist = 1000, tempDist;
        Point tempPos;
        ObsSurface nearest = null;

        for (ObsSurface obstacle: notYetTaken.values()) {
//            tempPos = obstacle.getPos();
            // neighbour cell of that surface
            tempPos = getNeighbour(obstacle.getPos(), obstacle.getSurface());
            tempDist = loc.distance(tempPos);
            if (tempDist < dist) {
                dist = tempDist;
                nearest = obstacle;
            }
        }
        return nearest;
    }
    public Cell nearestMovable(ObsSurface obsSurface) {
        double distance = 1000, tempDist;
        Cell nearest = null;
        Cell tempCell;
        int rowInc = 0, colInc = 0;
        int obsRow = obsSurface.getRow();
        int obsCol = obsSurface.getCol();
        switch (obsSurface.getSurface()) {
            case UP: {
                rowInc = 1;
                colInc = 0;
                break;
            }
            case DOWN:{
                rowInc = -1;
                colInc = 0;
                break;}
            case LEFT:{
                colInc = -1;
                rowInc = 0;
                break;
            }
            case RIGHT:{
                colInc = 1;
                rowInc = 0;
                break;
            }
        }

        // up or down
        if (rowInc == -1) {
            for (int row = obsRow + 5 * rowInc; row <= obsRow + 1 * rowInc; row++) {
                for (int col = obsCol - 1; col <= obsCol + 1; col++) {
                    if (checkValidMove(row, col)) {
                        tempCell = grid[row][col];
                        tempDist = obsSurface.getPos().distance(tempCell.getPos());
                        if (distance > tempDist) {
                            nearest = tempCell;
                            distance = tempDist;
                        }
                    }
                }
            }
        }
        if (rowInc == 1) {
            for (int row = obsRow + 1 * rowInc; row <= obsRow + 5 * rowInc; row++) {
                for (int col = obsCol - 1; col <= obsCol + 1; col++) {
                    if (checkValidMove(row, col)) {
                        tempCell = grid[row][col];
                        tempDist = obsSurface.getPos().distance(tempCell.getPos());
                        if (distance > tempDist) {
                            nearest = tempCell;
                            distance = tempDist;
                        }
                    }
                }
            }
        }
        else if (colInc == -1) {
            for (int col = obsCol + 5 * colInc; col <= obsCol + 1* colInc; col++) {
                for (int row = obsRow - 1; row <= obsRow + 1; row++) {
                    if (checkValidMove(row, col)) {
                        tempCell = grid[row][col];
                        tempDist = obsSurface.getPos().distance(tempCell.getPos());
                        if (distance > tempDist) {
                            nearest = tempCell;
                            distance = tempDist;
                        }
                    }
                }
            }
        }
        else if (colInc == 1) {
            for (int col = obsCol + 1 * colInc; col <= obsCol + 5 * colInc; col++) {
                for (int row = obsRow - 1; row <= obsRow + 1; row++) {
                    if (checkValidMove(row, col)) {
                        tempCell = grid[row][col];
                        tempDist = obsSurface.getPos().distance(tempCell.getPos());
                        if (distance > tempDist) {
                            nearest = tempCell;
                            distance = tempDist;
                        }
                    }
                }
            }
        }

        return nearest;
    }


    public boolean checkValidMove(int row, int col) {
//        System.out.println("check valid cell");
//        if(checkValidCell(row, col)){
//            System.out.println(getCell(row, col).getPos());
//            System.out.println(getCell(row, col).getIsVirtualWall());
//            System.out.println(getCell(row, col).getIsObstacle());
//            System.out.println(getCell(row, col).getIsExplored());
//        }
//        boolean OTW = checkObstacleOTW(bot.getRobotCurDir(),)
        boolean res = checkValidCell(row, col) && !getCell(row, col).getIsVirtualWall() && !getCell(row, col).getIsObstacle() && getCell(row,col).getIsExplored();
//        System.out.println("res is"+res);
        return res;
    }

    public Point getNeighbour(Point pos, Direction surfDir) {

        Point n = null;

        switch (surfDir) {
            case UP:
                n = new Point(pos.x , pos.y + 1);
                break;
            case DOWN:
                n = new Point(pos.x, pos.y - 1);
                break;
            case LEFT:
                n = new Point(pos.x - 1, pos.y);
                break;
            case RIGHT:
                n = new Point(pos.x + 1, pos.y);
                break;
        }
        return n;
    }

    public void appendSurfaceCoverage(ObsSurface surface){
        surfaceCoverage.add(surface);
    }

    public void setMakeUpImageRun(boolean flag){
        this.makeUpImageRun = flag;
    }
    public void setArriveAtImagePos(boolean flag){
        this.arriveAtImagePos = flag;
    }

}
