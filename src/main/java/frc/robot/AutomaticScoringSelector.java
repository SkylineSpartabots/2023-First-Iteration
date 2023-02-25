package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.Elevator.ElevatorStates;

public final class AutomaticScoringSelector {

    private static AutomaticScoringSelector instance;
    private ShuffleboardTab scoringGridDisplay = Shuffleboard.getTab("Scoring Display");
    
    private ScoringPosition[][] grid = new ScoringPosition[3][9]; //[row][column] 0 row is l1, 0 column is the one farthest from load zone
    private boolean[][] isSelected = new boolean[3][9];
    private GenericEntry[][] selectionDisplay = new GenericEntry[3][9];
    private GenericEntry currentGridSelected, selectedX, selectedY;

    private int currRow = 0, currColumn = 0;
    private int selectedRow, selectedColumn;

    public static AutomaticScoringSelector getInstance(){
        if(instance == null){
            instance = new AutomaticScoringSelector();
        }
        return instance;
    }

    public AutomaticScoringSelector(){
        for(int i = 1; i <= 9; i++){
            boolean isCube = i==2 || i==5 || i==8;
            for(int j = 0; j < 2; j++){
                if(isCube){
                    if(!isCube){
                        grid[j][i - 1] = ScoringPositions.ALLPOSES.coneStates.get(i + j - 1);
                    } else {
                        grid[j][i - 1] = ScoringPositions.ALLPOSES.cubeStates.get(i + j - 2);
                    }
                }
            }
        }
        isSelected[currRow][currColumn] = true;
        createDisplay();
    }

    public void moveUp(){
        isSelected[currRow][currColumn] = false;
        currRow++;
        isSelected[currRow][currColumn] = true;
    }

    public void moveDown(){
        isSelected[currRow][currColumn] = false;
        currRow--;
        isSelected[currRow][currColumn] = true;

    }

    public void moveRight(){
        isSelected[currRow][currColumn] = false;
        currColumn--;
        isSelected[currRow][currColumn] = true;

    }

    public void moveLeft(){
        isSelected[currRow][currColumn] = false;
        currColumn++;
        isSelected[currRow][currColumn] = true;
    }

    public void select(){
        selectedRow = currRow;
        selectedColumn = currColumn;
    }

    public void createDisplay(){
        for(int i = 0; i < 3; i++){
            for(int j = 8; j > -1; j--){
                selectionDisplay[i][j] = scoringGridDisplay.add("", isSelected[i][j])
                .getEntry();
            }
        }
        currentGridSelected = scoringGridDisplay.add("selection updated", currRow == selectedRow && currColumn == selectedColumn).getEntry();
        selectedX = scoringGridDisplay.add("Selected X", getSelectedPose().getX()).getEntry();
        selectedY = scoringGridDisplay.add("Selected Y", getSelectedPose().getY()).getEntry();
    }

    public void updateShuffleboard(){
        for(int i = 0; i < 3; i++){
            for(int j = 8; j > -1; j--){
                selectionDisplay[i][j].setBoolean(isSelected[i][j]);
            }
        }
        currentGridSelected.setBoolean(currRow == selectedRow && currColumn == selectedColumn);
        selectedX.setDouble(getSelectedPose().getX());
        selectedY.setDouble(getSelectedPose().getY());
    }

    public Pose2d getSelectedPose(){
        return grid[selectedRow][selectedColumn].targetPos;
    }

    public ArmStates getArmState(){
        return grid[selectedRow][selectedColumn].armState;
    }
    
    public ElevatorStates getElevStates(){
        return grid[selectedRow][selectedColumn].elevatorState;
    }

    public enum ScoringPositions {
        ALLPOSES();
        
        //botbot refers to the bottom grid of 9, and the bottom row in that grid. L1 refers to height level. Orientation is bird's eye, same as path planner
        ArrayList<ScoringPosition> coneStates; //it goes: [botbotL1, botbotL2, botbotL3, bottopL1... ...toptopL3] size: 18
        ArrayList<ScoringPosition> cubeStates; //it goes: [botmidL1, botmidL2, botmidL3, midmidL1... ...topmidL3] size: 9

        private ScoringPositions(){
            double x = 2.0; //should be the same for every default scoring position
            double y = 0.4; //only one that changes
            double yIncrem = 0.75; //how much each y varies by - currently unmeasured
            Rotation2d rot = new Rotation2d(180.0); // should be the same for every default scoring position
            ArmStates[] armStatesCone = {ArmStates.L1CONE, ArmStates.L2CONE, ArmStates.L3CONE};
            ArmStates[] armStatesCube = {ArmStates.L1CUBE, ArmStates.L2CUBE, ArmStates.L3CUBE};
            ElevatorStates[] elevStatesCone = {ElevatorStates.L1CONE, ElevatorStates.L2CONE, ElevatorStates.L3CONE};
            ElevatorStates[] elevStatesCube = {ElevatorStates.L1CUBE, ElevatorStates.L2CUBE, ElevatorStates.L3CUBE};

            for(int i = 1; i <= 3; i++){
                for(int j = 1; j <= 3; j++){
                    y += yIncrem;
                    Pose2d tempPose = new Pose2d(x, y, rot);
                    for(int b = 1; b <= 3; b++){
                        if(j%2==0){
                            cubeStates.add(AutomaticScoringSelector.getInstance().new ScoringPosition(tempPose, armStatesCube[b - 1], elevStatesCube[b - 1]));
                        } else {
                            coneStates.add(AutomaticScoringSelector.getInstance().new ScoringPosition(tempPose, armStatesCone[b - 1], elevStatesCone[b - 1]));
                        }
                    }
                }
            }
        }
    }

    class ScoringPosition {

        Pose2d targetPos;
        ArmStates armState;
        ElevatorStates elevatorState;

        private ScoringPosition(Pose2d pose, ArmStates armState, ElevatorStates elevState){
            targetPos = pose;
            this.armState = armState;
            this.elevatorState = elevState;
        }
    }
}
