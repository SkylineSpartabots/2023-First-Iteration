package frc.robot;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.Elevator.ElevatorStates;

public final class AutomaticScoringSelector {

    Swerve s_Swerve;
    public BooleanSupplier inPosition = () -> inPosition();
    private static AutomaticScoringSelector instance;
    private ShuffleboardTab scoringGridDisplay = Shuffleboard.getTab("Scoring Display");

    private ScoringPosition[][] grid = new ScoringPosition[3][9]; // [row][column] 0 row is l1, 0 column is the one
                                                                  // farthest from load zone
    private boolean[][] isSelected = new boolean[3][9];
    private GenericEntry[][] selectionDisplay = new GenericEntry[3][9];
    private GenericEntry currentGridSelected, selectedX, selectedY, selectedRot;

    public int currRow = 0;
    public int currColumn = 0;
    private int selectedRow = -1, selectedColumn = -1;

    public static AutomaticScoringSelector getInstance() {
        if (instance == null) {
            instance = new AutomaticScoringSelector();
        }
        return instance;
    }


    public enum ScoringPositions {
        ALLPOSES();

        // botbot refers to the bottom grid of 9, and the bottom row in that grid. L1
        // refers to height level. Orientation is bird's eye, same as path planner
        ArrayList<ScoringPosition> coneStates; // it goes: [botbotL1, botbotL2, botbotL3, bottopL1... ...toptopL3] size:
                                               // 18
        ArrayList<ScoringPosition> cubeStates; // it goes: [botmidL1, botmidL2, botmidL3, midmidL1... ...topmidL3] size:
                                               // 9

        private ScoringPositions() {
            double x = 2.0; // should be the same for every default scoring position
            double y = 0.4; // only one that changes
            double yIncrem = 0.5; // how much each y varies by - currently unmeasured
            Rotation2d rot = new Rotation2d(Math.PI); // should be the same for every default scoring position
            ArmStates[] armStatesCone = { ArmStates.L1CONE, ArmStates.L2CONE, ArmStates.L3CONE };
            ArmStates[] armStatesCube = { ArmStates.L1CUBE, ArmStates.L2CUBE, ArmStates.L3CUBE };
            ElevatorStates[] elevStatesCone = { ElevatorStates.L1CONE, ElevatorStates.L2CONE, ElevatorStates.L3CONE };
            ElevatorStates[] elevStatesCube = { ElevatorStates.L1CUBE, ElevatorStates.L2CUBE, ElevatorStates.L3CUBE };
            coneStates = new ArrayList<ScoringPosition>();
            cubeStates = new ArrayList<ScoringPosition>();

            for (int i = 1; i <= 3; i++) {
                for (int j = 1; j <= 3; j++) {
                    y += yIncrem;
                    for (int b = 1; b <= 3; b++) {
                        if (j % 2 == 0) {
                            cubeStates.add(new ScoringPosition(new Pose2d(x, y, rot), armStatesCube[b - 1],
                                    elevStatesCube[b - 1]));
                        } else {
                            coneStates.add(new ScoringPosition(new Pose2d(x, y, rot), armStatesCone[b - 1],
                                    elevStatesCone[b - 1]));
                        }
                    }
                }
            }
        }
    }

    public AutomaticScoringSelector() {
        s_Swerve = Swerve.getInstance();
        for (int i = 1; i <= 9; i++) {
            boolean isCube = i == 2 || i == 5 || i == 8;
            for (int j = 0; j < 3; j++) {
                if (!isCube) {
                    grid[j][i - 1] = ScoringPositions.ALLPOSES.coneStates.get(i + j - 1);
                } else {
                    grid[j][i - 1] = ScoringPositions.ALLPOSES.cubeStates.get(i + j - 2);
                }
            }
        }
        isSelected[currRow][currColumn] = true;
    }

    public void moveUp() {
        if (currRow >= 2) {
            return;
        }
        isSelected[currRow][currColumn] = false;
        currRow++;
        isSelected[currRow][currColumn] = true;
        updateShuffleboard();
    }

    public void moveDown() {
        if (currRow <= 0) {
            return;
        }
        isSelected[currRow][currColumn] = false;
        currRow--;
        isSelected[currRow][currColumn] = true;
        updateShuffleboard();

    }

    public void moveRight() {
        if (currColumn <= 0) {
            return;
        }
        isSelected[currRow][currColumn] = false;
        currColumn--;
        isSelected[currRow][currColumn] = true;
        updateShuffleboard();
    }

    public void moveLeft() {
        if (currColumn >= 8) {
            return;
        }
        isSelected[currRow][currColumn] = false;
        currColumn++;
        isSelected[currRow][currColumn] = true;
        updateShuffleboard();
    }

    public void select() {
        selectedRow = currRow;
        selectedColumn = currColumn;
        updateShuffleboard();
    }

    public void createDisplay() {
        for (int i = 0; i < 3; i++) {
            for (int j = 8; j > -1; j--) {
                selectionDisplay[i][j] = scoringGridDisplay.add(i + " " + j, isSelected[i][j])
                        .getEntry();
            }
        }
        currentGridSelected = scoringGridDisplay
                .add("selection updated", currRow == selectedRow && currColumn == selectedColumn).getEntry();
        selectedX = scoringGridDisplay.add("Selected X", getSelectedPose().getX()).getEntry();
        selectedY = scoringGridDisplay.add("Selected Y", getSelectedPose().getY()).getEntry();
        selectedRot = scoringGridDisplay.add("Selected Rot", getSelectedPose().getRotation().getDegrees()).getEntry();
    }

    public void updateShuffleboard() {
        for (int i = 0; i < 3; i++) {
            for (int j = 8; j > -1; j--) {
                selectionDisplay[i][j].setBoolean(isSelected[i][j]);
            }
        }
        currentGridSelected.setBoolean(currRow == selectedRow && currColumn == selectedColumn);
        selectedX.setDouble(getSelectedPose().getX());
        selectedY.setDouble(getSelectedPose().getY());
        selectedRot.setDouble(getSelectedPose().getRotation().getDegrees());
    }

    public Pose2d getSelectedPose() {
        if (selectedRow > -1 && selectedColumn > -1) {
            return grid[selectedRow][selectedColumn].targetPos;
        }
        return new Pose2d();
    }

    public ArmStates getArmState() {
        return grid[selectedRow][selectedColumn].armState;
    }

    public ElevatorStates getElevStates() {
        return grid[selectedRow][selectedColumn].elevatorState;
    }

    public boolean inPosition() {
        return (Math.abs(s_Swerve.getPose().getX() - getSelectedPose().getX()) < 1.5)
                && (Math.abs(s_Swerve.getPose().getX() - getSelectedPose().getX()) < 1.5)
                && (Math.abs(s_Swerve.getPose().getRotation().getDegrees()
                        - getSelectedPose().getRotation().getDegrees()) < 30);
    }

}

class ScoringPosition {

    Pose2d targetPos;
    ArmStates armState;
    ElevatorStates elevatorState;

    public ScoringPosition(Pose2d pose, ArmStates armState, ElevatorStates elevState) {
        this.targetPos = pose;
        this.armState = armState;
        this.elevatorState = elevState;
    }
}
