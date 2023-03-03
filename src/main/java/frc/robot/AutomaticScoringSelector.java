package frc.robot;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.CompleteMechanism.MechanismState;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.lib.util.COTSFalconSwerveConstants.driveGearRatios;
import frc.robot.ScoringPosition;

public final class AutomaticScoringSelector {

    Swerve s_Swerve;
    public BooleanSupplier inPosition = () -> inPosition();
    private static AutomaticScoringSelector instance;
    private ShuffleboardTab scoringGridDisplay = Shuffleboard.getTab("Scoring Display");

    // private ScoringPosition[][] grid = new ScoringPosition[3][9]; // [row][column] 0 row is l1, 0 column is the one
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

    Pose2d[] allPoses = new Pose2d[9];


    public AutomaticScoringSelector() {
        s_Swerve = Swerve.getInstance();
            double x = 1.9; //should be the same for every default scoring position
            double y = 0.59; //only one that changes
            double yIncrem = 0.56; //how much each y varies by - currently unmeasured

            
            for (int i = 0; i < 9; i++) {
                allPoses[DriverStation.getAlliance() == DriverStation.Alliance.Blue ? 8 - i : i] = new Pose2d(x, y + yIncrem * i, new Rotation2d(Math.PI));
            }
        
        isSelected[currRow][currColumn] = true;
    }

    public void moveDown() {
        if (currRow >= 2) {
            return;
        }
        isSelected[currRow][currColumn] = false;
        currRow++;
        isSelected[currRow][currColumn] = true;
        updateShuffleboard();
    }

    public void moveUp() {
        if (currRow <= 0) {
            return;
        }
        isSelected[currRow][currColumn] = false;
        currRow--;
        isSelected[currRow][currColumn] = true;
        updateShuffleboard();

    }

    public void moveLeft() {
        if (currColumn <= 0) {
            return;
        }
        isSelected[currRow][currColumn] = false;
        currColumn--;
        isSelected[currRow][currColumn] = true;
        updateShuffleboard();
    }

    public void moveRight() {
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

    public Pose2d convertToRed(Pose2d a) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            Pose2d b = new Pose2d(a.getX(), Constants.FIELD_WIDTH_METERS - a.getY(), a.getRotation());
            return b;
        } 
        return a;

    }

    public Pose2d getSelectedPose() {
        
        if (selectedRow > -1 && selectedColumn > -1) {
            return convertToRed(allPoses[selectedColumn]);
        }
        return new Pose2d();
    }

    public MechanismState getMechState() {
        if (selectedRow == 0) {
            if (selectedColumn % 3 == 1) {
                return MechanismState.LOWCUBE;
            } else {
                return MechanismState.LOWCONE;
            }
        } else if (selectedRow == 1) {
            if (selectedColumn % 3 == 1) {
                return MechanismState.MIDCUBE;
            } else {
                return MechanismState.MIDCONE;
            }
        } else {
            if (selectedColumn % 3 == 1) {
                return MechanismState.HIGHCUBE;
            } else {
                return MechanismState.HIGHCONE;
            }
        }
    }


    
    public boolean inPosition() {
        return (Math.abs(s_Swerve.getPose().getX() - getSelectedPose().getX()) < 0.5)
                && (Math.abs(s_Swerve.getPose().getX() - getSelectedPose().getX()) < 0.5)
                && (Math.abs(s_Swerve.getPose().getRotation().getDegrees()
                        - getSelectedPose().getRotation().getDegrees()) < 30);
    }

}
