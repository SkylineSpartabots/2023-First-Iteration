package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.CompleteMechanism.MechanismState;

public final class AutomaticScoringSelector {

    Swerve s_Swerve;
    public BooleanSupplier inPosition = () -> inPosition();
    private static AutomaticScoringSelector instance;
    public ShuffleboardTab scoringGridDisplay = Shuffleboard.getTab("Scoring Display");

    // private ScoringPosition[][] grid = new ScoringPosition[3][9]; // [row][column] 0 row is l1, 0 column is the one
                                                                  // farthest from load zone
    private boolean[][] isSelected = new boolean[3][9];
    private GenericEntry[][] selectionDisplay = new GenericEntry[3][9];
    public GenericEntry currPosEntry, currLevelEntry, currentXPosEntry, currentYPosEntry, realXEntry, realYEntry, realRotEntry;

    public int currPos = 0;
    public int currLevel = 0;
    public double currentXpos, currentYpos; 
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
        double x = 1.72; //should be the same for every default scoring position
        double y = 0.59 - 0.1; //only one that changes
        double yIncrem = Units.inchesToMeters(22); //how much each y varies by - currently unmeasured

        
        for (int i = 0; i < 9; i++) {
            allPoses[DriverStation.getAlliance() == DriverStation.Alliance.Blue ? 8 - i : i] = new Pose2d(x, y + yIncrem * i, new Rotation2d(Math.PI));
        }
        
        currPos = 0;
        currLevel = 0;
    }

    public void createDisplay() {
        currPosEntry = scoringGridDisplay.add("Current Pos", currPos).getEntry();
        currLevelEntry = scoringGridDisplay.add("Current Level", currLevel).getEntry();
        currentXPosEntry = scoringGridDisplay.add("Current X pos", currentXpos).getEntry();
        currentYPosEntry = scoringGridDisplay.add("Current Y pos", currentYpos).getEntry();
        realXEntry = scoringGridDisplay.add("RealX", 0).getEntry();
        realYEntry = scoringGridDisplay.add("RealY", 0).getEntry();
        realRotEntry = scoringGridDisplay.add("RealRot", 0).getEntry();

        // for (int i = 0; i < 3; i++) {
        //     for (int j = 8; j > -1; j--) {
        //         selectionDisplay[i][j] = scoringGridDisplay.add(i + " " + j, isSelected[i][j])
        //                 .getEntry();
        //     }
        // }


        // currentGridSelected = scoringGridDisplay
        //         .add("selection updated", currPos == selectedRow && currColumn == selectedColumn).getEntry();
        // selectedX = scoringGridDisplay.add("Selected X", getSelectedPose().getX()).getEntry();
        // selectedY = scoringGridDisplay.add("Selected Y", getSelectedPose().getY()).getEntry();
        // selectedRot = scoringGridDisplay.add("Selected Rot", getSelectedPose().getRotation().getDegrees()).getEntry();
        // currX = scoringGridDisplay.add("Curr X", Swerve.getInstance().getPose().getX()).getEntry();
        // currX = scoringGridDisplay.add("Curr X", Swerve.getInstance().getPose().getY()).getEntry();
        // seesTarg = scoringGridDisplay.add("Has Target", Limelight.getInstance().hasTarget()).getEntry();
        // inPos = scoringGridDisplay.add("Swerve in pos", inPosition()).getEntry();
    }

    public void updateShuffleboard() {
        currPosEntry.setDouble(currPos+1);
        currLevelEntry.setDouble(currLevel+1);
        currentXpos = convertToRed(allPoses[currPos]).getX();
        currentYpos = convertToRed(allPoses[currPos]).getY();    
        currentXPosEntry.setDouble(currentXpos);
        currentYPosEntry.setDouble(currentYpos);

        // for (int i = 0; i < 3; i++) {
        //     for (int j = 8; j > -1; j--) {
        //         selectionDisplay[i][j].setBoolean(isSelected[i][j]);
        //     }
        // }
        // currentGridSelected.setBoolean(currPos == selectedRow && currColumn == selectedColumn);
        // selectedX.setDouble(getSelectedPose().getX());
        // selectedY.setDouble(getSelectedPose().getY());
        // selectedRot.setDouble(getSelectedPose().getRotation().getDegrees());
        // currX.setDouble(Swerve.getInstance().getPose().getX());
        // currY.setDouble(Swerve.getInstance().getPose().getY());
        // seesTarg.setBoolean(frc.robot.subsystems.Limelight.getInstance().hasTarget());
        // inPos.setBoolean(inPosition());
    }

    public Pose2d convertToRed(Pose2d a) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            Pose2d b = new Pose2d(a.getX(), Constants.FIELD_WIDTH_METERS - a.getY(), a.getRotation());
            return b;
        } 
        return a;

    }

    public Pose2d getSelectedPose() {
        return convertToRed(allPoses[currPos]);
    }

    public MechanismState getMechState() {
        if (currLevel == 0) {
            if (currPos % 3 == 1) {
                return MechanismState.L1CUBE;
            } else {
                return MechanismState.L1CONE;
            }
        } else if (currLevel == 1) {
            if (currPos % 3 == 1) {
                return MechanismState.L2CUBE;
            } else {
                return MechanismState.L2CONE;
            }
        } else {
            if (currPos % 3 == 1) {
                return MechanismState.L3CUBE;
            } else {
                return MechanismState.L3CONE;
            }
        }
    }


    
    public boolean inPosition() {
        return (Math.abs(s_Swerve.getPose().getX() - getSelectedPose().getX()) < 0.3)
                && (Math.abs(s_Swerve.getPose().getY() - getSelectedPose().getY()) < 0.3)
                && (Math.abs(s_Swerve.getPose().getRotation().getDegrees()
                        - getSelectedPose().getRotation().getDegrees()) < 20);
    }

    public void setLevel(int level) {
        currLevel = level;
        updateShuffleboard();
    }

    public int getLevel() {
        return currLevel;
    }

    public void increasePos() {
        currPos = (currPos + 1) % 9;
        updateShuffleboard();
    }

    public void decreasePos() {
        currPos = (currPos + 8) % 9;
        updateShuffleboard();
    }
}
