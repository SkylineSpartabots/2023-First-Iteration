/*
 automatic scoring selector to generate poses for 
 targets and create display on shuffleboard
*/

package frc.robot;

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
    private static AutomaticScoringSelector instance;
    public ShuffleboardTab scoringGridDisplay = Shuffleboard.getTab("Scoring Display");

    public GenericEntry currPosEntry, currLevelEntry, currentXPosEntry, currentYPosEntry, realXEntry, realYEntry, realRotEntry;

    public int currPos = 0;
    public int currLevel = 0;
    public double currentXpos, currentYpos; 

    public static AutomaticScoringSelector getInstance() {
        if (instance == null) {
            instance = new AutomaticScoringSelector();
        }
        return instance;
    }

    Pose2d[] allPoses = new Pose2d[9];
    double x = 1.68; //should be the same for every default scoring position
    double y = 0.59 - 0.1; //only one that changes
    double yIncrem = Units.inchesToMeters(22); //how much each y varies by - currently unmeasured

    // generates all the poses for the targets 
    public AutomaticScoringSelector() {
        s_Swerve = Swerve.getInstance();
        
        
        for (int i = 0; i < 9; i++) {
            allPoses[DriverStation.getAlliance() == DriverStation.Alliance.Blue ? 8 - i : i] = new Pose2d(x, y + yIncrem * i, new Rotation2d(Math.PI));
        }
        
        currPos = 0;
        currLevel = 0;
    }

    // creates shuffleboard display
    public void createDisplay() {
        currPosEntry = scoringGridDisplay.add("Current Pos", currPos).getEntry();
        currLevelEntry = scoringGridDisplay.add("Current Level", currLevel).getEntry();
        currentXPosEntry = scoringGridDisplay.add("Current X pos", currentXpos).getEntry();
        currentYPosEntry = scoringGridDisplay.add("Current Y pos", currentYpos).getEntry();
        realXEntry = scoringGridDisplay.add("RealX", 0).getEntry();
        realYEntry = scoringGridDisplay.add("RealY", 0).getEntry();
        realRotEntry = scoringGridDisplay.add("RealRot", 0).getEntry();
    }

    // updates shuffleboard display
    public void updateShuffleboard() {
        currPosEntry.setDouble(currPos+1);
        currLevelEntry.setDouble(currLevel+1);
        currentXpos = convertToRed(allPoses[currPos]).getX();
        currentYpos = convertToRed(allPoses[currPos]).getY();    
        currentXPosEntry.setDouble(currentXpos);
        currentYPosEntry.setDouble(currentYpos);
    }

    // converts positions to red if the alliance is red
    public Pose2d convertToRed(Pose2d a) {
        for (int i = 0; i < 9; i++) {
            allPoses[DriverStation.getAlliance() == DriverStation.Alliance.Blue ? 8 - i : i] = new Pose2d(x, y + yIncrem * i, new Rotation2d(Math.PI));
        }
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            Pose2d b = new Pose2d(a.getX(), Constants.FIELD_WIDTH_METERS - a.getY(), a.getRotation());
            return b;
        } 
        return a;

    }

    // resturns pose of the current seleced target
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
