package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SmartPathGenerating extends CommandBase {
    Pose2d startPos;
    Pose2d endPos;
    
    public SmartPathGenerating(Pose2d startPos, Pose2d endPos) {
        this.startPos = startPos;
        this.endPos = endPos;


    }
}
