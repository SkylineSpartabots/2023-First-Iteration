package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SmartPathGenerating extends CommandBase {
    Pose2d startPos;
    Pose2d endPos;
    
    public SmartPathGenerating(Pose2d startPos, Pose2d endPos) {
        this.startPos = startPos;
        this.endPos = endPos;


    }


    public double crossProduct(Translation2d a, Translation2d b) {
        return a.getY() * b.getX() - a.getX() * b.getY();
    }

    public int orient(Translation2d a, Translation2d b, Translation2d c) {
        var temp = crossProduct(new Translation2d(b.getX() - a.getX(), b.getY() - a.getY()), new Translation2d(c.getX() - a.getX(), c.getY() - a.getY()));
        if (Math.abs(temp - 1e-6) < 0) return 0;
        return temp > 0 ? 1 : -1; 
    }

    public boolean intersectLines(Translation2d a1, Translation2d a2, Translation2d b1, Translation2d b2) {
        if (crossProduct( // parralel line case
            new Translation2d(a1.getX() - a2.getX(), a1.getY() - a2.getY()),
            new Translation2d(b2.getX() - b1.getX(), b2.getY() - b1.getY())) == 0) {
            return false; // technically not mathematically correct, but we dont care about parralel lines here
        }
        if (orient(a1, a2, b1) == orient(a1, a2, b2)) return false;
        if (orient(b1, b2, a1) == orient(b1, b2, a2)) return false;
        return true;
    }

    public boolean checkIfInterfere(Translation2d bottomLeft, Translation2d bottomRight, Translation2d topLeft, Translation2d topRight, Translation2d line1, Translation2d line2) {
        if (intersectLines(bottomLeft, bottomRight, line1, line2) ||
            intersectLines(bottomLeft, topLeft, line1, line2) ||
            intersectLines(bottomRight, bottomLeft, line1, line2) ||
            intersectLines(topRight, topLeft, line1, line2)) return true;
        return false;
    }
}
