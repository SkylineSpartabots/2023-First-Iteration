package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.factories.AutoCommandFactory;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartPathGenerating extends CommandBase {
    Pose2d startPos;
    Pose2d endPos;

    public SmartPathGenerating(Pose2d startPos, Pose2d endPos) {
        this.startPos = startPos;
        this.endPos = endPos;
    }

    public Translation2d translation(Pose2d x) {
        return new Translation2d(x.getX(), x.getY());
    }

    public PathPoint getPathPoint(Pose2d x) {
        return new PathPoint(translation(x), Rotation2d.fromDegrees(0), x.getRotation());
    }

    public Translation2d convertToRed(Translation2d x) {
        if (DriverStation.getAlliance() != DriverStation.Alliance.Red)
            return x;
        var y = new Translation2d(x.getX(), Constants.FIELD_WIDTH_METERS - x.getY());
        return y;
    }

    private double getDistance(Translation2d a, Translation2d b) {
        return Math.sqrt((a.getX() - b.getX()) * (a.getX() - b.getX()) + (a.getY() - b.getY()) * (a.getY() - b.getY()));
    }

    @Override
    public void initialize() {
        var tStart = translation(startPos);
        var tEnd = translation(endPos);

        Translation2d[] tempArray = new Translation2d[6];
        for (int i = 1; i <= 4; i++)
            tempArray[i] = convertToRed(Constants.cornersBlue[i - 1]);
        tempArray[0] = convertToRed(tStart);
        tempArray[5] = convertToRed(tEnd);

        double[] distance = new double[6];
        distance[0] = 0;
        for (int i = 1; i < 6; i++)
            distance[i] = 1e4;

        int[] last = new int[6];
        for (int i = 0; i < 6; i++)
            last[i] = -1;

        PriorityQueue<Node> pq = new PriorityQueue<Node>(6, new Node());
        pq.add(new Node(0, 0));
        while (!pq.isEmpty()) {
            Node top = pq.peek();
            pq.poll();
            if (top.cost > distance[top.node] + 1e-2) {
                continue;
            }
            for (int i = 0; i < 6; i++) {
                if (i == top.node)
                    continue;
                if (!checkIfInterfere(convertToRed(Constants.BOTTOM_LEFT_CHARGE),
                        convertToRed(Constants.BOTTOM_RIGHT_CHARGE), convertToRed(Constants.TOP_LEFT_CHARGE),
                        convertToRed(Constants.TOP_RIGHT_CHARGE), tempArray[top.node], tempArray[i])) {
                    // SmartDashboard.putNumber("FF2 " + top.node + i, distance[i]);
                    if (distance[i] > distance[top.node] + getDistance(tempArray[i], tempArray[top.node])) {
                        // SmartDashboard.putNumber("FF3 " + i, i);
                        distance[i] = distance[top.node] + getDistance(tempArray[i], tempArray[top.node]);
                        last[i] = top.node;

                        pq.add(new Node(i, distance[i]));
                    }
                }
            }
        }
        

        // for (int i = 0; i < 6; i++) {
        //     for (int j = i + 1; j < 6; j++) {
        //         if (!checkIfInterfere(convertToRed(Constants.BOTTOM_LEFT_CHARGE),
        //                 convertToRed(Constants.BOTTOM_RIGHT_CHARGE), convertToRed(Constants.TOP_LEFT_CHARGE),
        //                 convertToRed(Constants.TOP_RIGHT_CHARGE), tempArray[i], tempArray[j])) {
        //             SmartDashboard.putNumber("num " + i + j, 0);
        //         }
        //     }
        // }

        // for (int i = 0; i < 6; i++) {
        //     SmartDashboard.putNumber("dist " + i, distance[i]);
        //     SmartDashboard.putNumber("last " + i, last[i]);
        // }

        List<PathPoint> result = new ArrayList<>();
        int x = 5;
        while (x != -1) {
            PathPoint p = new PathPoint(convertToRed(tempArray[x]), Rotation2d.fromDegrees(0));
            if (x == 0) {
                p = new PathPoint(convertToRed(tempArray[x]), Rotation2d.fromDegrees(0), startPos.getRotation());
            } else if (x == 5) {
                p = new PathPoint(convertToRed(tempArray[x]), Rotation2d.fromDegrees(0), endPos.getRotation());
            }
            result.add(p);
            x = last[x];
        }
        Collections.reverse(result);
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                new PathConstraints(4, 3),
                result);
        CommandScheduler.getInstance().schedule(AutoCommandFactory.followPathCommand(trajectory));
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public double crossProduct(Translation2d a, Translation2d b) {
        return a.getY() * b.getX() - a.getX() * b.getY();
    }

    public int orient(Translation2d a, Translation2d b, Translation2d c) {
        var temp = crossProduct(new Translation2d(b.getX() - a.getX(), b.getY() - a.getY()),
                new Translation2d(c.getX() - a.getX(), c.getY() - a.getY()));
        if (Math.abs(temp - 1e-6) < 0)
            return 0;
        return temp > 0 ? 1 : -1;
    }

    public boolean intersectLines(Translation2d a1, Translation2d a2, Translation2d b1, Translation2d b2) {
        if (crossProduct( // parralel line case
                new Translation2d(a1.getX() - a2.getX(), a1.getY() - a2.getY()),
                new Translation2d(b2.getX() - b1.getX(), b2.getY() - b1.getY())) == 0) {
            return false; // technically not mathematically correct, but we dont care about parralel lines
                          // here
        }
        if (orient(a1, a2, b1) == orient(a1, a2, b2))
            return false;
        if (orient(b1, b2, a1) == orient(b1, b2, a2))
            return false;
        // SmartDashboard.putNumber("intersection", 0);
        return true;
    }

    public boolean equal(Translation2d a, Translation2d b) {
        if (Math.abs(a.getX() - b.getX()) < 1e-6 && Math.abs(a.getY() - b.getY()) < 1e-6) {
            return true;
        }
        return false;
    }

    public boolean checkIfInterfere(Translation2d bottomLeft, Translation2d bottomRight, Translation2d topLeft,
            Translation2d topRight, Translation2d line1, Translation2d line2) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < i; j++) {
                if (i == 3 && j == 0)
                    continue;
                if (i == 2 && j == 1)
                    continue;
                if (equal(convertToRed(Constants.cornersBlue[i]), line2)
                        || equal(convertToRed(Constants.cornersBlue[i]), line1) ||
                        equal(convertToRed(Constants.cornersBlue[j]), line1)
                        || equal(convertToRed(Constants.cornersBlue[j]), line2)) {
                    continue;
                }
                if (intersectLines(convertToRed(Constants.cornersBlue[i]), convertToRed(Constants.cornersBlue[j]),
                        line1, line2))
                    return true;
            }
        }

        return false;
    }

    class Node implements Comparator<Node> {
        public int node;
        public double cost;

        public Node() {
        }

        public Node(int node, double cost) {
            this.node = node;
            this.cost = cost;
        }

        @Override
        public int compare(Node node1, Node node2) {

            if (node1.cost < node2.cost)
                return -1;

            if (node1.cost > node2.cost)
                return 1;

            return 0;
        }
    }

}
