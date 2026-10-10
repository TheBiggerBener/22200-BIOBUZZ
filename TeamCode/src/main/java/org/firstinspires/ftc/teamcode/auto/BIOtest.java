package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Drives 12 inches forward with Pedro Pathing, then ends autonomous. That is all it does -
 * no intake, no launcher, no vision. It never touches those motors.
 *
 * COORDINATES ARE RELATIVE. The start pose is (0, 0, heading 0), meaning "wherever the
 * robot is set down", and the path runs 12 inches along the robot's own starting heading.
 * Point the robot the way you want it to go before pressing start - it drives whichever
 * way it is facing.
 *
 * FIRST RUN WARNING: the tuning numbers in Constants.java are placeholders, not measured
 * values, so this will not stop at exactly 12 inches yet. Run it with plenty of clear
 * space ahead and a hand on stop. Constants.maxPower is 0.5 to keep it slow.
 */
@Autonomous(name = "BIOtest - drive 12in", group = "auto")
public class BIOtest extends OpMode {

    private static final double DRIVE_INCHES = 12.0;

    // Stop anyway if the follower has not finished by now, so a tuning problem parks the
    // robot instead of leaving it pushing against something for the whole period.
    private static final double TIMEOUT_SECONDS = 5.0;

    private static final String LOG_TAG = "BIOtest";

    private Follower follower;
    private Timer pathTimer;
    private PathChain driveOut;
    private boolean finished = false;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose endPose = new Pose(DRIVE_INCHES, 0, Math.toRadians(0));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();

        // Straight line, heading held at the starting heading the whole way.
        driveOut = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        follower.setPose(startPose);

        telemetry.addLine("BIOtest: drives " + DRIVE_INCHES + " in forward, then ends.");
        telemetry.addLine("Point the robot where you want it to go. Keep the area clear.");
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        follower.followPath(driveOut, true);
        RobotLog.ii(LOG_TAG, "START driving %.1f in", DRIVE_INCHES);
    }

    @Override
    public void loop() {
        follower.update();

        if (!finished) {
            boolean arrived = !follower.isBusy();
            boolean timedOut = pathTimer.getElapsedTimeSeconds() > TIMEOUT_SECONDS;

            if (arrived || timedOut) {
                // Release the path so Pedro stops driving the motors to hold the end pose.
                follower.breakFollowing();
                finished = true;
                RobotLog.ii(LOG_TAG, "DONE (%s) t=%.2fs x=%.2f y=%.2f heading=%.1fdeg",
                        timedOut ? "timeout" : "arrived",
                        pathTimer.getElapsedTimeSeconds(),
                        follower.getPose().getX(), follower.getPose().getY(),
                        Math.toDegrees(follower.getPose().getHeading()));
            }
        } else {
            // Autonomous is over - end the OpMode instead of idling out the period.
            requestOpModeStop();
        }

        telemetry.addData("state", finished ? "DONE" : "DRIVING");
        telemetry.addData("target", "%.1f in forward", DRIVE_INCHES);
        telemetry.addData("x / y", "%.2f / %.2f in",
                follower.getPose().getX(), follower.getPose().getY());
        telemetry.addData("heading", "%.1f deg", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("t", "%.2f s", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
