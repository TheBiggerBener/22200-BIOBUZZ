package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SleepAction;
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
    private Action driveAction;
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

        // Drive the path, but race it against a timeout. RaceAction finishes as soon as
        // ANY of its actions finishes, so whichever happens first - arriving, or 5 seconds
        // elapsing - ends it. This replaces the hand-written elapsed-time check that used to
        // live in loop(), and it is the pattern that scales: swap RaceAction for
        // SequentialAction and this becomes a multi-step auto.
        driveAction = new RaceAction(
                new FollowPathAction(follower, driveOut, true),
                new SleepAction(TIMEOUT_SECONDS));

        follower.setPose(startPose);

        telemetry.addLine("BIOtest: drives " + DRIVE_INCHES + " in forward, then ends.");
        telemetry.addLine("Point the robot where you want it to go. Keep the area clear.");
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        // No followPath() call here any more - FollowPathAction hands the path over itself
        // on its first tick, inside loop().
        RobotLog.ii(LOG_TAG, "START driving %.1f in", DRIVE_INCHES);
    }

    @Override
    public void loop() {
        // MUST run every loop. FollowPathAction deliberately does not call this itself, and
        // without it the follower never moves and isBusy() never clears.
        follower.update();

        if (!finished) {
            // One tick of the composed action. Returns false once the race is settled -
            // either the path finished or the SleepAction timeout won.
            boolean stillRunning = driveAction.run(new TelemetryPacket());

            if (!stillRunning) {
                // Release the path so Pedro stops driving the motors to hold the end pose.
                follower.breakFollowing();
                finished = true;
                RobotLog.ii(LOG_TAG, "DONE t=%.2fs x=%.2f y=%.2f heading=%.1fdeg",
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
