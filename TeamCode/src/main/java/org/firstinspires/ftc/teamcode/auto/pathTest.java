package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

/**
 * EIGHT-PATH MOVEMENT TEST - driving only. No shooter, no intake, no vision.
 * It never constructs intakeOuttake, so it physically cannot move a mechanism.
 *
 * ================================ HOW TO USE IT =================================
 * There is ONE thing to edit: the WAYPOINTS array below. Put your eight poses in, run it,
 * and the robot drives start -> 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> 8 and stops.
 *
 * Each leg is the straight line from wherever the previous one ended to the next waypoint,
 * so every pose is typed ONCE - there is no "the end of path 3 is also the start of path 4"
 * duplication to keep in sync.
 *
 * To run fewer than eight, lower PATHS_TO_RUN. The spare waypoints stay in the file,
 * ignored, so you can try a shorter route by changing one number.
 * ================================================================================
 *
 * THE FIELD FRAME (same as gardenSide)
 *   (0, 0) is the bottom-left corner of the field, on the red garden side.
 *   +X goes RIGHT toward BLUE.        -X goes LEFT toward RED.
 *   +Y goes UP toward the blue garden / red parking.
 *   -Y goes DOWN toward the red garden / blue parking.
 *   Heading is where the robot's FRONT points: 0 = facing +X, 90 = facing +Y, 180 = -X.
 *
 * HOW IT RUNS
 *   loop() is called over and over, maybe 50 times a second. Each call, follower.update()
 *   lets Pedro take one step along the current path, then we check !follower.isBusy() -
 *   which means "Pedro says it arrived". When it has, we hand over the next leg and bump the
 *   index. Nothing blocks or sleeps; each loop is one thin slice of the drive.
 */
@Autonomous(name = "pathTest - 8 paths", group = "auto")
public class pathTest extends OpMode {

    // ============================ SET YOUR POSES HERE ===============================
    // Where the robot is physically placed before you press start.
    private final Pose startPose = new Pose(82.2, 136, Math.toRadians(0));

    // The eight waypoints, in the order the robot should visit them.
    // Pose(x, y, heading) - x and y in INCHES, heading via Math.toRadians(degrees).
    // These describe the CENTRE of the robot, not its bumper.
    private final Pose[] WAYPOINTS = new Pose[] {
            new Pose(118.0, 136.0, Math.toRadians(0)), // path 1
            new Pose(132.0, 136.0, Math.toRadians(0)), // path 2
            new Pose(106.46, 105.68, Math.toRadians(90)), // path 3
            new Pose(106.46, 16.06, Math.toRadians(90)), // path 4
            new Pose(83.26,  16.09, Math.toRadians(-90)), // path 5
            new Pose(89.63,  10.23, Math.toRadians(137)), // path 6
            new Pose( 83.73,  15.57, Math.toRadians(-42)), // path 7
            new Pose( 131.33, 25.38, Math.toRadians(180)), // path 8
    };

    // Lower this to run only the first N paths. Must be 1..8.
    private static final int PATHS_TO_RUN = 8;

    // ============================ FIELD AND ROBOT SIZE ==============================
    // Used only to check your poses keep the robot off the walls - see wallProblem().
    private static final double FIELD_SIZE_IN = 144.0; // standard FTC field, 12 ft square
    private static final double ROBOT_SIZE_IN = 17.0;  // 17 x 17 per the team

    // Give up on a leg that has not finished by now, so a bad coordinate or a tuning problem
    // parks the robot instead of grinding into a wall for the rest of the period.
    private static final double PATH_TIMEOUT_SECONDS = 6.0;

    private static final String LOG_TAG = "pathTest";

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // One Path per leg, built once up front.
    private final List<Path> paths = new ArrayList<>();

    // Which leg we are on. 0 = driving to WAYPOINTS[0], and so on.
    private int currentPath = 0;

    private boolean finished = false;

    // Non-empty means a pose would put the robot through a wall, and loop() refuses to drive.
    private final List<String> geometryProblems = new ArrayList<>();

    /**
     * Builds one straight-line leg per waypoint. Leg i runs from the previous pose (the
     * start pose for the first leg) to WAYPOINTS[i].
     *
     * In Pedro 3 a path is made with the static line(from, to) factory, and .linear(from, to)
     * is the heading interpolation - it turns the robot smoothly across the leg so it arrives
     * already pointing the right way instead of stopping and then spinning. If you would
     * rather hold one heading for a whole leg, use .constant(headingRadians) instead.
     *
     * line() is a straight line. For a leg that would clip a wall, Paths.curve(...) takes
     * extra control poses that pull the path aside.
     */
    public void buildPaths() {
        Pose from = startPose;

        for (int i = 0; i < PATHS_TO_RUN; i++) {
            Pose to = WAYPOINTS[i];

            paths.add(line(from, to).linear(from, to));

            from = to; // the end of this leg is the start of the next
        }
    }

    /**
     * Returns a complaint if this pose would put a 17 in robot through a wall, else null.
     * Poses are the robot's CENTRE, so the centre must stay half a robot from every edge -
     * on a 144 in field with a 17 in robot that is a box from 8.5 to 135.5 on both axes.
     */
    private String wallProblem(String name, Pose p) {
        double half = ROBOT_SIZE_IN / 2.0;
        double lo = half;
        double hi = FIELD_SIZE_IN - half;

        if (p.x() < lo || p.x() > hi || p.y() < lo || p.y() > hi) {
            return String.format("%s (%.1f, %.1f) outside safe box %.1f..%.1f",
                    name, p.x(), p.y(), lo, hi);
        }
        return null;
    }

    private void checkGeometry() {
        if (PATHS_TO_RUN < 1 || PATHS_TO_RUN > WAYPOINTS.length) {
            geometryProblems.add("PATHS_TO_RUN is " + PATHS_TO_RUN
                    + ", must be 1.." + WAYPOINTS.length);
            return; // the loop below would be unsafe with a bad count
        }

        String problem = wallProblem("startPose", startPose);
        if (problem != null) geometryProblems.add(problem);

        for (int i = 0; i < PATHS_TO_RUN; i++) {
            problem = wallProblem("waypoint " + (i + 1), WAYPOINTS[i]);
            if (problem != null) geometryProblems.add(problem);
        }

        for (String p : geometryProblems) {
            RobotLog.ww(LOG_TAG, "GEOMETRY: %s", p);
        }
    }

    /** Hands a leg to the follower. holdEnd keeps Pedro actively holding the end pose. */
    private void startLeg(int index) {
        follower.holdEnd.set(true);
        follower.follow(paths.get(index));
        pathTimer.reset();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        checkGeometry();

        if (geometryProblems.isEmpty()) {
            buildPaths();
            follower.setPose(startPose);
            telemetry.addLine("pathTest: driving " + PATHS_TO_RUN + " paths in order, then stopping.");
            telemetry.addLine("Movement only - no shooter, no intake.");
        } else {
            telemetry.addLine("!! WILL NOT DRIVE - a pose would hit a wall !!");
            for (String p : geometryProblems) telemetry.addLine("  " + p);
            telemetry.addLine("Fix the pose, or correct FIELD_SIZE_IN / ROBOT_SIZE_IN.");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.reset();
        pathTimer.reset();

        if (geometryProblems.isEmpty()) {
            // Hand over the first leg. Every later leg starts from loop() as the previous
            // one finishes.
            startLeg(0);
            RobotLog.ii(LOG_TAG, "START running %d paths", PATHS_TO_RUN);
        }
    }

    @Override
    public void loop() {
        if (!geometryProblems.isEmpty()) {
            // Touch nothing - no follower.update(), no motor commands.
            telemetry.addLine("BLOCKED: a pose would put the robot through a wall.");
            for (String p : geometryProblems) telemetry.addLine("  " + p);
            telemetry.update();
            return;
        }

        // One step along the current path. Must happen every loop or the robot never moves.
        follower.update();

        if (!finished) {
            boolean arrived = !follower.isBusy();
            boolean timedOut = pathTimer.seconds() > PATH_TIMEOUT_SECONDS;

            if (arrived || timedOut) {
                RobotLog.ii(LOG_TAG, "path %d of %d done (%s) t=%.2fs pose x=%.2f y=%.2f h=%.1f",
                        currentPath + 1, PATHS_TO_RUN, timedOut ? "TIMEOUT" : "arrived",
                        pathTimer.seconds(),
                        follower.pose().x(), follower.pose().y(),
                        Math.toDegrees(follower.pose().heading()));

                currentPath++;

                if (currentPath < PATHS_TO_RUN) {
                    startLeg(currentPath);
                } else {
                    // All legs done. stop() releases the follower so it quits driving the
                    // motors to hold the final pose (Pedro 2's breakFollowing()).
                    follower.stop();
                    finished = true;
                    RobotLog.ii(LOG_TAG, "ALL %d PATHS DONE in %.2fs",
                            PATHS_TO_RUN, opModeTimer.seconds());
                }
            }
        }

        // Deliberately no requestOpModeStop() when finished - ending the OpMode would wipe
        // the telemetry, and the final pose is the most useful thing to read after a run.

        Pose target = WAYPOINTS[Math.min(currentPath, PATHS_TO_RUN - 1)];

        telemetry.addData("status", finished ? "DONE" : "driving");
        telemetry.addData("path", "%d / %d", Math.min(currentPath + 1, PATHS_TO_RUN), PATHS_TO_RUN);
        telemetry.addData("target", "%.1f, %.1f @ %.0f deg",
                target.x(), target.y(), Math.toDegrees(target.heading()));
        telemetry.addData("actual", "%.1f, %.1f @ %.0f deg",
                follower.pose().x(), follower.pose().y(),
                Math.toDegrees(follower.pose().heading()));
        telemetry.addData("path time", "%.2f s", pathTimer.seconds());
        telemetry.addData("total time", "%.2f s", opModeTimer.seconds());
        telemetry.update();
    }
}
