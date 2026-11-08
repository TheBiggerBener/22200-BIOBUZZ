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
 * GARDEN SIDE AUTONOMOUS - pathTest's eight paths, plus shooting and collecting.
 *
 * ================================ HOW TO USE IT =================================
 * The driving is EXACTLY pathTest: startPose, then the eight WAYPOINTS in order. Those
 * numbers are copied from pathTest.java - if you retune a pose there, copy it here too.
 *
 * What this file adds is two little schedules, right under the waypoints:
 *
 *   SHOOT_AFTER_PATHS      - fire a volley when this many paths have finished.
 *                            0 means "before path 1 ever starts", i.e. from the start pose.
 *   INTAKE_ON_BEFORE_PATHS - switch the intake on just before this path number begins.
 *
 * Want to shoot somewhere else? Change a number in SHOOT_AFTER_PATHS. Want to start
 * collecting a path earlier? Change a number in INTAKE_ON_BEFORE_PATHS. Nothing else in
 * the file has to move.
 * ================================================================================
 *
 * THE FIELD FRAME (same as pathTest)
 *   (0, 0) is the bottom-left corner of the field, on the red garden side.
 *   +X goes RIGHT toward BLUE.        -X goes LEFT toward RED.
 *   +Y goes UP toward the blue garden / red parking.
 *   -Y goes DOWN toward the red garden / blue parking.
 *   Heading is where the robot's FRONT points: 0 = facing +X, 90 = facing +Y, 180 = -X.
 *
 * THE ROUTINE AS CURRENTLY SET
 *   volley 1   before path 1, standing on the start pose, four preloaded balls
 *   path 1     drive, intake off
 *   path 2     INTAKE ON - starts collecting, and stays on...
 *   paths 3-5  ...all the way through here
 *   volley 2   at the end of path 5, four balls
 *   path 6     INTAKE ON again
 *   path 7     still collecting
 *   volley 3   at the end of path 7, four balls
 *   path 8     drive to park, intake off
 *   Twelve balls in three volleys of four.
 *
 * THE SHOOTER IS NOT INSTALLED. Every launcher call is written out but commented. Each
 * "shot" is a timed wait with the intake still turning, standing in for the real thing.
 *
 * HOW IT RUNS
 *   loop() is called over and over, maybe 50 times a second. Each call, follower.update()
 *   lets Pedro take one step along the current path, then statePathUpdate() asks "is this
 *   step finished, and what is next?". Nothing blocks or sleeps - each loop is one thin
 *   slice of the whole routine. follower.update() runs during the volleys too, which is
 *   what keeps the robot parked on the shooting pose instead of drifting while it fires.
 */
@Autonomous(name = "gardenSide", group = "auto")
public class gardenSide extends OpMode {

    // ============================ SET YOUR POSES HERE ===============================
    // Copied from pathTest.java. Where the robot is physically placed before you press start.
    private final Pose startPose = new Pose(82.2, 135.0, Math.toRadians(0));

    // The eight waypoints, in the order the robot should visit them.
    // Pose(x, y, heading) - x and y in INCHES, heading via Math.toRadians(degrees).
    // These describe the CENTRE of the robot, not its bumper.
    private final Pose[] WAYPOINTS = new Pose[] {
            new Pose(118.0, 135.0, Math.toRadians(0)), // path 1
            new Pose(132.0, 135.0, Math.toRadians(0)), // path 2
            new Pose(106.46, 105.68, Math.toRadians(90)), // path 3
            new Pose(106.46, 16.06, Math.toRadians(90)), // path 4
            new Pose(83.26,  16.09, Math.toRadians(-90)), // path 5
            new Pose(89.63,  10.23, Math.toRadians(137)), // path 6
            new Pose( 83.73,  15.57, Math.toRadians(-42)), // path 7
            new Pose( 131.33, 25.38, Math.toRadians(180)), // path 8
    };

    // Lower this to run only the first N paths. Must be 1..8. Any shoot/intake event
    // scheduled past this number simply never happens.
    private static final int PATHS_TO_RUN = 8;

    // ============================== WHEN TO SHOOT ====================================
    // "Fire a volley once this many paths are complete."
    //   0 = before path 1 starts (from the start pose, the four preloaded balls)
    //   5 = the moment path 5 finishes, standing on WAYPOINTS[4] = (83.26, 16.09) @ -90
    //   7 = the moment path 7 finishes, standing on WAYPOINTS[6] = (83.73, 15.57) @ -42
    // The robot holds that pose while it fires - it does not start path 6 / 8 until the
    // volley is done.
    private static final int[] SHOOT_AFTER_PATHS = { 0, 5, 7 };

    // ============================= WHEN TO COLLECT ===================================
    // "Switch the intake on just before this path number begins." It then stays on through
    // every following path until the next volley finishes, so one number here covers a
    // whole collecting run.
    //   2 = intake on at the start of path 2, collecting through paths 2, 3, 4, 5
    //   6 = intake on at the start of path 6, collecting through paths 6 and 7
    private static final int[] INTAKE_ON_BEFORE_PATHS = { 2, 6 };

    // ============================ FIELD AND ROBOT SIZE ==============================
    // Used only to check your poses keep the robot off the walls - see wallProblem().
    // A standard FTC field is 144 in (12 ft) square. TODO confirm the BIOBUZZ field really
    // is 144 - nothing in the repo says so.
    private static final double FIELD_SIZE_IN = 144.0;
    private static final double ROBOT_SIZE_IN = 17.0;  // 17 x 17 per the team

    // =============================== TIMINGS / POWERS ================================
    // TODO the real launcher velocity is unknown - the shooter is not installed. Measure it
    // with shootTest and put it here. setVelocity() is in ENCODER TICKS PER SECOND, not RPM;
    // shootTest prints both so you can convert.
    private static final double LAUNCHER_VELOCITY_TICKS = 0.0; // TODO measure

    // TODO how long the launcher needs to reach speed before the first shot of a volley.
    private static final double LAUNCHER_SPINUP_SECONDS = 1.5;

    // Stand-in for one shot while there is no shooter: wait this long with the intake
    // turning, as though a ball went out. TODO replace with the real feed time.
    private static final double SHOT_DWELL_SECONDS = 0.8;

    // Balls in one volley. Three volleys of four = twelve.
    private static final int BALLS_PER_VOLLEY = 4;

    // Positive should pull balls IN. If it spits them out, negate this rather than rewiring.
    private static final double INTAKE_POWER = 1.0;

    // Give up on a leg that has not finished by now, so a bad coordinate or a tuning problem
    // parks the robot instead of grinding into a wall for the rest of the period.
    private static final double PATH_TIMEOUT_SECONDS = 6.0;

    private static final String LOG_TAG = "gardenSide";

    private Follower follower;
    private intakeOuttake take;
    private Timer pathTimer, opModeTimer;

    // One Path per leg, built once up front.
    private final List<Path> paths = new ArrayList<>();

    // How many legs have FINISHED. Also the 0-based index of the leg to start next, which is
    // why one counter is enough: 3 completed means "path 3 is done, next up is paths.get(3)".
    private int pathsCompleted = 0;

    private int ballsShotThisVolley = 0;
    private int totalBallsShot = 0;
    private int volley = 0; // how many volleys have been fired

    // Non-empty means a pose would put the robot through a wall, and loop() refuses to drive.
    private final List<String> geometryProblems = new ArrayList<>();

    /**
     * Every distinct step. Naming them makes the driver station readable - you can see
     * exactly where the robot thinks it is in the routine.
     */
    public enum PathState {
        DRIVE,            // following one of the eight legs
        SPIN_UP,          // launcher coming up to speed (a plain wait for now)
        SHOOT,            // four shots, one at a time - used by EVERY volley
        DONE,
        BLOCKED_BAD_GEOMETRY
    }

    private PathState pathState;

    /**
     * Builds one straight-line leg per waypoint. Leg i runs from the previous pose (the
     * start pose for the first leg) to WAYPOINTS[i], so every pose is typed ONCE.
     *
     * In Pedro 3 a path is made with the static line(from, to) factory, and .linear(from, to)
     * is the heading interpolation - it turns the robot smoothly across the leg so it arrives
     * already pointing the right way instead of stopping and then spinning. To hold one
     * heading for a whole leg use .constant(headingRadians) instead.
     *
     * line() is a straight line. For a leg that would clip a wall or a garden, Paths.curve(...)
     * takes extra control poses that pull the path aside.
     */
    public void buildPaths() {
        Pose from = startPose;

        for (int i = 0; i < PATHS_TO_RUN; i++) {
            Pose to = WAYPOINTS[i];

            paths.add(line(from, to).linear(from, to));

            from = to; // the end of this leg is the start of the next
        }
    }

    /** True if n appears in the schedule. Plain loop - these arrays are two or three long. */
    private static boolean scheduled(int[] schedule, int n) {
        for (int entry : schedule) {
            if (entry == n) return true;
        }
        return false;
    }

    /**
     * The state machine, called once per loop. Each case answers one question: "is this step
     * done, and what is next?" setPathState() resets pathTimer, so inside any case pathTimer
     * tells you how long THIS step has been running.
     */
    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE:
                boolean arrived = !follower.isBusy();
                boolean timedOut = pathTimer.seconds() > PATH_TIMEOUT_SECONDS;

                if (arrived || timedOut) {
                    pathsCompleted++;
                    RobotLog.ii(LOG_TAG, "path %d of %d done (%s) t=%.2fs pose x=%.2f y=%.2f h=%.1f",
                            pathsCompleted, PATHS_TO_RUN, timedOut ? "TIMEOUT" : "arrived",
                            pathTimer.seconds(),
                            follower.pose().x(), follower.pose().y(),
                            Math.toDegrees(follower.pose().heading()));

                    // Is a volley due right here? If so we stay parked on this pose and fire
                    // before touching the next leg.
                    if (scheduled(SHOOT_AFTER_PATHS, pathsCompleted)) {
                        beginVolley();
                    } else {
                        startNextLegOrFinish();
                    }
                }
                break;

            case SPIN_UP:
                // With no launcher this is just a pause. Once the shooter exists, swap the
                // timer for a real at-speed check - far more reliable than a fixed wait,
                // which is always too short on a low battery:
                //
                //   if (take.launcher.getVelocity() >= LAUNCHER_VELOCITY_TICKS * 0.95) { ... }
                //
                if (pathTimer.seconds() > LAUNCHER_SPINUP_SECONDS) {
                    setPathState(PathState.SHOOT);
                }
                break;

            case SHOOT:
                // One ball per SHOT_DWELL_SECONDS. The intake stays on the whole time - that
                // is what feeds the next ball up to the shooter.
                if (pathTimer.seconds() > SHOT_DWELL_SECONDS) {
                    ballsShotThisVolley++;
                    totalBallsShot++;
                    RobotLog.ii(LOG_TAG, "volley %d: shot %d of %d (total %d) - simulated",
                            volley, ballsShotThisVolley, BALLS_PER_VOLLEY, totalBallsShot);

                    // ---- REAL SHOOTER CODE, COMMENTED OUT UNTIL IT IS INSTALLED ----
                    // take.launcher.setVelocity(LAUNCHER_VELOCITY_TICKS);
                    // (whatever feeds one ball into the wheel goes here)

                    if (ballsShotThisVolley < BALLS_PER_VOLLEY) {
                        // Re-entering SHOOT resets pathTimer, which times the next ball.
                        setPathState(PathState.SHOOT);
                    } else {
                        // Volley finished. Spin the launcher down, stop collecting, drive on.
                        // ---- COMMENTED OUT UNTIL THE SHOOTER EXISTS ----
                        // take.launcher.setVelocity(0.0);

                        take.intake.setPower(0.0);
                        startNextLegOrFinish();
                    }
                }
                break;

            case DONE:
            case BLOCKED_BAD_GEOMETRY:
                // Deliberately no requestOpModeStop(). Ending the OpMode wipes telemetry, and
                // the final pose is the most useful thing to read after a run.
                break;
        }
    }

    /**
     * Starts a volley from wherever the robot is standing. The intake comes on because it is
     * the feed - it pushes the next ball up into the wheel between shots.
     */
    private void beginVolley() {
        volley++;
        ballsShotThisVolley = 0;
        take.intake.setPower(INTAKE_POWER);

        // ---- REAL SHOOTER CODE, COMMENTED OUT UNTIL IT IS INSTALLED ----
        // take.launcher.setVelocity(LAUNCHER_VELOCITY_TICKS);

        setPathState(PathState.SPIN_UP);
    }

    /**
     * Hands the next leg to the follower, turning the intake on first if this path number is
     * in INTAKE_ON_BEFORE_PATHS. If every leg is done, shuts everything down instead.
     */
    private void startNextLegOrFinish() {
        if (pathsCompleted >= PATHS_TO_RUN) {
            // stop() releases the follower so it quits driving the motors to hold the final
            // pose (this was breakFollowing() in Pedro 2).
            follower.stop();
            take.intake.setPower(0.0);
            setPathState(PathState.DONE);
            RobotLog.ii(LOG_TAG, "ALL %d PATHS DONE in %.2fs, %d balls",
                    PATHS_TO_RUN, opModeTimer.seconds(), totalBallsShot);
            return;
        }

        int nextPathNumber = pathsCompleted + 1; // 1-based, the way the schedules read

        if (scheduled(INTAKE_ON_BEFORE_PATHS, nextPathNumber)) {
            take.intake.setPower(INTAKE_POWER);
            RobotLog.ii(LOG_TAG, "intake ON for path %d", nextPathNumber);
        }

        // holdEnd keeps Pedro actively holding the end pose, which matters here because we
        // may well stand on it and shoot.
        follower.holdEnd.set(true);
        follower.follow(paths.get(pathsCompleted));
        setPathState(PathState.DRIVE);
    }

    /** Advances the state machine and restarts pathTimer so each step times itself. */
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.reset();
        RobotLog.ii(LOG_TAG, "state -> %s at t=%.2fs pose x=%.2f y=%.2f heading=%.1fdeg",
                newState, opModeTimer.seconds(),
                follower.pose().x(), follower.pose().y(),
                Math.toDegrees(follower.pose().heading()));
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

    /** How many volleys the schedule will actually fire, for the telemetry count. */
    private int plannedVolleys() {
        int n = 0;
        for (int entry : SHOOT_AFTER_PATHS) {
            if (entry >= 0 && entry <= PATHS_TO_RUN) n++;
        }
        return n;
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        take = new intakeOuttake(hardwareMap);

        checkGeometry();

        if (geometryProblems.isEmpty()) {
            buildPaths();
            follower.setPose(startPose);
            telemetry.addLine("gardenSide: " + PATHS_TO_RUN + " paths, "
                    + plannedVolleys() + " volleys of " + BALLS_PER_VOLLEY
                    + " = " + (plannedVolleys() * BALLS_PER_VOLLEY) + " balls.");
            telemetry.addLine("Shooter NOT installed - shots are timed waits.");
        } else {
            // Refuse to move rather than drive a pose that is inside a wall.
            pathState = PathState.BLOCKED_BAD_GEOMETRY;
            telemetry.addLine("!! WILL NOT DRIVE - a pose would hit a wall !!");
            for (String p : geometryProblems) telemetry.addLine("  " + p);
            telemetry.addLine("Fix the pose, or correct FIELD_SIZE_IN / ROBOT_SIZE_IN.");
        }

        telemetry.addLine("Foresight values in Constants.java are PLACEHOLDERS until the ForesightTuner has been run.");
        telemetry.addData("missing hardware", take.missingDevices());
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.reset();
        pathTimer.reset();

        if (!geometryProblems.isEmpty()) return;

        // A volley scheduled "after 0 paths" fires right here, from the start pose, before
        // anything drives. Otherwise we set off down path 1 immediately.
        if (scheduled(SHOOT_AFTER_PATHS, 0)) {
            beginVolley();
        } else {
            startNextLegOrFinish();
        }

        RobotLog.ii(LOG_TAG, "START problems=%d missing=%s",
                geometryProblems.size(), take.missingDevices());
    }

    @Override
    public void loop() {
        if (pathState == PathState.BLOCKED_BAD_GEOMETRY) {
            // Touch nothing - no follower.update(), no motor commands.
            telemetry.addLine("BLOCKED: a pose would put the robot through a wall.");
            for (String p : geometryProblems) telemetry.addLine("  " + p);
            telemetry.update();
            return;
        }

        // One step along the current path. Must happen every loop or the robot never moves -
        // and during a volley it is what holds the robot still on the shooting pose.
        follower.update();
        statePathUpdate();

        Pose target = WAYPOINTS[Math.min(pathsCompleted, PATHS_TO_RUN - 1)];

        telemetry.addData("state", pathState);
        telemetry.addData("path", "%d / %d",
                Math.min(pathsCompleted + 1, PATHS_TO_RUN), PATHS_TO_RUN);
        telemetry.addData("target", "%.1f, %.1f @ %.0f deg",
                target.x(), target.y(), Math.toDegrees(target.heading()));
        telemetry.addData("actual", "%.1f, %.1f @ %.0f deg",
                follower.pose().x(), follower.pose().y(),
                Math.toDegrees(follower.pose().heading()));
        telemetry.addData("volley", "%d of %d", volley, plannedVolleys());
        telemetry.addData("this volley", "%d / %d", ballsShotThisVolley, BALLS_PER_VOLLEY);
        telemetry.addData("total shot", "%d / %d",
                totalBallsShot, plannedVolleys() * BALLS_PER_VOLLEY);
        telemetry.addData("intake", "%.2f", take.intake.getPower());
        telemetry.addData("step time", "%.2f s", pathTimer.seconds());
        telemetry.addData("total time", "%.2f s", opModeTimer.seconds());
        telemetry.update();
    }
}
