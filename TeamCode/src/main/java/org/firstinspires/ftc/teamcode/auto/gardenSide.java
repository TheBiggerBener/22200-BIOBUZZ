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
 * GARDEN SIDE AUTONOMOUS - four nectar balls from the garden, shoot, park.
 *
 * THE FIELD FRAME (as described by the team)
 *   (0, 0) is the bottom-left corner of the field, on the red garden side.
 *   +X goes RIGHT, toward the BLUE side.      -X goes LEFT, toward the RED side.
 *   +Y goes UP, toward the blue garden and the red parking spot.
 *   -Y goes DOWN, toward the red garden and the blue parking spot.
 *   Heading is where the robot's FRONT points, 0 = facing +X, 90 = facing +Y.
 *
 * THE PLAN - eight balls total
 *   1. Start at (82.2, 141.4) facing 180, already loaded with four balls.
 *   2. Shoot those four WITHOUT MOVING. The shooting pose and the starting pose are the
 *      same spot, so the first volley costs no driving time at all.
 *   3. Drive to the garden (x = 120..144 along the top) and creep through it with the
 *      intake running, collecting four more.
 *   4. Drive back to the starting pose and shoot those four.
 *   5. Park at (134, 45) - far +X side of the zone, leaving the rest for our partner.
 *
 * THE SHOOTER IS NOT INSTALLED. Every launcher call is written out but commented. Each
 * "shot" is a timed wait with the intake still turning, standing in for the real thing.
 *
 * HOW IT RUNS
 *   This is an iterative OpMode. loop() is called over and over, maybe 50 times a second.
 *   Each call does two things: follower.update() lets Pedro take one step along the current
 *   path, then statePathUpdate() checks whether it is time to move to the next step. Nothing
 *   blocks or sleeps - each loop is one thin slice of the whole routine. The usual test for
 *   "has the drive finished" is !follower.isBusy(); Pedro reports busy while still working.
 */
@Autonomous(name = "gardenSide", group = "auto")
public class gardenSide extends OpMode {

    // ============================ FIELD AND ROBOT SIZE ===============================
    // Used to check every pose keeps the robot off the walls. A standard FTC field is 144 in
    // (12 ft) square. TODO confirm the BIOBUZZ field is actually 144 - nothing in the repo
    // says so, and the .fmap's own numbers are template defaults, not this field.
    private static final double FIELD_SIZE_IN = 144.0;
    private static final double ROBOT_SIZE_IN = 17.0; // 17 x 17 per the team

    // ============================== FIELD GEOMETRY ===================================
    // Poses are the CENTRE of the robot. With a 17 in robot the centre has to stay at least
    // 8.5 in from every wall, so the legal box is 8.5 .. 135.5 on both axes. The check in
    // init() enforces that and refuses to drive if any pose falls outside it.

    // Start, and also where we shoot from. Given as (82.2, 141.4) heading 180.
    // NOTE: y = 141.4 puts the robot's edge 5.9 in past the 144 wall - see init()'s warning.
    private final Pose startPose = new Pose(82.2, 141.4, Math.toRadians(180));

    // Shooting happens from the starting pose, so this is deliberately the same spot.
    private final Pose shootPose = new Pose(82.2, 137.0, Math.toRadians(180));

    // The garden runs x = 120..144 along the top of the field. The robot CENTRE cannot reach
    // x = 144 (that is the wall) or sit at y >= 140, so the collecting run travels along the
    // highest safe y with the intake reaching up into the garden.
    // TODO confirm GARDEN_COLLECT_Y against where the intake actually grabs from.
    private static final double GARDEN_COLLECT_Y = 134.0; // 1.5 in of wall clearance
    private static final double GARDEN_X_START = 120.0;   // low-x end of the garden
    private static final double GARDEN_X_END = 135.5;     // furthest +X the centre can go

    // Heading 90 points the robot's front at the garden (+Y) so the intake faces the balls
    // while the robot travels sideways through the strip.
    // TODO if your intake is not on the front, or you would rather drive through nose-first
    // along +X, change these headings to 0 and re-test.
    private final Pose gardenStartPose = new Pose(GARDEN_X_START, GARDEN_COLLECT_Y, Math.toRadians(90));
    private final Pose gardenEndPose = new Pose(GARDEN_X_END, GARDEN_COLLECT_Y, Math.toRadians(90));

    // Park far over on +X so our partner still has room in the zone.
    // TODO the parking heading was not specified - 180 just matches the start. Change freely.
    private final Pose parkPose = new Pose(134.0, 45.0, Math.toRadians(180));

    // =============================== TIMINGS / POWERS ================================
    // TODO the real launcher velocity is unknown - the shooter is not installed. Measure it
    // with shootTest and put it here. setVelocity() is in ENCODER TICKS PER SECOND, not RPM;
    // shootTest prints both so you can convert.
    private static final double LAUNCHER_VELOCITY_TICKS = 0.0; // TODO measure

    // TODO how long the launcher needs to reach speed before the first shot.
    private static final double LAUNCHER_SPINUP_SECONDS = 1.5;

    // Stand-in for one shot while there is no shooter: wait this long with the intake
    // turning, as though a ball went out. TODO replace with the real feed time.
    private static final double SHOT_DWELL_SECONDS = 0.8;

    // Four preloaded, then four collected from the garden = eight shots total.
    private static final int BALLS_PER_VOLLEY = 4;
    private static final int VOLLEYS = 2;

    // Positive should pull balls IN. If it spits them out, negate this rather than rewiring.
    private static final double INTAKE_POWER = 1.0;

    // Give up on a path that has not finished by now, so a tuning problem parks the robot
    // instead of grinding into a wall for the rest of the period.
    private static final double PATH_TIMEOUT_SECONDS = 6.0;

    private static final String LOG_TAG = "gardenSide";

    private Follower follower;
    private intakeOuttake take;
    private Timer pathTimer, opModeTimer;

    private Path toGarden, alongGarden, toShoot, toPark;

    private int ballsShotThisVolley = 0;
    private int totalBallsShot = 0;
    private int volley = 1; // 1 = the preloaded four, 2 = the four from the garden

    // Filled in by init(). Non-empty means at least one pose would hit a wall, and loop()
    // will refuse to drive.
    private final List<String> geometryProblems = new ArrayList<>();

    /**
     * Every distinct step. Naming them makes the driver station readable - you can see
     * exactly where the robot thinks it is in the routine.
     */
    public enum PathState {
        SPIN_UP,          // launcher coming up to speed (a plain wait for now)
        SHOOT,            // four shots, one at a time - used by BOTH volleys
        DRIVE_TO_GARDEN,  // after volley 1, go get four more
        COLLECT_BALLS,    // intake running, creeping through the garden
        DRIVE_TO_SHOOT,   // carrying four balls back to the shooting pose
        DRIVE_TO_PARK,    // after volley 2, park
        PARKED,
        BLOCKED_BAD_GEOMETRY
    }

    private PathState pathState;

    /**
     * Builds every path once, up front. A Path is an immutable recipe, so building it here
     * costs nothing at run time and keeps loop() fast.
     *
     * line(a, b) is a straight line between two poses. If a straight line would clip a wall
     * or the garden, swap in curve(a, control, b) with one or two control poses pulling the
     * path away from the obstacle - same idea, just more points.
     *
     * .linear(from, to) turns the robot smoothly across the path, so it arrives already
     * pointing the right way instead of stopping and then spinning.
     */
    public void buildPaths() {
        toGarden = line(startPose, gardenStartPose)
                .linear(startPose, gardenStartPose);

        // The collecting run - its own short path so the intake has time to pull balls in
        // as the robot creeps past them.
        alongGarden = line(gardenStartPose, gardenEndPose)
                .linear(gardenStartPose, gardenEndPose);

        // Back to the starting pose, because that is where we shoot from.
        toShoot = line(gardenEndPose, shootPose)
                .linear(gardenEndPose, shootPose);

        toPark = line(shootPose, parkPose)
                .linear(shootPose, parkPose);
    }

    /**
     * The state machine, called once per loop. Each case answers one question: "is this step
     * done, and what is next?" setPathState() resets pathTimer, so inside any case pathTimer
     * tells you how long THIS step has been running.
     */
    public void statePathUpdate() {
        boolean pathTimedOut = pathTimer.seconds() > PATH_TIMEOUT_SECONDS;

        switch (pathState) {
            case SPIN_UP:
                // Entered twice: once at the very start for the preloaded balls, and again
                // after returning from the garden. No driving happens here either time -
                // for volley 1 we have not moved yet, and for volley 2 DRIVE_TO_SHOOT
                // already put us back on the shooting pose.
                //
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
                    } else if (volley < VOLLEYS) {
                        // Volley 1 done - go fill up at the garden.
                        volley++;
                        ballsShotThisVolley = 0;
                        follower.follow(toGarden);
                        setPathState(PathState.DRIVE_TO_GARDEN);
                    } else {
                        // Volley 2 done - that is all eight. Shut down and park.
                        // ---- COMMENTED OUT UNTIL THE SHOOTER EXISTS ----
                        // take.launcher.setVelocity(0.0);

                        take.intake.setPower(0.0);
                        follower.follow(toPark);
                        setPathState(PathState.DRIVE_TO_PARK);
                    }
                }
                break;

            case DRIVE_TO_GARDEN:
                if (!follower.isBusy() || pathTimedOut) {
                    // Intake stays on from the first volley right through collecting, so it
                    // is already spinning when the first garden ball arrives.
                    follower.follow(alongGarden);
                    setPathState(PathState.COLLECT_BALLS);
                }
                break;

            case COLLECT_BALLS:
                // Intake runs for the whole creep through the garden.
                if (!follower.isBusy() || pathTimedOut) {
                    // Leave it RUNNING on purpose - it holds the balls in on the way back.
                    follower.follow(toShoot);
                    setPathState(PathState.DRIVE_TO_SHOOT);
                }
                break;

            case DRIVE_TO_SHOOT:
                if (!follower.isBusy() || pathTimedOut) {
                    // ---- REAL SHOOTER CODE, COMMENTED OUT UNTIL IT IS INSTALLED ----
                    // take.launcher.setVelocity(LAUNCHER_VELOCITY_TICKS);
                    setPathState(PathState.SPIN_UP);
                }
                break;

            case DRIVE_TO_PARK:
                if (!follower.isBusy() || pathTimedOut) {
                    // Release the path so Pedro stops driving the motors to hold the pose.
                    follower.stop();
                    take.intake.setPower(0.0);
                    setPathState(PathState.PARKED);
                }
                break;

            case PARKED:
            case BLOCKED_BAD_GEOMETRY:
                // Deliberately no requestOpModeStop(). Ending the OpMode wipes telemetry, and
                // the final pose is the most useful thing to read after a run. This also
                // matches how the alliance autos finish.
                break;
        }
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
     * The robot centre has to stay half a robot away from every edge.
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
        String[] names = {"startPose", "gardenStart", "gardenEnd", "shootPose", "parkPose"};
        Pose[] poses = {startPose, gardenStartPose, gardenEndPose, shootPose, parkPose};

        for (int i = 0; i < poses.length; i++) {
            String problem = wallProblem(names[i], poses[i]);
            if (problem != null) {
                geometryProblems.add(problem);
                RobotLog.ww(LOG_TAG, "GEOMETRY: %s", problem);
            }
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        take = new intakeOuttake(hardwareMap);

        buildPaths();
        follower.setPose(startPose);

        checkGeometry();

        if (geometryProblems.isEmpty()) {
            // Starts by shooting the preload - no driving needed, the shooting pose IS the
            // starting pose.
            pathState = PathState.SPIN_UP;
            telemetry.addLine("gardenSide: shoot 4 preloaded, collect 4, shoot, park (8 total).");
            telemetry.addLine("Shooter NOT installed - shots are timed waits.");
        } else {
            // Refuse to move rather than drive a pose that is inside a wall.
            pathState = PathState.BLOCKED_BAD_GEOMETRY;
            telemetry.addLine("!! WILL NOT DRIVE - a pose would hit a wall !!");
            for (String problem : geometryProblems) {
                telemetry.addLine("  " + problem);
            }
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

        if (geometryProblems.isEmpty()) {
            // Intake on for the whole routine: it feeds the shooter during both volleys and
            // collects in between. It only stops once all eight shots are away.
            take.intake.setPower(INTAKE_POWER);

            // ---- REAL SHOOTER CODE, COMMENTED OUT UNTIL IT IS INSTALLED ----
            // take.launcher.setVelocity(LAUNCHER_VELOCITY_TICKS);
        }

        RobotLog.ii(LOG_TAG, "START problems=%d missing=%s",
                geometryProblems.size(), take.missingDevices());
    }

    @Override
    public void loop() {
        if (pathState == PathState.BLOCKED_BAD_GEOMETRY) {
            // Touch nothing - no follower.update(), no motor commands.
            telemetry.addLine("BLOCKED: a pose would put the robot through a wall.");
            for (String problem : geometryProblems) {
                telemetry.addLine("  " + problem);
            }
            telemetry.update();
            return;
        }

        // One step along the current path. Must happen every loop or the robot never moves.
        follower.update();
        statePathUpdate();

        telemetry.addData("state", pathState);
        telemetry.addData("volley", "%d of %d", volley, VOLLEYS);
        telemetry.addData("this volley", "%d / %d", ballsShotThisVolley, BALLS_PER_VOLLEY);
        telemetry.addData("total shot", "%d / %d", totalBallsShot, BALLS_PER_VOLLEY * VOLLEYS);
        telemetry.addData("x / y", "%.2f / %.2f in",
                follower.pose().x(), follower.pose().y());
        telemetry.addData("heading", "%.1f deg", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("step time", "%.2f s", pathTimer.seconds());
        telemetry.addData("total time", "%.2f s", opModeTimer.seconds());
        telemetry.update();
    }
}
