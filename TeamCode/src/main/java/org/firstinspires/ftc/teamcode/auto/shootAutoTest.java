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
 * STEP-LIST AUTONOMOUS - driving, shooting and waiting, all written as one readable list.
 *
 * ================================ HOW TO USE IT =================================
 * There is ONE thing to edit: the ROUTINE array below. Each entry is one step, and the robot
 * does them strictly in order. Four kinds of step:
 *
 *     drive(pose)      drive in a straight line to that pose
 *     shoot(n)         fire n balls (spin up, then one shot per SHOT_DWELL_SECONDS)
 *     pause(seconds)   sit still for a bit - let a ball settle, wait for a partner, etc.
 *     intake(power)    set the intake motor, 1.0 for in, 0.0 for off, -1.0 to reverse
 *
 * So a whole match plan reads like the plan:
 *
 *     shoot(4),                                    // fire the preload where we start
 *     intake(1.0),                                 // intake on before the garden
 *     drive(new Pose(120, 135.5, rad(90))),        // over to the garden
 *     drive(new Pose(135.5, 135.5, rad(90))),      // creep through it
 *     pause(0.5),                                  // let the last ball settle
 *     drive(new Pose(82.2, 135.5, rad(180))),      // back to the shooting spot
 *     shoot(4),                                    // fire those four
 *     intake(0.0),                                 // intake off
 *     drive(new Pose(134, 45, rad(0))),            // park
 *
 * Add, remove or reorder lines freely - nothing else in the file needs to change. Drive steps
 * chain automatically: each one starts wherever the previous drive ended, so every pose is
 * typed once.
 * ================================================================================
 *
 * THE SHOOTER IS NOT INSTALLED YET. Every launcher call is written out but commented, and a
 * shoot(n) step is currently just a spin-up wait followed by n dwell waits, with the intake
 * left running. When the shooter goes on, uncomment those lines - the timing structure is
 * already correct.
 *
 * THE FIELD FRAME (same as gardenSide / pathTest)
 *   (0, 0) is the bottom-left corner of the field, on the red garden side.
 *   +X goes RIGHT toward BLUE.      -X goes LEFT toward RED.
 *   +Y goes UP toward the blue garden / red parking.
 *   -Y goes DOWN toward the red garden / blue parking.
 *   Heading is where the robot's FRONT points: 0 = +X, 90 = +Y, 180 = -X.
 */
@Autonomous(name = "shootAutoTest", group = "auto")
public class shootAutoTest extends OpMode {

    /** Shorthand so the routine below stays readable. */
    private static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    // ============================ SET YOUR ROUTINE HERE =============================
    // Where the robot is physically placed before you press start.
    private final Pose startPose = new Pose(82.2, 135.5, rad(0));

    // The eight path positions are copied straight from pathTest. Shooting and intaking are
    // slotted in between them:
    //   - shoot before path 1 (the preload), after path 4, and after path 6  = 12 balls
    //   - intake running across path 1 -> 2, and across path 5 -> 6
    //
    // Reminder on the numbering: path N is the LEG that ENDS at waypoint N. Path 1 runs from
    // the start pose to waypoint 1, path 2 from waypoint 1 to waypoint 2, and so on - which
    // is why "intake from path 1 to 2" is an intake(1.0) sitting between those two drives.
    private final Step[] ROUTINE = new Step[] {
            shoot(4),                                    // before path 1 - the preloaded four
            intake(0.0),                                 // stop the feed once they are away

            drive(new Pose(100.0, 135.5, rad(0))),     // path 1
            intake(1.0),                                 // intake ON for path 1 -> 2
            drive(new Pose(120.0, 135.5, rad( 0))),     // path 2
            intake(0.0),                                 // intake OFF

            drive(new Pose(135.5, 135.5, rad( 90))),     // path 3
            drive(new Pose(135.5, 100.0, rad(  90))),     // path 4

            shoot(4),                                    // after path 4
            intake(0.0),                                 // stop the feed

            drive(new Pose(134.0,  45.0, rad(  -90))),     // path 5
            intake(1.0),                                 // intake ON for path 5 -> 6
            drive(new Pose(100.0,  45.0, rad(137))),     // path 6

            shoot(4),                                    // after path 6 (keeps the intake on)
            intake(0.0),                                 // stop the feed

            drive(new Pose( 60.0,  45.0, rad(-42))),     // path 7
            drive(new Pose( 60.0, 100.0, rad( 180))),     // path 8
    };

    // =============================== TIMINGS / POWERS ================================
    // TODO the real launcher velocity is unknown - the shooter is not installed. Measure it
    // with shootTest and put it here. setVelocity() is in ENCODER TICKS PER SECOND, not RPM;
    // shootTest prints both so you can convert.
    private static final double LAUNCHER_VELOCITY_TICKS = 0.0; // TODO measure

    // TODO how long the launcher needs to reach speed before the first shot of a volley.
    private static final double LAUNCHER_SPINUP_SECONDS = 1.0;

    // TODO one shot takes this long. With no shooter it is simply the stand-in wait.
    private static final double SHOT_DWELL_SECONDS = 0.8;

    // Give up on a drive that has not finished by now, so a bad coordinate or a tuning
    // problem parks the robot instead of grinding into a wall for the rest of the period.
    private static final double DRIVE_TIMEOUT_SECONDS = 6.0;

    // ============================ FIELD AND ROBOT SIZE ==============================
    private static final double FIELD_SIZE_IN = 144.0; // standard FTC field, 12 ft square
    private static final double ROBOT_SIZE_IN = 17.0;  // 17 x 17 per the team

    private static final String LOG_TAG = "shootAutoTest";

    // ================================= STEP TYPES ====================================

    private enum Kind { DRIVE, SHOOT, PAUSE, INTAKE }

    /**
     * One line of the routine. Only the field matching its Kind is meaningful; the factory
     * methods below mean you never build one of these by hand.
     */
    private static final class Step {
        final Kind kind;
        final Pose pose;      // DRIVE
        final int count;      // SHOOT - how many balls
        final double value;   // PAUSE - seconds; INTAKE - motor power

        private Step(Kind kind, Pose pose, int count, double value) {
            this.kind = kind;
            this.pose = pose;
            this.count = count;
            this.value = value;
        }
    }

    private static Step drive(Pose pose)      { return new Step(Kind.DRIVE,  pose, 0, 0); }
    private static Step shoot(int balls)      { return new Step(Kind.SHOOT,  null, balls, 0); }
    private static Step pause(double seconds) { return new Step(Kind.PAUSE,  null, 0, seconds); }
    private static Step intake(double power)  { return new Step(Kind.INTAKE, null, 0, power); }

    // ================================== RUNTIME ======================================

    private Follower follower;
    private intakeOuttake take;
    private Timer stepTimer, opModeTimer;

    // Built at init: paths.get(i) is the Path for ROUTINE[i], or null if that step is not a
    // drive. Pre-building keeps loop() fast and catches geometry mistakes before moving.
    private final List<Path> paths = new ArrayList<>();

    private int stepIndex = 0;
    private boolean finished = false;

    // SHOOT bookkeeping, reset at the start of each shoot step.
    private boolean spinUpDone = false;
    private int ballsFiredThisStep = 0;
    private int totalBallsFired = 0;

    private final List<String> geometryProblems = new ArrayList<>();

    /**
     * Walks the routine and builds a Path for every drive step. Each drive starts from the
     * previous drive's pose (or the start pose), so the route chains without you repeating
     * coordinates. Non-drive steps get a null placeholder to keep indices lined up.
     *
     * line(from, to) is Pedro 3's straight-line factory; .linear(from, to) turns the robot
     * smoothly across the leg so it arrives already pointing the right way. For a constant
     * heading use .constant(radians) instead, and for a leg that would clip something use
     * Paths.curve(...) with extra control poses.
     */
    private void buildPaths() {
        Pose from = startPose;

        for (Step step : ROUTINE) {
            if (step.kind == Kind.DRIVE) {
                paths.add(line(from, step.pose).linear(from, step.pose));
                from = step.pose; // next drive starts here
            } else {
                paths.add(null);
            }
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
        String problem = wallProblem("startPose", startPose);
        if (problem != null) geometryProblems.add(problem);

        for (int i = 0; i < ROUTINE.length; i++) {
            if (ROUTINE[i].kind == Kind.DRIVE) {
                problem = wallProblem("step " + (i + 1) + " drive", ROUTINE[i].pose);
                if (problem != null) geometryProblems.add(problem);
            }
        }

        for (String p : geometryProblems) {
            RobotLog.ww(LOG_TAG, "GEOMETRY: %s", p);
        }
    }

    /**
     * Sets up whichever step we have just arrived at. Called once per step, not per loop -
     * this is where a drive gets handed to the follower and where an intake power is applied.
     */
    private void beginStep() {
        if (stepIndex >= ROUTINE.length) {
            // Nothing left. Release the follower so it stops driving the motors to hold the
            // final pose (this is Pedro 3's replacement for breakFollowing()).
            follower.stop();
            take.intake.setPower(0.0);
            finished = true;
            RobotLog.ii(LOG_TAG, "ROUTINE DONE in %.2fs, %d balls fired",
                    opModeTimer.seconds(), totalBallsFired);
            return;
        }

        Step step = ROUTINE[stepIndex];
        stepTimer.reset();

        switch (step.kind) {
            case DRIVE:
                follower.holdEnd.set(true);
                follower.follow(paths.get(stepIndex));
                RobotLog.ii(LOG_TAG, "step %d: DRIVE to %.1f, %.1f @ %.0fdeg",
                        stepIndex + 1, step.pose.x(), step.pose.y(),
                        Math.toDegrees(step.pose.heading()));
                break;

            case SHOOT:
                spinUpDone = false;
                ballsFiredThisStep = 0;

                // Intake on so it feeds balls up to the wheel while shooting.
                take.intake.setPower(1.0);

                // ---- REAL SHOOTER CODE, COMMENTED OUT UNTIL IT IS INSTALLED ----
                // take.launcher.setVelocity(LAUNCHER_VELOCITY_TICKS);

                RobotLog.ii(LOG_TAG, "step %d: SHOOT %d", stepIndex + 1, step.count);
                break;

            case PAUSE:
                RobotLog.ii(LOG_TAG, "step %d: PAUSE %.2fs", stepIndex + 1, step.value);
                break;

            case INTAKE:
                take.intake.setPower(step.value);
                RobotLog.ii(LOG_TAG, "step %d: INTAKE power %.2f", stepIndex + 1, step.value);
                break;
        }
    }

    /**
     * Asks "is the current step finished?" once per loop. Returning true means move on.
     * Each kind has its own idea of done: a drive waits for the follower, a pause watches
     * the clock, shooting counts balls, and an intake change is instant.
     */
    private boolean stepIsDone() {
        Step step = ROUTINE[stepIndex];

        switch (step.kind) {
            case DRIVE:
                // isBusy() is Pedro saying "still working on it". The timeout is the escape
                // hatch for a path that can never finish.
                boolean timedOut = stepTimer.seconds() > DRIVE_TIMEOUT_SECONDS;
                if (timedOut) {
                    RobotLog.ww(LOG_TAG, "step %d TIMED OUT after %.2fs",
                            stepIndex + 1, stepTimer.seconds());
                }
                return !follower.isBusy() || timedOut;

            case SHOOT:
                // First wait for spin-up, then one ball per SHOT_DWELL_SECONDS.
                if (!spinUpDone) {
                    // Once the shooter exists, replace this timer with a real at-speed check -
                    // far more reliable than a fixed wait, which is always too short on a low
                    // battery and silently throws the first shot:
                    //
                    //   if (take.launcher.getVelocity() >= LAUNCHER_VELOCITY_TICKS * 0.95)
                    //
                    if (stepTimer.seconds() > LAUNCHER_SPINUP_SECONDS) {
                        spinUpDone = true;
                        stepTimer.reset(); // start timing the first shot
                    }
                    return false;
                }

                if (stepTimer.seconds() > SHOT_DWELL_SECONDS) {
                    ballsFiredThisStep++;
                    totalBallsFired++;
                    RobotLog.ii(LOG_TAG, "  shot %d of %d (total %d) - simulated, no shooter",
                            ballsFiredThisStep, step.count, totalBallsFired);

                    // ---- REAL SHOOTER CODE, COMMENTED OUT UNTIL IT IS INSTALLED ----
                    // (whatever feeds one ball into the wheel goes here)

                    stepTimer.reset(); // time the next shot
                }

                if (ballsFiredThisStep >= step.count) {
                    // ---- COMMENTED OUT UNTIL THE SHOOTER EXISTS ----
                    // take.launcher.setVelocity(0.0);
                    return true;
                }
                return false;

            case PAUSE:
                return stepTimer.seconds() > step.value;

            case INTAKE:
                // Setting a motor power happens instantly - nothing to wait for.
                return true;
        }
        return true;
    }

    @Override
    public void init() {
        stepTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        take = new intakeOuttake(hardwareMap);

        checkGeometry();

        if (geometryProblems.isEmpty()) {
            buildPaths();
            follower.setPose(startPose);
            telemetry.addLine("shootAutoTest: " + ROUTINE.length + " steps.");
            telemetry.addLine("Shooter NOT installed - shoot steps are timed waits.");
        } else {
            telemetry.addLine("!! WILL NOT DRIVE - a pose would hit a wall !!");
            for (String p : geometryProblems) telemetry.addLine("  " + p);
            telemetry.addLine("Fix the pose, or correct FIELD_SIZE_IN / ROBOT_SIZE_IN.");
        }
        telemetry.addData("missing hardware", take.missingDevices());
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.reset();
        if (geometryProblems.isEmpty()) {
            beginStep(); // kick off step 1
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

        // One step along the current path. Must happen every loop or the robot never moves,
        // and isBusy() would never clear.
        follower.update();

        if (!finished && stepIsDone()) {
            stepIndex++;
            beginStep();
        }

        // Deliberately no requestOpModeStop() - ending the OpMode wipes the telemetry, and
        // the final pose is the most useful thing to read after a run.

        String what = finished ? "done"
                : ROUTINE[stepIndex].kind + (ROUTINE[stepIndex].kind == Kind.SHOOT
                        ? " " + ballsFiredThisStep + "/" + ROUTINE[stepIndex].count : "");

        telemetry.addData("step", "%d / %d  (%s)",
                Math.min(stepIndex + 1, ROUTINE.length), ROUTINE.length, what);
        telemetry.addData("balls fired", totalBallsFired);
        telemetry.addData("actual", "%.1f, %.1f @ %.0f deg",
                follower.pose().x(), follower.pose().y(),
                Math.toDegrees(follower.pose().heading()));
        telemetry.addData("step time", "%.2f s", stepTimer.seconds());
        telemetry.addData("total time", "%.2f s", opModeTimer.seconds());
        telemetry.update();
    }
}
