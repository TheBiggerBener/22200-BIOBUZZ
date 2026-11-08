package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Mechanism + AprilTag helper, ported from last season's DECODE code so the autonomous
 * side can drive the non-drivetrain hardware and align to tags through Road Runner Actions.
 *
 * ===================== THE HARDWARE THIS CLASS EXPECTS ===========================
 * Exactly the devices LlDrivemode.java configures, by the same config names, so teleop and
 * autonomous can never disagree about what is on the robot:
 *
 *   REQUIRED  frontLeft  backLeft  frontRight  backRight   (drivetrain, DcMotorEx)
 *             intake                                        (DcMotorEx)
 *             imu
 *
 *   OPTIONAL  garden  garden2                               (Servo)
 *             launcherMotor                                 (DcMotorEx)
 *             limelight                                     (Limelight3A)
 *
 * Required devices use hardwareMap.get(), which throws at init if the config is missing one -
 * that is deliberate, because an auto cannot drive without a drivetrain and you want to know
 * at init, not halfway down a path. Optional devices use hardwareMap.tryGet(), which returns
 * null instead of throwing, and every use below is null-guarded. A missing launcher means
 * "that part does nothing", not "the OpMode crashed". Call missingDevices() and put it on
 * telemetry so it is obvious which pieces are absent.
 *
 * Last season's leftFeeder, rightFeeder, topWheel, boxMotor and laserAnalogInput are GONE
 * from this class - none of them is on this robot or in its config, so they could not be
 * used from an autonomous anyway.
 *
 * The goBILDA Pinpoint ("odo") is deliberately NOT fetched here even though LlDrivemode has
 * it. In autonomous, Pedro's PinpointLocalizer owns that device - see Constants.java. Two
 * objects talking to the same I2C device fight over it, so autonomous reads the robot's pose
 * from follower.pose() instead.
 * =================================================================================
 */
public class intakeOuttake {

    // ---- TODO: THESE TAG IDS ARE A GUESS AND MUST BE CONFIRMED ----
    // Last season (DECODE) the alliance goals were tag 20 (blue) and 24 (red). This field
    // is different: BIOBUZZ.fmap places tags 30-37 and no 20 or 24 at all, so last year's
    // numbers cannot be right here. In that fmap, 30-33 sit in one column (x = -0.32 m) and
    // 34-37 in the other (x = +0.32 m), so one column per alliance is the obvious reading -
    // hence 30 and 34 below. CONFIRM against the real field before trusting an auto to it.
    public static final int BLUE_TARGET_TAG = 30;
    public static final int RED_TARGET_TAG = 34;

    // Intake power used by intakeBalls() and shootBalls(). ONE place to change it - the
    // intake has been overheating, and if you lower it, lower it here.

    // TODO these are last season's launcher gains, for last season's launcher. Re-tune.
    private static final double LAUNCHER_P = 430.0;
    private static final double LAUNCHER_F = 14.8;

    // --- required: the OpMode will not start without these ---
    public final DcMotorEx frontLeft, backLeft, frontRight, backRight;
    public final DcMotorEx intake;
    public final IMU imu;

    // --- optional: null when not in the robot config, every use below is guarded ---
    // The garden servos. Present in LlDrivemode as "garden" and "garden2".
    public final Servo garden1, garden2;

    // NOT ON THE ROBOT RIGHT NOW (Oct 2026). Bolt the hardware on, add it to the robot
    // config, and these start working again with no code change - nothing to uncomment.
    public final Limelight3A limelight;
    public final DcMotorEx launcher;

    private final List<String> missing = new ArrayList<>();
    private final ElapsedTime turningTime = new ElapsedTime();

    public intakeOuttake(HardwareMap hardwareMap) {
        // --- drivetrain: names and directions match LlDrivemode.java ---
        // NOTE these assign the FIELDS. Writing "DcMotorEx frontLeft = hardwareMap.get(...)"
        // here declares a local that shadows the field and leaves the field null, which is
        // how this class ended up not compiling before.
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // --- IMU: orientation copied from LlDrivemode.java (UP / FORWARD).
        // Last season's file said RIGHT / UP; that was a different robot. Do not "restore" it.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )));

        // --- optional hardware: null rather than an exception if not configured ---
        garden1 = track(hardwareMap.tryGet(Servo.class, "garden"), "garden");
        garden2 = track(hardwareMap.tryGet(Servo.class, "garden2"), "garden2");
        launcher = track(hardwareMap.tryGet(DcMotorEx.class, "launcherMotor"), "launcherMotor");
        limelight = track(hardwareMap.tryGet(Limelight3A.class, "limelight"), "limelight");

        if (launcher != null) {
            launcher.setDirection(DcMotorEx.Direction.FORWARD);
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (limelight != null) {
            limelight.pipelineSwitch(TagDistance.APRILTAG_PIPELINE);
            limelight.start();
        }

        if (!missing.isEmpty()) {
            RobotLog.ww("intakeOuttake", "not configured on this robot: %s", missingDevices());
        }
    }

    private <T> T track(T device, String name) {
        if (device == null) missing.add(name);
        return device;
    }

    /** Comma separated list of hardware this class wanted but the robot is not configured for. */
    public String missingDevices() {
        return missing.isEmpty() ? "none" : String.join(", ", missing);
    }

    public class intakeBalls implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeBalls();   // the immediate method below - one implementation, not two
            return false;
        }
    }

    public class stopBalls implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            stopBalls();
            return false;
        }
    }

    public class shootBalls implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            shootBalls();
            return false;
        }
    }

    public class revLauncher implements Action {
        private final double velocity;

        public revLauncher(double velocity) {
            this.velocity = velocity;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            revLauncher(velocity);
            return false;
        }
    }

    /**
     * Search for the alliance's AprilTag and rotate until it is at the aim offset.
     * Call once per loop tick; returns true while it still needs more ticks.
     */
    public class turnToTag implements Action {
        private static final double SEARCH_POWER = 0.3;
        private static final double GIVE_UP_SECONDS = 2.0;
        private static final double MAX_SEARCH_SECONDS = 3.0;
        private static final double AIM_OFFSET_DEGREES = -3.0;
        private static final double AIM_TOLERANCE_DEGREES = 1.0;
        private static final double kP = 0.015;
        private static final double MAX_TURN_POWER = 0.18;

        private boolean turn;
        private final boolean blue;
        public boolean everSeenTag = false;
        public double lastTx = Double.NaN;

        public turnToTag(boolean turn, boolean blue) {
            this.turn = turn;
            this.blue = blue;
            turningTime.reset();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!turn) {
                stopDrive();
                return false;
            }

            // No camera configured means there is nothing to align to. Say so and finish
            // rather than spinning blind for three seconds.
            if (limelight == null) {
                RobotLog.ww("intakeOuttake", "turnToTag: no limelight configured, skipping");
                turn = false;
                stopDrive();
                return false;
            }

            if (turningTime.seconds() > MAX_SEARCH_SECONDS) {
                turn = false;
                stopDrive();
                return false;
            }

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            LLResult llResult = limelight.getLatestResult();
            boolean isValid = llResult != null && llResult.isValid();

            // Pick OUR tag out of everything in frame. Keeping whichever tag came last in
            // the list means that with several tags visible the right one gets missed while
            // the camera is staring straight at it.
            int wantedId = blue ? BLUE_TARGET_TAG : RED_TARGET_TAG;
            LLResultTypes.FiducialResult target = null;
            if (isValid) {
                for (LLResultTypes.FiducialResult fid : llResult.getFiducialResults()) {
                    if (fid.getFiducialId() == wantedId) {
                        target = fid;
                        break;
                    }
                }
            }

            if (target != null) {
                everSeenTag = true;

                // This tag's own angle. llResult.getTx() is the primary target's angle, which
                // is a different tag whenever ours is not the one the pipeline sorted first.
                double tx = target.getTargetXDegrees();
                lastTx = tx;

                // Drive the error toward the aim offset and test the same error for done, so
                // the control term and the exit condition cannot disagree.
                double error = tx - AIM_OFFSET_DEGREES;
                if (Math.abs(error) < AIM_TOLERANCE_DEGREES) {
                    turn = false;
                    stopDrive();
                    return false;
                }

                double turnPower = Math.max(-MAX_TURN_POWER,
                        Math.min(MAX_TURN_POWER, kP * error));
                frontLeft.setPower(-turnPower);
                backLeft.setPower(-turnPower);
                frontRight.setPower(turnPower);
                backRight.setPower(turnPower);
                return true;
            }

            // Never saw it at all - give up rather than spinning for the full cap.
            if (!everSeenTag && turningTime.seconds() > GIVE_UP_SECONDS) {
                turn = false;
                stopDrive();
                return false;
            }

            // Lost it mid-align: keep sweeping the way it was last correcting toward.
            double sweepPower = (!Double.isNaN(lastTx) && lastTx < 0) ? -SEARCH_POWER : SEARCH_POWER;
            frontLeft.setPower(-sweepPower);
            backLeft.setPower(-sweepPower);
            frontRight.setPower(sweepPower);
            backRight.setPower(sweepPower);
            return true;
        }

        private void stopDrive() {
            frontLeft.setPower(0.0);
            backLeft.setPower(0.0);
            frontRight.setPower(0.0);
            backRight.setPower(0.0);
        }
    }

    // =====================================================================================
    //  cellDist() - distance to an AprilTag in inches.
    //
    //  The math lives in TagDistance, so LlDrivemode can use the identical code without
    //  constructing this whole class (which would re-initialise the IMU and fight over the
    //  launcher PIDF that teleop sets every loop).
    //
    //  Returns Double.NaN whenever it cannot answer - never 0.0. Always check
    //  Double.isNaN(). See TagDistance for what must be true before it can answer at all.
    // =====================================================================================

    /** Inches to the closest AprilTag in view, or NaN if none. */
    public double cellDist() {
        return TagDistance.closest(latestResult());
    }

    /** Inches to one specific tag id, or NaN if that tag is not in view. */
    public double cellDist(int tagId) {
        return TagDistance.forTag(latestResult(), tagId);
    }

    /** Inches to every tag in view, keyed by tag id. */
    public Map<Integer, Double> allCellDists() {
        return TagDistance.all(latestResult());
    }

    private LLResult latestResult() {
        return limelight == null ? null : limelight.getLatestResult();
    }

    /**
     * Move both garden servos to a position. Does nothing if the servos are not configured.
     *
     * The two positions LlDrivemode toggles between are 1.0 and 0.4 (gamepad2 X). Which one
     * is "open" is not written down anywhere, so they are named by value rather than by a
     * guess at their meaning - check on the robot and rename them once you know.
     */
    public class setGarden implements Action {
        private final double position;

        public setGarden(double position) {
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setGarden(position);
            return false;
        }
    }

    /** The two garden servo positions LlDrivemode toggles between. */
    public static final double GARDEN_POSITION_ONE = 1.0;
    public static final double GARDEN_POSITION_FOUR_TENTHS = 0.4;

    // =====================================================================================
    //  THE MECHANISM METHODS - these DO the thing the moment you call them.
    //
    //  Call take.intakeBalls() and the intake turns on. No Action to run, no loop to tick,
    //  nothing to remember. This is what an iterative OpMode (gardenSide, pathTest,
    //  shootAutoTest, blueAuto) should use.
    //
    //  Every one of them is null-safe: no launcher configured means revLauncher() quietly
    //  does nothing instead of throwing.
    // =====================================================================================

    /** Intake on, pulling balls in. */
    public void intakeBalls() {
        intake.setPower(1.0);
    }

    /** Intake off, and the launcher braked to a stop if there is one. */
    public void stopBalls() {
        intake.setPower(0.0);
        if (launcher != null) {
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launcher.setPower(0.0);
        }
    }

    /** Intake on - it is the feed that pushes the next ball into the launcher wheel. */
    public void shootBalls() {
        intake.setPower(1.0);
    }

    /** Spin the launcher up to a velocity in TICKS PER SECOND. No-op without a launcher. */
    public void revLauncher(double velocity) {
        if (launcher == null) return; // no launcher on the robot yet
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(LAUNCHER_P, 0.0, 0.0, LAUNCHER_F));
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setVelocity(velocity);
    }

    /** Move both garden servos. No-op for whichever of them is not configured. */
    public void setGarden(double position) {
        if (garden1 != null) garden1.setPosition(position);
        if (garden2 != null) garden2.setPosition(position);
    }

    // =====================================================================================
    //  THE ACTION WRAPPERS - for Road Runner sequences only.
    //
    //  An Action does NOT run when you build it. Actions.runBlocking(...) or a SequentialAction
    //  runs it. Writing "take.intakeBallsAction();" on its own line builds an object and
    //  throws it away - the intake never moves, and the compiler will not warn you.
    //
    //  That is why these all end in "Action": if you see that word and no runner, it is a
    //  bug. If you just want the mechanism to move now, call the method above instead.
    // =====================================================================================

    public Action intakeBallsAction() { return new intakeBalls(); }
    public Action stopBallsAction() { return new stopBalls(); }
    public Action shootBallsAction() { return new shootBalls(); }
    public Action revLauncherAction(double velocity) { return new revLauncher(velocity); }
    public Action setGardenAction(double position) { return new setGarden(position); }

    /**
     * Search for the alliance tag and rotate onto it. This one is genuinely multi-tick -
     * there is no instant version, because aiming takes time. Drive it with startAction()
     * and updateAction() below, or put it in a Road Runner sequence.
     */
    public Action turnToTagAction(boolean turn, boolean blue) { return new turnToTag(turn, blue); }

    // =====================================================================================
    //  RUNNING AN ACTION FROM AN ITERATIVE OpMode
    //
    //  OpMode (as opposed to LinearOpMode) cannot block, so Actions.runBlocking() is not an
    //  option. Instead: startAction(...) once, then updateAction() every loop until it
    //  returns false. That is what Road Runner's runBlocking does internally, just spread
    //  across loop() calls so the Driver Station stop button still works.
    //
    //      // once
    //      take.startAction(take.turnToTagAction(true, true));
    //
    //      // every loop()
    //      if (take.updateAction()) return;   // still aiming, do nothing else yet
    // =====================================================================================

    private Action currentAction = null;

    /** Hand over an Action to be ticked by updateAction(). Replaces any current one. */
    public void startAction(Action action) {
        currentAction = action;
    }

    /** Tick the current Action once. Returns true while it still needs more loops. */
    public boolean updateAction() {
        if (currentAction == null) return false;

        boolean wantsMoreTime = currentAction.run(new TelemetryPacket());
        if (!wantsMoreTime) currentAction = null;
        return wantsMoreTime;
    }

    public boolean isActionRunning() {
        return currentAction != null;
    }

    /** Drop the current Action without finishing it. */
    public void cancelAction() {
        currentAction = null;
    }

    /**
     * Run a one-shot Action right now, in this call. Safe for the wrappers above, which all
     * finish in a single tick. If the Action asks for more time this logs a warning and does
     * NOT loop - a blocking loop inside an OpMode would freeze the robot and ignore the stop
     * button. Use startAction()/updateAction() for anything multi-tick.
     */
    public void runOnce(Action action) {
        if (action == null) return;

        if (action.run(new TelemetryPacket())) {
            RobotLog.ww("intakeOuttake",
                    "runOnce(): %s wanted more than one tick and was cut short - "
                            + "use startAction()/updateAction() instead",
                    action.getClass().getSimpleName());
        }
    }
}
