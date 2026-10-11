package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
 * ================= WHY THIS DOES NOT CRASH ON A HALF-BUILT ROBOT =================
 * Last season's robot had more hardware than this one is configured for. Checked against
 * LlDrivemode.java, this robot's config knows about:
 *
 *     frontLeft  backLeft  frontRight  backRight  imu  intake  launcherMotor  odo  limelight
 *
 * Last season's version also wanted leftFeeder, rightFeeder, topWheel, boxMotor and
 * laserAnalogInput. Those are NOT in this robot's config. Asking hardwareMap.get() for a
 * device that is not configured throws, which would make every OpMode using this class die
 * at init. So the devices this robot is known to have are fetched normally, and the rest go
 * through hardwareMap.tryGet(), which returns null instead of throwing.
 *
 * Every action below checks for null before touching optional hardware, so a missing feeder
 * means "that part does nothing", not "the OpMode crashed". Call missingDevices() and put it
 * on telemetry so it is obvious which pieces are not responding rather than silently absent.
 *
 * Also note: last year's intakeMotor is called "intake" on this robot. That rename is done.
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

    // TODO these are last season's launcher gains, for last season's launcher. Re-tune.
    private static final double LAUNCHER_P = 430.0;
    private static final double LAUNCHER_F = 14.8;

    // Confirmed present on this robot.
    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public final DcMotor intake;
    public final DcMotorEx launcher;
    public final IMU imu;

    // Present in this robot's config but commented out in LlDrivemode, so treated as
    // optional until someone confirms the camera is actually plugged in.
    public final Limelight3A limelight;

    // Not in this robot's config at all. null unless somebody adds them.
    public final AnalogInput laserAnalog;

    private final List<String> missing = new ArrayList<>();
    private final ElapsedTime turningTime = new ElapsedTime();

    public intakeOuttake(HardwareMap hardwareMap) {
        // --- drivetrain: names and directions match LlDrivemode.java ---
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        rightBack = hardwareMap.get(DcMotorEx.class, "backRight");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- mechanisms this robot has ---
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);

        launcher = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcher.setDirection(DcMotorEx.Direction.FORWARD);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(LAUNCHER_P, 0.0, 0.0, LAUNCHER_F));

        // --- IMU: orientation copied from LlDrivemode.java:92-96 (UP / FORWARD).
        // Last season's file said RIGHT / UP; that was a different robot. Do not "restore" it.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )));

        // --- optional hardware: null rather than an exception if not configured ---
        limelight = track(hardwareMap.tryGet(Limelight3A.class, "limelight"), "limelight");
        laserAnalog = track(hardwareMap.tryGet(AnalogInput.class, "laserAnalogInput"), "laserAnalogInput");

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
            intake.setPower(1.0);
            return false;
        }
    }

    public class stopBalls implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(0.0);
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launcher.setPower(0.0);
            return false;
        }
    }

    public class shootBalls implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(1.0);
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
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(LAUNCHER_P, 0.0, 0.0, LAUNCHER_F));
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            launcher.setVelocity(velocity);
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
                leftFront.setPower(-turnPower);
                leftBack.setPower(-turnPower);
                rightFront.setPower(turnPower);
                rightBack.setPower(turnPower);
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
            leftFront.setPower(-sweepPower);
            leftBack.setPower(-sweepPower);
            rightFront.setPower(sweepPower);
            rightBack.setPower(sweepPower);
            return true;
        }

        private void stopDrive() {
            leftFront.setPower(0.0);
            leftBack.setPower(0.0);
            rightFront.setPower(0.0);
            rightBack.setPower(0.0);
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

    public Action intakeBalls() { return new intakeBalls(); }
    public Action stopBalls() { return new stopBalls(); }
    public Action shootBalls() { return new shootBalls(); }
    public Action revLauncher(double velocity) { return new revLauncher(velocity); }
    public Action turnToTag(boolean turn, boolean blue) { return new turnToTag(turn, blue); }
}
