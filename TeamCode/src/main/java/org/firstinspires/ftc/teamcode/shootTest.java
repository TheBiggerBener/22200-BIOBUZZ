package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.auto.TagDistance;

import java.util.Map;

// For limelight


@TeleOp
public class shootTest extends LinearOpMode {
    // General variable set up
    double launcher_velocity = 3000.0;
    GoBildaPinpointDriver odo;
    //public Pose2D autoPos;
    Pose2D pos = null; // filled in from odo.getPosition() when the Pinpoint is present
    double oldTime = 0;
    boolean alliance = true;
    public double highVelocity = 1500.0;
    public double lowVelocity = 900.0;
    double curTargetVelocity = highVelocity;
    double F = 0.0;
    double P = 0.0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;
    private DcMotorEx launcher;
    Servo rgbLight; // For color
    private double distance;


    // HERE //
    // Enabled for cellDist(). tryGet below means a missing camera leaves this null rather
    // than killing the OpMode, so teleop still drives if the Limelight is unplugged.

    private IMU imu;

    /**
     * Distance in INCHES to the closest AprilTag in view, or Double.NaN if it cannot answer.
     * Same math intakeOuttake.cellDist() uses - see TagDistance for what has to be
     * configured before it returns a real number. NEVER returns 0.0 for "no idea", so
     * always check Double.isNaN() before using it.
     */

    @Override
    public void runOpMode() {

        // Motor/servo/limelight config

        // NO LAUNCHER ON THE ROBOT RIGHT NOW (Oct 2026). tryGet leaves this null instead
        // of throwing at init, and every use below is guarded. Bolt the launcher back on,
        // add it to the robot config, and it works again with nothing to uncomment.
        launcher = hardwareMap.tryGet(DcMotorEx.class, "launcherMotor");

        if (launcher != null) {
            launcher.setDirection(DcMotorEx.Direction.FORWARD);
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        // Limelight initalization HERE!


        // IMU HERE!
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));




        //telemetry.addData("Autoposition: ", autoPos);
        //telemetry.addData("Status", "Initialized");
        //telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        //telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        //telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        //telemetry.addData("Heading Scalar", odo.getYawScalar());
        //telemetry.update();


        final int CYCLE_MS = 5;


        waitForStart();
        if (isStopRequested()) return;


        // HERE //


        while (opModeIsActive()) {
            // HERE //
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            //limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            //odo.update();

            if (gamepad1.x) {
                alliance = false;
            } else if (gamepad1.b) {
                alliance = true;
            }

            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            oldTime = newTime;

           /*
           gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
            */
            //pos = odo.getPosition();
            //String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            //telemetry.addData("Position", data);

           /*
           gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
            */
            //String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            //telemetry.addData("Velocity", velocity);


            if (gamepad2.dpad_left) { // medium
                launcher_velocity = 2100.0;
            } else if (gamepad2.dpad_right) { // low-mid (new)
                launcher_velocity = 1800.0;
            } else if (gamepad2.dpad_down) { // low
                launcher_velocity = 1500.0;
            }

            if (launcher != null) {
                if (gamepad2.right_trigger > 0) {
                    launcher.setVelocity(launcher_velocity);
                } else if (gamepad2.b) {
                    launcher.setPower(-1.0);
                } else {
                    launcher.setPower(0.0);
                }
            }
            



            if (gamepad1.y) {
                if (curTargetVelocity == highVelocity) {
                    curTargetVelocity = lowVelocity;
                } else {
                    curTargetVelocity = highVelocity;
                }
            }

            // PID STUFF

            if (gamepad1.b) {

                stepIndex = (stepIndex + 1) % stepSizes.length;
            }

            if (gamepad1.dpad_right) {
                F += stepSizes[stepIndex];
            }
            if (gamepad1.dpad_left) {
                F -= stepSizes[stepIndex];
            }

            if (gamepad1.dpad_up) {
                P += stepSizes[stepIndex];
            }

            if (gamepad1.dpad_down) {
                P -= stepSizes[stepIndex];
            }

            /*if (launcher != null) {
                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(175.0, 0, 0, 12.663387);
                launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            }
            */
            // NaN rather than 0.0 while there is no launcher, so the readout says so plainly
            // instead of looking like a launcher that is spun down.
            double curVelocity = launcher != null ? launcher.getVelocity() : Double.NaN;
            double error = curTargetVelocity - curVelocity;

            telemetry.addData("Target Velocity PIDF", curTargetVelocity);
            telemetry.addData("Current Velocity PIDF", curVelocity);
            telemetry.addData("Error PIDF", error);
            telemetry.addLine("-------------");
            telemetry.addData("Tuning P PIDF", P);
            telemetry.addData("Tuning F PIDF", F);
            telemetry.addData("Step Size PIDF", stepSizes[stepIndex]);
            telemetry.addLine("--------------");

            // AprilTag distance. NaN until the Limelight has 3D enabled with the right tag
            // size, or CAMERA_HEIGHT_IN / CAMERA_PITCH_DEG are measured in TagDistance.



            /*if (isValid) {
                Pose3D botpose = llResult.getBotpose_MT2();
                //distance = getDistanceFromTag(llResult.getTy());
                telemetry.addData("Distance", distance);
                telemetry.addData("LL Timestamp", llResult.getTimestamp());
                telemetry.addLine("AprilTag Detected");
            } else {
                telemetry.addLine("No AprilTag Detected");
            }
            */

            /* Press a to turn on auto-aim (limelight)
            if (gamepad1.right_trigger > 0.0 && isValid) {
                double tx = llResult.getTx();
                // 'amt' of turn
                double kP = 0.02;
                double turnPower = kP * (tx+3);
                turnPower = Math.max(-0.3, Math.min(0.3, turnPower));
                if (Math.abs(tx) < 1.0) turnPower = 0;


                // Rotate robot via above
                fL_Motor += -turnPower;
                bL_Motor += -turnPower;
                fR_Motor += turnPower;
                bR_Motor += turnPower;




                // Telemetry for data
                telemetry.addData("Left/Right offset: ", tx);
                telemetry.addData("Turn Power: ", turnPower);
            }
            */




            /* launcher brake toggler
            if (gamepad2.left_bumper) {
                launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            } else {
                launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            }
            */

            //Incremental velocity power for the launcher
            if (gamepad2.left_stick_y > 0.0) {
                launcher_velocity += 20;
            } else if (gamepad2.left_stick_y < 0.0) {
                launcher_velocity -= 20;
            }
            if (gamepad2.left_trigger > 0.0) {
                launcher_velocity += gamepad2.left_trigger;
            }

            // General info output for the robot in the console
            //telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.addData("Velocity", launcher_velocity);
            telemetry.addData("leftStick", gamepad2.left_stick_y);
            telemetry.addData("left_trigger", gamepad2.left_trigger);
            telemetry.addData("Acc Velocity", launcher.getVelocity());
            telemetry.addData("launcher is real", launcher != null);
            telemetry.update();
            sleep(CYCLE_MS);
            idle();
        }
    }
}