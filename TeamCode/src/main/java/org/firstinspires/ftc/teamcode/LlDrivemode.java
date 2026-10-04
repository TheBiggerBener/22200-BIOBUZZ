package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;


// For limelight
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;


import java.util.Locale;


@TeleOp
public class LlDrivemode extends LinearOpMode {
    // General variable set up
    double launcher_velocity = 3000.0;
    GoBildaPinpointDriver odo;
    public Pose2D autoPos;
    Pose2D pos = null;
    double oldTime = 0;
    boolean alliance = true;
    public double highVelocity = 1500.0;
    public double lowVelocity = 900.0;
    double curTargetVelocity = highVelocity;
    double F = 0.0;
    double P = 0.0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;
    private DcMotor intake;
    private DcMotorEx launcher;
    Servo rgbLight; // For color
    private double distance;


    // HERE //
    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void runOpMode() {

        // Motor/servo/limelight config
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intake = hardwareMap.dcMotor.get("intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcherMotor");


        // Motor directions
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotorEx.Direction.FORWARD);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Limelight initalization HERE!
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);


        // IMU HERE!
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        // Odometry setup
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");


        // Type of odometry arm that the robot is using.
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);


        //Direction
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);


       /*
       Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
       The IMU will automatically calibrate when first powered on, but recalibrating before running
       the robot is a good idea to ensure that the calibration is "good".
       resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
       This is recommended before you run your autonomous, as a bad initial calibration can cause
       an incorrect starting value for x, y, and heading.
        */
        odo.recalibrateIMU();
        odo.resetPosAndIMU();
        // Measure in milimeters at the meeting, this is not currently accurate.
        odo.setOffsets(-22.1, -136.4, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.update();
        pos = odo.getPosition();
        odo.setPosition(new Pose2D(DistanceUnit.MM,
                609.6,
                -1828.8,
                AngleUnit.RADIANS,
                0
        ));
        telemetry.addData("Autoposition: ", autoPos);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();


        final int CYCLE_MS = 5;


        waitForStart();
        if (isStopRequested()) return;


        // HERE //
        limelight.start();


        while (opModeIsActive()) {
            // HERE //
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            odo.update();

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
            pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

           /*
           gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
            */
            String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);


            if (gamepad2.dpad_left) { // medium
                launcher_velocity = 2100.0;
            } else if (gamepad2.dpad_right) { // low-mid (new)
                launcher_velocity = 1800.0;
            } else if (gamepad2.dpad_down) { // low
                launcher_velocity = 1500.0;
            }

            if (gamepad2.right_trigger > 0) {
                launcher.setVelocity(launcher_velocity);
            } else if (gamepad2.left_trigger > 0) {
                launcher.setPower(-1.0);
            } else {
                launcher.setPower(0.0);
            }


            double speedMultiplier = 1.0;
            if (gamepad1.left_trigger > 0.5) {
                speedMultiplier *= 0.4; // Original - prev was 0.5
                frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            } else {
                frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            }


            // Drive calculations
            double y = -gamepad1.left_stick_y * speedMultiplier;
            double x = gamepad1.left_stick_x * 1.1 * speedMultiplier;
            double rx = -gamepad1.right_stick_x * speedMultiplier;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);


            // To decrease 'noise' via small movements
            if (Math.abs(x) < 0.05) x = 0;
            if (Math.abs(y) < 0.05) y = 0;
            if (Math.abs(rx) < 0.05) rx = 0;


            double fL_Motor = (y + x + rx) / denominator;
            double bL_Motor = (y - x + rx) / denominator;
            double fR_Motor = (y - x - rx) / denominator;
            double bR_Motor = (y + x - rx) / denominator;




            // Limelight alignment HERE //
            LLResult llResult = limelight.getLatestResult();
            boolean isValid = llResult != null && llResult.isValid();

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

            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(175.0, 0, 0, 12.663387);
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


            double curVelocity = launcher.getVelocity();
            double error = curTargetVelocity - curVelocity;

            telemetry.addData("Target Velocity PIDF", curTargetVelocity);
            telemetry.addData("Current Velocity PIDF", curVelocity);
            telemetry.addData("Error PIDF", error);
            telemetry.addLine("-------------");
            telemetry.addData("Tuning P PIDF", P);
            telemetry.addData("Tuning F PIDF", F);
            telemetry.addData("Step Size PIDF", stepSizes[stepIndex]);
            telemetry.addLine("--------------");


            if (isValid) {
                Pose3D botpose = llResult.getBotpose_MT2();
                //distance = getDistanceFromTag(llResult.getTy());
                telemetry.addData("Distance", distance);
                telemetry.addData("LL Timestamp", llResult.getTimestamp());
                telemetry.addLine("AprilTag Detected");
            } else {
                telemetry.addLine("No AprilTag Detected");
            }


            // Press a to turn on auto-aim (limelight)
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
            // Regular Motor Controls
            frontLeft.setPower(fL_Motor);
            backLeft.setPower(bL_Motor);
            frontRight.setPower(fR_Motor);
            backRight.setPower(bR_Motor);

            // Intake motor's control
            if (gamepad2.right_trigger > 0.0) { intake.setPower(gamepad2.right_trigger); } else { intake.setPower(0.0); }

            // launcher brake toggler
            if (gamepad2.left_bumper) {
                launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            } else {
                launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            }


            //Incremental velocity power for the launcher
            if (gamepad2.left_stick_y > 0.0) {
                launcher_velocity += 100;
            } else if (gamepad2.left_stick_y < 0.0) {
                launcher_velocity -= 100;
            }

            // General info output for the robot in the console
            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.addData("Alliance selected: ", alliance);
            telemetry.addData("Position on the field measured by inches: ", odo.getPosition());
            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.addData("Launcher target velocity : ", launcher_velocity);
            telemetry.addData("Acc target velocity: ", launcher.getVelocity());
            telemetry.update();
            sleep(CYCLE_MS);
            idle();
        }
    }
}