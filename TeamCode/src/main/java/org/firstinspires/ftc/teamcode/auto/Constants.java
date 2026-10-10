package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing configuration for this robot.
 *
 * ============================ READ THIS BEFORE TRUSTING IT ============================
 * The DRIVETRAIN NAMES and the PINPOINT SETTINGS below are real - they are copied from
 * LlDrivemode.java, which is this robot's own teleop.
 *
 * The TUNING NUMBERS are NOT real yet. mass, the zero-power accelerations, xVelocity,
 * yVelocity and the PIDF coefficients are placeholders marked TODO. Pedro cannot drive
 * an accurate distance until they are measured on THIS robot with Pedro's own tuning
 * OpModes. Until then expect the 12 inch move to overshoot or undershoot.
 *
 * maxPower is deliberately set to 0.5, not 1.0, so the first runs are slow.
 *
 * ALSO UNCONFIRMED: this assumes the goBILDA Pinpoint is wired up and configured as "odo".
 * In LlDrivemode.java every odo line is commented out, so it may not be on this robot at
 * all. If it is not, Pedro has no idea where the robot is and this config must change to a
 * different localizer before anything is run.
 * ======================================================================================
 */
public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            // TODO measure the robot's actual mass in kg and put it here.
            .mass(8.0)
            // TODO tune with Pedro's ForwardZeroPowerAccelerationTuner.
            .forwardZeroPowerAcceleration(-40.0)
            // TODO tune with Pedro's LateralZeroPowerAccelerationTuner.
            .lateralZeroPowerAcceleration(-80.0)
            // TODO tune. Starting points only.
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.025, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(-1.4, 0, -0.05, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0.0, 0.00005, 0.6, 0.01))
            .centripetalScaling(0.0005);

    // Motor names and directions are taken from LlDrivemode.java:65-77 - the names this
    // robot is actually configured with. Do not change these without changing teleop too.
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.5) // kept low on purpose for first autonomous tests
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorEx.Direction.REVERSE)
            // TODO tune both with Pedro's ForwardVelocityTuner / StrafeVelocityTuner.
            .xVelocity(55.0)
            .yVelocity(40.0);

    // Pod type, offsets and encoder directions all come from LlDrivemode.java:106-124.
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .strafePodX(-22.1)
            .forwardPodY(-136.4)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99, 100, 1.35, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
