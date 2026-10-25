package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing 3.0 configuration for this robot.
 *
 * Pedro 3.0 builds a Follower from three pieces: a Localizer (where am I?), a Drivetrain
 * (how do I move the wheels?) and an Algorithm (how do I follow the path?). The algorithm
 * here is Foresight, which looks ahead and brakes early instead of stopping at each point.
 *
 * ============================ READ THIS BEFORE TRUSTING IT ============================
 * The DRIVETRAIN NAMES/DIRECTIONS and the PINPOINT SETTINGS below are real - they carried
 * over unchanged from the old Pedro 2.0.6 file, which copied them from LlDrivemode.java.
 *
 * The FORESIGHT NUMBERS ARE NOT REAL YET. Every value in foresightConfig is a placeholder
 * marked TODO. Foresight predicts how far the robot will coast, so with wrong numbers it will
 * start braking too early or too late. Run Pedro's ForesightTuner on THIS robot (it prints a
 * finished foresightConfig block) and paste its output over the placeholders before trusting
 * any distance.
 *
 * The old 0.5 power cap is now maxPathSpeed(0.5): the robot may use at most half its top
 * speed, so the first runs are slow. Raise it only after a coach has reviewed the tuning.
 *
 * ALSO UNCONFIRMED: this assumes the goBILDA Pinpoint is wired up and configured as "odo".
 * In LlDrivemode.java every odo line is commented out, so it may not be on this robot at
 * all. If it is not, Pedro has no idea where the robot is and this config must change to a
 * different localizer before anything is run.
 * ======================================================================================
 */
public class Constants {

    // Motor names and directions are taken from LlDrivemode.java:65-77 - the names this
    // robot is actually configured with. Do not change these without changing teleop too.
    public static MecanumConfig mecanumConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.backLeftName.set("backLeft");
        c.frontRightName.set("frontRight");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    // Pod type, offsets and encoder directions all come from LlDrivemode.java:106-124.
    // Pedro 3.0 names the pods by axis: the X pod is the one that measures FORWARD travel
    // and the Y pod is the one that measures STRAFE travel. The numbers below are the same
    // ones the old file called forwardPodY / strafePodX, just under their new names.
    public static PinpointConfig pinpointConfig = new PinpointConfig(c -> {
        c.name.set("odo");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        c.offsetUnits.set(DistanceUnit.MM);
        c.xPodOffset.set(-22.1);   // was strafePodX
        c.yPodOffset.set(-136.4);  // was forwardPodY
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);  // was forwardEncoderDirection
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);  // was strafeEncoderDirection
    });

    // TODO ALL OF THESE ARE PLACEHOLDERS. Replace this whole block with the code printed by
    // the ForesightTuner OpMode after running it on the robot (with a coach present).
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        // TODO tuner: "forwardTranslational / strafeTranslational Primary and Secondary kP"
        Controller forwardPrimary = Controller.proportional(0.1);
        Controller forwardSecondary = Controller.proportional(0.1);
        Controller strafePrimary = Controller.proportional(0.1);
        Controller strafeSecondary = Controller.proportional(0.1);
        c.forwardTranslational.set(Controller.piecewise(forwardSecondary).put(2.5, forwardPrimary));
        c.strafeTranslational.set(Controller.piecewise(strafeSecondary).put(2.5, strafePrimary));

        // TODO tuner: "coast kV" and "brake kV"
        c.coast.set(Controller.proportionalFeedforward(0.02));
        c.brake.set(Controller.proportionalFeedforward(0.02));

        // TODO tuner: "heading kP" and the heading braking coefficients
        c.headingFeedback.set(Controller.proportional(1.0));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.1, 0.01));

        // TODO tuner: forward / strafe braking linear + quadratic coefficients
        c.linearBrakeCoefficients.set(Matrix.diag(0.1, 0.1));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.001, 0.001));

        // TODO tuner: max achievable velocity and natural deceleration, in inches/s.
        // Starting points come from the old file's xVelocity 55 / yVelocity 40 and the
        // zero-power accelerations -40 / -80 (Foresight wants them as positive numbers).
        c.maxAchievableForwardVelocity.set(55.0);
        c.maxAchievableStrafeVelocity.set(40.0);
        c.naturalForwardDeceleration.set(40.0);
        c.naturalStrafeDeceleration.set(80.0);

        // The old maxPower(0.5): never use more than half of top speed until tuning is reviewed.
        c.maxPathSpeed.set(0.5);

        // Do NOT actively brake at the end of every path. Default is true, which makes the
        // robot stop dead at each waypoint - fine for accuracy, but it turns a multi-leg auto
        // into a series of lurches and wastes a lot of time. With this false the legs blend
        // into each other and the whole routine runs faster.
        //
        // The trade: it coasts into each endpoint instead of planting itself, so it overshoots
        // more and may still be drifting when a path reports done. Where the robot genuinely
        // has to be still - lining up a shot, for instance - add a short pause() in the
        // routine rather than turning braking back on everywhere.
        c.brakeAtEnd.set(false);
    });

    public static Drivetrain createDrivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, mecanumConfig);
    }

    public static Localizer createLocalizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, pinpointConfig);
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                createLocalizer(hardwareMap),
                createDrivetrain(hardwareMap),
                new Foresight(foresightConfig));
    }
}
