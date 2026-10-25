package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.auto.Constants;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * Pedro Pathing's tuning menu. Every method marked @Tuner shows up as a tuner in Pedro's
 * tuning web page - nothing here is an OpMode you pick from the Driver Station list.
 *
 * These use the SAME Constants (auto/Constants.java) that gardenSide uses, so what you tune
 * here is exactly what the autonomous runs. Suggested order on the real robot, with a coach
 * present and plenty of clear floor:
 *   1. Pinpoint tuner  - checks the odometry pod directions and offsets.
 *   2. Mecanum tuner   - checks motor names and directions.
 *   3. Foresight tuner - measures the numbers that replace the placeholders in Constants.
 *   4. Tests           - lines, curves and holds, to see it all working together.
 *
 * The other tuners in the procedures folder (OTOS, OctoQuad, two/three-wheel) are for
 * localizers this robot does not use. They are here so the folder matches Pedro's own.
 */
public class Tuning {
    @Tuner(name = "Pinpoint")
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner(name = "Mecanum")
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner(name = "Foresight")
    public static Procedure foresightTuner() {
        return new ForesightTuner(Constants::createLocalizer, Constants::createDrivetrain);
    }

    @Tuner(name = "Follower tests")
    public static Procedure tests() {
        return new Tests(
                Constants::createDrivetrain,
                Constants::createLocalizer,
                () -> new Foresight(Constants.foresightConfig));
    }
}
