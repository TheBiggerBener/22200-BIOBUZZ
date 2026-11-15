/*
THIS FILE IS FOR PARKING ON THE RED BOX
 */

package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.line;
import com.pedropathing.api.PoseFactory;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;
@Autonomous
public class redBioBuzzPark extends OpMode {
    private ElapsedTime pathTimer, opModeTimer; // 2. Change timer type to ElapsedTime
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private intakeOuttake take;
    private Path Path1;

    public enum PathState {
        PARK,
        COMPLETED,
    }

    private PathState pathState;

    private final Pose start = poseFactory.of(9.634, 76.5048, 90);
    private final Pose point1 = poseFactory.of(9.8428, 94.23, 90);

    public void buildPaths() {
        Path1 = line(start, point1).linear(start, point1);
    }

    public void statePathUpdate() {
        switch(pathState) {
            case PARK:
                if (!follower.isBusy()) {
                    follower.follow(Path1);
                    setPathState(PathState.COMPLETED);
                }
                break;
            case COMPLETED:
                break;
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState) { // nice helper function
        pathState = newState;
        pathTimer.reset();
    }

    public void init() {
        pathState = PathState.PARK;
        pathTimer = new ElapsedTime();
        opModeTimer = new ElapsedTime();
        follower = Constants.createFollower(hardwareMap);

        // take = new intakeOuttake(hardwareMap);

        buildPaths();
        follower.setPose(start);
    }

    public void runOpMode() {
        opModeTimer.reset();
        setPathState(pathState);
    }

    public void loop() {
        follower.update();
        statePathUpdate();

        // telemetry
        telemetry.addData("Path State: ", pathState.toString());
        telemetry.addData("path time in seconds", pathTimer.seconds());

    }
}
