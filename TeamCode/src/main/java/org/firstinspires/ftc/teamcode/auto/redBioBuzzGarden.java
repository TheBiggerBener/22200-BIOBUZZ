package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.line;

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
public class redBioBuzzGarden extends OpMode {
    private ElapsedTime pathTimer, opModeTimer; // 2. Change timer type to ElapsedTime
    private Follower follower;
    private intakeOuttake take;
    private Path Path1, Path2, Path3, Path4;


    public enum PathState {
        SHOOT_FIRST_BATCH,
        PICK_GARDEN,
        TRAVEL_ACROSS,
        SHOOT_SECOND_BATCH,
        PICK_FLOWER,
        SHOOT_THIRD_BATCH,
        PARK,
        COMPLETED,
    }

    private PathState pathState;

    private final Pose startPose = new Pose(56.0, 8.0, Math.toRadians(180));
    private final Pose toGarden = new Pose(9, 8, Math.toRadians(180));
    private final Pose travelPose = new Pose(41.51, 123.36, Math.toRadians(90));
    private final Pose pickFlower = new Pose(46.65, 130.25, Math.toRadians(90));
    private final Pose park = new Pose(10.79, 113.7, Math.toRadians(90));
    public void buildPaths() {
        Path1 = line(startPose, toGarden).linear(startPose, toGarden);
        Path2 = line(toGarden, travelPose).linear(toGarden, travelPose);
        Path3 = line(travelPose, pickFlower).linear(travelPose, pickFlower);
        Path4 = line(pickFlower, park).linear(pickFlower, park);
    }

    public void statePathUpdate() {
        switch(pathState) {
            case SHOOT_FIRST_BATCH:
                if (!follower.isBusy()) {
                    // temporary - might have 2 motors
                    take.launcher.setVelocity(1580.0);


                    if (pathTimer.seconds() > 0.0) {
                        take.intake.setPower(1.0);
                    }
                    if (pathTimer.seconds() > 4.0) {
                        take.launcher.setVelocity(0.0);
                        setPathState(PathState.PICK_GARDEN);
                    }
                }
                break;

            case PICK_GARDEN:
                if (!follower.isBusy() && pathTimer.seconds() > 0.75) {
                    take.intake.setPower(1.0);
                    follower.follow(Path1);
                    setPathState(PathState.TRAVEL_ACROSS);
                }
                break;

            case TRAVEL_ACROSS:
                if (!follower.isBusy() && pathTimer.seconds() > 4.5) {
                    follower.follow(Path2);
                    setPathState(PathState.SHOOT_SECOND_BATCH);
                }
                break;

            case SHOOT_SECOND_BATCH:
                if (!follower.isBusy()) {
                    // temporary - might have 2 motors
                    take.launcher.setVelocity(1580.0);


                    if (pathTimer.seconds() > 0.5) {
                        take.intake.setPower(1.0);
                    }
                    if (pathTimer.seconds() > 4.0) {
                        take.launcher.setVelocity(0.0);
                        setPathState(PathState.PICK_FLOWER);
                    }
                }
                break;

            case PICK_FLOWER:
                if (!follower.isBusy() && pathTimer.seconds() > 0.75) {
                    take.intake.setPower(1.0);
                    follower.follow(Path3);
                    setPathState(PathState.SHOOT_THIRD_BATCH);
                }
                break;

            case SHOOT_THIRD_BATCH:
                if (follower.isBusy()) {
                    // temporary - might have 2 motors
                    take.launcher.setVelocity(1580.0);

                    if (pathTimer.seconds() > 0.5) {
                        take.intake.setPower(1.0);
                    }
                    if (pathTimer.seconds() > 4.0) {
                        take.launcher.setVelocity(0.0);
                        take.intake.setPower(0.0);
                        setPathState(PathState.PARK);
                    }
                }

            case PARK:
                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    follower.follow(Path4);
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
        pathState = PathState.SHOOT_FIRST_BATCH;
        pathTimer = new ElapsedTime();
        opModeTimer = new ElapsedTime();
        follower = Constants.createFollower(hardwareMap);

        // take = new intakeOuttake(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
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
