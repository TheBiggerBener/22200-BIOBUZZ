package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.utils.Timer;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

@Autonomous(name = "blueAuto", group = "auto")
public class blueAuto extends OpMode {
    private ElapsedTime pathTimer, opModeTimer;
    private Follower follower;
    private intakeOuttake take;


    public enum PathState {
        SHOOT,
        GARDENBALLS,
        TRAVELDOWN,
        MOVESHOOT,
        SHOOT1,
        GETFLOWER,
        FLOWERSHOOT,
        SHOOT2,
        PARK,
        COMPLETED,
    }// state machine to keep track of diff paths

    private PathState pathState;
    private final Pose startPose = new Pose(82.2, 136, Math.toRadians(90));

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(82.2, 136, -90);
    private final Pose point1 = poseFactory.of(132, 136, -90);
    private final Pose point2Start = poseFactory.of(132, 136, -90);
    private final Pose point2 = poseFactory.of(105.8529, 22.8494, -90);
    private final Pose point3 = poseFactory.of(84.0974, 22.0254, -177.8309);
    private final Pose point4 = poseFactory.of(90.2629, 7.2601, -155);
    private final Pose point5 = poseFactory.of(84.1197, 21.6301, 113.1469);
    private final Pose point6 = poseFactory.of(128.9235, 23.246, -177.9344);
    private Path Path1, Path2, Path3, Path4, Path5, Path6;


    public void buildPaths() {
        Path1 = Paths.line(start, point1).linear(start, point1);
        Path2 = Paths.line(point2Start, point2).linear(point2Start, point2);
        Path3 = Paths.line(point2, point3).tangent();
        Path4 = Paths.line(point3, point4).constant(point4);
        Path5 = Paths.line(point4, point5).tangent();
        Path6 = Paths.line(point5, point6).reverseTangent();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case SHOOT:
                if (!follower.isBusy()) {
                    //take.launcher.setVelocity(1580.0);
                    // take.turnToTag(true, true);
                    if (pathTimer.seconds() > 2.0) {
                        setPathState(PathState.GARDENBALLS);
                    }
                }
                break;
            case GARDENBALLS:
                // Wait for path to complete, then shoot
                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    take.intakeBalls();
                    follower.follow(Path1);
                    setPathState(PathState.TRAVELDOWN);
                }
                break;
            case TRAVELDOWN:
                // Start path to grab first 3 samples
                if (!follower.isBusy() && pathTimer.seconds() > 0.75) {
                    follower.follow(Path2);
                    take.stopBalls();
                    setPathState(PathState.MOVESHOOT);
                }
                break;
            case MOVESHOOT:
                // Navigate back to shoot position
                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    follower.follow(Path3);
                    setPathState(PathState.SHOOT1);
                }
                break;
            case SHOOT1:
                if (!follower.isBusy()) {
                    //take.launcher.setVelocity(1580.0);
                    // take.turnToTag(true, true);
                    if (pathTimer.seconds() > 2.0) {

                        if (pathTimer.seconds() > 4.0) {
                            take.launcher.setVelocity(0.0);

                            setPathState(PathState.GETFLOWER);
                        }
                    }
                }
                break;
            case GETFLOWER:
                if (!follower.isBusy() && pathTimer.seconds() > 0.75) {
                    take.intakeBalls();

                    if (pathTimer.seconds() > 0.25) {
                        follower.follow(Path4);
                        setPathState(PathState.FLOWERSHOOT);
                    }
                }
                break;
            case FLOWERSHOOT:
                // Grab second set of 3 artifacts
                if (!follower.isBusy() && pathTimer.seconds() > 0.3) {
                    follower.follow(Path5);
                    setPathState(PathState.SHOOT2);
                }
                break;

            case SHOOT2:
                if (!follower.isBusy()) {
                    //take.launcher.setVelocity(1580.0);
                    // take.turnToTag(true, true);
                    if (pathTimer.seconds() > 2.0) {
                        if (pathTimer.seconds() > 4.0) {
                            take.launcher.setVelocity(0.0);

                            setPathState(PathState.PARK);
                        }
                    }
                }
                break;
            case PARK:
                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    follower.follow(Path6);
                    setPathState(PathState.COMPLETED);
                }
                break;
            case COMPLETED:
                // stop all machinery - completed in shoot_3
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
        pathState = PathState.SHOOT;
        pathTimer = new ElapsedTime();
        opModeTimer = new ElapsedTime();
        follower = Constants.createFollower(hardwareMap); // TODO IN Constants.java
        // TODO add any init (limelight, flywheel, mechanisms)

        take = new intakeOuttake(hardwareMap);

        buildPaths();
        follower.setPose(start);
    }

    public void start() {
        opModeTimer.reset();
        setPathState(pathState);

    }

    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());
        telemetry.addData("path time in seconds", pathTimer.seconds());
    }
}
