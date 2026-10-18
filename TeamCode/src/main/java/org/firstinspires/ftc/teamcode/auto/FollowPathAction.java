package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

/**
 * Wraps a Pedro Pathing path as a Road Runner Action, so driving somewhere can be composed
 * with mechanism steps on one timeline instead of being tracked by a hand-written PathState
 * switch:
 *
 *     new SequentialAction(
 *         new FollowPathAction(follower, Path1),
 *         new ParallelAction(                       // rev WHILE driving, not after
 *             take.revLauncher(1580),
 *             new FollowPathAction(follower, Path2)
 *         ),
 *         take.turnToTag(true, true),
 *         take.shootBalls()
 *     )
 *
 * ============================= THE ONE RULE THAT MATTERS =============================
 * SOMETHING ELSE MUST CALL follower.update() EVERY LOOP. This Action deliberately does not,
 * because Pedro integrates motion per call and double-updating would corrupt its estimate.
 *
 * That means Actions.runBlocking(new FollowPathAction(...)) WILL HANG FOREVER. runBlocking
 * spins on run() but never updates the follower, so isBusy() never becomes false. Tick this
 * from an OpMode loop() that already calls follower.update() - see BIOtest.
 * =====================================================================================
 *
 * No built-in timeout, on purpose. Race it against a SleepAction instead, which is the
 * composable way to say "give up after 5 seconds":
 *
 *     new RaceAction(new FollowPathAction(follower, path), new SleepAction(5.0))
 */
public class FollowPathAction implements Action {

    private final Follower follower;
    private final PathChain path;
    private final boolean holdEnd;

    private boolean started = false;

    /** holdEnd true keeps Pedro actively holding the final pose once it arrives. */
    public FollowPathAction(Follower follower, PathChain path, boolean holdEnd) {
        this.follower = follower;
        this.path = path;
        this.holdEnd = holdEnd;
    }

    public FollowPathAction(Follower follower, PathChain path) {
        this(follower, path, true);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!started) {
            follower.followPath(path, holdEnd);
            started = true;

            // Claim "still working" for this tick no matter what. isBusy() can still read
            // false in the moment between handing the path over and the follower's next
            // update(), and returning false here would report the drive finished instantly.
            return true;
        }

        return follower.isBusy();
    }

    /** True once the path has been handed over and the follower reports it is done. */
    public boolean isFinished() {
        return started && !follower.isBusy();
    }
}
