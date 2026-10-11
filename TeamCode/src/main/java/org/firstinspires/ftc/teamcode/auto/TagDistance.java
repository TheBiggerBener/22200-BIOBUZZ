package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * How far the robot is from an AprilTag, in INCHES. Pure math on an LLResult - it owns no
 * hardware, so both intakeOuttake and LlDrivemode can use it without either one dragging in
 * the other's motor, IMU or launcher setup.
 *
 * EVERY METHOD RETURNS Double.NaN WHEN IT CANNOT ANSWER, never 0.0. A silent zero reads as
 * "the tag is touching the robot" and would poison a distance-based RPM table. Always check
 * Double.isNaN() before using a result.
 *
 * Two ways to measure, and it prefers the accurate one automatically:
 *
 *  1. SOLVEPNP - the camera solves the tag's full 3D pose from its four corners. Needs the
 *     pipeline setting fiducial_skip3d = 0 and fiducial_size matching the real tag. This
 *     camera currently has skip3d = 1 and fiducial_size = 101.6 mm while the BIOBUZZ field
 *     tags 30-37 are 82.5 mm, so this path is dark. Fix both on the Limelight dashboard (or
 *     load the .fmap, which stores a size per tag - the only clean way to handle a field
 *     carrying both 82.5 and 165.1 mm tags) and it starts working with no code change.
 *
 *  2. TRIG fallback -   distance = (tagHeight - cameraHeight) / tan(cameraPitch + ty)
 *     Needs CAMERA_HEIGHT_IN and CAMERA_PITCH_DEG measured on the robot.
 *
 * Distances are the horizontal GROUND run, not slant range, because that is what a launcher
 * lookup table wants. They are measured from the CAMERA unless the camera's pose in robot
 * space is configured on the Limelight dashboard - it currently is not (status reports
 * cameraQuat 0,0,0,0). For shooting you want launcher-to-target, so measure that offset and
 * add it rather than assuming the camera and launcher sit at the same point.
 */
public class TagDistance {

    // TODO confirm which pipeline index on THIS camera is the AprilTag pipeline. Last
    // season's robot used 8; this is a guess until somebody checks the dashboard.
    public static final int APRILTAG_PIPELINE = 8;

    // TODO MEASURE BOTH on the robot. While they are both zero the trig fallback refuses to
    // answer rather than handing back a fabricated number.
    public static double CAMERA_HEIGHT_IN = 0.0;  // lens centre above the floor, inches
    public static double CAMERA_PITCH_DEG = 0.0;  // upward tilt in degrees, 0 = dead level

    // Tag centre height above the floor. BIOBUZZ.fmap puts tags 30-37 at z = 1.075 m.
    public static final double TAG_HEIGHT_IN = 1.075 * 39.3701; // 42.32 in

    private TagDistance() {} // static helper, never instantiated

    /** Inches to every tag currently in view, keyed by tag id. Empty if none. */
    public static Map<Integer, Double> all(LLResult result) {
        Map<Integer, Double> out = new LinkedHashMap<>();
        if (result == null || !result.isValid()) return out;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return out;

        for (LLResultTypes.FiducialResult fid : fiducials) {
            out.put(fid.getFiducialId(), to(fid));
        }
        return out;
    }

    /** Inches to the closest tag in view, or NaN if none. */
    public static double closest(LLResult result) {
        double best = Double.NaN;
        for (double d : all(result).values()) {
            if (!Double.isNaN(d) && (Double.isNaN(best) || d < best)) best = d;
        }
        return best;
    }

    /** Inches to one specific tag id, or NaN if that tag is not in view. */
    public static double forTag(LLResult result, int tagId) {
        Double d = all(result).get(tagId);
        return d == null ? Double.NaN : d;
    }

    /** One tag: solvePnP if the camera is giving 3D, otherwise trig. NaN if neither works. */
    public static double to(LLResultTypes.FiducialResult fid) {
        double fromPose = fromPose(fid);
        if (!Double.isNaN(fromPose)) return fromPose;
        return fromAngle(fid);
    }

    private static double fromPose(LLResultTypes.FiducialResult fid) {
        Pose3D pose = fid.getTargetPoseRobotSpace();
        if (pose == null || pose.getPosition() == null) return Double.NaN;

        Position p = pose.getPosition().toUnit(DistanceUnit.INCH);

        // With fiducial_skip3d = 1 the camera reports no 3D, which arrives as all zeros.
        // Treat that as "no answer" rather than "zero inches away".
        if (p.x == 0.0 && p.y == 0.0 && p.z == 0.0) return Double.NaN;

        // Horizontal run to the tag, ignoring how high it is. For straight-line slant
        // distance instead: Math.sqrt(p.x*p.x + p.y*p.y + p.z*p.z).
        return Math.hypot(p.x, p.y);
    }

    private static double fromAngle(LLResultTypes.FiducialResult fid) {
        // Refuse to guess until the camera mounting has actually been measured.
        if (CAMERA_HEIGHT_IN == 0.0 && CAMERA_PITCH_DEG == 0.0) return Double.NaN;

        // That tag's OWN vertical angle. result.getTy() describes only the primary target,
        // which is a different tag whenever yours is not the one the pipeline sorted first.
        double angleDeg = CAMERA_PITCH_DEG + fid.getTargetYDegrees();
        double heightDiff = TAG_HEIGHT_IN - CAMERA_HEIGHT_IN;

        // Near-level sightline: tan goes to zero and the distance blows up to nonsense.
        if (angleDeg <= 0.5) return Double.NaN;

        return heightDiff / Math.tan(Math.toRadians(angleDeg));
    }
}
