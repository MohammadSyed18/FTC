package dev.ftc.control;

/** Field pose: x forward, y left, heading counterclockwise in radians. */
public record Pose2(double x,double y,double heading) {
    public Pose2 {
        if (!Double.isFinite(x)||!Double.isFinite(y)||!Double.isFinite(heading)) throw new IllegalArgumentException("finite pose required");
        heading = wrap(heading);
    }
    public double distanceTo(Pose2 other) { return Math.hypot(other.x-x,other.y-y); }
    public static double wrap(double angle) { return Math.atan2(Math.sin(angle),Math.cos(angle)); }
}
