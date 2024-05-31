package dev.ftc.control;

import java.util.List;

/** Deterministic waypoint state machine, independent of an FTC hardware map. */
public final class WaypointFollower {
    public record Command(double forward,double strafe,double turn,int waypoint,boolean complete,boolean failed) {}
    private final List<Pose2> waypoints;
    private final PIDController x,y,heading;
    private final double tolerance,headingTolerance,timeout;
    private int index;
    private double elapsed;
    private boolean failed;
    public WaypointFollower(List<Pose2> waypoints,double tolerance,double headingTolerance,double timeout) {
        if(waypoints==null||waypoints.isEmpty()||!Double.isFinite(tolerance)||tolerance<=0||!Double.isFinite(headingTolerance)||headingTolerance<=0||headingTolerance>Math.PI||!Double.isFinite(timeout)||timeout<=0)throw new IllegalArgumentException("invalid follower settings");
        this.waypoints=List.copyOf(waypoints);this.tolerance=tolerance;this.headingTolerance=headingTolerance;this.timeout=timeout;
        x=new PIDController(.06,0,.002,.7,0,.05);y=new PIDController(.06,0,.002,.7,0,.05);heading=new PIDController(.8,0,0,.5,0,.05);
    }
    public Command update(Pose2 pose,double dt) {
        if(pose==null||!Double.isFinite(dt)||dt<=0||dt>.25)throw new IllegalArgumentException("pose and bounded dt required");
        if(failed||index==waypoints.size())return zero();
        Pose2 target=waypoints.get(index);
        double angleError=Pose2.wrap(target.heading()-pose.heading());
        if(pose.distanceTo(target)<=tolerance&&Math.abs(angleError)<=headingTolerance) {
            index++;elapsed=0;x.reset();y.reset();heading.reset();
            return zero();
        }
        elapsed+=dt;
        if(elapsed>=timeout){failed=true;return zero();}
        double fieldX=x.calculate(target.x(),pose.x(),dt),fieldY=y.calculate(target.y(),pose.y(),dt);
        double forward=fieldX*Math.cos(pose.heading())+fieldY*Math.sin(pose.heading());
        double strafe=-fieldX*Math.sin(pose.heading())+fieldY*Math.cos(pose.heading());
        double normalizer=Math.max(1,Math.abs(forward)+Math.abs(strafe));
        return new Command(forward/normalizer,strafe/normalizer,heading.calculate(angleError,0,dt),index,false,false);
    }
    public void cancel(){failed=true;}
    private Command zero(){return new Command(0,0,0,index,index==waypoints.size(),failed);}
}
