package dev.ftc.control;

/** Three-wheel incremental odometry with an SE(2) exponential integration step. */
public final class EncoderOdometry {
    private final double ticksPerUnit,trackWidth,lateralOffset;
    private long previousLeft,previousRight,previousLateral;
    private boolean initialized;
    private Pose2 pose;
    public EncoderOdometry(double ticksPerUnit,double trackWidth,double lateralOffset,Pose2 initial) {
        if (!Double.isFinite(ticksPerUnit)||ticksPerUnit<=0||!Double.isFinite(trackWidth)||trackWidth<=0||!Double.isFinite(lateralOffset)||initial==null) throw new IllegalArgumentException("invalid odometry configuration");
        this.ticksPerUnit=ticksPerUnit;this.trackWidth=trackWidth;this.lateralOffset=lateralOffset;pose=initial;
    }
    public Pose2 update(long left,long right,long lateral) {
        if (!initialized) {baseline(left,right,lateral);return pose;}
        double dl=((double)left-previousLeft)/ticksPerUnit;
        double dr=((double)right-previousRight)/ticksPerUnit;
        double dp=((double)lateral-previousLateral)/ticksPerUnit;
        double theta=(dr-dl)/trackWidth;
        double forward=(dl+dr)/2,sideways=dp-lateralOffset*theta;
        double sinc=Math.abs(theta)<1e-6?1-theta*theta/6:Math.sin(theta)/theta;
        double cosc=Math.abs(theta)<1e-6?theta/2-theta*theta*theta/24:(1-Math.cos(theta))/theta;
        double localX=sinc*forward-cosc*sideways,localY=cosc*forward+sinc*sideways;
        double cos=Math.cos(pose.heading()),sin=Math.sin(pose.heading());
        pose=new Pose2(pose.x()+cos*localX-sin*localY,pose.y()+sin*localX+cos*localY,pose.heading()+theta);
        baseline(left,right,lateral);return pose;
    }
    public void reset(Pose2 initial,long left,long right,long lateral) {
        if(initial==null)throw new IllegalArgumentException("pose required");pose=initial;baseline(left,right,lateral);
    }
    public Pose2 pose() {return pose;}
    private void baseline(long left,long right,long lateral) {previousLeft=left;previousRight=right;previousLateral=lateral;initialized=true;}
}
