package dev.ftc.control;
import java.util.List;

public class ControlTest {
    private static void near(double actual,double expected) {if(Math.abs(actual-expected)>1e-6)throw new AssertionError(actual+" != "+expected);}
    public static void main(String[] args) {
        var odometry=new EncoderOdometry(100,10,2,new Pose2(0,0,0));
        odometry.update(0,0,0);var pose=odometry.update(1000,1000,0);near(pose.x(),10);near(pose.y(),0);
        pose=odometry.update(1000,1000,500);near(pose.y(),5);
        var rotation=new EncoderOdometry(100,10,2,new Pose2(0,0,0));rotation.update(0,0,0);
        pose=rotation.update(-500,500,200);near(pose.x(),0);near(pose.y(),0);near(pose.heading(),1);
        var pid=new PIDController(1,1,0,1,10,0);
        for(int index=0;index<1000;index++)near(pid.calculate(100,0,.02),1);
        near(pid.integral(),0);near(pid.calculate(0,0,.02),0);
        var follower=new WaypointFollower(List.of(new Pose2(10,0,0)),.5,.1,2);
        if(follower.update(new Pose2(0,0,0),.02).forward()<=0)throw new AssertionError("wrong direction");
        if(!follower.update(new Pose2(10,0,0),.02).complete())throw new AssertionError("not completed");
        var timeout=new WaypointFollower(List.of(new Pose2(10,0,0)),.5,.1,.1);
        for(int index=0;index<10;index++)timeout.update(new Pose2(0,0,0),.02);
        if(!timeout.update(new Pose2(0,0,0),.02).failed())throw new AssertionError("timeout failed");
        boolean rejected=false;
        try {pid.calculate(Double.MAX_VALUE,-Double.MAX_VALUE,.02);} catch(IllegalArgumentException expected){rejected=true;}
        if(!rejected)throw new AssertionError("nonfinite arithmetic accepted");
        System.out.println("FTC control checks passed");
    }
}
