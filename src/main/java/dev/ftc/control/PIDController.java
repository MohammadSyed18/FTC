package dev.ftc.control;

/** Bounded PID with derivative filtering and conditional-integration anti-windup. */
public final class PIDController {
    private final double kp,ki,kd,limit,integralLimit,filterSeconds;
    private double integral,previousMeasurement,derivative;
    private boolean initialized;
    public PIDController(double kp,double ki,double kd,double limit,double integralLimit,double filterSeconds) {
        if (!finite(kp,ki,kd,limit,integralLimit,filterSeconds)||kp<0||ki<0||kd<0||limit<=0||integralLimit<0||filterSeconds<0) throw new IllegalArgumentException("invalid PID settings");
        this.kp=kp;this.ki=ki;this.kd=kd;this.limit=limit;this.integralLimit=integralLimit;this.filterSeconds=filterSeconds;
    }
    public double calculate(double target,double measurement,double dt) {
        if (!finite(target,measurement,dt)||dt<=0||dt>.25) throw new IllegalArgumentException("finite input and dt in (0,.25] required");
        double error=target-measurement;
        double raw=initialized?-(measurement-previousMeasurement)/dt:0;
        if (!finite(error,raw)) throw new IllegalArgumentException("PID arithmetic exceeded finite range");
        double alpha=filterSeconds==0?1:dt/(filterSeconds+dt);
        derivative+=alpha*(raw-derivative);
        double candidate=clamp(integral+error*dt,integralLimit);
        double unconstrained=kp*error+ki*candidate+kd*derivative;
        // Keep integration only if unsaturated or the error drives out of saturation.
        if(Math.abs(unconstrained)<=limit||Math.signum(error)!=Math.signum(unconstrained))integral=candidate;
        previousMeasurement=measurement;initialized=true;
        return clamp(kp*error+ki*integral+kd*derivative,limit);
    }
    public void reset() {integral=0;derivative=0;initialized=false;}
    public double integral() {return integral;}
    private static double clamp(double value,double limit) {return Math.max(-limit,Math.min(limit,value));}
    private static boolean finite(double... values) {for(double value:values)if(!Double.isFinite(value))return false;return true;}
}
