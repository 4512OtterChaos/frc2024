package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.OCSparkMax;
import frc.robot.util.TalonUtil;
import frc.robot.util.OCSparkMax.OCRelativeEncoder;

public class Shooter extends SubsystemBase {
    private OCSparkMax leftMotor = new OCSparkMax(kLeftMotorID, MotorType.kBrushless);
    private OCSparkMax rightMotor = new OCSparkMax(kRightMotorID, MotorType.kBrushless);

    private final OCRelativeEncoder leftEncoder = leftMotor.getEncoder();
    private final OCRelativeEncoder rightEncoder = rightMotor.getEncoder();

    private SparkPIDController leftPid = leftMotor.getPIDController();
    private SparkPIDController rightPid = rightMotor.getPIDController();

    private double targetLeftVel = 0;
    private double targetRightVel = 0;

    private double lastShotTime;


    public Shooter() {
        leftMotor.enableVoltageCompensation(12);
        rightMotor.enableVoltageCompensation(12);

        leftPid.setP(kP);
        leftPid.setI(kI);
        leftPid.setD(kD);

        rightPid.setP(kP);
        rightPid.setI(kI);
        rightPid.setD(kD);
    }

    public void setLeftVoltage(double voltage){
        leftMotor.setVoltage(voltage);
        if (voltage!=0){
            setLastShotTime();
        }
    }
    public void setRightVoltage(double voltage){
        rightMotor.setVoltage(voltage);
        if (voltage!=0){
            setLastShotTime();
        }
    }

    public void setLeftVelocity(double velocity){
        leftMotor.setVoltage(TalonUtil.velocityToRadians(velocity, 24/18)/kV);
        if (velocity!=0){
            setLastShotTime();
        }
        targetLeftVel = velocity;
    }

    public void setRightVelocity(double velocity){
        rightMotor.setVoltage(velocity/kV);
        if (velocity!=0){
            setLastShotTime();
        }
        targetRightVel = velocity;
    }

    public void setLeftRPM(double rpm){ //TODO: create method

    }

    public void setRightRPM(double rpm){ //TODO: create method
        
    }

    public void shootSubwoof(){
        setLeftVoltage(9);
        setRightVoltage(8);
    }

    public void shootAmp(){
        setLeftVoltage(2);
        setRightVoltage(2);
    }

    public void stop(){ 
        setLeftVoltage(0);
        setRightVoltage(0);
    }

    public void shootTable(double distance){
    // to be completed once we have the table
    }

    public double getLeftVel(){
        return leftEncoder.getVelocity();
    }

    public double getRightVel(){
        return rightEncoder.getVelocity();
    }

    public double getTargetLeftVel(){
        return targetLeftVel;
    }

    public double getTargetRightVel(){
        return targetRightVel;
    }

    private void setLastShotTime(){
        lastShotTime = Timer.getFPGATimestamp(); 
    }

    public boolean shotNote(){
        if (getTargetLeftVel()-getLeftVel()>kNoteShotVelocityDrop && ((Timer.getFPGATimestamp()-lastShotTime)>0.1)){
            return true;
        }
        return false;
    }
    






    public Command shootSubwoofC(){
        return runOnce(()->shootSubwoof());
    }

    public Command CShootAmp(){
        return run(()->shootAmp());
    }

    public Command CShootTable(double distance){
        return run(()->shootTable(distance));
    }

    public Command stopC() {
        return runOnce(()-> {
            stop();
        });
    }



    
    /**
     * Represents a state of flywheel rpm and hood angle achievable by the shooter.
     * Shooter states can be transitioned from by using {@link #interpolate(State, double)}.
     */
    public static class State implements Interpolatable<Shooter.State>{
        public final double leftRPM;
        public final double rightRPM;
        public final double armDeg;
        
        public State(double leftRPM, double rightRPM, double armDeg){
            this.leftRPM = leftRPM;
            this.rightRPM = rightRPM;
            this.armDeg = armDeg;
        }
        public State(){
            this(0, 0, 0);
        }
        public State plus(State other){
            return new State(this.leftRPM+other.leftRPM, this.rightRPM+other.rightRPM, this.armDeg+other.armDeg);
        }
        public boolean withinTolerance(State other){
            return Math.abs(other.leftRPM - leftRPM) <= kToleranceRPM && Math.abs(other.rightRPM - rightRPM) <= kToleranceRPM && Math.abs(other.armDeg - armDeg) < kArmAngleTolerance;
        }
        
        @Override
        public State interpolate(State endValue, double t) {
            if (t < 0) {
                return this;
            } else if (t >= 1) {
                return endValue;
            } else {
                double newLeftRPM = MathUtil.interpolate(leftRPM, endValue.leftRPM, t);
                double newRightRPM = MathUtil.interpolate(leftRPM, endValue.rightRPM, t);
                double newArmDeg = MathUtil.interpolate(armDeg, endValue.armDeg, t);
                return new State(newLeftRPM, newRightRPM, newArmDeg);
            }
        }
    }
}
