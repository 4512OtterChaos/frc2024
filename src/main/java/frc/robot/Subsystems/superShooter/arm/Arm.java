package frc.robot.subsystems.superShooter.arm;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.superShooter.arm.ArmConstants.*;

public class Arm extends SubsystemBase {
    private TalonFX leftMotor = new TalonFX(1);
    private TalonFX rightMotor = new TalonFX(2);
    private ProfiledPIDController pid = new ProfiledPIDController(kP, kI, kI, null);
    private SimpleMotorFeedforward mff = new SimpleMotorFeedforward(1, 0.5);
    

    private double targetAngle = 0;
    private boolean isManual = false;

    public Arm() {
        rightMotor.setInverted(true);

        pid.enableContinuousInput(-Math.PI,Math.PI);
        leftMotor.getPosition();
    }

    @Override
    public void periodic() {
        double currentAngle = getEncoderAngle();
        if (!isManual){setVoltage(pid.calculate(currentAngle,targetAngle)+mff.calculate(getVelocity()));}
        


    }

    public void setVoltage(double volts){
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
    }

    public void setSpeed(double desiredSpeed){
        isManual = true;
        MathUtil.clamp(desiredSpeed, -1, 1);
        setVoltage(kShoulderVoltage*desiredSpeed);
    }

    public void setAngle(double targetAngle){
        isManual = false;
        this.targetAngle = Math.toRadians(MathUtil.clamp(targetAngle,0,kMaxRotationDegrees));
    }

    public double getEncoderDistance(){
        return (rightMotor.getPosition().getValueAsDouble()+leftMotor.getPosition().getValueAsDouble())/2;
    }

    public double getVelocity(){
        return (leftMotor.getVelocity().getValueAsDouble()+rightMotor.getVelocity().getValueAsDouble())/2;
    }

    public double getEncoderAngle(){
        return Math.toRadians((getEncoderDistance()/kCountsPerRevolution)*360);
    }

    public void resetEncoders(){
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
    }

    public double getCurrentFlow(){
        double rightCurrent = rightMotor.getTorqueCurrent().getValueAsDouble();
        double leftCurrent = leftMotor.getTorqueCurrent().getValueAsDouble();
        return (leftCurrent+rightCurrent)/2;
    }

    

    // public Command sendArmHome(){
        
    //     return (
    //         run(()->setSpeed(-0.1)).until(()->getStalled()).finallyDo(()->resetEncoders())
    //     );
    // }


    public Command CSetAngle(double targetAngle){
        return runOnce(()->setAngle(targetAngle));
    }

}
