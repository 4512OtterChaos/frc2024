package frc.robot.subsystems.superShooter.arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.superShooter.arm.ArmConstants.*;

public class Arm extends SubsystemBase {
    private TalonFX leftMotor = new TalonFX(1);
    private TalonFX rightMotor = new TalonFX(2);
    private ProfiledPIDController pid = new ProfiledPIDController(kP, kI, kI, null);
    private StatusSignal<Double> lefStatusSignal = leftMotor.getPosition();
    private StatusSignal<Double> righStatusSignal = rightMotor.getPosition();
    

    private double targetAngle = 0;

    public Arm() {
        rightMotor.setInverted(true);

        pid.enableContinuousInput(-Math.PI,Math.PI);
        leftMotor.getPosition();
    }

    @Override
    public void periodic() {
        double currentAngle = (lefStatusSignal.getValueAsDouble()+righStatusSignal.getValueAsDouble())/2;
        setSpeed(pid.calculate(currentAngle,targetAngle));


    }

    public void setSpeed(double desiredSpeed){
        MathUtil.clamp(desiredSpeed, -1, 1);
        leftMotor.setVoltage(kShoulderVoltage*desiredSpeed);
        rightMotor.setVoltage(kShoulderVoltage*desiredSpeed);
    }

    public void setAngle(double targetAngle){
        this.targetAngle = targetAngle;
    }

    

}
