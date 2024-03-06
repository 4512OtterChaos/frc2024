package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.OCSparkMax;
import frc.robot.util.OCSparkMax.OCRelativeEncoder;

public class Intake extends SubsystemBase{
    private OCSparkMax floorMotor = new OCSparkMax(kFloorMotorID, MotorType.kBrushless);

    OCRelativeEncoder encoder = floorMotor.getEncoder();
    
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.25, 0.005);
    private PIDController pid = new PIDController(0.005, 0, 0);
    boolean isManual = true;
    
    public Intake(){
        // intakeMotor.setCANTimeout(100);
        // OCConfig.configMotors(kMotorStallLimit, kMotorStallLimit, kRampRate, leftMotor, rightMotor);
        // OCConfig.setStatusNormal(leftMotor, rightMotor);
        // OCConfig.setIdleMode(IdleMode.kBrake, leftMotor, rightMotor);
        // intakeMotor.setCANTimeout(0);
		// OCConfig.saveConfig(leftMotor, rightMotor);
    }

    // @Override
    public void periodic() {
        if(!isManual) {
            floorMotor.setVoltage(ff.calculate(pid.getSetpoint())+pid.calculate(encoder.getVelocity()));
        }
    }

    public void setVoltage(double voltage){
        isManual=true;
        floorMotor.setVoltage(voltage);
    }

    public void setVelocity(double rpm) {
        isManual = false;
        pid.setSetpoint(rpm);
    }

    public Command setVoltageC(double voltage){
        return run(()->setVoltage(voltage));
    }

    public Command setVoltageInC(){
        return run(()->setVoltage(7));
    }

    public Command setVoltageOutC(){
        return run(()->setVoltage(-7));
    }

    public Command setVelocityC(double velocity){
        return run(()->setVelocity(velocity));
    }

    public Command setVelocityInC(){
        return run(()->setVelocity(30));
    }

    public Command setVelocityOutC(){
        return run(()->setVelocity(-30));
    }
}
