package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.OCSparkMax;
import frc.robot.util.OCSparkMax.OCRelativeEncoder;

public class Intake extends SubsystemBase{
    private OCSparkMax floorMotor = new OCSparkMax(kFloorMotorID, MotorType.kBrushless);
    private OCSparkMax centeringMotor = new OCSparkMax(kCenteringMotorID, MotorType.kBrushless);

    OCRelativeEncoder encoder = floorMotor.getEncoder();
    
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.25, 0.005);
    private PIDController floorPID = new PIDController(0.005, 0, 0);
    private PIDController centeringPID = new PIDController(0.0037, 0, 0);

    boolean isManual = true;
    
    double lastFreeTime = Timer.getFPGATimestamp();
    
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
            floorMotor.setVoltage(ff.calculate(floorPID.getSetpoint())+floorPID.calculate(encoder.getVelocity()));
            centeringMotor.setVoltage(ff.calculate(centeringPID.getSetpoint())+centeringPID.calculate(encoder.getVelocity()));
        }

        if (getFloorCurrent() <= kStallCurrent){
            lastFreeTime = Timer.getFPGATimestamp();
        }

        log();
    }

    public void setVoltage(double floorVoltage, double centeringVoltage){
        isManual=true;
        floorMotor.setVoltage(floorVoltage);
        centeringMotor.setVoltage(-centeringVoltage);
    }

    public void setVelocity(double floorRPM, double centeringRPM) {
        isManual = false;
        floorPID.setSetpoint(floorRPM);
        centeringPID.setSetpoint(centeringRPM);
    }

    public boolean isStalled(){
        return Timer.getFPGATimestamp() >= (lastFreeTime + kStallTime);
    }

    public double getFloorCurrent(){
        return floorMotor.getOutputCurrent();
    }

    public double getCenteringCurrent(){
        return centeringMotor.getOutputCurrent();
    }

    public Command setVoltageC(double floorVoltage, double centeringVoltage){
        return run(()->setVoltage(floorVoltage, centeringVoltage));
    }

    public Command setVoltageInC(){
        return run(()->setVoltage(7, 5.5));
    }

    public Command setVoltageOutC(){
        return run(()->setVoltage(-7, -5.5));
    }

    public Command setVelocityC(double floorRPM, double centeringRPM){
        return run(()->setVelocity(floorRPM, centeringRPM));
    }

    public Command setVelocityInC(){
        return run(()->setVelocity(30, 24));
    }

    public Command setVelocityOutC(){
        return run(()->setVelocity(-30, -24));
    }

    public void log() {
        SmartDashboard.putNumber("Intake/Floor Motor Voltage", floorMotor.getAppliedOutput()*12);
        SmartDashboard.putNumber("Intake/Centering Motor Voltage", centeringMotor.getAppliedOutput()*12);
        SmartDashboard.putNumber("Intake/Floor Motor Current", getFloorCurrent());
        SmartDashboard.putNumber("Intake/Centering Motor Current", getCenteringCurrent());
        SmartDashboard.putBoolean("Intake/isStalled", isStalled());
    }
}
