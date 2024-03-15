// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.OCSparkMax;

public class Climber extends SubsystemBase {
    private final OCSparkMax leftMotor = new OCSparkMax(kLeftMotorID, MotorType.kBrushless);
    private final OCSparkMax rightMotor = new OCSparkMax(kRightMotorID, MotorType.kBrushless);
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final ProfiledPIDController leftController = new ProfiledPIDController(kP, kI, kD, kConstraints);
    private final ProfiledPIDController rightController = new ProfiledPIDController(kP, kI, kD, kConstraints);

    private boolean isManual = true;
    private double leftTargetVolts = 0;
    private double rightTargetVolts = 0;
    private double leftTargetRotations = 0;
    private double rightTargetRotations = 0;

    /** Creates a new Climber. */
    public Climber() {
        leftMotor.setCANTimeout(kCANTimeout);
        rightMotor.setCANTimeout(kCANTimeout);
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        leftMotor.setInverted(kMotorInverted);
        rightMotor.setInverted(kMotorInverted);
        rightMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.enableVoltageCompensation(kVoltageSaturation);
        rightMotor.enableVoltageCompensation(kVoltageSaturation);
        leftMotor.setSmartCurrentLimit(kPeakCurrentLimit, kContinuousCurrentLimit);
        rightMotor.setSmartCurrentLimit(kPeakCurrentLimit, kContinuousCurrentLimit);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double leftAdjustedVoltage = leftTargetVolts;
        double rightAdjustedVoltage = rightTargetVolts;

        if(!isManual){
            leftAdjustedVoltage = leftController.calculate(leftEncoder.getPosition(), leftTargetRotations);
            rightAdjustedVoltage = rightController.calculate(rightEncoder.getPosition(), rightTargetRotations);
        }

        if(leftEncoder.getPosition() >= kTopHeightRotations) leftAdjustedVoltage = Math.min(leftAdjustedVoltage, 0);
        if(rightEncoder.getPosition() >= kTopHeightRotations) rightAdjustedVoltage = Math.min(rightAdjustedVoltage, 0);
        if(leftEncoder.getPosition() <= kBottomHeightRotations) leftAdjustedVoltage = Math.max(leftAdjustedVoltage, 0);
        if(rightEncoder.getPosition() <= kBottomHeightRotations) rightAdjustedVoltage = Math.max(rightAdjustedVoltage, 0);

        rightMotor.setVoltage(rightAdjustedVoltage);
        leftMotor.setVoltage(leftAdjustedVoltage);
    }

    public boolean isWithinTolerance() {
        double leftError = Math.abs(leftTargetRotations - leftEncoder.getPosition());
        double rightError = Math.abs(rightTargetRotations - rightEncoder.getPosition());
        return leftError <= kToleranceRotations && rightError <= kToleranceRotations;
    }
    
    public void setVolts(double leftVolts, double rightVolts) {
        isManual = true;
        leftTargetVolts = leftVolts;
        rightTargetVolts = rightVolts;
    }

    public void stop() { setVolts(0, 0); }

    public void setRotations(double leftRot, double rightRot) {
        if(isManual){
            leftController.reset(leftEncoder.getPosition(), leftEncoder.getVelocity());
            rightController.reset(rightEncoder.getPosition(), rightEncoder.getVelocity());
        }
        isManual = false;
        leftTargetRotations = leftRot;
        rightTargetRotations = rightRot; 
    }
    public void setTopHeightRotations() { setRotations(kTopHeightRotations, kTopHeightRotations); }
    public void setBottomHeightRotations() { setRotations(kBottomHeightRotations, kBottomHeightRotations); }

    public void log(){
        SmartDashboard.putNumber("Climber/Left Pos", leftEncoder.getPosition());
        SmartDashboard.putNumber("Climber/Right Pos", rightEncoder.getPosition());

        // New code 
        SmartDashboard.putNumber("Climber/Right point", rightController.getSetpoint().position);
        SmartDashboard.putNumber("Climber/Left point", leftController.getSetpoint().position);

    }

    public double getCurrentDraw(){
        return leftMotor.getOutputCurrent() + rightMotor.getOutputCurrent();
    }

    public Command CSetMinHeight(){
        return run(()->setBottomHeightRotations()).until(this::isWithinTolerance);
    }
    
    public Command CSetMaxHeight(){
        return run(()->setTopHeightRotations()).until(this::isWithinTolerance);
    }

    public Command CStop(){
        return runOnce(()->stop());
    }

    public Command holdPositionC(){
        return startEnd(
            ()->{ // get position at start and hold it
                setRotations(leftEncoder.getPosition(), rightEncoder.getPosition());
            },
            ()->{}
        );
    }

    public Command setVoltageC(double leftVolts, double rightVolts){
        return runOnce(()->setVolts(leftVolts, rightVolts));
    }

    public Command setVoltageUpC(){
        return setVoltageC(kVoltageUp, kVoltageUp);
    }
    public Command setVoltageUpLeftC(){
        return setVoltageC(kVoltageUp, 0);
    }
    public Command setVoltageUpRightC(){
        return setVoltageC(0, kVoltageUp);
    }

    public Command setVoltageDownC(){
        return setVoltageC(kVoltageDown, kVoltageDown);
    }
    public Command setVoltageDownLeftC(){
        return setVoltageC(kVoltageDown, 0);
    }
    public Command setVoltageDownRightC(){
        return setVoltageC(0, kVoltageDown);
    }


    // simulation init
    {
        REVPhysicsSim.getInstance().addSparkMax(leftMotor, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(rightMotor, DCMotor.getNEO(1));
    }
    @Override
    public void simulationPeriodic() {
    }
}
