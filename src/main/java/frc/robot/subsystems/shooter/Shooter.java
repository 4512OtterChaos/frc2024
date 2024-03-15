package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShotMap;
import frc.robot.util.OCSparkMax;
import frc.robot.util.OCSparkMax.OCRelativeEncoder;

public class Shooter extends SubsystemBase {
    private OCSparkMax leftMotor = new OCSparkMax(kLeftMotorID, MotorType.kBrushless);
    private OCSparkMax rightMotor = new OCSparkMax(kRightMotorID, MotorType.kBrushless);

    private final OCRelativeEncoder leftEncoder = leftMotor.getEncoder();
    private final OCRelativeEncoder rightEncoder = rightMotor.getEncoder();

    private SparkPIDController leftPid = leftMotor.getPIDController();
    private SparkPIDController rightPid = rightMotor.getPIDController();

    private double targetLeftRPM = 0;
    private double targetRightRPM = 0;

    private double lastAverageErrorRPM = 0;
    private boolean shotDetected = false;

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

    @Override
    public void periodic() {
        double avgErrorRPM = getAverageErrorRPM();
        double diff = avgErrorRPM - lastAverageErrorRPM;
        shotDetected = diff > kShotDetectThresholdRPM;
        lastAverageErrorRPM = avgErrorRPM;
    }

    public double getLeftRPM() {
        return leftEncoder.getVelocity();
    }

    public double getRightRPM() {
        return rightEncoder.getVelocity();
    }

    public double getTargetLeftRPM() {
        return targetLeftRPM;
    }

    public double getTargetRightRPM() {
        return targetRightRPM;
    }

    public double getAverageErrorRPM() {
        return ((targetLeftRPM - getLeftRPM()) + (targetRightRPM - getRightRPM()) / 2.0);
    }

    public boolean isReady() {
        return getAverageErrorRPM() < kToleranceRPM;
    }

    public boolean isShotDetected() {
        return shotDetected;
    }

    public void setVoltage(double leftVolts, double rightVolts) {
        leftMotor.setVoltage(leftVolts);
        targetLeftRPM = kFF.maxAchievableVelocity(leftVolts, 0);
        rightMotor.setVoltage(rightVolts);
        targetRightRPM = kFF.maxAchievableVelocity(rightVolts, 0);
        lastAverageErrorRPM = getAverageErrorRPM();
    }

    public void setVelocity(ShotMap.State state) {
        setVelocity(state.leftRPM, state.rightRPM);
    }

    public void setVelocity(double leftRPM, double rightRPM) {
        if (leftRPM == 0 && rightRPM == 0) {
            stop();
        }
        else {
            targetLeftRPM = leftRPM;
            double leftRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(leftRPM);
            leftPid.setReference(leftRPM, ControlType.kVelocity, 0, kFF.calculate(leftRadPerSec));
            targetRightRPM = rightRPM;
            double rightRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rightRPM);
            rightPid.setReference(rightRPM, ControlType.kVelocity, 0, kFF.calculate(rightRadPerSec));
        }
    }

    public void stop() { 
        setVoltage(0, 0);
    }

    //---------- Command factories

    /** Sets the shooter voltages and ends immediately. */
    public Command setVoltageC(double leftVolts, double rightVolts) {
        return runOnce(()->setVoltage(leftVolts, rightVolts));
    }

    /** Sets the shooter velocities and ends when the velocity is within tolerance. */
    public Command setVelocityC(ShotMap.State state) {
        return setVelocityC(state.leftRPM, state.rightRPM);
    }

    /** Sets the shooter velocities and ends when the velocity is within tolerance. */
    public Command setVelocityC(double leftRPM, double rightRPM) {
        return run(()->setVelocity(leftRPM, rightRPM)).until(this::isReady);
    }

    /** Sets the shooter voltages to 0 and ends immediately. */
    public Command stopC() {
        return setVoltageC(0, 0);
    }
}
