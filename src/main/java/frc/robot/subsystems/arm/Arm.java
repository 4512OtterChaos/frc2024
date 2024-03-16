package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShotMap;

public class Arm extends SubsystemBase {
    private TalonFX leftMotor = new TalonFX(kLeftMotorID);
    private TalonFX rightMotor = new TalonFX(kRightMotorID);

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private Rotation2d targetAngle = kHomeAngle;
    private boolean isManual = false;
    private double targetVoltage = 0;

    private double lastNonStallTime = Timer.getFPGATimestamp();

    private boolean isHoming = false;

    private final StatusSignal<Double> dutyStatus = leftMotor.getDutyCycle();
    private final StatusSignal<Double> voltageStatus = leftMotor.getMotorVoltage();
    private final StatusSignal<Double> positionStatus = leftMotor.getPosition();
    private final StatusSignal<Double> velocityStatus = leftMotor.getVelocity();
    private final StatusSignal<Double> statorStatus = leftMotor.getStatorCurrent();

    public Arm() {
        // try applying motor configs
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = leftMotor.getConfigurator().apply(kConfig);
            rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
            if (status.isOK()) break;
        }
        if (!status.isOK()) DriverStation.reportWarning("Failed applying Arm motor configuration!", false);

        dutyStatus.setUpdateFrequency(100);
        voltageStatus.setUpdateFrequency(100);
        positionStatus.setUpdateFrequency(100);
        velocityStatus.setUpdateFrequency(50);
        statorStatus.setUpdateFrequency(50);
        ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor);
    }

    @Override
    public void periodic() {
        // Angle safety
        double currentRot = getArmRotations();
        double currentKG = Math.cos(getArmRotations()) * kConfig.Slot0.kG;
        double adjustedVoltage = targetVoltage + currentKG;

        if (currentRot >= kMaxAngle.getRotations()) {
            adjustedVoltage = Math.min(adjustedVoltage, currentKG);
        }
        if (!isHoming && currentRot <= kHomeAngle.plus(kAngleTolerance).getRotations()) {
            adjustedVoltage = Math.max(adjustedVoltage, 0);
            if (!isManual) { // go limp at home angle
                isManual = true;
                adjustedVoltage = 0;
            }
        }

        // Voltage/Position control
        if (!isManual) {
            leftMotor.setControl(mmRequest.withPosition(targetAngle.getRotations()));
        }
        else {
            leftMotor.setControl(voltageRequest.withOutput(adjustedVoltage));
        }

        // Stall detection
        if (getCurrent() < kStallThresholdAmps) {
            lastNonStallTime = Timer.getFPGATimestamp();
        }
    }

    public double getArmRotations() {
        return positionStatus.getValueAsDouble();
    }

    public boolean isWithinTolerance() {
        double error = targetAngle.minus(Rotation2d.fromRotations(getArmRotations())).getRotations();
        return Math.abs(error) < kAngleTolerance.getRotations();
    }

    public void resetArmRotations(double rotations){
        leftMotor.setPosition(rotations);
    }

    public void setVoltage(double volts){
        isManual = true;
        targetVoltage = volts;
    }

    public void setRotation(Rotation2d targetRot){
        isManual = false;
        this.targetAngle = Rotation2d.fromRotations(MathUtil.clamp(targetRot.getRotations(), kHomeAngle.getRotations(), kMaxAngle.getRotations()));
    }

    public void stop(){ 
        setVoltage(0);
    }

    public double getVelocity(){
        return velocityStatus.getValueAsDouble();
    }

    public double getCurrent(){
        return statorStatus.getValueAsDouble();
    }

    public boolean getStalled(){
        return (Timer.getFPGATimestamp() - lastNonStallTime) > kStallThresholdSeconds;
    }

    //---------- Command factories

    /** Sets the arm voltage and ends immediately. */
    public Command setVoltageC(double volts) {
        return runOnce(()->setVoltage(volts));
    }

    /** Sets the target arm rotation and ends when it is within tolerance. */
    public Command setRotationC(ShotMap.State state) {
        return setRotationC(state.armAngle);
    }

    /** Sets the target arm rotation and ends when it is within tolerance. */
    public Command setRotationC(Rotation2d targetRot){
        return run(()->setRotation(targetRot)).until(this::isWithinTolerance);
    }

    /** Runs the arm into the hardstop, detecting a current spike and resetting the arm angle. */
    public Command homingSequenceC(){
        return startEnd(
            () -> {
                isHoming = true;
                setVoltage(-2);
            },
            () -> {
                isHoming = false;
                setVoltage(0);
                resetArmRotations(kHomeAngle.getRotations());
            }
        ).until(()->getStalled());
    }

    public void log() {
        SmartDashboard.putNumber("lastNonStallTime", lastNonStallTime);
        SmartDashboard.putNumber("Arm Rotations", getArmRotations());
        SmartDashboard.putNumber("Arm Target Degrees", targetAngle.getDegrees());
        SmartDashboard.putNumber("Motor Current", getCurrent());
        SmartDashboard.putNumber("Motor Target Voltage", targetVoltage);
        SmartDashboard.putNumber("Motor Velocity", getVelocity());
        SmartDashboard.putBoolean("Motor Stalled", getStalled());
        SmartDashboard.putData("Current Arm Command", getCurrentCommand());
        SmartDashboard.putData("Default Arm Command", getDefaultCommand());
    }
}
