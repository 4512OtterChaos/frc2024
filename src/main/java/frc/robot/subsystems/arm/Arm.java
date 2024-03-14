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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private TalonFX leftMotor = new TalonFX(kLeftMotorID);
    private TalonFX rightMotor = new TalonFX(kRightMotorID);

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private Rotation2d targetAngle = kHomeAngle;
    private boolean isManual = false;
    private double targetVoltage = 0;

    private final StatusSignal<Double> dutyStatus = leftMotor.getDutyCycle();
    private final StatusSignal<Double> voltageStatus = leftMotor.getMotorVoltage();
    private final StatusSignal<Double> positionStatus = leftMotor.getPosition();
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
        statorStatus.setUpdateFrequency(50);
        ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor);
    }

    @Override
    public void periodic() {
        
        if (!isManual) {
            leftMotor.setControl(mmRequest.withPosition(targetAngle.getRotations()));
        }
        else {
            leftMotor.setControl(voltageRequest.withOutput(targetVoltage));
        }
    }

    public double getArmRotations() {
        return positionStatus.getValueAsDouble();
    }

    public void setArmRotations(double rotations){
        leftMotor.setPosition(rotations);
    }

    public void setVoltage(double volts){
        isManual = true;
        targetVoltage = volts + kConfig.Slot0.kG;
    }

    public void setAngle(double targetAngle){
        isManual = false;
        this.targetAngle = Rotation2d.fromRotations(MathUtil.clamp(targetAngle, kHomeAngle.getRotations(), kMaxAngle.getRotations()));
    }

    public double getVelocity(){
        return (leftMotor.getVelocity().getValueAsDouble()+rightMotor.getVelocity().getValueAsDouble())/2;
    }

    public double getCurrent(){
        return statorStatus.getValueAsDouble();
    }

    public boolean getStalled(){
        return false;
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
