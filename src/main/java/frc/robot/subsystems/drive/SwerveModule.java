package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.SwerveConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.drive.SwerveConstants.Module;
import frc.robot.util.TalonUtil;

public class SwerveModule {

    // Module Constants
    private final Module moduleConstants;

    private SwerveModuleState lastDesiredState = new SwerveModuleState();
    private double lastTargetTotalAngle = 0;

    // Hardware
    private final WPI_TalonFX driveMotorOLD; //TODO: DELETE
    private final TalonFX driveMotor;
    private final WPI_TalonFX steerMotorOLD;//TODO: DELETE
    private final TalonFX steerMotor;
    private final WPI_CANCoder steerEncoderOLD;//TODO: DELETE
    private final CANcoder steerEncoder;

    private double lastSeed = Timer.getFPGATimestamp();

    public SwerveModule(Module moduleConstants){
        this.moduleConstants = moduleConstants;

        driveMotorOLD = new WPI_TalonFX(moduleConstants.driveMotorID);
        steerMotorOLD = new WPI_TalonFX(moduleConstants.steerMotorID);
        steerEncoderOLD = new WPI_CANCoder(moduleConstants.cancoderID);

        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        steerMotor = new TalonFX(moduleConstants.steerMotorID);
        steerEncoder = new CANcoder(moduleConstants.cancoderID);

        setupDriveMotor(true);
        setupCancoder(true);
        setupSteerMotor(true);

        // Simulation
        driveMotorSimOLD = driveMotorOLD.getSimCollection(); //TODO: DELETE
        steerMotorSimOLD = steerMotorOLD.getSimCollection(); //TODO: DELETE
        steerEncoderSimOLD = steerEncoderOLD.getSimCollection(); //TODO: DELETE

        driveMotorSim = driveMotor.getSimState();
        steerMotorSim = steerMotor.getSimState();
        steerEncoderSim = steerEncoder.getSimState();
    }

    private void setupDriveMotor(boolean init){
        if(init){
            driveMotor.getConfigurator().apply(driveConfig);
        }
        driveMotor.enableVoltageCompensation(true);
        driveMotor.setSelectedSensorPosition(0);
        driveMotor.setInverted(kInvertDrive);
        TalonUtil.configStatusSolo(driveMotor);
        if(Robot.isSimulation()) TalonUtil.configStatusSim(driveMotor);
    }
    private void setupCancoder(boolean init){
        steerEncoder.getConfigurator().apply(cancoderConfig);
        // steerEncoder.configMagnetOffset(moduleConstants.angleOffset, 50);
        steerEncoder.configMagnetOffset(0, 50);

    }
    private void setupSteerMotor(boolean init){
        if(init){
            steerMotor.getConfigurator().apply(steerConfig);
        }
        steerMotor.enableVoltageCompensation(true);
        steerMotor.setInverted(kInvertSteer);
        resetToAbsolute();
        TalonUtil.configStatusSolo(steerMotor);
        if(Robot.isSimulation()) TalonUtil.configStatusSim(steerMotor);
    }

    public void periodic(){
        // check if the motors had an oopsie, reapply settings
        if(driveMotor.hasResetOccurred()){
            setupDriveMotor(false);
        }
        if(steerMotor.hasResetOccurred()){
            setupSteerMotor(false);
        }

        if(Timer.getFPGATimestamp() - lastSeed > 1 && DriverStation.isDisabled()) {
            resetToAbsolute();
        }
    }

    /**
     * Reset the steering motor integrated encoder to the position of the steering cancoder.
     * We want to use the integrated encoder for control, but need the absolute cancoder for determining our startup rotation.
     */
    public void resetToAbsolute(){
        double absolutePosition = TalonUtil.degreesToPosition(getAbsoluteHeading().getDegrees(), kSteerGearRatio);
        steerMotor.setPosition(absolutePosition);
        lastSeed = Timer.getFPGATimestamp();
    }

    /**
     * Command this swerve module to the desired angle and velocity.
     * Falcon onboard control is used instead of on-RIO control to avoid latency.
     * @param steerInPlace If modules should steer to target angle when target velocity is 0
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean openLoop, boolean steerInPlace){
        Rotation2d currentRotation = getIntegratedHeading();
        // avoid turning more than 90 degrees by inverting speed on large angle changes
        desiredState = SwerveModuleState.optimize(desiredState, currentRotation);
        
        // our desired angle in [-pi, pi]
        double targetConstrainedAngle = desiredState.angle.getRadians();
        // our total current angle. This is not constrained to [-pi, pi]
        double currentTotalAngle = currentRotation.getRadians();
        // our current angle in [-pi, pi]
        double currentConstrainedAngle = MathUtil.angleModulus(currentTotalAngle);
        // convert our constrained target to the closest "total" angle near our current total
        double targetTotalAngle = currentTotalAngle + MathUtil.angleModulus(targetConstrainedAngle - currentConstrainedAngle);

        // if the module is not driving, maintain last angle setpoint
        if(!steerInPlace && Math.abs(desiredState.speedMetersPerSecond) < 0.01){
            targetTotalAngle = lastTargetTotalAngle;
            this.lastDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        }
        else{
            lastTargetTotalAngle = targetTotalAngle;
            this.lastDesiredState = desiredState;
        }

        // convert our target radians to falcon position units
        double angleNative = TalonUtil.radiansToPosition(targetTotalAngle, kSteerGearRatio);
        // perform onboard PID to steer the module to the target angle
        final PositionVoltage request = new PositionVoltage(0).withSlot(0); //TODO: Fix
        steerMotor.set(ControlMode.MotionMagic, angleNative);

        // convert our target meters per second to falcon velocity units
        double velocityNative = TalonUtil.metersToVelocity(
            desiredState.speedMetersPerSecond,
            kDriveGearRatio,
            kWheelCircumference
        );

        double angleError = MathUtil.angleModulus(targetConstrainedAngle - currentConstrainedAngle);
        // perform onboard PID with inputted feedforward to drive the module to the target velocity
        double driveFFOutput = kDriveFF.calculate(this.lastDesiredState.speedMetersPerSecond*Math.cos(angleError))/kVoltageSaturation;
        if(!openLoop){
            driveMotor.set(
                ControlMode.Velocity, velocityNative, // Native falcon counts per 100ms
                DemandType.ArbitraryFeedForward, driveFFOutput // feedforward voltage to percent output
            );
        }
        else{
            driveMotor.set(driveFFOutput);
        }
    }

    public void setDriveBrake(boolean is){
        driveMotor.setNeutralMode(is ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
    public void setSteerBrake(boolean is){
        steerMotor.setNeutralMode(is ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    /**
     * Module heading reported by steering motor integrated encoder.
     * <br></br>
     * NOT constrained to [-pi, pi]
     */
    public Rotation2d getIntegratedHeading(){
        return Rotation2d.fromDegrees(TalonUtil.positionToDegrees(steerMotor.getRotorPosition().getValue(), kSteerGearRatio));
    }
    /**
     * Module heading reported by steering cancoder
     */
    public Rotation2d getAbsoluteHeading(){
        var rot = Rotation2d.fromDegrees(steerEncoder.getAbsolutePosition().getValue());
        if(RobotBase.isSimulation()) return rot;
        return rot.plus(Rotation2d.fromDegrees(moduleConstants.angleOffset));
    }

    /**
     * @return State describing integrated module rotation and velocity in meters per second
     */
    public SwerveModuleState getIntegratedState(){
        double velocity = TalonUtil.velocityToMeters(
            driveMotor.getRotorVelocity().getValue(),
            kDriveGearRatio, kWheelCircumference
        );
        Rotation2d angle = getIntegratedHeading();
        return new SwerveModuleState(velocity, angle);
    }
    /**
     * @return State describing absolute module rotation and velocity in meters per second
     */
    public SwerveModuleState getAbsoluteState(){
        double velocity = TalonUtil.velocityToMeters(
            driveMotor.getRotorVelocity().getValue(),
            kDriveGearRatio, kWheelCircumference
        );
        Rotation2d angle = getAbsoluteHeading();
        return new SwerveModuleState(velocity, angle);
    }

    public void resetPosition(){
        driveMotor.setSelectedSensorPosition(0);
    }
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            TalonUtil.positionToMeters(driveMotor.getRotorPosition().getValue(), kDriveGearRatio, kWheelCircumference),
            getAbsoluteHeading()
        );
    }

    /**
     * @return Constants about this module, like motor IDs, cancoder angle offset, and translation from center
     */
    public Module getModuleConstants(){
        return moduleConstants;
    }

    public void log(){
        SwerveModuleState state = getAbsoluteState();
        int num = moduleConstants.moduleNum;
        String prefix = "Drive/Module "+num+"/";
        
        SmartDashboard.putNumber(prefix+"Steer Degrees", state.angle.getDegrees());

        // SmartDashboard.putNumber(prefix+"Steer Absolute Degrees", steerEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(prefix+"Steer Target Degrees", lastDesiredState.angle.getDegrees());
        SmartDashboard.putNumber(prefix+"Steer Native", steerMotor.getRotorPosition().getValue());
        // SmartDashboard.putNumber(prefix+"Steer Target Native", steerMotor.getClosedLoopTarget());
        SmartDashboard.putNumber(prefix+"Steer Velocity Native", steerMotor.getRotorVelocity().getValue());
        SmartDashboard.putNumber(prefix+"Drive Rotations", TalonUtil.positionToRotations(driveMotor.getRotorPosition().getValue(), kDriveGearRatio));
        SmartDashboard.putNumber(prefix+"Drive Velocity Feet", Units.metersToFeet(state.speedMetersPerSecond));
        SmartDashboard.putNumber(prefix+"Drive Velocity Target Feet", Units.metersToFeet(lastDesiredState.speedMetersPerSecond));
    }


    // Simulation
    private final TalonFXSimCollection driveMotorSimOLD; //TODO: DELETE
    private final TalonFXSimState driveMotorSim;
    private final FlywheelSim driveWheelSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            kDriveFF.kv * kWheelCircumference / (2*Math.PI),
            kDriveFF.ka * kWheelCircumference / (2*Math.PI)
        ),
        DCMotor.getFalcon500(1),
        kDriveGearRatio
    );
    private final TalonFXSimCollection steerMotorSimOLD; //TODO: DELETE
    private final TalonFXSimState steerMotorSim;
    private final FlywheelSim steeringSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(kSteerFF.kv, kSteerFF.ka),
        DCMotor.getFalcon500(1),
        kSteerGearRatio
    );
    private final CANCoderSimCollection steerEncoderSimOLD; //TODO: DELETE
    private final CANcoderSimState steerEncoderSim;

    public void simulationPeriodic(){
        // apply our commanded voltage to our simulated physics mechanisms
        double driveVoltage = driveMotorSim.getMotorVoltage(); //TODO:Was getMotorOutputLeadVoltage
        if(driveVoltage >= 0) driveVoltage = Math.max(0, driveVoltage-kSteerFF.ks);
        else driveVoltage = Math.min(0, driveVoltage+kSteerFF.ks);
        driveWheelSim.setInputVoltage(driveVoltage);

        double steerVoltage = steerMotorSim.getMotorVoltage();
        if(steerVoltage >= 0) steerVoltage = Math.max(0, steerVoltage-kSteerFF.ks);
        else steerVoltage = Math.min(0, steerVoltage+kSteerFF.ks);
        steeringSim.setInputVoltage(steerVoltage);
        
        driveWheelSim.update(0.02);
        steeringSim.update(0.02);

        // update our simulated devices with our simulated physics results
        double driveMotorVelocityNative = TalonUtil.rotationsToVelocity(driveWheelSim.getAngularVelocityRPM()/60, kDriveGearRatio);
        double driveMotorPositionDeltaNative = driveMotorVelocityNative*10*0.02;
        driveMotorSim.setRotorVelocity((int)driveMotorVelocityNative);
        driveMotorSim.addRotorPosition((int)(driveMotorPositionDeltaNative));
        driveMotorSim.setSupplyCurrent(driveWheelSim.getCurrentDrawAmps()/2);

        //SmartDashboard.putNumber("Steer Sim Model Velocity", steeringSim.getAngularVelocityRPM());
        double steerMotorVelocityNative = TalonUtil.rotationsToVelocity(steeringSim.getAngularVelocityRPM()/60, kSteerGearRatio);
        double steerMotorPositionDeltaNative = steerMotorVelocityNative*10*0.02;
        steerMotorSim.setRotorVelocity((int)steerMotorVelocityNative);
        steerMotorSim.addRotorPosition((int)(steerMotorPositionDeltaNative));
        steerMotorSim.setSupplyCurrent(steeringSim.getCurrentDrawAmps()/2);
        
        steerEncoderSim.setVelocity((int)(TalonUtil.rotationsToVelocity(steeringSim.getAngularVelocityRPM()/60, 1)*2));
        steerEncoderSim.setRawPosition((int)(getIntegratedHeading().getDegrees()/360.0*4096));

        driveMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());//TODO:Used to be setBusVoltage
        steerMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        steerEncoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

    public double getDriveCurrentDraw(){
        return driveMotor.getSupplyCurrent().getValue();
    }
    public double getSteerCurrentDraw(){
        return steerMotor.getSupplyCurrent().getValue();
    }
}