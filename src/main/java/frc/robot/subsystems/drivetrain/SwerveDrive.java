package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drivetrain.SwerveConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SwerveDrive extends SwerveDrivetrain implements Subsystem {    
    private static final double kSimLoopPeriod = 0.0025; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation RotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();

    /* Use one of these sysidroutines for your particular test */
    private SysIdRoutine SysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(TranslationCharacterization.withVolts(volts)),
                    null,
                    this));

    private final SysIdRoutine SysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(RotationCharacterization.withVolts(volts)),
                    null,
                    this));
    private final SysIdRoutine SysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(7),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(SteerCharacterization.withVolts(volts)),
                    null,
                    this));
    
    /* Change this to the sysid routine you want to test */
    private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric() // field-centric
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // with open-loop control

    // Logging
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Drive");
    private final DoubleArrayPublisher posePub = table.getDoubleArrayTopic("RobotPose").publish();
    private final DoublePublisher velocityXPub = table.getDoubleTopic("Velocity X").publish();
    private final DoublePublisher velocityYPub = table.getDoubleTopic("Velocity Y").publish();
    private final DoublePublisher velocityAngularPub = table.getDoubleTopic("Velocity Angular").publish();
    private final DoublePublisher speedPub = table.getDoubleTopic("Speed").publish();
    private final DoublePublisher odomFreqPub = table.getDoubleTopic("Odometry Frequency").publish();
    private final StructArrayPublisher<SwerveModuleState> measuredModuleStatePub = 
            table.getStructArrayTopic("Measured Module States", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> targetModuleStatePub = 
            table.getStructArrayTopic("Target Module States", SwerveModuleState.struct).publish();

    public SwerveDrive() {
        super(kPhoenixSwerveConstants, 100, kPhoenixModuleFL, kPhoenixModuleFR, kPhoenixModuleBL, kPhoenixModuleBR);
        getDaqThread().setThreadPriority(99);
        CommandScheduler.getInstance().registerSubsystem(this);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        // hook logging into odometry loop
        registerTelemetry(state -> {
            var pose = state.Pose;
            posePub.set(new double[] {
                pose.getX(), pose.getY(), pose.getRotation().getRadians()
            });
            var fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(state.speeds, pose.getRotation());
            velocityXPub.set(fieldSpeeds.vxMetersPerSecond);
            velocityYPub.set(fieldSpeeds.vyMetersPerSecond);
            velocityAngularPub.set(fieldSpeeds.omegaRadiansPerSecond);
            speedPub.set(Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond));

            measuredModuleStatePub.set(state.ModuleStates);
            targetModuleStatePub.set(state.ModuleTargets);

            odomFreqPub.set(1.0 / state.OdometryPeriod);
        });
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
    
    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }

    public Command cApplyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> setControl(requestSupplier.get()));
    }

    public Command cDrive(DoubleSupplier xVelocity, DoubleSupplier yVelocity, DoubleSupplier angularVelocity) {
        return cApplyRequest(() -> driveRequest
            .withVelocityX(xVelocity.getAsDouble())
            .withVelocityY(yVelocity.getAsDouble())
            .withRotationalRate(angularVelocity.getAsDouble())
        );
    }

    public Command cDrivePercent(DoubleSupplier xPercent, DoubleSupplier yPercent, DoubleSupplier angularPercent) {
        return cDrive(
            ()->xPercent.getAsDouble()*kMaxLinearVelocity,
            ()->yPercent.getAsDouble()*kMaxLinearVelocity,
            ()->angularPercent.getAsDouble()*kMaxAngularVelocity
        );
    }

    /*
     * Both the sysid commands are specific to one particular sysid routine, change
     * which one you're trying to characterize
     */
    public Command cSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return RoutineToApply.quasistatic(direction);
    }

    public Command cSysIdDynamic(SysIdRoutine.Direction direction) {
        return RoutineToApply.dynamic(direction);
    }
}
