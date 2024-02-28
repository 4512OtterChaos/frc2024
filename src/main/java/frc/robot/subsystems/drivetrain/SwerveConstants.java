package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    
    public static final double kTrackWidth = Units.inchesToMeters(20.5);
    public static final double kTrackLength = Units.inchesToMeters(18.5);
    public static final double kWheelDiameter = Units.inchesToMeters(4);
    public static final double kWheelCircumference = kWheelDiameter * Math.PI;

    public static final double kDriveGearRatio = 6.12; // 6.12:1 SDS MK4 L3 ratio
    public static final double kSteerGearRatio = 12.8; // 12.8:1

    public static final double kLinearAcceleration = Units.feetToMeters(20); //TODO: tune these to this year's robot (currently just last year's robot)
    public static final double kLinearDeceleration = Units.feetToMeters(30);
    public static final double kRotationalAcceleration = Units.rotationsToRadians(6);
    public static final double kRotationalDeceleration = Units.rotationsToRadians(10);

    private static final int kPigeonId = 1;
    
    public enum Module {
        FL(1,1,2,1,0,kTrackWidth/2,kTrackLength/2),
        FR(2,3,4,0,0,kTrackWidth/2,-kTrackLength/2),
        BL(3,5,6,3,0,-kTrackWidth/2,kTrackLength/2),
        BR(4,7,8,8,0,-kTrackWidth/2,-kTrackLength/2);

        public final int moduleNum;
        public final int driveMotorID;
        public final int steerMotorID;
        public final int cancoderID;
        public final double angleOffset;
        public final Translation2d centerOffset;
        private Module(int moduleNum, int driveMotorID, int steerMotorID, int cancoderID, double angleOffset, double xOffset, double yOffset){
            this.moduleNum = moduleNum;
            this.driveMotorID = driveMotorID;
            this.steerMotorID = steerMotorID;
            this.cancoderID = cancoderID;
            this.angleOffset = angleOffset;
            centerOffset = new Translation2d(xOffset, yOffset);
        }
    }

    // Feedforward TODO: sysid
    // Linear drive feed forward
    public static final SimpleMotorFeedforward kDriveFF = new SimpleMotorFeedforward(
        0.25, // Voltage to break static friction
        2.5, // Volts per meter per second
        0.3 // Volts per meter per second squared
    );
    // Steer feed forward
    public static final SimpleMotorFeedforward kSteerFF = new SimpleMotorFeedforward(
        0.5, // Voltage to break static friction
        0.25, // Volts per radian per second
        0.01 // Volts per radian per second squared
    );

    //----------- Phoenix Swerve API
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.05)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);
    
    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 60.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kMaxLinearVelocity = Units.feetToMeters(17);
    public static final double kMaxAngularVelocity = Units.rotationsToRadians(2);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "rio";

    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    public static final SwerveDrivetrainConstants kPhoenixSwerveConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory kPhoenixConstantsFactory = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(Units.metersToInches(kWheelDiameter) / 2.0) // why is this in inches? ? ???
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kMaxLinearVelocity)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);

    public static final SwerveModuleConstants kPhoenixModuleFL = kPhoenixConstantsFactory.createModuleConstants(
        Module.FL.steerMotorID, Module.FL.driveMotorID, Module.FL.cancoderID,
        Module.FL.angleOffset, Module.FL.centerOffset.getX(), Module.FL.centerOffset.getY(),
        kInvertLeftSide
    );
    public static final SwerveModuleConstants kPhoenixModuleFR = kPhoenixConstantsFactory.createModuleConstants(
        Module.FR.steerMotorID, Module.FR.driveMotorID, Module.FR.cancoderID,
        Module.FR.angleOffset, Module.FR.centerOffset.getX(), Module.FR.centerOffset.getY(),
        kInvertRightSide
    );
    public static final SwerveModuleConstants kPhoenixModuleBL = kPhoenixConstantsFactory.createModuleConstants(
        Module.BL.steerMotorID, Module.BL.driveMotorID, Module.BL.cancoderID,
        Module.BL.angleOffset, Module.BL.centerOffset.getX(), Module.BL.centerOffset.getY(),
        kInvertLeftSide
    );
    public static final SwerveModuleConstants kPhoenixModuleBR = kPhoenixConstantsFactory.createModuleConstants(
        Module.BR.steerMotorID, Module.BR.driveMotorID, Module.BR.cancoderID,
        Module.BR.angleOffset, Module.BR.centerOffset.getX(), Module.BR.centerOffset.getY(),
        kInvertRightSide
    );
}
