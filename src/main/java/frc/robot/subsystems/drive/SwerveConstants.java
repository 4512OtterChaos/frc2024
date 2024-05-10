package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.TalonUtil;

public class SwerveConstants {

    public static final double kLinearAcceleration = Units.feetToMeters(30); //m/s/s
    public static final double kLinearDeceleration = Units.feetToMeters(40);
    public static final double kRotationalAcceleration = Units.rotationsToRadians(6);
    public static final double kRotationalDeceleration = Units.rotationsToRadians(10);

    private static final boolean isReal = RobotBase.isReal();
    public static final int kPigeonID = 0;

    // Inversions
    public static final boolean kInvertGyro = false;
    public static final boolean kInvertDrive = false;
    public static final boolean kInvertSteer = false;
    public static final SensorDirectionValue kCancoderDirection = SensorDirectionValue.CounterClockwise_Positive;
    // Physical properties
    public static final double kTrackWidth = Units.inchesToMeters(20.5);
    public static final double kTrackLength = Units.inchesToMeters(18.5);
    public static final double kRobotWidth = Units.inchesToMeters(27.375 + 3.25*2);
    public static final double kRobotLength = Units.inchesToMeters(25.188 + 5.5 + 3.25*2);
    public static final double kRobotBackHalfLength = Units.inchesToMeters(25.0 / 2.0 + 0.188 + 3.25);
    
    public static final double kSwerveCenterRadius = Math.hypot(kTrackLength, kTrackWidth)/2;
    
    public static final double kMaxLinearSpeed = Units.feetToMeters(17);
    public static final double kMaxAngularSpeed = Units.rotationsToRadians(2.2);
    public static final double kWheelDiameter = Units.inchesToMeters(3.8715);
    public static final double kWheelCircumference = kWheelDiameter*Math.PI;
    public static final double kDriveGearRatio = 6.12; // 6.12:1 L3 ratio
    public static final double kSteerGearRatio = 12.8; // 12.8:1

    public enum Module {
        FL(1, 2, 6, 1, -172.353, kTrackLength/2, kTrackWidth/2), // Front Right
        FR(2, 1, 5, 4, -104.414, kTrackLength/2, -kTrackWidth/2), // Back Left
        BL(3, 7, 3, 3, 68.554, -kTrackLength/2, kTrackWidth/2), // Front left
        BR(4, 4, 8, 2, 66.357, -kTrackLength/2, -kTrackWidth/2); // Back Right

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

    // Current limits
    public static final int kDriveContinuousCurrentLimit = 40;
    public static final int kDrivePeakCurrentLimit = 60;
    public static final double kDrivePeakCurrentDuration = 0.1;
    public static final int kSteerContinuousCurrentLimit = 25;
    public static final int kSteerPeakCurrentLimit = 40;
    public static final double kSteerPeakCurrentDuration = 0.1;
    // Voltage compensation
    public static final double kVoltageSaturation = 12;
    public static final int kVoltageMeasurementSamples = 32;

    // Feedforward
    // Linear drive feed forward
    public static final SimpleMotorFeedforward kDriveFF = new SimpleMotorFeedforward( // real
        0.2, // Voltage to break static friction
        2.25, // Volts per meter per second
        0.17 // Volts per meter per second squared
    );
    // Steer feed forward
    public static final SimpleMotorFeedforward kSteerFF = new SimpleMotorFeedforward( // real
        0.55, // Voltage to break static friction
        0.23, // Volts per radian per second
        0.0056 // Volts per radian per second squared
    );

    // PID
    public static final double kDriveKP = 0.025;
    public static final double kDriveKI = 0;
    public static final double kDriveKD = 0;

    public static final double kSteerKP = 0.4;
    public static final double kSteerKI = 0;
    public static final double kSteerKD = 0.5;
    public static final double kSteerVelocity = 8; // rotations per second
    public static final double kSteerAcceleration = 40; // rotations per second squared
    public static final int kAllowableSteeringError = 80;

    // The configurations applied to swerve CTRE devices
    public static final TalonFXConfiguration driveConfigOLD = new TalonFXConfiguration(); //TODO: DELETE
    public static final Slot0Configs driveConfig = new Slot0Configs();
    public static final TalonFXConfiguration steerConfigOLD = new TalonFXConfiguration(); //TODO: DELETE
    public static final Slot0Configs steerConfig = new Slot0Configs();
    public static final CANCoderConfiguration cancoderConfigOLD = new CANCoderConfiguration(); //TODO: DELETE
    public static final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    public static final MagnetSensorConfigs cancoderMagnetConfig = cancoderConfig.MagnetSensor;
    public static final Pigeon2Configuration kPigeon2Config = new Pigeon2Configuration();

    
    public static final int kCANTimeout = 100;

    static {
        driveConfig.initializationStrategy = SensorInitializationStrategy.BootToZero; //TODO: Find new version
        driveConfig.kP = kDriveKP;
        driveConfig.kI = kDriveKI;
        driveConfig.kD = kDriveKD;
        driveConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
            true,
            kDriveContinuousCurrentLimit,
            kDrivePeakCurrentLimit,
            kDrivePeakCurrentDuration
        );
        driveConfig.voltageCompSaturation = kVoltageSaturation;
        driveConfig.voltageMeasurementFilter = kVoltageMeasurementSamples;
        driveConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_20Ms;
        driveConfig.velocityMeasurementWindow = 32;

        steerConfig.initializationStrategy = SensorInitializationStrategy.BootToZero; //TODO: Find new version
        steerConfig.kP = kSteerKP;
        steerConfig.kI = kSteerKI;
        steerConfig.kD = kSteerKD;
        steerConfig.kV = kSteerFF.kv;
        steerConfig.motionCruiseVelocity = TalonUtil.rotationsToVelocity(kSteerVelocity, kSteerGearRatio);
        steerConfig.motionAcceleration = TalonUtil.rotationsToVelocity(kSteerAcceleration, kSteerGearRatio);
        steerConfig.allowableClosedloopError = kAllowableSteeringError;
        steerConfig.neutralDeadband = isReal ? 0.01 : 0.001;
        steerConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
            true,
            kSteerContinuousCurrentLimit,
            kSteerPeakCurrentLimit,
            kSteerPeakCurrentDuration
        );
        steerConfig.voltageCompSaturation = kVoltageSaturation;
        steerConfig.voltageMeasurementFilter = kVoltageMeasurementSamples;
        steerConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_20Ms;
        steerConfig.velocityMeasurementWindow = 32;

        cancoderMagnetConfig.withAbsoluteSensorRange(AbsoluteSensorRangeValue.valueOf(180)); //TODO:Check if this is right
        cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition; //TODO: Find new version
        cancoderMagnetConfig.SensorDirection = kCancoderDirection;
        cancoderMagnetConfig.MagnetOffset = 0; //TODO: Used to have timeout of 50ms
    }
}
