package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {
    
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

    public static SwerveModuleConstants FL = new SwerveModuleConstants();
    public static SwerveModuleConstants FR = new SwerveModuleConstants();
    public static SwerveModuleConstants BL = new SwerveModuleConstants();
    public static SwerveModuleConstants BR = new SwerveModuleConstants();
    
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

    //the power of an affront to both god and alan turing is what makes this code possible.
    static {
        FL.withDriveMotorId(Module.FL.driveMotorID);
        FL.withSteerMotorId(Module.FL.steerMotorID);
        FL.withCANcoderId(Module.FL.cancoderID);
        FL.withCANcoderOffset(Module.FL.angleOffset);
        FL.withLocationX(Module.FL.centerOffset.getX());
        FL.withLocationY(Module.FL.centerOffset.getY());

        FR.withDriveMotorId(Module.FR.driveMotorID);
        FR.withSteerMotorId(Module.FR.steerMotorID);
        FR.withCANcoderId(Module.FR.cancoderID);
        FR.withCANcoderOffset(Module.FR.angleOffset);
        FR.withLocationX(Module.FR.centerOffset.getX());
        FR.withLocationY(Module.FR.centerOffset.getY());

        BL.withDriveMotorId(Module.BL.driveMotorID);
        BL.withSteerMotorId(Module.BL.steerMotorID);
        BL.withCANcoderId(Module.BL.cancoderID);
        BL.withCANcoderOffset(Module.BL.angleOffset);
        BL.withLocationX(Module.BL.centerOffset.getX());
        BL.withLocationY(Module.BL.centerOffset.getY());

        BR.withDriveMotorId(Module.BR.driveMotorID);
        BR.withSteerMotorId(Module.BR.steerMotorID);
        BR.withCANcoderId(Module.BR.cancoderID);
        BR.withCANcoderOffset(Module.BR.angleOffset);
        BR.withLocationX(Module.BR.centerOffset.getX());
        BR.withLocationY(Module.BR.centerOffset.getY());
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

    // PID TODO: tune
    public static final double kDriveKP = 0.025;
    public static final double kDriveKI = 0;
    public static final double kDriveKD = 0;

    public static final double kSteerKP = 0.4;
    public static final double kSteerKI = 0;
    public static final double kSteerKD = 0.5;
}
