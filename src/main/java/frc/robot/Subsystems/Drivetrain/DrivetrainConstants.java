package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Translation2d;

public class DrivetrainConstants {
    
    public static final double kTrackWidth = 0;
    public static final double kTrackLength = 0;

    public static SwerveModuleConstants FL = new SwerveModuleConstants();
    public static SwerveModuleConstants FR = new SwerveModuleConstants();
    public static SwerveModuleConstants BL = new SwerveModuleConstants();
    public static SwerveModuleConstants BR = new SwerveModuleConstants();
    

    public enum Module {
        FL(1,1,2,1,0,kTrackWidth/2,kTrackLength/2),
        FR(2,3,4,0,0,-kTrackWidth/2,kTrackLength/2),
        BL(3,5,6,3,0,kTrackWidth/2,-kTrackLength/2),
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
    public DrivetrainConstants(){
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
    }
