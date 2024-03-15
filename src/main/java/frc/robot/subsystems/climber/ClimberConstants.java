package frc.robot.subsystems.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ClimberConstants {
    public static final int kLeftMotorID = 6;
    public static final int kRightMotorID = 7;
    public static final boolean kMotorInverted = false;

    public static final double kTopHeightRotations = 144; // motor rotations
    public static final double kBottomHeightRotations = 0;
    public static final double kToleranceRotations = 1;

    public static final double kP = .3;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(55, 75);

    public static final int kContinuousCurrentLimit = 35;
    public static final int kPeakCurrentLimit = 50;
    public static final double kVoltageSaturation = 12;
    public static final int kCANTimeout = 100;

    public static final double kVoltageUp = 10;
    public static final double kVoltageDown = -10;
}
