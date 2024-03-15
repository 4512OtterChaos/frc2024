package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ShooterConstants {
    public static final int kLeftMotorID = 4;
    public static final int kRightMotorID = 5;

    public static final double kGearing = 18.0 / 24.0; // 0.75 motor rotations per wheel rotation

    public static final double kMaxMotorRPM = 5700;
    public static final double kToleranceRPM = 100;
    
    public static final double kShotDetectThresholdRPM = 100;

    public static final double kP = 0.01;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final SimpleMotorFeedforward kFF = new SimpleMotorFeedforward(
        0,
        0, // volts per radian
        0
    );
}
