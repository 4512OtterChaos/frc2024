package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.util.FieldUtil;

/**
 * A lookup table for finding the "best" state for shooting at any given distance.
 */
public abstract class ShotMap {
    /**
     * The state of a potential shot at any moment. This includes the left and right shooter wheel speeds,
     * and the angle of the shooter.
     */
    public static class State implements Interpolatable<State> {
        public final double leftRPM;
        public final double rightRPM;
        public final Rotation2d armAngle;
        public final double timeofflight;

        public State(double leftRPM, double rightRPM, Rotation2d armAngle, double timeofflight) {
            this.leftRPM = leftRPM;
            this.rightRPM = rightRPM;
            this.armAngle = armAngle;
            this.timeofflight = timeofflight;
        }

        @Override
        public State interpolate(State endValue, double t) {
            return interpolate(this, endValue, t);
        }

        public static State interpolate(State startValue, State endValue, double t) {
            if (t <= 0) {
                return startValue;
            } else if (t >= 1) {
                return endValue;
            } else {
                return new State(
                    MathUtil.interpolate(startValue.leftRPM, endValue.leftRPM, t),
                    MathUtil.interpolate(startValue.leftRPM, endValue.rightRPM, t),
                    startValue.armAngle.interpolate(endValue.armAngle, t),
                    MathUtil.interpolate(startValue.timeofflight, endValue.timeofflight, t)
                );
            }
        }
    }

    //----------

    private static final InterpolatingTreeMap<Double, ShotMap.State> map =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), (a, b, t) -> State.interpolate(a, b, t));
    private static double rpmOffset = 0;

    public static final State kIdle = new State(0, 0, ArmConstants.kHomeAngle, 0);
    public static final State kSubwoofer = new State(3500, 3000, ArmConstants.kHomeAngle, 0.15);
    public static final State kAmp = new State(1000, 1000, Rotation2d.fromDegrees(90), 0.5);
    
    static {
        // TODO: tune numbers
        map.put(
            // Subwoofer shot
            FieldUtil.kWallToSubwooferFrontDist + SwerveConstants.kRobotBackHalfLength,
            kSubwoofer
        );
        map.put(2.0, new State(4500, 4000, Rotation2d.fromDegrees(-3), 0.2));
        map.put(3.0, new State(4750, 4250, Rotation2d.fromDegrees(9), 0.25));
        map.put(4.0, new State(5000, 4500, Rotation2d.fromDegrees(14), 0.3));
        map.put(5.0, new State(5250, 4750, Rotation2d.fromDegrees(18), 0.35));
    }

    // public static void setRPMOffset(double rpm){
    //     rpmOffset = rpm;
    // }

    public static State find(double distanceMeters) {
        return map.get(distanceMeters);
    }
}
