package frc.robot.subsystems.shooter;

import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.util.FieldUtil;

public final class ShotMap {

    private static final TreeMap<Double, ShotMapEntry> map = new TreeMap<>();
    public static final Shooter.State kSubwoofer = new Shooter.State(1500, 1500, 25); //TODO: Fix numbers
    private static double rpmOffset = 0;
    static {
        // shooter shot states at distance in inches
        map.put(
            // Subwoofer shot
            Units.metersToInches(FieldUtil.kWallToSubwooferFrontDist+SwerveConstants.kRobotWidth/2.0), //TODO: fix numbers
            new ShotMapEntry(2900, 2900, 0, 1.5)
        );
        map.put(100.0, new ShotMapEntry(2600, 2600, 23, 1.5)); //TODO: fix numbers
        map.put(200.0, new ShotMapEntry(3400, 3400, 41, 1.75)); //TODO: fix numbers
        map.put(300.0, new ShotMapEntry(4800, 4800, 53, 2)); //TODO: fix numbers
    }

    private static ShotMapEntry findEntry(double distanceMeters){
        double distanceInches = Units.metersToInches(distanceMeters);
        distanceInches = MathUtil.clamp(distanceInches, map.firstKey(), map.lastKey());
        // close side nearest map entry
        double closeDist = map.floorKey(distanceInches);
        // far side nearest map entry
        double farDist = map.ceilingKey(distanceInches);
        double closeToFar = farDist - closeDist;
        double closeToTarget = distanceInches - closeDist;

        if(closeToFar == 0) return map.get(closeDist);
        return map.get(closeDist).interpolate(map.get(farDist), closeToTarget / closeToFar).plus(new ShotMapEntry(rpmOffset, rpmOffset, 0, 0));
    }
    /**
     * Finds shooter state appropriate for shooting into the high goal at the given distance
     * @param distanceMeters Distance in meters from the center of the hub
     */
    public static Shooter.State find(double distanceMeters){
        return findEntry(distanceMeters).state;
    }
    /**
     * Finds the time-of-flight in seconds for the cargo if shot from the given distance
     * with the corresponding shooter state.
     * @param distanceMeters Distance in meters from the center of the hub
     * @return Time-of-flight of the cargo in seconds (exit shooter to contact hub)
     */
    public static double findToF(double distanceMeters){
        return findEntry(distanceMeters).tof;
    }
    public static void setRPMOffset(double rpm){
        rpmOffset = rpm;
    }

    private static class ShotMapEntry implements Interpolatable<ShotMapEntry>{
        public final Shooter.State state;
        public final double tof;

        /**
         * A shooter state with an additional time-of-flight for
         * estimating note speed.
         * @param tof Time-of-flight seconds at given distance
         */
        public ShotMapEntry(double leftRPM, double rightRPM, double armAngle, double tof){
            this.state = new Shooter.State(leftRPM, rightRPM, armAngle);
            this.tof = tof;
        }
        public ShotMapEntry plus(ShotMapEntry other){
            Shooter.State newState = other.state.plus(state);
            return new ShotMapEntry(newState.leftRPM, newState.rightRPM, newState.armDeg,tof);
        }

        @Override
        public ShotMapEntry interpolate(ShotMapEntry endValue, double t) {
            if (t < 0) {
                return this;
            } else if (t >= 1) {
                return endValue;
            } else {
                Shooter.State newState = state.interpolate(endValue.state, t);
                return new ShotMapEntry(
                    newState.leftRPM,
                    newState.rightRPM,
                    newState.armDeg,
                    MathUtil.interpolate(tof, endValue.tof, t)
                );
            }
        }
    }
}
