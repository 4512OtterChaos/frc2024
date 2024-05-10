package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Utility methods for using the Phoenix library with Falcon 500s
 */
public class TalonUtil {

    /**
     * Configures the status frame periods of given motors
     * @return Success
     */
    public static boolean configStatusSolo(TalonFX... motors){
        return configStatus1(20, 20) == StatusCode.OK;
    }
    /**
     * Configures the status frame periods of given motors
     * @return Success
     */
    public static boolean configStatusFollower(TalonFX... motors){
        boolean success = true;
        StatusCode result1 = configStatus1(255, 20, motors);
        StatusCode result2 = configStatus2(255, 20, motors);
        success = success && (result1 == StatusCode.OK) && (result2 == StatusCode.OK);
        return success;
    }
    /**
     * Configures the status frame periods of given motors
     * @return Success
     */
    public static boolean configStatusCurrent(TalonFX... motors){
        boolean success = true;
        for(TalonFX motor : motors){
            StatusCode result = motor.getStatorCurrent().setUpdateFrequency(50, .02);
            success = success && (result == StatusCode.OK);
        }
        return success;
    }
    /**
     * Configures the status frame periods of given motors
     * @return Success
     */
    public static boolean configStatusSim(TalonFX... motors){
        boolean success = true;
        StatusCode result1 = configStatus2(10, 20, motors);
        // StatusCode result2 = motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, 20);//TODO: Make this work/update from phoenix 5 to 6
        success = success && (result1 == StatusCode.OK); //&& (result2 == StatusCode.OK);
        return success;
    }


    private static StatusCode configStatus1(double periodMs, double timeoutMs, TalonFX... motors){//TODO: Status 1 DOESN'T include limit switches 
        StatusCode success = StatusCode.OK;
        double frequencyHz = 1000/periodMs;
        double timeoutSec = timeoutMs*.001;
        for(TalonFX motor : motors){
            StatusCode result1_1 = motor.getClosedLoopOutput().setUpdateFrequency(frequencyHz, timeoutSec);
            StatusCode result1_2 = motor.getFaultField().setUpdateFrequency(frequencyHz, timeoutSec);
            if ((success == StatusCode.OK) && (result1_1 == StatusCode.OK) && (result1_2 == StatusCode.OK)){
                success = StatusCode.OK;
            }
            else{
                success = null;
            }
        }
        return success;
    }

    private static StatusCode configStatus2(double periodMs, double timeoutMs, TalonFX... motors){
        StatusCode success = StatusCode.OK;
        double frequencyHz = 1000/periodMs;
        double timeoutSec = timeoutMs*.001;
        for(TalonFX motor : motors){
            StatusCode result1 = motor.getPosition().setUpdateFrequency(frequencyHz, timeoutSec);
            StatusCode result2 = motor.getVelocity().setUpdateFrequency(frequencyHz, timeoutSec);
            StatusCode result3 = motor.getSupplyCurrent().setUpdateFrequency(frequencyHz, timeoutSec);
            StatusCode result4 = motor.getStickyFaultField().setUpdateFrequency(frequencyHz, timeoutSec);
            if ((success == StatusCode.OK) && (result1 == StatusCode.OK) && (result2 == StatusCode.OK) && (result3 == StatusCode.OK) && (result4 == StatusCode.OK)){
                success = StatusCode.OK;
            }
            else{
                success = null;
            }
        }
        return success;
    }


    //Conversions yuck
    public static double positionToRotations(double nativePosition, double motorRotationsPerMechanismRotation){
        return nativePosition / 2048 / motorRotationsPerMechanismRotation;
    }
    public static double positionToDegrees(double nativePosition, double motorRotationsPerMechanismRotation){
        return positionToRotations(nativePosition, motorRotationsPerMechanismRotation) * 360;
    }
    public static double positionToRadians(double nativePosition, double motorRotationsPerMechanismRotation){
        return positionToRotations(nativePosition, motorRotationsPerMechanismRotation) * 2 * Math.PI;
    }
    public static double positionToMeters(double nativePosition, double motorRotationsPerMechanismRotation, double circumferenceMeters){
        return positionToRotations(nativePosition, motorRotationsPerMechanismRotation) * circumferenceMeters;
    }
    public static double rotationsToPosition(double rotations, double motorRotationsPerMechanismRotation){
        return rotations * 2048 * motorRotationsPerMechanismRotation;
    }
    public static double degreesToPosition(double degrees, double motorRotationsPerMechanismRotation){
        return rotationsToPosition(degrees/(360), motorRotationsPerMechanismRotation);
    }
    public static double radiansToPosition(double radians, double motorRotationsPerMechanismRotation){
        return rotationsToPosition(radians/(2*Math.PI), motorRotationsPerMechanismRotation);
    }
    public static double metersToPosition(double meters, double motorRotationsPerMechanismRotation, double circumferenceMeters){
        return rotationsToPosition(meters/(circumferenceMeters), motorRotationsPerMechanismRotation);
    }

    public static double velocityToRotations(double nativeVelocity, double motorRotationsPerMechanismRotation){
        return nativeVelocity * 10 / 2048 / motorRotationsPerMechanismRotation;
    }
    public static double velocityToDegrees(double nativeVelocity, double motorRotationsPerMechanismRotation){
        return velocityToRotations(nativeVelocity, motorRotationsPerMechanismRotation) * 360;
    }
    public static double velocityToRadians(double nativeVelocity, double motorRotationsPerMechanismRotation){
        return velocityToRotations(nativeVelocity, motorRotationsPerMechanismRotation) * 2 * Math.PI;
    }
    public static double velocityToMeters(double nativeVelocity, double motorRotationsPerMechanismRotation, double circumferenceMeters){
        return velocityToRotations(nativeVelocity, motorRotationsPerMechanismRotation) * circumferenceMeters;
    }
    public static double rotationsToVelocity(double rotations, double motorRotationsPerMechanismRotation){
        return rotations * 2048 / 10 * motorRotationsPerMechanismRotation;
    }
    public static double degreesToVelocity(double degrees, double motorRotationsPerMechanismRotation){
        return rotationsToVelocity(degrees/(360), motorRotationsPerMechanismRotation);
    }
    public static double radiansToVelocity(double radians, double motorRotationsPerMechanismRotation){
        return rotationsToVelocity(radians/(2*Math.PI), motorRotationsPerMechanismRotation);
    }
    public static double metersToVelocity(double meters, double motorRotationsPerMechanismRotation, double circumferenceMeters){
        return rotationsToVelocity(meters/(circumferenceMeters), motorRotationsPerMechanismRotation);
    }
}
