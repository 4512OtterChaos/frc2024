package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;


import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;


public class Drivetrain extends SubsystemBase {
    private SwerveModule[] swerveMods = {
        new SwerveModule(FL, getName()),
        new SwerveModule(FR, getName()),
        new SwerveModule(BL, getName()),
        new SwerveModule(BR, getName())
    };
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    DrivetrainConstants.Module.FL.centerOffset,
    DrivetrainConstants.Module.FR.centerOffset,
    DrivetrainConstants.Module.BL.centerOffset,
    DrivetrainConstants.Module.BR.centerOffset
    
    );

    SwerveDriveAccelLimiter limiter = new SwerveDriveAccelLimiter(kLinearAcceleration, kLinearDeceleration, kRotationalAcceleration, kRotationalDeceleration);

    private ChassisSpeeds lastTargetSpeeds = new ChassisSpeeds();


    public void drive(double vX,double vY,double omegaDegrees){

        ChassisSpeeds targetSpeeds = new ChassisSpeeds(vX, vY, omegaDegrees);
        targetSpeeds = limiter.calculate(targetSpeeds, lastTargetSpeeds, 0.02);
        lastTargetSpeeds = targetSpeeds;
        setChassisSpeeds(targetSpeeds);
    }
    

    public void setSwerveStates(SwerveModuleState[] states){
        for (int i = 0; i < swerveMods.length; i++) {
            swerveMods[i].apply(states[i], null);
        }
    }

    public void setChassisSpeeds(double vX, double vY, double omegaDegrees){
        setSwerveStates(kinematics.toSwerveModuleStates(new ChassisSpeeds(vX, vY, omegaDegrees)));
        
    }

    public void setChassisSpeeds(ChassisSpeeds targetSpeeds){
        setSwerveStates(kinematics.toSwerveModuleStates(targetSpeeds));
        
    }

    public Command CDrive(double vX,double vY,double omegaDegrees){
        return run(()->drive(vX, vY, omegaDegrees));
    }
    
}
