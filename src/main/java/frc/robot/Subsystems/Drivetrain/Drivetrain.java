package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;


import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;


public class Drivetrain extends SubsystemBase {
    SwerveModule[] swerveMods = {
        new SwerveModule(FL, getName()),
        new SwerveModule(FR, getName()),
        new SwerveModule(BL, getName()),
        new SwerveModule(BR, getName())
    };
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    DrivetrainConstants.Module.FL.centerOffset,
    DrivetrainConstants.Module.FR.centerOffset,
    DrivetrainConstants.Module.BL.centerOffset,
    DrivetrainConstants.Module.BR.centerOffset
    
    );


    public   void drive(double vX,double vY,double omegaDegrees){

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
    
}
