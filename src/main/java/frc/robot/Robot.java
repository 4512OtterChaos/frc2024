// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.AutoOptions;
import frc.robot.subsystems.ShotMap;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.OCXboxController;

public class Robot extends TimedRobot {
    private SwerveDrive swerve = new SwerveDrive();
    // private Climber climber = new Climber();
    private Intake intake = new Intake();
    private Feeder feeder = new Feeder();
    private Arm arm = new Arm();
    // private Arm arm = null;
    private Shooter shooter = new Shooter();

    private Vision vision = new Vision();

    private Superstructure superstructure = new Superstructure(swerve, intake, arm, shooter, feeder);

    private AutoOptions autos = new AutoOptions(swerve, intake, arm, shooter, feeder, superstructure);
    
    private OCXboxController driver = new OCXboxController(0);
    private OCXboxController operator = new OCXboxController(1);

    @Override
    public void robotInit() {
        configureDriverBinds(driver);
        // configureOperatorBinds(operator);

        //TODO: plug usb drive into roborio
        DataLogManager.start();
        SignalLogger.start();

        shooter.setDefaultCommand(shooter.stopC());
        arm.setDefaultCommand(arm.setRotationC(ShotMap.kIdle));
        feeder.setDefaultCommand(feeder.setVoltageC(0));
        // climber.setDefaultCommand(climber.holdPositionC());
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        autos.periodic();
    }
    
    @Override
    public void autonomousInit() {
        autos.getAuto().schedule();
    }
    
    @Override
    public void autonomousPeriodic() {}
    
    @Override
    public void teleopInit() {}
    
    @Override
    public void teleopPeriodic() {}
    
    @Override
    public void disabledInit() {}
    
    @Override
    public void disabledPeriodic() {}
    
    @Override
    public void testInit() {}
    
    @Override
    public void testPeriodic() {}

    //----- Bindings

    private void configureDriverBinds(OCXboxController controller){
        swerve.setDefaultCommand(
            run(()->{
                    int invert=1;
                    if (swerve.getIsDrivingMirrored()){
                        invert=-1;
                    }
                    swerve.drive(
                        controller.getForward()*swerve.getMaxLinearVelocityMeters()*invert,
                        controller.getStrafe()*swerve.getMaxLinearVelocityMeters()*invert,
                        controller.getTurn()*swerve.getMaxAngularVelocityRadians(),
                        true
                    );
                },
                swerve
            )
        );

        // RIGHT TRIGGER: INTAKE
        controller.rightTrigger().whileTrue(superstructure.intake().alongWith(runOnce(()->driver.rumble(0))).andThen(run(()->driver.rumble(0.5)).asProxy().withTimeout(0.3).finallyDo(()->driver.rumble(0))));

        // RIGHT BUMPER ONLY: SUBWOOFER
        controller.rightBumper().and(controller.leftBumper().negate())
                .whileTrue(superstructure.shootSubwoof());
        // LEFT BUMPER: ARM TO AMP
        // Runs until let go
        controller.leftBumper().whileTrue(
            parallel(
                arm.setRotationC(ShotMap.kAmp),
                idle() // hold rotation until button is let go
            )
        );
        // LEFT AND RIGHT BUMPERS: SHOOT AMP
        // Runs until let go
        controller.rightBumper().and(controller.leftBumper()).whileTrue(
            sequence(
                shooter.setVoltageC(3, 3),
                waitSeconds(0.25),
                feeder.setVoltageInC(),
                idle()
            )
        );
        // LEFT TRIGGER: AUTO SHOOT
        // controller.leftTrigger()
    
        // controller.povUp().onTrue(climber.CSetMaxHeight());
        // controller.povDown().onTrue(climber.CSetMinHeight());

        controller.povLeft().whileTrue(arm.setVoltageC(2).repeatedly().finallyDo(()->arm.setVoltage(0)));
        controller.povRight().whileTrue(arm.setVoltageC(-2).repeatedly().finallyDo(()->arm.setVoltage(0)));

        controller.leftStick()
            .whileTrue(superstructure.outtake());        

        // controller.y()
        //     .whileTrue(climber.setVoltageUpC().repeatedly());

        // controller.x().and(controller.b().negate())
        //     .whileTrue(climber.setVoltageUpLeftC().repeatedly());
        // controller.b().and(controller.x().negate())
        //     .whileTrue(climber.setVoltageUpRightC().repeatedly());

        // controller.a()
        //     .whileTrue(climber.setVoltageDownC().repeatedly());
                
        // reset the robot heading forward
        controller.start()
            .onTrue(runOnce(()->
                swerve.resetOdometry(
                    new Pose2d(
                        swerve.getPose().getTranslation(),
                        swerve.getIsDrivingMirrored() ? new Rotation2d(Math.PI) : new Rotation2d()
                    )
                )
            )
        );

        // measure drive wheel rotations versus gyro rotation
        controller.back().whileTrue(sequence(
            swerve.runOnce(()->{
                swerve.stop();
            }),
            waitSeconds(0.5),
            swerve.runOnce(()->{
                swerve.zeroGyro();
                swerve.zeroModulePositions();
            }),
            waitSeconds(0.5),
            swerve.run(()->{
                swerve.drive(0, 0, 4, false);
            })
        ));
    }

    private void configureOperatorBinds(OCXboxController controller) {
        // controller.povUp().onTrue(climber.CSetMaxHeight());
        // controller.povDown().onTrue(climber.CSetMinHeight());

        // controller.y()
        //     .whileTrue(climber.setVoltageUpC().repeatedly());

        // controller.x().and(controller.b().negate())
        //     .whileTrue(climber.setVoltageUpLeftC().repeatedly());
        // controller.b().and(controller.x().negate())
        //     .whileTrue(climber.setVoltageUpRightC().repeatedly());

        // controller.a()
        //     .whileTrue(climber.setVoltageDownC().repeatedly());
    }


    //----- Simulation

    @Override
    public void simulationInit() {}
    
    @Override
    public void simulationPeriodic() {
        // Update camera simulation
        // TODO: dont use the pose estimator pose
        vision.simulationPeriodic(swerve.getPose());

        // Calculate battery voltage sag due to current draw
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(swerve.getCurrentDraw()));
    }
}
