package frc.robot;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.AutoOptions;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superShooter.arm.Arm;
import frc.robot.subsystems.superShooter.feeder.Feeder;
import frc.robot.subsystems.superShooter.shooter.Shooter;

public class RobotContainer {
    private Drivetrain drivetrain = new Drivetrain();
    private Climber climber = new Climber();
    private Intake intake = new Intake();
    private Feeder feeder = new Feeder();
    private Shooter shooter = new Shooter();
    private Arm arm = new Arm();
    private AutoOptions autos = new AutoOptions(drivetrain,intake,arm,shooter,feeder);
    private CommandXboxController driver = new CommandXboxController(0);
    
    public RobotContainer(){
        configureDriverBinds();
    }

    public Command getAuto(){
        return autos.getAuto();
    }


    private void configureDriverBinds(){
        drivetrain.setDefaultCommand(drivetrain.CDrive(driver.getLeftX(), driver.getLeftY(), driver.getRightX()));
        intake.setDefaultCommand(intake.setVoltageC(0));
        climber.setDefaultCommand(null);
    








        driver.rightTrigger().whileTrue(shooter.CShootTable(2));
        driver.leftTrigger().whileTrue(intake.setVoltageInC());

        driver.leftStick().whileTrue(intake.setVoltageOutC());

        driver.x().onTrue(arm.CSetAngle(0));
        driver.a().whileTrue(shooter.CShootSubwoof());
        driver.b().whileTrue(shooter.CShootAmp());

        driver.povUp().onTrue(climber.CSetMaxHeight());
        driver.povDown().onTrue(climber.CSetMinHeight());
    }   


}
