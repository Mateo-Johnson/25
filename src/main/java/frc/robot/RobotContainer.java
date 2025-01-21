package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.coral.commands.C_Intake;
import frc.robot.subsystems.algae.commands.A_Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.Constants.OIConstants;

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;

    // The robot's subsystems
    private final Drivetrain drivetrain = new Drivetrain(); // Drivetrain Subsystem
    private final Lights lights = new Lights();  // Lights Subsystem
    private final Coral coral = new Coral(); // Coral Subsystem
    private final Algae algae = new Algae(); // Algae Subsystem
    // private final Vision vision = new Vision(drivetrain); // Vision Subsystem

    // The driver's controller
    public static final CommandXboxController primary = new CommandXboxController(OIConstants.primaryPort);

    // Speed factor to reduce drive speed (e.g., 50% speed)
    private static final double slowFactor = 0.5;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        registerNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        drivetrain.setDefaultCommand(
            new RunCommand(
                () -> {
                    // Get the controller inputs
                    double ySpeed = -MathUtil.applyDeadband(primary.getLeftY(), OIConstants.driveDeadband);
                    double xSpeed = -MathUtil.applyDeadband(primary.getLeftX(), OIConstants.driveDeadband);
                    double rot = -MathUtil.applyDeadband(primary.getRightX(), OIConstants.driveDeadband);

                    // Apply slow factor if B button is pressed
                    if (primary.b().getAsBoolean()) {  // Fixed: Use b().getAsBoolean() instead of getBButton()
                        ySpeed *= slowFactor;
                        xSpeed *= slowFactor;
                        rot *= slowFactor;
                    }

                    drivetrain.drive(ySpeed, xSpeed, rot, true);
                },
                drivetrain));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link Trigger} class.
     */
    private void configureButtonBindings() {
        // Example of how to bind a button to a command using the new command-based framework
        //primary.a()
        //     .whileTrue(new ExampleCommand());
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("IntakeCoral", new C_Intake(coral, lights));
        NamedCommands.registerCommand("IntakeAlgae", new A_Intake(algae, lights));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * This method is called when the autonomous period begins.
     * Start flashing the lights blue here.
     */
    public void autonomousInit() {
        // Start flashing blue during autonomous
        lights.flash("Blue", 2);
    }

    /**
     * This method is called periodically during the autonomous period.
     * Keep calling the flashing method to update the lights.
     */
    public void autonomousPeriodic() {
        // Ensure flashing is updated periodically during autonomous
        lights.flash("Blue", 2);  // Flash blue at a rate of 2
    }

    /**
     * Stop the flashing or lights when teleop starts.
     */
    public void teleopInit() {
        lights.turnOff();  // Turn off the lights when teleop starts
    }
}