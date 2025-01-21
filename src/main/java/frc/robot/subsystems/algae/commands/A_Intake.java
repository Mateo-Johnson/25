package frc.robot.subsystems.algae.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.lights.Lights;

public class A_Intake extends Command {
    private final Algae intake;
    private final Lights lights;
    
    public A_Intake(Algae intake, Lights lights) {
        this.intake = intake;
        this.lights = lights;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        intake.intake();
    }

    @Override
    public void execute() {
        lights.flash("Yellow", 2);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Switch to hold mode when we stop intaking
        intake.hold();
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}