package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.lights.Lights;

public class C_Intake extends Command {
    private final Coral intake;
    private final Lights lights;
    
    public C_Intake(Coral intake, Lights lights) {
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