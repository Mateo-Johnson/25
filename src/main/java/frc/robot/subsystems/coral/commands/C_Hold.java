package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.lights.Lights;

public class C_Hold extends Command {
    private final Coral intake;
    private final Lights lights;
    
    public C_Hold(Coral intake, Lights lights) {
        this.intake = intake;
        this.lights = lights;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        intake.hold();
        lights.setColor("SeaGreen");
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop if interrupted
        intake.stop();
    }
}