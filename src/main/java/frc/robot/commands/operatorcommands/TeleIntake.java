package frc.robot.commands.operatorcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

import frc.robot.subsystems.intake.Intake;

public class TeleIntake extends CommandBase{

    private final Intake intake;
    private final Supplier<Double> percentSupplier;

    public TeleIntake(Intake intake, Supplier<Double> percentSupplier) {
        this.intake = intake;
        this.percentSupplier = percentSupplier;

        addRequirements(intake);
    }
    
    @Override
    public void execute() {
        double percent = percentSupplier.get();
        intake.runIntakePercent(percent);
    }

    @Override
    public void end(boolean interupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
