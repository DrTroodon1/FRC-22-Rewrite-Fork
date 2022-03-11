package frc.robot.commands.operatorcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class TeleClimbDown extends CommandBase{
    
    // r 16

    private final Climber climber;

    public TeleClimbDown(Climber climber){
        this.climber = climber;
        climber.setBrakeMode(true);
    }

    @Override
    public void execute(){
        climber.setLeftPercent(-0.7);
        climber.setRightPercent(-0.7);
    }

    @Override
    public void end(boolean interrupted){
        climber.setLeftVelocity(0);
        climber.setRightVelocity(0);
    }

    public boolean isFinished(){
        return false;
    }

}
