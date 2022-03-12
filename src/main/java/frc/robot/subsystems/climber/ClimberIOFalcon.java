package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.opencv.core.Mat;

public class ClimberIOFalcon implements ClimberIO {
   
    public static final int leftClimberPort = 15;
    public static final int rightClimberPort = 16;
    
    private final WPI_TalonFX climberMotor;

    public ClimberIOFalcon(int port){
        this.climberMotor = new WPI_TalonFX(port);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        NeutralMode mode = enable ? NeutralMode.Brake : NeutralMode.Coast;
        climberMotor.setNeutralMode(mode);
    }
    
    @Override
    public void setVelocity(double velRadPerSec){
        double velTicksPerSecond = (velRadPerSec*2048)/(Math.PI*2);
        climberMotor.set(ControlMode.Velocity, velTicksPerSecond);
        System.out.println(velTicksPerSecond);
    }

    @Override
    public void setPercentPower(double power){
        climberMotor.set(ControlMode.PercentOutput, power);
    }
    
    @Override
    public void setTargetPoint(double degrees){
        // Convert degrees to motor values
        climberMotor.set(ControlMode.Position, degrees);
    }
}
