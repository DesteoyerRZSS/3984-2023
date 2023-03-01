package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
public class Intake extends CommandBase{
    private BooleanSupplier In;
    private BooleanSupplier Out;
    private Claw claw;
    public Intake(Claw claw, BooleanSupplier In, BooleanSupplier Out){
        this.claw = claw;
        addRequirements(claw);
        this.In = In;
        this.Out = Out;
    }
    public void execute(){
        boolean in = In.getAsBoolean();
        boolean out = Out.getAsBoolean();
        if (in){
            claw.Intake();
        }
        else if (out){
            claw.Outtake();
        }
        else{
            claw.Stop();
        }

    }

}
