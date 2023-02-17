package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmSubsystem;

public class Arm extends CommandBase{
    private ArmSubsystem Armm;
    private BooleanSupplier Intake;
    private BooleanSupplier Low;
    private BooleanSupplier Medium;
    private BooleanSupplier High;
    private BooleanSupplier Retract;
    private DoubleSupplier JoyFL;
    private DoubleSupplier JoyFR;

    //Constructor for command
    public Arm (ArmSubsystem Armm, DoubleSupplier JoyFL, DoubleSupplier JoyFR){
        this.Armm = Armm;
        addRequirements(Armm);
        this.JoyFL = JoyFL;
        this.JoyFR = JoyFR;

    }
    public void execute(){
    Armm.moveToAngle(JoyFL.getAsDouble(), JoyFR.getAsDouble());
    }

}
