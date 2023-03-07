package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ManualArm extends CommandBase {
    private DoubleSupplier speedJoint;
    private DoubleSupplier speedShoulder;
    private Arm arm;
    
    public ManualArm(Arm arm, DoubleSupplier joint, DoubleSupplier shoulder){
        this.arm = arm;
        speedJoint = joint;
        speedShoulder = shoulder;

    }
    @Override
    public void execute() {
        double Jspeed = speedJoint.getAsDouble();
        double Sspeed = speedShoulder.getAsDouble();
        arm.ManualControl(Jspeed, Sspeed);
        // TODO Auto-generated method stub
        super.execute();
    }

}
