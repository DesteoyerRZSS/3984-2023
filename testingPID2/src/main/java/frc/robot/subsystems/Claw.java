package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Claw extends SubsystemBase{
    private Spark rightMotor;
    private Spark leftMotor;
    private boolean yes;
    private boolean neutral;
    public Claw() {
        rightMotor = new Spark(11);
        leftMotor= new Spark(12);
        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
        yes = false;
        neutral = true;
    }

    public void Intake(){ 
        rightMotor.set(1);
        leftMotor.set(1);
        yes = true;
        neutral = false;
    }
    public void Outtake(){ 
        rightMotor.set(-1);
        leftMotor.set(-1);
        yes = false;
        neutral = false;
    }
    public void periodic(){
        if (neutral){
            SmartDashboard.putString("Claw State", "Neutral");
        }
        else{
            if (yes == true){
                SmartDashboard.putString("Claw State", "In");
            }
            else if (yes == false){
                SmartDashboard.putString("ClawState", "Out");
            }

        }
    }
    /*public void getNewPoint(){
        goal[0] = goal[0] + 0.05;
        goal[1] = goal[0] + 0.05;
    }*/
}
