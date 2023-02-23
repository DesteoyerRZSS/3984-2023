package frc.robot.subsystems;
import javax.management.ConstructorParameters;
import org.opencv.core.Mat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.Swerve.arm;
import frc.robot.Constants.Swerve.arm.Shoulder;


public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax shoulderMotor; 
    private CANSparkMax jointMotor;
    public RelativeEncoder EncoderShoulder;
    public RelativeEncoder EncoderJoint;
    public ArmFeedforward ShoulderFF;
    public ArmFeedforward JointFF;
    public SparkMaxPIDController ShoulderPID;
    public SparkMaxPIDController JointPID;
    
    public ArmSubsystem() {
        // Initialize feedforwards with the Ks Kg and Kv values.
        ShoulderFF = new ArmFeedforward(
            arm.Shoulder.Ks, 
            arm.Shoulder.Kg, 
            arm.Shoulder.Kv
        );
        JointFF = new ArmFeedforward(
            arm.Joint.Ks,
            arm.Joint.Kg, 
            arm.Joint.Kv
        );
        // Initialize the Motors
        shoulderMotor = new CANSparkMax(
            arm.Shoulder.rotMotorID, 
            MotorType.kBrushless
        );
        jointMotor = new CANSparkMax(
            arm.Joint.rotMotorID, 
            MotorType.kBrushless
        );
        // Initialize the built in motor encoders 
        EncoderShoulder = shoulderMotor.getEncoder(
            SparkMaxRelativeEncoder.Type.kHallSensor, 
            42
        );
        // Set the position to zero
        EncoderShoulder.setPosition(0);
        EncoderShoulder.setPositionConversionFactor(
            360/arm.Shoulder.gearRatio // in degrees
        );
        EncoderShoulder.setVelocityConversionFactor(
            (360 / arm.Shoulder.gearRatio) / 60.0 // in degrees/second
        );
        EncoderJoint = jointMotor.getEncoder(
            SparkMaxRelativeEncoder.Type.kHallSensor, 
            42
        );
        EncoderJoint.setPosition(0);
        EncoderJoint.setPositionConversionFactor(
            360/arm.Joint.gearRatio
        );
        EncoderShoulder.setVelocityConversionFactor(
            (360 / arm.Shoulder.gearRatio) / 60.0
        );
        // Initialize the built in Motor PID controllers
        ShoulderPID = shoulderMotor.getPIDController();
        ShoulderPID.setP(arm.Shoulder.Kp);
        ShoulderPID.setI(arm.Shoulder.Ki);
        ShoulderPID.setD(arm.Shoulder.Kd);

        /*JointPID = jointMotor.getPIDController();
        JointPID.setP(arm.Joint.Kp);
        JointPID.setI(arm.Joint.Ki);
        JointPID.setD(arm.Joint.Kd);*/
        
    }
    

    public Rotation2d[] getAngles(double x, double y){
        double AngleShoulder;
        double AngleJoint;
        Rotation2d[] angles = new Rotation2d[2];
        //double arm.Shoulder.Length;
		double hypotenuse = Math.hypot(x, y);
		AngleJoint = Math.pow(hypotenuse,  2) - Math.pow(arm.Shoulder.Length, 2) - Math.pow(arm.Joint.Length, 2);
		AngleJoint = AngleJoint/(-2*arm.Shoulder.Length*arm.Joint.Length);
		AngleJoint = Math.acos(AngleJoint);
        double theta2 = Math.pow(arm.Joint.Length,2)-Math.pow(arm.Shoulder.Length,2)-Math.pow(hypotenuse,2);
        theta2 = theta2/(-2*arm.Shoulder.Length*hypotenuse);
        double j = Math.atan(y/x);
        double k = theta2-j;
        AngleShoulder = ((Math.PI/2)-k);
        angles[0] = new Rotation2d(AngleShoulder);
        angles[1] = new Rotation2d(AngleJoint);
        return angles;
    }
    public void moveToAngle(Rotation2d angleShoulder, Rotation2d angleJoint){
        System.out.println("Trying to run PID loop");
        ShoulderPID.setReference(angleShoulder.getDegrees(), ControlType.kPosition, 0, ShoulderFF.calculate(angleShoulder.getRadians(), 0));
        //JointPID.setReference(angleJoint.getDegrees(), ControlType.kPosition, 0, JointFF.calculate(angleJoint.getRadians(), 0));
    }
    public Rotation2d[] getPosition(){
        Rotation2d[] poss = new Rotation2d[2];
        poss[0] = Rotation2d.fromDegrees( EncoderShoulder.getPosition());
        poss[1] = Rotation2d.fromDegrees( EncoderJoint.getPosition());
        return poss;
    }
    public Rotation2d[] getError(double goalShoulder, double goalJoint){
        Rotation2d[] poss = new Rotation2d[2];
        poss[0] = Rotation2d.fromDegrees( goalShoulder - getPosition()[0].getDegrees());
        poss[1] = Rotation2d.fromDegrees( goalJoint - getPosition()[1].getDegrees());
        return poss;
    }
    public Command moveTo(double[] point){
        Rotation2d[] angles = getAngles(point[0], point[1]);
        return run(() -> 
            moveToAngle(
                angles[0], 
                angles[1]
            )
        ).until(()->
            (Math.abs(getError(angles[0].getDegrees(), angles[1].getDegrees())[0].getDegrees())  < 2) //&& 
            //(Math.abs(getError(angles[0].getDegrees(), angles[1].getDegrees())[1].getDegrees())  < 2) 
        );
    }
    public void periodic(){
        SmartDashboard.putNumber("EncoderShoulder", getPosition()[0].getDegrees());
        

    }
    /*public void getNewPoint(){
        goal[0] = goal[0] + 0.05;
        goal[1] = goal[0] + 0.05;
    }*/
}
