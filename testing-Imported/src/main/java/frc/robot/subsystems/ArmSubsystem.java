package frc.robot.subsystems;
import org.opencv.core.Mat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
    public double ShoulderValue = 0;
    public double JointValue = 0;
    // Initialize the goal point for the arm.
    private double[] goal = new double[2];
    private double currAngle = 0;
    public PIDController goToAngleShoulder =
     new PIDController(Constants.Swerve.sarmKP, 
                        Constants.Swerve.sarmKI, 
                        Constants.Swerve.sarmKD);
    public PIDController goToAngleJoint = 
    new PIDController(Constants.Swerve.jarmKP,
                        Constants.Swerve.jarmKI,
                        Constants.Swerve.jarmKD);
    public ArmFeedforward Shoulderff = 
    new ArmFeedforward(Constants.Swerve.sarmKS,
                        Constants.Swerve.sarmKG, 
                        Constants.Swerve.sarmKV, 
                        Constants.Swerve.sarmKA);
    public double PIDShoulder = 0;
    public double PIDJoint = 0;
    public ArmFeedforward Jointff = new ArmFeedforward(0, 0, 0, 0);
    public ArmSubsystem() {
        shoulderMotor = new CANSparkMax(arm.Shoulder.rotMotorID, MotorType.kBrushless);
        jointMotor = new CANSparkMax(arm.Joint.rotMotorID, MotorType.kBrushless);
        EncoderShoulder = shoulderMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        EncoderShoulder.setPosition(0);
        EncoderJoint = jointMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        shoulderMotor.setInverted(false);
        EncoderJoint.setPosition(0);
        EncoderJoint.setPositionConversionFactor(2*Math.PI);
    }
    public void moveToAngle(){
        Rotation2d[] angles = new Rotation2d[2];
        //get angles needed for each joint
        
        //ignore this
        if (goal[0] == 0 && goal[1] == 0){
            angles[0] = Rotation2d.fromRadians(0);
            angles[1] = Rotation2d.fromRadians(0);
        }
        else{
            angles = this.getAngles(goal[0], goal[1]);
        }
        double angle = angles[0].getRadians();
        goToAngleShoulder.setSetpoint(angle);
        goToAngleShoulder.setTolerance(1, 0.1);
        SmartDashboard.putNumber("Angle Goal Shoulder", angle);

        angle = angles[1].getRadians();
        goToAngleJoint.setSetpoint(angle);
        goToAngleJoint.setTolerance(1, 0.1);
        SmartDashboard.putNumber("Angle Goal Shoulder", angle);
        
        SmartDashboard.putNumberArray("GoalPoint: ", goal);
      /*   if (!goToAngleJoint.atSetpoint()/*  && !goToAngleShoulder.atSetpoint()/){
             shoulderMotor.setVoltage(-PIDShoulder 
                                    /*+ Shoulderff.calculate(ShoulderValue- arm.Shoulder.angleOffset.getRadians()
                                    , EncoderShoulder.getVelocity())*.12);
            SmartDashboard.putNumber("ShoulderV", PIDShoulder);
            jointMotor.setVoltage((goToAngleJoint.calculate(JointValue - arm.Joint.angleOffset.getRadians())
                                    + Jointff.calculate(JointValue- arm.Joint.angleOffset.getRadians()
                                    , EncoderJoint.getVelocity()))*.12);
        }*/
        if ((ShoulderValue/(2*Math.PI)) * 360 < 4500){
            shoulderMotor.set(difference);
            
        }
        else{
            stop();
        }

    }
    /************************************************/
    public Rotation2d[] getAngles(double x, double y){
        double AngleShoulder = 0;
        double AngleJoint = 0;
        Rotation2d[] angles = new Rotation2d[2];
        //Implement get angle code here Jouji
        double arm1Length = 34;
        
		double jointLength = 23.5;
		double hypotenuse = Math.hypot(x, y);

		AngleJoint = Math.pow(hypotenuse,  2) - Math.pow(arm1Length, 2) - Math.pow(jointLength, 2);

		AngleJoint = AngleJoint/(-2*arm1Length*jointLength);

		AngleJoint = Math.acos(AngleJoint);
		
        
        double theta2 = Math.pow(jointLength,2)-Math.pow(arm1Length,2)-Math.pow(hypotenuse,2);

        theta2 = theta2/(-2*arm1Length*hypotenuse);

        double j = Math.atan(0/23);

        double k = theta2-j;

        AngleShoulder = ((Math.PI/2)-k);
        
        //convert angle to radians
        angles[0] = new Rotation2d(-AngleShoulder);
        angles[1] = new Rotation2d(AngleJoint);
        return angles;
    }
    /*************************************************/
    public void stop(){
        jointMotor.stopMotor();;
        shoulderMotor.stopMotor();;
    }


    public void setPoint(boolean intake, boolean low, boolean medium, boolean high, boolean retract){
        if (intake){
            goal[0] = arm.INTAKE[0];
            goal[1] = arm.INTAKE[1];
        }
        else if (low){
            goal[0] = arm.LOWGOAL[0];
            goal[1] = arm.LOWGOAL[1];
        }
        else if (medium){
            goal[0] = arm.MIDGOAL[0];
            goal[1] = arm.MIDGOAL[1];
        }
        else if (high){
            goal[0] = arm.HIGHGOAL[0];
            goal[1] = arm.HIGHGOAL[1];
            
        }
        else if (retract){
            goal[0] = 0;
            goal[1] = 0;
        }

    }
    public double difference = 0;
    public void updateSensor(){
        JointValue = -EncoderJoint.getPosition();
        ShoulderValue = EncoderShoulder.getPosition();
        difference =  4500 - ShoulderValue/(2*Math.PI) * 360;
    }
    public void zero(){
        EncoderJoint.setPosition(0);
        EncoderJoint.setPosition(0);
    }

    public void periodic(){
        
        updateSensor();
        SmartDashboard.putNumber("ShoulderV", ((goToAngleShoulder.calculate(ShoulderValue - arm.Shoulder.angleOffset.getRadians()) ))*.12);
        PIDShoulder = ((goToAngleShoulder.calculate(ShoulderValue - arm.Shoulder.angleOffset.getRadians()) ))*.12;
        double joi = EncoderJoint.getPosition()/(2*Math.PI) * 360;
        double sho = EncoderShoulder.getPosition()/(2*Math.PI) * 360;
        SmartDashboard.putNumber("Shoulder", sho);
        SmartDashboard.putNumber("Joint", joi);
        Rotation2d[] angles = this.getAngles(goal[0], goal[1]);
        double angle = angles[0].getDegrees();
        SmartDashboard.putNumber("ShoulderGoal ", angle);
        angle = angles[1].getDegrees();
        SmartDashboard.putNumber("JointGoal ", angle);
        SmartDashboard.putNumber("Direction", (difference));
    }
    /*public void getNewPoint(){
        goal[0] = goal[0] + 0.05;
        goal[1] = goal[0] + 0.05;
    }*/
}
