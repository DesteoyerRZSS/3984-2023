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


public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax shoulderMotor; 
    private CANSparkMax jointMotor;
    public RelativeEncoder shoulderEncoder;
    public RelativeEncoder jointEncoder;
    public double jointPos = 0;
    public double shoulderPos = 0;
    // Initialize the goal point for the arm.
    /* 
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
            
    public ArmFeedforward Jointff = new ArmFeedforward(0, 0, 0, 0);
    */
    public ArmSubsystem() {
        shoulderMotor = new CANSparkMax(arm.Shoulder.rotMotorID, MotorType.kBrushless);
        jointMotor = new CANSparkMax(arm.Joint.rotMotorID, MotorType.kBrushless);
        shoulderEncoder = shoulderMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        shoulderEncoder.setPosition(0);        
        jointEncoder = jointMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        jointEncoder.setPosition(0);
        jointEncoder.setPositionConversionFactor(360);
    }
    public void moveToAngle(boolean move){
        Rotation2d[] angles = new Rotation2d[2];
        new Rotation2d();
        //get angles needed for each joint
        Rotation2d Sangle = Rotation2d.fromDegrees(-45);
        //new Rotation2d();
        //Rotation2d Jangle = Rotation2d.fromDegrees(0);// Don't let it move for now
        if (move){
            while (Sangle.getDegrees() != shoulderPos){
                shoulderMotor.setVoltage((Sangle.getDegrees() - shoulderPos)*0.5);
            }
        }
        /* 
        goToAngleShoulder.setSetpoint(angle);
        goToAngleShoulder.setTolerance(1, 0.1);
        SmartDashboard.putNumber("Angle Goal Shoulder", angle);

        angle = angles[1].getRadians();
        goToAngleJoint.setSetpoint(angle);
        goToAngleJoint.setTolerance(1, 0.1);
        SmartDashboard.putNumber("Angle Goal Shoulder", angle);
        
        SmartDashboard.putNumberArray("GoalPoint: ", goal);
        if (!goToAngleJoint.atSetpoint() && !goToAngleShoulder.atSetpoint()){
            shoulderMotor.setVoltage((goToAngleShoulder.calculate(EncoderShoulder.getPosition() - arm.Shoulder.angleOffset.getRadians()) 
                                    + Shoulderff.calculate(EncoderShoulder.getPosition()- arm.Shoulder.angleOffset.getRadians()
                                    , EncoderShoulder.getVelocity()))*.12);
            jointMotor.setVoltage((goToAngleJoint.calculate(EncoderJoint.getPosition() - arm.Joint.angleOffset.getRadians())
                                    + Jointff.calculate(EncoderJoint.getPosition()- arm.Joint.angleOffset.getRadians()
                                    , EncoderJoint.getVelocity()))*.12);
        }
        */
    }








    /************************************************
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
        angles[0] = new Rotation2d(AngleShoulder);
        angles[1] = new Rotation2d(AngleJoint);
        return angles;
    }
    *************************************************/
    public void Update(){
        jointPos = jointEncoder.getPosition();
        shoulderPos = shoulderEncoder.getPosition();

    }

    public void periodic(){
        Update();
        SmartDashboard.putNumber("ShoulderDeg", shoulderPos);
        SmartDashboard.putNumber("JointDeg", jointPos);
        /* 
        Rotation2d[] angles = this.getAngles(goal[0], goal[1]);
        double angle = angles[0].getDegrees();
        SmartDashboard.putNumber("ShoulderGoal ", angle);
        angle = angles[1].getDegrees();
        SmartDashboard.putNumber("JointGoal ", angle);
         */

    }
    /*public void getNewPoint(){
        goal[0] = goal[0] + 0.05;
        goal[1] = goal[0] + 0.05;
    }*/
}
