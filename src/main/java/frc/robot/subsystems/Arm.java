package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{

    public final DutyCycleEncoder armPivotEnc;

    private final CANSparkMax leftShoulder;
    private final CANSparkMax rightShoulder;

    private final int leftShoulderID = 17;
    private final int rightShoulderID = 18;

    private volatile double targetAngle;

    private final double ARM_PIVOT_MAX_ANGLE = 220; //TODO: check max angle  //Robot 0 deg = Arm pointing straight down
    private final double ARM_PIVOT_MIN_ANGLE = 0;  //TODO: check min angle  //TBD, the reason for 110 range is to give wiggle room for oscillations

    private final int ARM_PIVOT_ENCODER_PORT = 0; //reference the following for pinout configurations on the NavX: https://pdocs.kauailabs.com/navx-mxp/installation/io-expansion/

    private final double PIVOT_OFFSET = 0.0; //TODO: set the resting state of the arm to 0 degrees

    private boolean armTargetHit = false;

    private final double INPUT_THRESHOLD = 1.0E-3;

    public Arm(){
        armPivotEnc = new DutyCycleEncoder(ARM_PIVOT_ENCODER_PORT);
        armPivotEnc.setPositionOffset(PIVOT_OFFSET/360);

        leftShoulder = new CANSparkMax(leftShoulderID, MotorType.kBrushless);
        rightShoulder = new CANSparkMax(rightShoulderID, MotorType.kBrushless);

        leftShoulder.setIdleMode(IdleMode.kCoast);
        rightShoulder.setIdleMode(IdleMode.kCoast);

        leftShoulder.setSmartCurrentLimit(10);
        rightShoulder.setSmartCurrentLimit(10);
        
    }

    public double getPivotAngle() 
    {   //returns a decimal 0-1 of a full rotation (hopefuly)
        return ((armPivotEnc.getAbsolutePosition()-armPivotEnc.getPositionOffset()) * 360.0);
    }
  
    public double getPivotTargetAngle()
    {
        return targetAngle;
    }

    public void setPivotTargetAngle(double angle)
    {
        targetAngle = angle;
    }

    public void increaseTargetAngle()
    {
        targetAngle+=5;
    }

    public void decreaseTargetAngle()
    {
        targetAngle-=5; //smaller because gravity pulls the arm that is already trying to go down
    }

    public void setArmTargetHit(boolean state)
    {
        armTargetHit = state;
    }

    public boolean getArmTargetHit()
    {
        return armTargetHit;
    }


    public void setPosition(double pivotTargetAngle)
    {
        RobotContainer.arm.setPivotTargetAngle(pivotTargetAngle);

        RobotContainer.arm.setArmTargetHit(false);
    }

    
    public void pivotPID()
    {
        Thread t = new Thread(() ->
        {
            final double ARM_PIVOT_THREAD_WAITING_TIME = 0.005;
            final double kP = 0.018;//TODO: Calibrate kP second
            final double kD = 0.0012;//TODO: Calibrate kD last
            final double kI = 0.0;
            final double kA = 0.33;//TODO: Calibrate kA first
            final double kF = 0.0;

            double power;            
            double kPpower;
            double kIpower;
            double kDpower;
            double kApower;
            double kFpower;

            Timer armTimer = new Timer();
            armTimer.start();

            double previousError = 0;
            double currentError; 
            double deltaError = 0; 

            double previousDerivative = 0;
            double currentDerivative;    
            double filteredDerivative;  // filtered to prevent derivative jumps, and provide 
                                        //a smoother transition to a new slope of the dE v. dt graph
            
            double previousTime = 0;
            
            double deltaTime;
            
            double currentTime;
            double currentAngle;

            double integral = 0;

            while(true)
            {
                if(targetAngle == ArmConstants.INVALID_ANGLE)
                {
                    Timer.delay(ArmConstants.CONTROLLER_INPUT_WAIT_TIME);
                }
                else
                {
                    //SmartDashboard.putBoolean("pivot pid state", runPivotPID);
                    currentTime  = armTimer.get();
                    currentAngle = getPivotAngle();

                    currentError = targetAngle - currentAngle;
                    
                    deltaError = currentError - previousError;
                    deltaTime  = currentTime  - previousTime;

                    integral += deltaTime * currentError;

                    currentDerivative = (deltaError / deltaTime);
                    //filteredDerivative = (0.7 * currentDerivative) + (0.3 * previousDerivative);


                        kPpower = kP * currentError;
                        kIpower = kI * integral;
                        kDpower = kD * currentDerivative;//filteredDerivative;
                        kApower = (kA * (Math.cos(Math.toRadians(195.0 - currentAngle))));//Compensates for angle of the arm
                        kFpower = kF;

                        power = kPpower + kIpower + kDpower + kApower + kFpower;

                        pivotArm(-power);
                    
                        //Set up 
                        previousError = currentError;
                        previousTime = currentTime;

                        previousDerivative = currentDerivative;

                        SmartDashboard.putNumber("P Power", kPpower);
                        SmartDashboard.putNumber("I Power", kIpower);
                        SmartDashboard.putNumber("D Power", kDpower);
                        SmartDashboard.putNumber("A Power", kApower);
                        SmartDashboard.putNumber("F Power", kFpower);
                        SmartDashboard.putNumber("Total Arm Power", power);
                    
                        Timer.delay(ARM_PIVOT_THREAD_WAITING_TIME);
                    //}
                }
            }
        });
        t.start();
    }

    public void pivotArm(double power){
        //using limits

        if (power > 1)
        {
            power = 1;
        }
        if (power < -1)
        {
            power = -1;
        }

        double pivotAngle = this.getPivotAngle();

        if(Math.abs(power) <= INPUT_THRESHOLD)
            {
                //setPivotTargetAngle(pivotAngle);
            }
            else if (power > 0.0)
            {
                //setPivotTargetAngle(BananaConstants.INVALID_ANGLE);

                if (pivotAngle > ARM_PIVOT_MAX_ANGLE || Math.abs(power) > 1.0)
                {
                    leftShoulder.set(0.0);
                    rightShoulder.set(0.0);
                    setPivotTargetAngle(ARM_PIVOT_MAX_ANGLE); //CHANGED HERE
                }
                else if (pivotAngle < ARM_PIVOT_MIN_ANGLE || Math.abs(power) > 1.0)
                {
                    leftShoulder.set(0.0);
                    rightShoulder.set(0.0);
                    setPivotTargetAngle(ARM_PIVOT_MIN_ANGLE);
                }
                    else //if (Math.abs(power) > INPUT_THRESHOLD)
                    {
                        leftShoulder.set(power);
                        rightShoulder.set(power); //rotate arm clockwise which means up
                    }
            }
            else if (power < 0.0)
            {
                //setPivotTargetAngle(BananaConstants.INVALID_ANGLE);

                if (pivotAngle < ARM_PIVOT_MIN_ANGLE || Math.abs(power) > 1.0)
                {
                    leftShoulder.set(0.0);
                    rightShoulder.set(0.0);
                    setPivotTargetAngle(ARM_PIVOT_MIN_ANGLE);
                }
                else if (pivotAngle > ARM_PIVOT_MAX_ANGLE || Math.abs(power) > 1.0)
                {
                    leftShoulder.set(0.0);
                    rightShoulder.set(0.0);
                    setPivotTargetAngle(ARM_PIVOT_MAX_ANGLE); //CHANGED HERE
                }
                    else //if ( Math.abs(power) > INPUT_THRESHOLD)
                    {
                        leftShoulder.set(power);
                        rightShoulder.set(power); //rotate arm counterclockwise which means down
                    }
                    
            }
            

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("PIVOT: Target Angle", getPivotTargetAngle());
        SmartDashboard.putNumber("PIVOT: Encoder Angle", getPivotAngle());
    }
    
}
