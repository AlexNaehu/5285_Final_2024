package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase{
    private final CANSparkMax intake;
    private final CANSparkMax wrist;
    public final DutyCycleEncoder wristEnc;

    private final int intakeID = 16;
    private final int wristMotorID = 12;
    private final int wristEncoderID = 0;

    private volatile double targetAngle;

    private boolean WristTargetHit = false;

    private final double INPUT_THRESHOLD = 1.0E-3;

    private final double WRIST_PIVOT_MAX_ANGLE = 190.0; //TODO: check max angle  //Robot 0 deg = Wrist pointing straight down
    private final double WRIST_PIVOT_MIN_ANGLE = 0;  //TODO: check min angle  //TBD, the reason for 110 range is to give wiggle room for oscillations


    public Intake(){
        intake = new CANSparkMax(intakeID, MotorType.kBrushless);
        wrist = new CANSparkMax(wristMotorID, MotorType.kBrushless);
        wristEnc = new DutyCycleEncoder(wristEncoderID);
        intake.setIdleMode(IdleMode.kBrake);
        intake.setIdleMode(IdleMode.kBrake);
    }

    public void vacuum(double power){
        intake.set(power*0.5); //sucks in game pieces
    }

    public void feed(double power){
        intake.set(power*0.5); //spits out game pieces (into the flywheel)
    }

    public double getPivotAngle() 
    {   //returns a decimal (hopefuly)
        return ((wristEnc.getAbsolutePosition()-wristEnc.getPositionOffset()) * 360.0);
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
        targetAngle-=5; //smaller because gravity pulls the Wrist that is already trying to go down
    }

    public void setWristTargetHit(boolean state)
    {
        WristTargetHit = state;
    }

    public boolean getWristTargetHit()
    {
        return WristTargetHit;
    }


    public void setPosition(double pivotTargetAngle)
    {
        RobotContainer.intake.setPivotTargetAngle(pivotTargetAngle);

        RobotContainer.intake.setWristTargetHit(false);
    }

    
    public void pivotPID()
    {
        Thread t = new Thread(() ->
        {
            final double WRIST_PIVOT_THREAD_WAITING_TIME = 0.005;
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

            Timer WristTimer = new Timer();
            WristTimer.start();

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
                if(targetAngle == WristConstants.INVALID_ANGLE)
                {
                    Timer.delay(WristConstants.CONTROLLER_INPUT_WAIT_TIME);
                }
                else
                {
                    //SmartDashboard.putBoolean("pivot pid state", runPivotPID);
                    currentTime  = WristTimer.get();
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
                        kApower = (kA * (Math.cos(Math.toRadians(195.0 - currentAngle))));//Compensates for angle of the Wrist
                        kFpower = kF;

                        power = kPpower + kIpower + kDpower + kApower + kFpower;

                        pivotWrist(-power);
                    
                        //Set up 
                        previousError = currentError;
                        previousTime = currentTime;

                        previousDerivative = currentDerivative;

                        SmartDashboard.putNumber("P Power", kPpower);
                        SmartDashboard.putNumber("I Power", kIpower);
                        SmartDashboard.putNumber("D Power", kDpower);
                        SmartDashboard.putNumber("A Power", kApower);
                        SmartDashboard.putNumber("F Power", kFpower);
                        SmartDashboard.putNumber("Total Wrist Power", power);
                    
                        Timer.delay(WRIST_PIVOT_THREAD_WAITING_TIME);
                    //}
                }
            }
        });
        t.start();
    }

    public void pivotWrist(double power){
        //using limits

        if (power > 0.3)
        {
            power = 0.3;
        }
        if (power < -0.3)
        {
            power = -0.3;
        }

        double pivotAngle = this.getPivotAngle();

        if(Math.abs(power) <= INPUT_THRESHOLD)
            {
                //setPivotTargetAngle(pivotAngle);
            }
            else if (power > 0.0)
            {
                //setPivotTargetAngle(BananaConstants.INVALID_ANGLE);

                if (pivotAngle > WRIST_PIVOT_MAX_ANGLE || Math.abs(power) > 1.0)
                {
                    wrist.set(0);
                    setPivotTargetAngle(WRIST_PIVOT_MAX_ANGLE); //CHANGED HERE
                }
                    else //if (Math.abs(power) > INPUT_THRESHOLD)
                    {
                        wrist.set(power);
                    }
            }
            else if (power < 0.0)
            {
                //setPivotTargetAngle(BananaConstants.INVALID_ANGLE);

                if (pivotAngle < WRIST_PIVOT_MIN_ANGLE || Math.abs(power) > 1.0)
                {
                    wrist.set(0);
                    setPivotTargetAngle(WRIST_PIVOT_MIN_ANGLE);
                }
                    else //if ( Math.abs(power) > INPUT_THRESHOLD)
                    {
                        wrist.set(0);//rotate Wrist counterclockwise which means down
                    }
                    
            }
            

    }

    public double getIntakePower(){
        return intake.get();
    }

    @Override
    public void periodic() {
        //Put Intake Data
        SmartDashboard.putNumber("Wrist Target Angle", getPivotTargetAngle());
        SmartDashboard.putNumber("Wirst Encoder Angle", getPivotAngle());
        SmartDashboard.putNumber("Intake: Power", getIntakePower());
    }
}
