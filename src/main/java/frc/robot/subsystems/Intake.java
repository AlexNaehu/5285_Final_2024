package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase{
    private final TalonSRX intake;
    private final TalonSRX wrist;
    

    public final DutyCycleEncoder wristEnc;

    private final int intakeID = 5; //TalonSRX ID (not CanSparkMax ID)
    private final int wristMotorID = 3; //TalonSRX ID (not CanSparkMax ID)
    private final int wristEncoderID = 9; //IO ID

    private volatile double targetAngle;

    private boolean WristTargetHit = false;

    private final double INPUT_THRESHOLD = 0.00001;

    private final double PIVOT_OFFSET = 275;

    private final double WRIST_PIVOT_MAX_ANGLE = 220.0; 
    private final double WRIST_PIVOT_MIN_ANGLE = -30;


    public Intake(){
        intake = new TalonSRX(intakeID);
        wrist = new TalonSRX(wristMotorID);
        wristEnc = new DutyCycleEncoder(wristEncoderID);

        wristEnc.setPositionOffset(0);

        intake.setNeutralMode(NeutralMode.Brake);
        wrist.setNeutralMode(NeutralMode.Brake);



        intake.configContinuousCurrentLimit(20);
        intake.configPeakCurrentLimit(25);
        intake.configPeakCurrentDuration(100);

        wrist.configContinuousCurrentLimit(20);
        wrist.configPeakCurrentLimit(25);
        wrist.configPeakCurrentDuration(100);

    
    }

    public void vacuum(double power){
        //deadband
        if(power<0.05){
            power = 0;
        }
        intake.set(ControlMode.PercentOutput, -power); //sucks in game pieces
    }

    public void feed(double power){
        //deadband
        if(power<0.05){
            power = 0;
        }
        intake.set(ControlMode.PercentOutput, power); //spits out game pieces (into the flywheel)
    }

    public double getPivotAngle() 
    {   //returns a decimal (hopefuly)
        return -(wristEnc.getAbsolutePosition()*360-PIVOT_OFFSET);//%360-294);
        //return wristEnc.getAbsolutePosition();//*360;
    }
  
    public double getPivotTargetAngle()
    {
        return targetAngle;
    }

    public void setPivotTargetAngle(double angle)
    {
        targetAngle = angle;
    }

    public void setWristPower(double power){
        wrist.set(ControlMode.PercentOutput, power);
    }

    public void increaseTargetAngle()
    {
        targetAngle+=2.5;
    }

    public void decreaseTargetAngle()
    {
        targetAngle-=2.5; //smaller because gravity pulls the Wrist that is already trying to go down
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

    
    public void wristPivotPID()
    {
        Thread t = new Thread(() ->
        {
            final double WRIST_PIVOT_THREAD_WAITING_TIME = 0.005;
            final double kP = 0.02;//TODO: Calibrate kP second
            final double kD = 0.00;//TODO: Calibrate kD last
            final double kI = 0.0;
            final double kA = 0.0;//TODO: Calibrate kA first
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

                    currentError = targetAngle - currentAngle;  //TODO: check if cw or ccw means increase or decrease in angle in order to make sure that positive power values calculate when the target angle is ABOVE the current angle
                    
                    deltaError = currentError - previousError;
                    deltaTime  = currentTime  - previousTime;

                    integral += deltaTime * currentError;

                    currentDerivative = (deltaError / deltaTime);
                    //filteredDerivative = (0.7 * currentDerivative) + (0.3 * previousDerivative);

                        //0 degrees : 275 degrees
                        kPpower = kP * currentError;
                        kIpower = kI * integral;
                        kDpower = kD * currentDerivative;//filteredDerivative;
                        //kApower = (kA * (Math.cos(Math.toRadians(195.0 - currentAngle))));//Compensates for angle of the Wrist
                        kFpower = kF;

                        power = kPpower + kIpower + kDpower + kFpower;

                        pivotWrist(-power);
                    
                        //Set up 
                        previousError = currentError;
                        previousTime = currentTime;

                        previousDerivative = currentDerivative;

                        /*SmartDashboard.putNumber("P Power", kPpower);
                        SmartDashboard.putNumber("I Power", kIpower);
                        SmartDashboard.putNumber("D Power", kDpower);
                        //SmartDashboard.putNumber("A Power", kApower);
                        SmartDashboard.putNumber("F Power", kFpower);*/
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

        if (power > 0.65)
        {
            power = 0.65;
        }
        if (power < -0.65)
        {
            power = -0.65;
        }

        double pivotAngle = this.getPivotAngle();

        if(Math.abs(power) <= INPUT_THRESHOLD)
            {
                power = 0;
            }
            
                /*if (pivotAngle >= WRIST_PIVOT_MAX_ANGLE) //added the >'(=)' to prevent trying to push past mechanical stops
                {
                    wrist.set(ControlMode.PercentOutput, 0);
                    setPivotTargetAngle(WRIST_PIVOT_MAX_ANGLE); //CHANGED HERE
                }
                else if (pivotAngle <= WRIST_PIVOT_MIN_ANGLE) //added the >'(=)' to prevent trying to push past mechanical stops
                {
                    wrist.set(ControlMode.PercentOutput, 0);
                    setPivotTargetAngle(WRIST_PIVOT_MIN_ANGLE);
                }*/
                    else //if ( Math.abs(power) > INPUT_THRESHOLD)
                    {
                        wrist.set(ControlMode.PercentOutput, power);   //posititve (ccw) = wrist up
                    }                       //negative (cw) = wrist down
                    
            
            

    }

    public double getIntakePower(){
        return intake.getMotorOutputPercent();
    }

    @Override
    public void periodic() {
        //Put Intake Data
        SmartDashboard.putNumber("Wrist Target Angle", getPivotTargetAngle());
        SmartDashboard.putNumber("Wirst Encoder Angle", getPivotAngle());
        //SmartDashboard.putNumber("Intake: Power", getIntakePower());
    }
}
