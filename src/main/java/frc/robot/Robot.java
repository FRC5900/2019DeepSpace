/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/*----------------------------------------------------------------------------*/
/* These are libraries used for ZotBot2019                                    */
/*----------------------------------------------------------------------------*/ 
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogOutput;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
	private DifferentialDrive m_myRobot;
	private Joystick ControllerL = new Joystick(0);
	private Joystick ControllerR = new Joystick(1);

	double RobotMaxSpeed = .75;  
	Compressor c = new Compressor(0);
	
	private SpeedController DriveMotorFront = new Spark(0);
	private SpeedController DriveMotorBack = new Spark (1);
	private SpeedController liftMotor = new Spark(2);
	private SpeedController Clamplift = new Spark (3);
	private Timer my_timer = new Timer();
	
	Solenoid MastLower = new Solenoid(0);
	Solenoid MastRaise = new Solenoid(1);
	
	Solenoid GripperClose = new Solenoid(2);
	Solenoid GripperOpen = new Solenoid(3);	   
	  
// Not using Gyro	
// private ADXRS450_Gyro myGyro = new ADXRS450_Gyro();
	
	private int Maststate = 0;
	private int Gripperstate = 0;
	private int ATonOfMoose_State = 0;
	private int ATonOfMoose_Count = 0;
	private int Strategy_value = 0;
	private int AutoMove_State = 0;
	private int AutoMove_Count = 0;
	private int AutoMove_TargetCount = 0;
	private int Auto_Turn_State = 0;
	private int Auto_Turn_Count = 0;
	private int Auto_LiftMotor_State = 0;
	private int Auto_LiftMotor_Count = 0;
	private int Auto_ClampLift_State = 0;
	private int Auto_ClampLift_Count = 0;
	
	Preferences prefs;
	private boolean Center_Field;
	private boolean Left_Field;
	private boolean Right_Field;
	private boolean Switch_Target;
	private boolean Scale_Target;
	
	private boolean TurnRight;
	private boolean TurnLeft;
	
	
	double Kp = 0.03; 
	double angle;
		
	int TimeToSwitch = 1;
	double TurnAngle = 90.0;
	int TimeToTurn = 6;
	int TimeToBaseline = 8;
	int TimeToRight = 25;
	int TimeToDelta = 12;
	int TimeToLeft = 25;
	int TimeToLeft_Scale = 25;
	int TimeToRight_Scale = 25;
	int TimeToRight_Switch = 25;
	int TimeToLeft_Switch = 25;
	
	double TargetTime;
    double SpeedToSwitch;
    int TimeToScale = 5;
    int TimeToLiftMast = 2;
    double SpeedToMast = .3;
    double SpeedToLift = 0.5;
    double SpeedToClamp = 0.5;
    double VFront = 0.0;
    double VBack = 2.5;
        
    double leftspeed;
	double rightspeed;
	double rightturnangle;
	double leftturnangle;
	double frontangle;
	double backangle;
	
	boolean btnA;
	boolean btnB;
	boolean btnX;
	boolean btnY;
	boolean btnRightTrigger;
	boolean btnLeftTrigger;
	int POV_Angle;
	double lefttrigger;
	double righttrigger;
	boolean btnStart;
	boolean btnBack;
	
	double Deadband_Upper = 0.05;
	double Deadband_Lower = -0.05;
	
	
	DigitalInput limitSwitch = new DigitalInput(1);
	DigitalInput limitSwitch2 = new DigitalInput(2);
	DigitalInput limitSwitch3 = new DigitalInput(3);
	DigitalInput limitSwitch4 = new DigitalInput(4);
	DigitalOutput testOutput = new DigitalOutput(9);
	 


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() 
  {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putString("Mode", "Robot Init" );
		m_myRobot = new DifferentialDrive(new Spark(5), new Spark(6));
		
		// CameraServer.getInstance() .startAutomaticCapture(0);
	  c.setClosedLoopControl(true);
		
		prefs = Preferences.getInstance();
		Center_Field = prefs.getBoolean("Center Field", false);
		Left_Field = prefs.getBoolean("Left Field", false);
		Right_Field = prefs.getBoolean("Right Field", false);
		Switch_Target = prefs.getBoolean("Switch Target", false);
		Scale_Target = prefs.getBoolean("Scale Target", false);
		Auto_LowerMast();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() 
  {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() 
  {
    switch (m_autoSelected) 
    {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopInit() 
  {
    super.teleopInit();
    SmartDashboard.putString("Mode", "Teleop Init" );
		Maststate = 0;
		Gripperstate = 0;	 		
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  {
    // Get JoyStick information
    leftspeed = ControllerR.getX();
    rightspeed = ControllerR.getY();

    /* Compensate for drift voltage of the Joysticks */
    if(( leftspeed < Deadband_Upper ) && ( leftspeed > Deadband_Lower ))
      leftspeed = 0.0;

    if( leftspeed > RobotMaxSpeed )
      leftspeed = RobotMaxSpeed;
      
    if( leftspeed < -RobotMaxSpeed )
      leftspeed = -RobotMaxSpeed;

    if(( rightspeed < Deadband_Upper ) && ( rightspeed > Deadband_Lower ))
      rightspeed = 0.0;
   
    if( rightspeed > RobotMaxSpeed )
      rightspeed = RobotMaxSpeed;
      
    if( rightspeed < -RobotMaxSpeed )
      rightspeed = -RobotMaxSpeed;  
    
    // Remove when in competition  
    SmartDashboard.putNumber("RightSpeed", rightspeed);
    SmartDashboard.putNumber("LeftSpeed", leftspeed);
  
   
  m_myRobot.arcadeDrive( rightspeed, leftspeed );
  
  
  if(ControllerL.getRawButton(3)) 
    liftMotor.set(-1.0);
  else 
  {	
      if(ControllerL.getRawButton(2))
        liftMotor.set(1.0);
      else 
        liftMotor.set(0.0);
  }
  
  if(ControllerR.getRawButton(3)) 
    Clamplift.set(-0.5);			
  else
  {	
    if(ControllerR.getRawButton(2)) 
          Clamplift.set(0.5);				
      else 
        Clamplift.set(0.0);
  }
  
    
  Mast_Cntl();
  Gripper_Cntl();

  
  
}


/* This controls the raise and lower the mast.           */
/* Pulling trigger will raise the mast.                  */ 
/* Release the trigger and pull the trigger again lowers */
/* the mast.                                             */
public void Mast_Cntl() 
{   
  switch (Maststate) 
  {
  case 0://waiting for trigger press
    if (ControllerL.getRawButton(1) == true) 
    { 
        MastRaise.set(true); 
        MastLower.set(false);
        Maststate = 1;
    }
    break;
    
  case 1: //waiting for trigger release
    if (ControllerL.getRawButton(1) == false) 
        Maststate = 2;
    break;
    
  case 2: 
    if (ControllerL.getRawButton(1) == true) 
    { 
      MastRaise.set(false);
      MastLower.set(true);
      Maststate = 3;
    }
    break;
    
  case 3: //waiting for trigger release
    if (ControllerL.getRawButton(1) == false)
        Maststate = 0;
    break; 
    
  default: 
    Maststate = 0;
    MastRaise.set(false);
    MastLower.set(false); 
    break;

  }
}
    

/* This controls the opening and closing of the gripper  */
	/* Pulling trigger will close the gripper                */
	/* Release the trigger and pull the trigger again opens  */
	/* the gripper.                                          */
	public void Gripper_Cntl() 
	{
		switch (Gripperstate) 
		{
		case 0://waiting for trigger press
			if (ControllerR.getRawButton(1) == true) 
			{ 
			    Auto_GripperClose();
			    Gripperstate = 1;
			}
			break;
			
		case 1: //waiting for trigger release
			if (ControllerR.getRawButton(1) == false) 
			    Gripperstate = 2;
			break;
			
		case 2: 
			if (ControllerR.getRawButton(1) == true) 
			{ 
				Auto_GripperOpen();
				Gripperstate = 3;
			}
			break;
			
		case 3: //waiting for trigger release
			if (ControllerR.getRawButton(1) == false)
			    Gripperstate = 0;
			break;
			
		default: 
			Gripperstate = 0;
			GripperClose.set(false);
			GripperOpen.set(false);
			break;

		}
	}
	
	
 

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() 
  {
  }

  @Override public void disabledPeriodic() 
	{   
	}

	/***************************************************************************/
	/* Auto_LiftMotor() - automatically Lifts up the mass.                       */
	/*                                                                         */
	/***************************************************************************/
	public void Auto_LiftMotor(int Target_Count)
	{
		
		switch (Auto_LiftMotor_State)
		{
		case 0:
			Auto_LiftMotor_Count = 0;
			Auto_LiftMotor_State = 1;
			break; 
			
		case 1:
			if ( ++Auto_LiftMotor_Count > Target_Count )
			{
				liftMotor.set(0.0);
				Auto_LiftMotor_State = 100;
			}
			else
			{
				liftMotor.set(SpeedToLift);
			}
			break;
			
		case 2:
			if ( ++Auto_LiftMotor_Count > Target_Count )
			{
				liftMotor.set(0.0);
				Auto_LiftMotor_State = 100;
			}
			else
			{
				liftMotor.set(-SpeedToLift);
			}
			break;
			
		case 100:
			liftMotor.set(0.0);
			break;
			
		default:
			Auto_LiftMotor_State = 100;
			break;
		}
	}
	
	/***************************************************************************/
	/* Auto_LiftMotor() - automatically Lifts up the mass.                       */
	/*                                                                         */
	/***************************************************************************/
	public void Auto_ClampLift(int Target_Count)
	{
		
		switch (Auto_ClampLift_State)
		{
	  case 0:
			Auto_ClampLift_Count = 0;
			Auto_ClampLift_State = 1;
			break; 
			
		case 1:
			if ( ++Auto_ClampLift_Count > Target_Count )
			{
				Clamplift.set(0.0);
				Auto_ClampLift_State = 100;
			}
			else
			{
				Clamplift.set(SpeedToClamp);
			}
			break;
			
		case 2:
			if ( ++Auto_ClampLift_Count > Target_Count )
			{
				Clamplift.set(0.0);
				Auto_ClampLift_State = 100;
			}
			else
			{
				Clamplift.set(-SpeedToClamp);
			}
			break;
			
		case 100:
			Clamplift.set(0.0);
			break;
			
		default:
			Auto_ClampLift_State = 100;
			break;
		}
	}
	
	/***************************************************************************/
	/* Auto_GripperOpen() - automatically opens gripper.                       */
	/*                                                                         */
	/***************************************************************************/
	public void Auto_GripperOpen()
	{
		GripperClose.set(false);
		GripperOpen.set(true);	
	}
	
	
	/***************************************************************************/
	/* Auto_GripperClose() - automatically opens gripper.                       */
	/*                                                                         */
	/***************************************************************************/
	public void Auto_GripperClose()
	{
		GripperClose.set(true);
		GripperOpen.set(false);	
	}
	
	
	/***************************************************************************/
	/* Auto_RaiseMast() - automatically raise mast.                            */
	/*                                                                         */
	/***************************************************************************/
	public void Auto_RaiseMast()
	{
		MastRaise.set(true);
		MastLower.set(false);	
	}

	
	/***************************************************************************/
	/* Auto_LowerMast() - automatically lowers mast.                           */
	/*                                                                         */
	/***************************************************************************/
	public void Auto_LowerMast()
	{
		MastRaise.set(false);
		MastLower.set(true);	
	}
	 
}






    



	
	

	 
	

	
	
	
 
					





