/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  DifferentialDrive drivetrain;
  Spark driveMotorLeft1, driveMotorLeft2, driveMotorRight1,driveMotorRight2;
  Joystick leftStick;
  Joystick rightStick;
  JoystickButton button1;
  Spark motor_Intake;
  WPI_TalonSRX motor_Elevator;
  Encoder Encoder1_isUsedForWhat;

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //Some Variables to store Vision Target Data
  NetworkTable _netorktable_LimeLight;
  NetworkTableEntry _LimeLight_tx;
  NetworkTableEntry _LimeLight_ty;
  NetworkTableEntry _LimeLight_ta;
  double VisionTarget_x_error;
  double VisionTarget_y_error;
  double VisionTarget_area;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    driveMotorLeft1=new Spark(0);
    driveMotorLeft2=new Spark(1);
    driveMotorRight1=new Spark(2);
    driveMotorRight2=new Spark(3);
    motor_Intake=new Spark(4);
    motor_Elevator = new WPI_TalonSRX(0);
    SpeedControllerGroup leftMotorGroup=new SpeedControllerGroup(driveMotorLeft1, driveMotorLeft2);
    SpeedControllerGroup rightMotorGroup=new SpeedControllerGroup(driveMotorRight1, driveMotorRight2);
	  drivetrain=new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    leftStick=new Joystick(0);
    rightStick=new Joystick(1);
    button1= new JoystickButton(leftStick, 2);
    Encoder1_isUsedForWhat=new Encoder(0,1,false);
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //Set up NetworkTables Lime Light tages
    _netorktable_LimeLight = NetworkTableInstance.getDefault().getTable("limelight");
    _LimeLight_tx = _netorktable_LimeLight.getEntry("tx");
    _LimeLight_ty = _netorktable_LimeLight.getEntry("ty");
    _LimeLight_ta = _netorktable_LimeLight.getEntry("ta");
    
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
  public void robotPeriodic() {
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
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    //Storage variable for DriveTrain state
    int m_drivetrainCase = 0;
    boolean _drivewithLimeLight = false;

    //Storage variable for CargoIntake
    int m_cargoIntakeCase = 0;
    double m_intake_OpenLoop_Power = 0;

    //Storage variable for Elevator
    int m_elevatorCase = 0;
    double m_elevaotor_OpenLoop_Power = 0;


    
    while(isOperatorControl()&&isEnabled()){

      //Lets force the Drivetrain to default to Manual
      m_drivetrainCase = 0; 

      // Add if statment or set equal to a button to enable
      _drivewithLimeLight = leftStick.getRawButtonPressed(2); 

      if(_drivewithLimeLight){
        m_drivetrainCase = 1;
      }

      /*********************************************************** */
      // DriveTrain Case Statment
      /*********************************************************** */

      double leftValue;
      double rightValue;
      switch(m_drivetrainCase){
        case 0: //Drive Normal
          leftValue=leftStick.getRawAxis(1)*-1;
          rightValue=leftStick.getRawAxis(5)*-1;
          myDrive(leftValue, rightValue);
          break;

        case 1:
          my_getLimeLightData();
          double kp = .05; //This is a proportional value to multiply the vision error
          double _throttle = 0; //Added a joystick here to drive forward while seeking
          leftValue = (VisionTarget_x_error * kp) + _throttle;
          rightValue= -(VisionTarget_x_error * kp) + _throttle;
          myDrive(leftValue, rightValue);
          break;

        default: //If something is wrong shut down
          myDrive(0,0);

      }

      /*********************************************************** */
      // Intake Case Statment
      /*********************************************************** */

      //Default to off
      m_cargoIntakeCase = 0;

      if(rightStick.getRawAxis(1)>.2){
        m_cargoIntakeCase = 1;
        m_intake_OpenLoop_Power = rightStick.getRawAxis(1);
      }

      switch(m_cargoIntakeCase){
        case 0: //Cargo Intake Off
          myIntake(0);
          break;

        case 1: //Intake
          myIntake(m_intake_OpenLoop_Power);
          break;
        
        default: //If something is wrong shut down
          myIntake(0);
      }


      /*********************************************************** */
      // Elevator Case Statment
      // 
      //  This is more complacated because we need to get the Encoder working
      //  But for now I will sample the code as you have it as Open Loop / Jog
      //
      /*********************************************************** */

      //Default to off
      m_elevatorCase = 0; //off

      if(Math.abs(rightStick.getRawAxis(5))>.2){
        m_elevatorCase = 1;
        m_elevaotor_OpenLoop_Power = rightStick.getRawAxis(5);
      }
      
      switch(m_elevatorCase){
        case 0: //Cargo Intake Off
          myElevator_OpenLoop_Jog(0);
          break;

        case 1: //Intake
        myElevator_OpenLoop_Jog(m_elevaotor_OpenLoop_Power);
          break;
        
        default: //If something is wrong shut down
          myIntake(0);
      }
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }




  /*********************************************************** */
  //         SUBSYSTEMS
  /*********************************************************** */



  /**
   * Only have one instance of the physical drivetrain and pass parrameters to it!
   * 
   * 
   * @param leftCommand
   * @param rightCommand
   */
  private void myDrive(double leftCommand, double rightCommand){
    drivetrain.tankDrive(leftCommand,rightCommand,true);
  }

  /**
   * Only have one instance of the physical Intake motor and pass parrameters to it!
   * @param power
   */
  private void myIntake(double power){
    motor_Intake.set(power);
  }

  /**
   * Open Loop Contorl of the Elevator
   * @param power
   */
  private void myElevator_OpenLoop_Jog(double power){
    //We need to get the TalonSRX on Can and Motion Magic working
      motor_Elevator.set(ControlMode.PercentOutput,power);
  }


  /**
   * There is alot more setup needed here
   * @param setpoint
   */
  private void myElevator_MotionMagic(double setpoint){
    //We need to get the TalonSRX on Can and Motion Magic working
      motor_Elevator.set(ControlMode.MotionMagic, setpoint);
  }



  /**
   * WE can go get some Vision Data
   */
  public void my_getLimeLightData(){
    
    VisionTarget_x_error = _LimeLight_tx.getDouble(0.0);
    VisionTarget_y_error = _LimeLight_ty.getDouble(0.0);
    VisionTarget_area = _LimeLight_ta.getDouble(0.0);
    SmartDashboard.putNumber("LimelightX", VisionTarget_x_error);
    SmartDashboard.putNumber("LimelightY", VisionTarget_y_error);
    SmartDashboard.putNumber("LimelightArea", VisionTarget_area);
  }
}
