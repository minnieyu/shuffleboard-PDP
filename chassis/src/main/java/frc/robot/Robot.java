/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
//import edu.wpi.first.wpilibj.PWMVictorSPX;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


  public class Robot extends TimedRobot {
    public static XboxController joy1 = new XboxController(0);
    private final WPI_VictorSPX frontLeftMotor=new WPI_VictorSPX(0);
    private final WPI_VictorSPX frontRightMotor=new WPI_VictorSPX(4);
    private final WPI_VictorSPX backLeftMotor=new WPI_VictorSPX(1);
    private final WPI_VictorSPX backRightMotor=new WPI_VictorSPX(5);
    private final VictorSPX frontsuckball=new VictorSPX(6);
    private final VictorSPX backsuckball=new VictorSPX(3);
    private Encoder encoder=new Encoder(0,1);
    int setpoint=10;
    private final double unknown=1.0/128*6*Math.PI/12; 
    double errorsum;
    double lastTimestamp;
    double derror;
    double lastError;
    private final PowerDistributionPanel powerDistributionPanel= new PowerDistributionPanel(0);
    private final DifferentialDrive m_tankDrive = new DifferentialDrive(new WPI_VictorSPX(0),
    new WPI_VictorSPX(4));

   
    @Override
   public void robotInit() {
    



  }

  
  
  @Override
  public void autonomousInit() {
  encoder.reset();
  errorsum=0;
  lastError=0;
  lastTimestamp=Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
  final double kp=0.3;
  double sensorpoistion=encoder.get()*unknown;
  double error=setpoint-sensorpoistion;
  
  final double ki=0.5;
  double dt=Timer.getFPGATimestamp()-lastTimestamp;
  final double limit=1;
  
  final double kd=0.1;
  double derror=(error-lastError)/dt;
  double output=kp*error + ki*errorsum + kd*derror;
  
  
  if(Math.abs(error)<limit){
    errorsum += error*dt;
  }

  if(joy1.getRawButton(1)){
    setpoint=10;
  }
  else if(joy1.getRawButton(2)){
    setpoint=0;
  }
  
  frontLeftMotor.set(ControlMode.PercentOutput,output);
  frontRightMotor.set(ControlMode.PercentOutput,-output);
  backLeftMotor.set(ControlMode.PercentOutput,output);
  backRightMotor.set(ControlMode.PercentOutput,-output);
   lastError=error;
  }




  @Override
  public void teleopInit() {

    }
    
  @Override
  public void teleopPeriodic() {
    
    final double speed = joy1.getRawAxis(0)*0.2;
    final double turn = -joy1.getRawAxis(1)*0.2;
    
    frontLeftMotor.set(ControlMode.PercentOutput,speed+turn);;
    frontRightMotor.set(ControlMode.PercentOutput,speed-turn);
    backLeftMotor.set(ControlMode.PercentOutput,speed+turn);
    backRightMotor.set(ControlMode.PercentOutput,speed-turn);
    
    
    SmartDashboard.putData("PDP",powerDistributionPanel);
    SmartDashboard.putData("Tank Drive",m_tankDrive);
    //SmartDashboard.putNumber("speed",speed);
    //SmartDashboard.putNumber("turn",turn);
    //SmartDashboard.putNumber("Front Left Motor",speed+turn);
    //SmartDashboard.putNumber("Front Right Motor", speed-turn);
    //ShuffleboardTab robotTab = Shuffleboard.getTab("Robot");
    //robotTab.add("Front Left Motor", frontLeftMotor);
    //robotTab.add("Front Right Motor", frontRightMotor);
    //robotTab.add("Back Left Motor", backLeftMotor);
    //robotTab.add("Back Right Motor", backRightMotor);
    //robotTab.add("speed", speed);
    //robotTab.add("turn",turn);
  
    backsuckball.set(ControlMode.PercentOutput, 0.4);
    if(joy1.getRawButton(6)){
      frontsuckball.set(ControlMode.PercentOutput, 0.4);
    }else if(joy1.getRawButton(5)){
      frontsuckball.set(ControlMode.PercentOutput, -0.4);
    }


    

    }

  

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}

  
  