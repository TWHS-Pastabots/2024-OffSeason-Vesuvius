package frc.robot.subsystems.climber;
import javax.print.CancelablePrintJob;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import frc.robot.Ports;


public class Climber {

    private CANSparkMax TomahawkR;
    private CANSparkMax TomahawkL;
    private CANSparkMax TomahawkRWheel;
    private CANSparkMax TomahawkLWheel;
    private CANSparkMax LifterR;
    private CANSparkMax LifterL;

    public static Climber instance;
 

    



    public Climber() {
        TomahawkR = new CANSparkMax(16, MotorType.kBrushless);
        TomahawkR.restoreFactoryDefaults();

        TomahawkR.setSmartCurrentLimit(60);
        TomahawkR.setIdleMode(IdleMode.kBrake);
        TomahawkR.setInverted(false);
        TomahawkR.setOpenLoopRampRate(3);
        TomahawkR.burnFlash();

        TomahawkL = new CANSparkMax(15, MotorType.kBrushless);
        TomahawkL.restoreFactoryDefaults();

        TomahawkL.setSmartCurrentLimit(60);
        TomahawkL.setIdleMode(IdleMode.kBrake);
        TomahawkL.setInverted(false);
        TomahawkL.setOpenLoopRampRate(3);

        TomahawkL.burnFlash();

        TomahawkRWheel = new CANSparkMax(3, MotorType.kBrushless);
        TomahawkRWheel.restoreFactoryDefaults();

        TomahawkRWheel.setSmartCurrentLimit(60);
        TomahawkRWheel.setIdleMode(IdleMode.kBrake);
        TomahawkRWheel.setInverted(false);
        TomahawkRWheel.burnFlash();

        TomahawkLWheel = new CANSparkMax(12, MotorType.kBrushless);
        TomahawkLWheel.restoreFactoryDefaults();

        TomahawkLWheel.setSmartCurrentLimit(60);
        TomahawkLWheel.setIdleMode(IdleMode.kBrake);
        TomahawkLWheel.setInverted(false);
        TomahawkLWheel.burnFlash();

        LifterR = new CANSparkMax(14, MotorType.kBrushless);
        LifterR.restoreFactoryDefaults();

        LifterR.setSmartCurrentLimit(60);
        LifterR.setIdleMode(IdleMode.kBrake);
        LifterR.setInverted(false);
        LifterR.burnFlash();

        LifterL = new CANSparkMax(6, MotorType.kBrushless);
        LifterL.restoreFactoryDefaults();

        LifterL.setSmartCurrentLimit(60);
        LifterL.setIdleMode(IdleMode.kBrake);
        LifterL.setInverted(true);
        LifterL.burnFlash();

      
    }

    public void setTommysOn(){
        TomahawkR.set(1);
        TomahawkL.set(1);
      

    }

    public void setTommysReverse(){
        TomahawkR.set(-1);
        TomahawkL.set(-1);
        
    }

    public void setTommysOff(){
        TomahawkR.set(0.02);
        TomahawkL.set(0.0);
        

    }

    public void setTommysWheelsOn(){
        
        TomahawkLWheel.set(.9);
        TomahawkRWheel.set(.9);

    }

    public void setTommysWheelsReverse(){
      
        TomahawkLWheel.set(-.9);
        TomahawkRWheel.set(-.9);
    }

    public void setTommysWheelsOff(){
       
        TomahawkLWheel.set(0.0);
        TomahawkRWheel.set(0.0);

    }

     public void setLifterOn(){
        LifterL.set(.9);
        LifterR.set(.9);
    }

    public void setLifterReverse(){
        LifterL.set(-.9);
        LifterR.set(-.9);
    }

    public void setLifterOff(){
        LifterR.set(0.0);
        LifterL.set(0.0);
    }



    public static Climber getInstance() {
        if (instance == null)
            instance = new Climber();
        return instance;
    }
}

