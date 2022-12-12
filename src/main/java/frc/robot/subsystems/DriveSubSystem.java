package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.COMMAND;

public class DriveSubSystem extends SubsystemBase{
        private TalonFX Leftm1;
        private TalonFX Leftm2;
        private TalonFX Rightm1;
        private TalonFX Rightm2;
        private PigeonIMU gyro = new PigeonIMU(Constants.GYRO_PORT);

        public DriveSubSystem()
        {
            this.Leftm1 = new TalonFX(Constants.LEFT);
            this.Leftm2 = new TalonFX(Constants.LEFT1);
            this.Rightm1 = new TalonFX(Constants.RIGHT);
            this.Rightm2 = new TalonFX(Constants.RIGHT1);
            Leftm1.setInverted(Constants.LeftInverted);
            Leftm2.setInverted(Constants.LeftInverted);
            Rightm1.setInverted(Constants.RightInverted);
            Rightm2.setInverted(Constants.RightInverted);
            setDefaultCommand(new COMMAND(this));
            this.Leftm1.config_kP(0, 0.02);
            this.Rightm1.config_kP(0, 0.02);
            this.Leftm2.config_kP(0, 0.02);
            this.Rightm2.config_kP(0, 0.02);
        }
        public void setPower(double L, double R)
        {
            Leftm1.set(ControlMode.PercentOutput, L);
            Leftm2.set(ControlMode.PercentOutput, L);
            Rightm1.set(ControlMode.PercentOutput, R);
            Rightm2.set(ControlMode.PercentOutput, R);
        }
        public double getLeftPosition(){
            double dist = Leftm1.getSelectedSensorPosition()/Constants.pulse_to_meter;
            return dist;
        }
        public double getRightPosition(){
            double dist = Rightm1.getSelectedSensorPosition()/Constants.pulse_to_meter;
            return dist;
        }
        public double getPosition(){
            double dist = (getRightPosition()+getLeftPosition())/2;
            return dist;
        }
        @Override
        public void periodic() {
        SmartDashboard.putNumber("Left Position", getLeftPosition());
        SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
        SmartDashboard.putNumber("Right Velocity", getRightVelocity());
        SmartDashboard.putNumber("Angle", getAngle());
        super.periodic();
        }
        public double getLeftVelocity(){
            double p = Leftm1.getSelectedSensorVelocity();
            return p*10/Constants.pulse_to_meter;
        }
        public double getRightVelocity(){
            double p = Rightm1.getSelectedSensorVelocity();
            return p*10/Constants.pulse_to_meter;
        }
        public void setVelocity(double VL , double VR){
            Leftm2.follow(Leftm1);
            Leftm1.set(ControlMode.Velocity , VL*Constants.pulse_to_meter/10.,
            DemandType.ArbitraryFeedForward ,Constants.Ks*Math.signum(VL) + VL*Constants.Kv);
            Rightm2.follow(Rightm1);
            Rightm1.set( ControlMode.Velocity , VR*Constants.pulse_to_meter/10.,
            DemandType.ArbitraryFeedForward , Constants.Ks*Math.signum(VR) + VR*Constants.Kv);
        }
        public double getAngle(){
            return gyro.getFusedHeading();
        }
        
}
