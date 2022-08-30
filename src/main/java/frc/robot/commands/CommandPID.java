package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubSystem;

public class CommandPID extends CommandBase{
    private DriveSubSystem drive;
    private double startdist;
    private double enddist;
    private PIDController pid;
    private double power;
    public CommandPID(DriveSubSystem drive, double power, double enddist){
        this.drive = drive;
        this.enddist = enddist;
        this.power = power;
        this.pid = new PIDController(0.7,0.07,0);
        this.pid.setSetpoint(this.enddist);
    }
    @Override
    public void initialize() {
        startdist = drive.getLeftPosition();
    }
    @Override
    public void execute() {
        double p = pid.calculate(drive.getLeftPosition()-startdist);
        if(p > power) {
            p = power;
        }
        System.out.println(" dist = " + (drive.getLeftPosition()-startdist) + " p=" + p );
        drive.setPower(p,p);
    }
    @Override
    public boolean isFinished() {
        return drive.getLeftPosition()-startdist >= enddist;
    }
    @Override
    public void end(boolean interrupted) {
        drive.setPower(0,0);;
    }
    
}
