package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubSystem;

public class CommandAuto extends CommandBase {
    private double power;
    private double dist;
    private DriveSubSystem drive;
    private double target;

    public CommandAuto(DriveSubSystem drive, double power, double dist) {
        this.power = power;
        this.dist = dist;
        this.drive = drive;
        addRequirements(drive);
        
    }

    

    @Override
    public void execute() {
        drive.setPower(this.power,this.power);
        System.out.println(power);
    }

    @Override
    public void initialize() {
        target = dist + drive.getLeftPosition();
    }

    @Override
    public boolean isFinished() {
        return drive.getLeftPosition() >= target;
    }

    @Override
    public void end(boolean interrupted) {
        drive.setPower(0,0);
        super.end(interrupted);
    }

}
