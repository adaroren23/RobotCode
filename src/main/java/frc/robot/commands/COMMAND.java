package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubSystem;

public class COMMAND extends CommandBase {
    protected DriveSubSystem drive;

    public COMMAND(DriveSubSystem drive)
    {
        this.drive = drive;
        addRequirements(drive);
    }
    @Override
    public void execute() {
        Double L = RobotContainer.LJOY.getY();
        Double R = RobotContainer.RJOY.getY();
        if ((L > -0.1) && (L<0.1))
        {
            L = 0.0;
        } 
        if ((R > -0.1) && (R<0.1))
        {
            R = 0.0;
        }
        L = L*L*Math.signum(L);
        R = R*R*Math.signum(R);
        drive.setPower(L, R);
    }
}
