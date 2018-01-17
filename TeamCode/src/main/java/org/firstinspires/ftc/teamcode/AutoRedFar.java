package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Dragonoids on 2/27/2017.
 */
@Autonomous(name="Auto Red Far", group="Red")
public class AutoRedFar extends DragonoidsAuto{

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        while (opModeIsActive()) {
            forward(-1.3,.3);
            strafe(.3,.3);
            //turn(-180);
            break;
        }
    }
}
