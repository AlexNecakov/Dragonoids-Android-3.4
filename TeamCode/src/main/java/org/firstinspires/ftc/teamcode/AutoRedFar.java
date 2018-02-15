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

            liftGlyph();
            int key = photoSense();
            knock(true);
            forward(-1,.3);
            turn(-180);
            strafe(-.15,.3);
            chooseGlyph(key, true);
            releaseGlyph();
            break;
        }
    }
}
