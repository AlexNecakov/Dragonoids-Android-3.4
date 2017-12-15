package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Dragonoids on 2/27/2017.
 */
@Autonomous(name="Auto Blue Far", group="Blue")
public class AutoBlueFar extends DragonoidsAuto{

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        while (opModeIsActive()) {

            /*int key = photoSense();
            strafe(-.1875,.3);
            knock(false);*/

            forward(1.4,.3);
            strafe(.5,.3);
            //chooseGlyph(key);

            break;
        }
    }
}
