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

            liftGlyph();
            int key = photoSense();
            knock(false);

            forward(1.1,.3);
            strafe(.3,.3);
            chooseGlyph(key, false);
            forward(.3,.3);
            releaseGlyph();

            break;
        }
    }
}
