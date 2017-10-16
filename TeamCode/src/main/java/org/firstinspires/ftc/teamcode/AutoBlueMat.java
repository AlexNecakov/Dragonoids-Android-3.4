package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Dragonoids on 2/27/2017.
 */
@Autonomous(name="Auto Blue Mat", group="Blue")
public class AutoBlueMat extends DragonoidsAuto{

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        while (opModeIsActive()) {

            int key = photoSense();
            //strafe(-.1,.3);
            knock(false);

           /* forward(1.5,.8);
            turn(-90);
            chooseGlyph(key);*/
            break;
        }
    }
}

