package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Dragonoids on 2/27/2017.
 */
@Disabled
@Autonomous(name="Auto Template", group="Blue")
public class AutoRedMat extends DragonoidsAuto{

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        while (opModeIsActive()) {

            break;
        }
    }
}
/*
package org.firstinspires.ftc.teamcode;

        import android.content.Context;
        import android.speech.tts.TextToSpeech;
        import android.util.Log;

        import java.util.Locale;

/**
 * Created by trucc on 3/1/2017.


public class RkrTTS {

    private static TextToSpeech tts;

    private static class TTSListener implements TextToSpeech.OnInitListener {

        @Override
        public void onInit(int initStatus) {
            if(initStatus == TextToSpeech.SUCCESS) {
                tts.setLanguage(Locale.US);
            } else {
                Log.d("RKR", "ERROR!");
            }
        }
    }

    public void init(Context context) {
        tts = new TextToSpeech(context, new TTSListener());
    }

    public void speakWords(String speech) {
        tts.speak(speech, TextToSpeech.QUEUE_ADD, null);
    }
}
 */