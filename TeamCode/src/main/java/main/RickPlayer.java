import android.media.MediaPlayer;
public class RickPlayer {
  //The player handling the audio
  private static MediaPlayer mediaPlayer = null;
  //Start the wubs
  public static void start(Context context) {
    if (mediaPlayer == null) mediaPlayer = MediaPlayer.create(context, R.raw.rick);
    mediaPlayer.seekTo(0);
    mediaPlayer.start();
  }
  //Stop the wubs
  public static void stop() {
    if (mediaPlayer != null) {
      mediaPlayer.stop();
      try { mediaPlayer.prepare(); }
      catch (IOException e) {}
    }
  }
}
public class SoundOpMode extends OpMode {
}