# Music player

[Documentation index](README.md)

Open an NSF or NSFe file to play its music through the normal CPU, APU, and
expansion-sound implementations. The window title shows the file title, selected
track, and elapsed emulated time. The Music Player panel exposes the title,
artist, copyright, ripper, track names, position, and transport controls.

## Transport

| Control | Shortcut | Behavior |
| --- | --- | --- |
| Play / Pause | Ctrl+Space | Pause or resume the selected track |
| Stop | Ctrl+End | Restart the selected track and leave playback paused |
| Next track | Page Up | Select the next track, wrapping after the last |
| Previous track | Page Down | Select the previous track during the first two seconds; otherwise restart the current track |
| Restart track | Ctrl+Home | Restart the current track without changing its pause state |
| Track selector | Music Player panel | Select a named or numbered track directly |

Pause uses the application's execution control. It does not advance the CPU or
track position. Manual track changes reset the music program and clear its
queued audio under the audio-device lock. Selecting a track while paused keeps
it paused. Music shortcuts are available only while an NSF or NSFe file is
loaded.

## Automatic track changes

Automatic progression is enabled initially. For an NSFe track with a positive
`time` value, playback continues through that duration and the positive `fade`
duration. The existing mixer applies the fade to the music output. At the next
completed emulation frame, the player selects the next track. Audio already
queued for the completed track remains ahead of the new track's samples.

An NSF track without timing metadata has no assumed duration. Silence detection
can advance it: both output channels must remain at or below the configured
amplitude for the configured delay. The defaults are 0.0005 on the normalized
audio scale and 3,000 milliseconds. The detector observes the complete stereo
mix, including expansion sound. Pause does not add to the silence interval, and
changing emulation speed does not change the interval measured in emulated time.

The Music Player panel controls automatic progression, silence detection, its
delay and threshold, repeat, and shuffle. Repeat restarts the selected track.
Shuffle chooses a different track when the file contains more than one; it does
not reorder or modify the file. Repeat takes precedence when both are enabled.
Track changes wrap at either end during ordinary sequential playback.

Loading another image or restoring a session clears the silence history.
Automatic and manual music transport are unavailable during movie recording,
playback, netplay, rewind, and speculative execution. The host player state does
not become part of an emulated hardware snapshot.

## File support

NSFe author, track-label, timing, fade, and bank metadata are read by the music
loader. Playlist-order and free-text chunks are accepted but do not change the
track order or add a text display. Unknown required chunks are rejected. Music
images may combine VRC6, VRC7, FDS, MMC5, Namco 163, and Sunsoft 5B audio.

The [hardware guide](hardware.md) describes music banking, initialization,
regional play timing, and the expansion chips. The music-player regression
groups exercise transport and panel actions, duration and fade boundaries,
queued stereo continuity, repeat and shuffle, expansion-audio silence detection,
and isolation from replay and speculative execution.
