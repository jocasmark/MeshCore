#pragma once

#include <Arduino.h>
#include <NonBlockingRtttl.h>

/* class abstracts underlying RTTTL library 

    Just a simple imlementation to start.  At the moment use same
    melody for message and discovery
    Suggest enum type for different sounds
    - on message
    - on discovery

    TODO
    - make message ring tone configurable

*/

class genericBuzzer
{
    public:
        void begin();  // set up buzzer port
        void play(const char *melody); // Generic play function
        void loop();  // loop driven-nonblocking
        void startup();  // play startup sound
        void shutdown();  // play shutdown sound
        bool isPlaying();  // returns true if a sound is still playing else false
        void quiet(bool buzzer_state);  // enables or disables the buzzer
        bool isQuiet();  // get buzzer state on/off

    private:
        // gemini's picks:
        const char *startup_song = "Startup:b=210,o=3,d=4:8d5,8e5,8g5,8d5,b5,8p,b5,8p,2a5,p";
        const char *shutdown_song = "Shutdown:d=4,o=5,b=100:8g5,16e5,16c5";

        bool _is_quiet = true;
};
