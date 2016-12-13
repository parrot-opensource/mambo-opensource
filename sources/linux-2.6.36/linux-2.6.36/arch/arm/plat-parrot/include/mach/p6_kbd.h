#ifndef _P6_KBD_H_
#define _P6_KBD_H_

struct p6_platform_kbd_input {
    int gpio;
    int keycode;
    int debounced;
    int no_long_press;
    //Will send Down and Up Event when Button will be pressed
    //else will wait the Up Event when it really happens
    int inverted; //1 means yes
    int delay; //0 means send all events
    int max_pressed_time;
    //0 means do not use this
    //else in case the button is pressed for more than this time no message are send
    //This add a delay before sending any pressed down event unit are microsecond
    //It's not compatible with "delay" settings
};
#endif
