#!/usr/bin/env python3

from pynput import keyboard
import curses
from time import sleep
from class1 import FourXFourBotControl


def main(stdscr):
    def on_press(key):
        if key in pressed_keys and not pressed_keys[key]:
            pressed_keys[key] = True
            # X
            if key == keyboard.Key.up:
                preset_vel[0] = 'UP'
            elif key == keyboard.Key.down:
                preset_vel[0] = 'DOWN'
            # Y
            elif key == keyboard.Key.right:
                preset_vel[1] = 'RIGHT'
            elif key == keyboard.Key.left:
                preset_vel[1] = 'LEFT'

    def on_release(key):
        if key in pressed_keys and pressed_keys[key]:
            pressed_keys[key] = False
            # X
            if key in (keyboard.Key.up, keyboard.Key.down):
                if not pressed_keys[keyboard.Key.up] and not pressed_keys[keyboard.Key.down]:
                    preset_vel[0] = 'NONE'
                elif pressed_keys[keyboard.Key.up]:
                    preset_vel[0] = 'UP'
                elif pressed_keys[keyboard.Key.down]:
                    preset_vel[0] = 'DOWN'
            # Y
            elif key in (keyboard.Key.right, keyboard.Key.left):
                if not pressed_keys[keyboard.Key.right] and not pressed_keys[keyboard.Key.left]:
                    preset_vel[1] = 'NONE'
                elif pressed_keys[keyboard.Key.right]:
                    preset_vel[1] = 'RIGHT'
                elif pressed_keys[keyboard.Key.left]:
                    preset_vel[1] = 'LEFT'

    # CURSES
    curses.curs_set(0)
    curses.use_default_colors()
    curses.init_pair(1, -1, 2)
    curses.init_pair(2, 2, -1)
    stick = curses.newwin(5, 10, 1, 2)
    info_win = curses.newwin(1, 11, 7, 2)

    # Static graphics
    stick.border()
    stick.refresh()

    # KEYBOARD
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()

    FFBC = FourXFourBotControl()

    while True:
        try:
            st_x = 0
            if preset_vel[1] == 'LEFT':
                st_x = -2
            if preset_vel[1] == 'RIGHT':
                st_x = 2
            st_x += 4
            st_y = 0
            if preset_vel[0] == 'UP':
                st_y = -1
            if preset_vel[0] == 'DOWN':
                st_y = 1
            st_y += 2
            for i in range(1, 4):
                stick.addstr(i, 1, '        ')
            stick.addstr(st_y, st_x, '  ', curses.color_pair(1))
            stick.refresh()

            while preset_vel[0] == 'UP' and preset_vel[1] == 'LEFT':
                FFBC.set_wheels_frequency(-30, 60, 60, -30)
                sleep(0.01)
            while preset_vel[0] == 'UP' and preset_vel[1] == 'NONE':
                FFBC.set_wheels_frequency(-60, 60, 60, -60)
                sleep(0.01)
            while preset_vel[0] == 'UP' and preset_vel[1] == 'RIGHT':
                FFBC.set_wheels_frequency(-60, 30, 30, -60)
                sleep(0.01)

            while preset_vel[0] == 'NONE' and preset_vel[1] == 'LEFT':
                FFBC.set_wheels_frequency(60, 60, 60, 60)
                sleep(0.01)
            while preset_vel[0] == 'NONE' and preset_vel[1] == 'NONE':
                FFBC.set_wheels_frequency(0, 0, 0, 0)
                sleep(0.01)
            while preset_vel[0] == 'NONE' and preset_vel[1] == 'RIGHT':
                FFBC.set_wheels_frequency(-60, -60, -60, -60)
                sleep(0.01)

            while preset_vel[0] == 'DOWN' and preset_vel[1] == 'LEFT':
                FFBC.set_wheels_frequency(30, -60, -60, 30)
                sleep(0.01)
            while preset_vel[0] == 'DOWN' and preset_vel[1] == 'NONE':
                FFBC.set_wheels_frequency(60, -60, -60, 60)
                sleep(0.01)
            while preset_vel[0] == 'DOWN' and preset_vel[1] == 'RIGHT':
                FFBC.set_wheels_frequency(60, -30, -30, 60)
                sleep(0.01)
            info_win.addstr(0, 0, '          ')
            info_win.addstr(0, 0, preset_vel[0])
            info_win.addstr(0, 5, preset_vel[1])
            info_win.refresh()

        except KeyboardInterrupt:
            for i in range(10):
                FFBC.set_wheels_frequency(0, 0, 0, 0)
                info_win.addstr(0, 0, 'NONE NONE ')
                info_win.refresh()
                sleep(0.1)
            break


if __name__ == '__main__':
    preset_vel = ['NONE', 'NONE']

    pressed_keys = {keyboard.Key.up: False,
                    keyboard.Key.down: False,
                    keyboard.Key.right: False,
                    keyboard.Key.left: False}

    curses.wrapper(main)
