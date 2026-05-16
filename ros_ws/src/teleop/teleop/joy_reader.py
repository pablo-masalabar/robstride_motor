import os
import sys
import time

# SDL must be told to use dummy drivers before pygame.init() is called,
# otherwise it tries to open a display/audio device which fails in a container.
os.makedirs('/tmp/xdg-runtime', exist_ok=True)
os.environ.setdefault('XDG_RUNTIME_DIR', '/tmp/xdg-runtime')
os.environ.setdefault('SDL_VIDEODRIVER', 'offscreen')
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')

import pygame


def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print('No joystick detected.')
        sys.exit(1)

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f'Joystick: {js.get_name()}')
    print(f'  axes={js.get_numaxes()}  buttons={js.get_numbuttons()}  hats={js.get_numhats()}')
    print()

    _last_t = time.monotonic()
    _hz     = 0.0

    try:
        while True:
            pygame.event.pump()

            now   = time.monotonic()
            dt    = now - _last_t
            _last_t = now
            _hz   = 1.0 / dt if dt > 0 else 0.0

            axes    = '  '.join(f'a[{i}]={js.get_axis(i):+.2f}'   for i in range(js.get_numaxes()))
            buttons = '  '.join(f'b[{i}]={js.get_button(i)}'       for i in range(js.get_numbuttons()))
            hats    = '  '.join(f'h[{i}]={js.get_hat(i)}'          for i in range(js.get_numhats()))

            print(f'HZ      {_hz:.1f}')
            print(f'AXES    {axes}')
            print(f'BUTTONS {buttons}')
            if js.get_numhats():
                print(f'HATS    {hats}')
            print()

            pygame.time.wait(50)   # 20 Hz print rate

    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()


if __name__ == '__main__':
    main()
