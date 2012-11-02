#!/usr/bin/env python
# coding=utf-8
# Author: Pedro I. López
# Contact: dreilopz@gmail.com

'''Numerical controller for the printer73x system.

**printerc** is an interactive command line interface for the numerical control
of the **printer73x** system.  printerc drives printerm through mcircuit;
printerc stablishes a serial connection with mcircuit, and mcircuit is coupled
to printerm through the motors.  mcircuit (actually MM12, its subsystem) loads
in its memory the routines that correspond to the translations across the
:math:`X`, :math:`Y` and :math:`Z` axis, and then printerc execute these
routines in order to produce the trajectory of the tool printing the image.
'''

# Standard library imports.
from __future__ import division
from pprint import pprint
import sys
import os
import atexit
import gc
import time

# Related third party imports.
import serial
import IPython
import numpy as np # remove
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import matplotlib.cm as cm

# Same as **printer73x**.
__version__ = '0.09'

# GLOBAL CONSTANT names.  *if main* section at bottom sets global names too.
# ==========================================================================
# ==========================================================================
LOGF = 'log.rst'
'''Path to the log file.'''

INTRO_MSG = '''\

**printerc**

Welcome!

'''
'''Welcome message for command line interface.'''

# MM12
# ==========================================================================
TRANSITIONS_PER_STEP = 2
'''The board sets the microstepping format with the jumpers *MS1* and *MS2*.
Use the following table to set this constant:

============ ============ ===============
MS1          MS2          TRANSITIONS_PER_STEP
============ ============ ===============
connected    connected    1
disconnected connected    2
connected    disconnected 4
disconnected disconnected 8
============ ============ ===============

.. note:: Both stepper motor driver boards must have the same jumper
          configuration.
'''

STEPS_PER_PIXEL = 90
'''Number of steps the stepper motor needs to translate 1 pixel across the
:math:`X` or :math:`Y` axes.'''

TRANSITIONS_PER_PIXEL = STEPS_PER_PIXEL * TRANSITIONS_PER_STEP
'''Number of low-to-high transitions the stepper motors need to translate 1
pixel across the :math:`X` or :math:`Y` axes.'''

SRV_SIGNAL_CHANNEL_TARGET_OFF = 940
''':math:`Z` axis servo motor pulse width in units of quarter-:math:`\\mu s`
that enables printing (moves the tool down).'''

SRV_SIGNAL_CHANNEL_TARGET_ON = 2175
SRV_SIGNAL_CHANNEL_TARGET_ON = 1580
''':math:`Z` axis servo motor pulse width in units of quarter-:math:`\\mu s`
that disables printing (moves the tool up).'''

STEPPER_CHANNELS_TARGET_ON = 6800
'''Target value in units of quarter-:math:`\\mu s` that drives the stepper
channels high.'''

STEPPER_CHANNELS_TARGET_OFF = 5600
'''Target value in units of quarter-:math:`\\mu s` that drives the stepper
channels low.'''

MM12_AXES_CHANNELS = {
    'X' : {
        'dir_channel' : 0,
        'dir_positive' : STEPPER_CHANNELS_TARGET_OFF,
        'dir_negative' : STEPPER_CHANNELS_TARGET_ON,
        'step_channel': 1,
    },
    'Y' : {
        'dir_channel' : 2,
        'dir_positive' : STEPPER_CHANNELS_TARGET_ON,
        'dir_negative' : STEPPER_CHANNELS_TARGET_OFF,
        'step_channel': 3,
    },
    'Z' : {
        'channel' : 4,
        'on'      : SRV_SIGNAL_CHANNEL_TARGET_OFF,
        'off'     : SRV_SIGNAL_CHANNEL_TARGET_ON,
    },
}
'''Configuration of the MM12 channels for the servo and stepper motors outputs.'''

SUB_STEPPER_PIXEL_TEMPLATE = '''sub {name}
  {{{{ntransitions}}}}
  {dir} {dir_channel} servo          # set direction
  begin
    dup
    while
    {off} {step_channel} servo
    {{delay}} delay
    {on} {step_channel} servo
    {{delay}} delay
    1 minus
  repeat
  quit
'''
'''Template for the MM12 script subroutines that drive a stepper motor in units
of pixels,'''

SUB_STEPPER_PULSE_TEMPLATE = '''sub {name}
  {dir} {dir_channel} servo          # set direction
  {off} {step_channel} servo
  {{delay}} delay
  {on} {step_channel} servo
  {{delay}} delay
  quit
'''
'''Template for the MM12 script subroutines that drive a stepper motor in units
of low-to-high transitions, for a precise but slow translation.'''

SUB_SERVO_TEMPLATE = '''sub {name}
  {position} {channel} servo
  begin
    get_moving_state
  while
    # wait until is is no longer moving.
  repeat
  75 delay
  quit
'''
'''Template for the MM12 script subroutine that drives the servo motor.'''

MM12_SCRIPT_INIT = '''\
{{servo_acceleration}} {servo_channel} acceleration
{{servo_speed}} {servo_channel} speed
'''.format(servo_channel=MM12_AXES_CHANNELS['Z']['channel'])
'''MM12 script initialization.'''

MM12_SUBROUTINES = {
    'X-p' : {
        'subroutine_id'       : 0,
        'subroutine_body' :
            SUB_STEPPER_PULSE_TEMPLATE.format(
                name='x_neg_pulse', dir=MM12_AXES_CHANNELS['X']['dir_negative'],
                dir_channel=MM12_AXES_CHANNELS['X']['dir_channel'],
                off=STEPPER_CHANNELS_TARGET_OFF,
                step_channel=MM12_AXES_CHANNELS['X']['step_channel'],
                on=STEPPER_CHANNELS_TARGET_ON),
    },
    'X+p' : {
        'subroutine_id'       : 1,
        'subroutine_body' :
            SUB_STEPPER_PULSE_TEMPLATE.format(
                name='x_pos_pulse', dir=MM12_AXES_CHANNELS['X']['dir_positive'],
                dir_channel=MM12_AXES_CHANNELS['X']['dir_channel'],
                off=STEPPER_CHANNELS_TARGET_OFF,
                step_channel=MM12_AXES_CHANNELS['X']['step_channel'],
                on=STEPPER_CHANNELS_TARGET_ON),
    },
    'X-P' : {
        'subroutine_id'       : 2,
        'subroutine_body' :
            SUB_STEPPER_PIXEL_TEMPLATE.format(
                name='x_neg_pixel', dir=MM12_AXES_CHANNELS['X']['dir_negative'],
                dir_channel=MM12_AXES_CHANNELS['X']['dir_channel'],
                off=STEPPER_CHANNELS_TARGET_OFF,
                step_channel=MM12_AXES_CHANNELS['X']['step_channel'],
                on=STEPPER_CHANNELS_TARGET_ON ),
    },
    'X+P' : {
        'subroutine_id'       : 3,
        'subroutine_body' :
            SUB_STEPPER_PIXEL_TEMPLATE.format(
                name='x_pos_pixel', dir=MM12_AXES_CHANNELS['X']['dir_positive'],
                dir_channel=MM12_AXES_CHANNELS['X']['dir_channel'],
                off=STEPPER_CHANNELS_TARGET_OFF,
                step_channel=MM12_AXES_CHANNELS['X']['step_channel'],
                on=STEPPER_CHANNELS_TARGET_ON ),
    },
    'Y-p' : {
        'subroutine_id'       : 4,
        'subroutine_body' :
            SUB_STEPPER_PULSE_TEMPLATE.format(
                name='y_neg_pulse', dir=MM12_AXES_CHANNELS['Y']['dir_negative'],
                dir_channel=MM12_AXES_CHANNELS['Y']['dir_channel'],
                off=STEPPER_CHANNELS_TARGET_OFF,
                step_channel=MM12_AXES_CHANNELS['Y']['step_channel'],
                on=STEPPER_CHANNELS_TARGET_ON),
    },
    'Y+p' : {
        'subroutine_id'       : 5,
        'subroutine_body' :
            SUB_STEPPER_PULSE_TEMPLATE.format(
                name='y_pos_pulse', dir=MM12_AXES_CHANNELS['Y']['dir_positive'],
                dir_channel=MM12_AXES_CHANNELS['Y']['dir_channel'],
                off=STEPPER_CHANNELS_TARGET_OFF,
                step_channel=MM12_AXES_CHANNELS['Y']['step_channel'],
                on=STEPPER_CHANNELS_TARGET_ON),
    },
    'Y-P' : {
        'subroutine_id'       : 6,
        'subroutine_body' :
            SUB_STEPPER_PIXEL_TEMPLATE.format(
                name='y_neg_pixel', dir=MM12_AXES_CHANNELS['Y']['dir_negative'],
                dir_channel=MM12_AXES_CHANNELS['Y']['dir_channel'],
                off=STEPPER_CHANNELS_TARGET_OFF,
                step_channel=MM12_AXES_CHANNELS['Y']['step_channel'],
                on=STEPPER_CHANNELS_TARGET_ON ),
    },
    'Y+P' : {
        'subroutine_id'       : 7,
        'subroutine_body' :
            SUB_STEPPER_PIXEL_TEMPLATE.format(
                name='y_pos_pixel', dir=MM12_AXES_CHANNELS['Y']['dir_positive'],
                dir_channel=MM12_AXES_CHANNELS['Y']['dir_channel'],
                off=STEPPER_CHANNELS_TARGET_OFF,
                step_channel=MM12_AXES_CHANNELS['Y']['step_channel'],
                on=STEPPER_CHANNELS_TARGET_ON ),
    },
    'Z-' : {
        'subroutine_id'       : 8,
        'subroutine_body' :
            SUB_SERVO_TEMPLATE.format(
                name='z_position_off',
                channel=MM12_AXES_CHANNELS['Z']['channel'],
                position=MM12_AXES_CHANNELS['Z']['off']*4)
    },
    'Z+' : {
        'subroutine_id'       : 9,
        'subroutine_body' :
            SUB_SERVO_TEMPLATE.format(
                name='z_position_on',
                channel=MM12_AXES_CHANNELS['Z']['channel'],
                position=MM12_AXES_CHANNELS['Z']['on']*4)
    },
}
'''Structure that builds and identifies the MM12 script subroutines.'''

MM12_SCRIPT_RUNNING = '\x00'
'''Byte value that the MM12 returns when the script is running.'''

MM12_SCRIPT_STOPPED = '\x01'
'''Byte value that the MM12 returns when the script is stopped.'''
# ==========================================================================

# ==========================================================================
# ==========================================================================

class Getch:
    """Gets a single character from standard input.  Does not echo to the
    screen.

    References
    ==========

    .. [GETCHRECIPE] http://code.activestate.com/recipes/134892/
    """
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self):
        return self.impl()

class _GetchUnix:
    '''Unix implementation of class ``Getch``.'''
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

class _GetchWindows:
    '''Windows implementation of class ``Getch``.'''
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()

def on_exit():
    '''Actions to do on exit.'''

    try:
        print >>logf, 'Closing ``{0}`` *command port*'.format(sp.port)
        sp.close()
    except NameError:
        pass

    print >>logf, 'END'
    logf.close()

    print '\nThanks for using ``printerc``!\n'

    # Invoke the garbage collector.
    gc.collect()

def scan_serial_ports():
    '''Scan system for available physical or virtual serial ports.

    Returns
    -------
    available : list of tuples
        Each element of the list is a ``(num, name)`` tuple with the number and
        name of the port.

    Notes
    -----
    Directly copied from example from `pyserial
    <http://sourceforge.net/projects/pyserial/files/package>`_ project.
    '''
    available = []
    for i in range(256):
        try:
            s = serial.Serial(i)
            available.append( (i, s.portstr))
            s.close()
        except serial.SerialException:
            pass
    return available

def mm12_script_status():
    '''Indicate whether the MM12 script is running or stopped.

    Returns
    -------
    script_status : {``MM12_SCRIPT_RUNNING``, ``MM12_SCRIPT_STOPPED``}
    '''
    assert sp.write('\xae') == 1
    return sp.read(1)

def translate(adm, confirm=False):
    '''Translate the printerm tool across the :math:`XYZ` space.

    printer73x can only perform translations across a single axis at a time.

    Parameters
    ----------

    adm: str
        *adm* stands for Axis, Direction, Mode.  Use the following table to
        select the kind of translation you want to perform (where :math:`n` is
        the number of pulses for the printerm tool to translate a pixel unit
        across the respective axis).

        ======= ================================================================
        *adm*   translation
        ======= ================================================================
        ``X-p`` send 1 single pulse for negative translation across :math:`X`.
        ``X+p`` send 1 single pulse for positive translation across :math:`X`.
        ``X-P`` send :math:`n` pulses for negative translation across :math:`X`.
        ``X+P`` send :math:`n` pulses for positive translation across :math:`X`.
        ``Y-p`` send 1 single pulse for negative translation across :math:`Y`.
        ``Y+p`` send 1 single pulse for positive translation across :math:`Y`.
        ``Y-P`` send :math:`n` pulses for negative translation across :math:`Y`.
        ``Y+P`` send :math:`n` pulses for positive translation across :math:`Y`.
        ``Z-``  move the tool to the off position (:math:`Z`).
        ``Z+``  move the tool to the on position (:math:`Z`).
        ======= ================================================================
    confirm: boolean, optional
        If ``True``, the user must confirm the translation by pressing Enter
        (default is ``False``).
    '''
    # Start until script is not running.
    while mm12_script_status() == MM12_SCRIPT_RUNNING:
        pass

    subroutine_id = chr( MM12_SUBROUTINES[adm]['subroutine_id'])
    str2write = ''.join(['\xa7', subroutine_id])
    if confirm:
        raw_input()
    assert sp.write(str2write) == 2

    #if 'Z' in adm:
        #time.sleep(0.1)

def build_mm12_script(fpath, ntransitions=TRANSITIONS_PER_PIXEL, delay=1,
                      servo_acceleration=0, servo_speed=100):
    '''Build a script to be loaded on the MM12.

    Parameters
    ----------
    fpath : str-like
        Path location where to save the script file.
    ntransitions : int, optional
        Number of low-to-high transitions to perform in the subroutines that
        performs translation in units of pixels through the stepper motor
        (default is ``TRANSITIONS_PER_PIXEL``).
    delay : int, optional
        Delay (in milliseconds) between each transition in the subroutines that
        perform translation through the stepper motors (default is 1).
    servo_acceleration : int, optional
        Sets the acceleration of the servo signal channel in units of (0.25
        us)/(10 ms)/(80 ms) (default is 0).
    servo_speed : int, optional
        Sets the speed of the servo signal channel in units of (0.25 us)/(10
        ms) (default is 100).
    '''

    def get_subroutine_key_by_id(subroutine_id):
        for key, value in MM12_SUBROUTINES.items():
            if subroutine_id is value['subroutine_id']:
                return key

    for intarg in (ntransitions, delay, servo_acceleration, servo_speed):
        assert isinstance(intarg, int)

    with open(fpath, 'w') as f:
        print >>f, MM12_SCRIPT_INIT.format(servo_acceleration=servo_acceleration,
                                       servo_speed=servo_speed)

        for i in range(len( MM12_SUBROUTINES)):
            subroutine_key = get_subroutine_key_by_id(i)
            subroutine_body = MM12_SUBROUTINES[subroutine_key]['subroutine_body']

            if 'Z' not in subroutine_key:
                subroutine_body = subroutine_body.format(delay=delay)

                if 'P' in subroutine_key:
                    subroutine_body = subroutine_body.format(ntransitions=ntransitions)

            print >>f, subroutine_body

def prepare_img(imgpath, invert=False, show=False):
    '''Perform any necessary processing for the input image to be reproduced by
    printerm.

    Parameters
    ----------
    imgpath : str-like
        Path to the image file.  Must be PNG, 8-bit grayscale, non-interlaced.
    invert : boolean, optional
        Invert the image if ``True`` (default is ``False``).
    show : boolean, optional
        Show the image if ``True`` (default is ``False``).

    Notes
    -----
    This function sets the following global names:

    **img** : array of booleans
        2-d array representation of the image.
    **b** : int
        Image's height, number of rows in the array representation.
    **w** : int
        Image's width, number of columns in the array representation.
    '''
    global img, b, w
    print 'Loading ``{0}``...'.format(imgpath)
    img = mpimg.imread(fname=imgpath, format='png')
    b, w = img.shape
    npixels = b * w
    nprints = nnotprints = 0
    assert (b > 0) and (w > 0)

    print 'Processing the image...'
    # only total black and white, no grays.
    for i in range(b):
        for j in range(w):
            if img[i][j] < 0.9:
                img[i][j] = 0.0
            else:
                img[i][j] = 1.0

    if invert:
        print 'Inverting image...'
        for i in range(b):
            for j in range(w):
                if img[i][j] > 0.0:
                    img[i][j] = 0.0
                else:
                    img[i][j] = 1.0

    # Check for pixel with and without color.
    for i in range(b):
        for j in range(w):
            if img[i][j] > 0.0:
                nnotprints += 1
            else:
                nprints += 1
    assert (nnotprints + nprints) == npixels

    # If ``nprints == 0`` then no pixel will be printed.
    assert nprints > 0

    print 'Loaded ``{0}`` with {1} pixels, {2} of which have color'.format(
             imgpath, npixels, nprints)
    plt.close('all')
    if show:
        plt.imshow(img, cmap=cm.gray)
        plt.show()

def connect_printerm(commandport_id):
    '''Connect printerc with printerm through the MM12 command port.

    Parameters
    ----------
    commandport_id : str or int
        Serial device name or port number number of the MM12 serial command
        port.
    '''

    global sp

    sp = serial.Serial(port=commandport_id)
    assert sp.isOpen()
    print >>logf, '``{0}`` just opened *command port* ``{1}``'.format(PN, sp.port)


    msg = '``{0}`` is now connected to ``printerm`` through ``{1}``'.format(
        PN, sp.port)
    for f in (logf, sys.stdout):
        print >>f, msg

def manual_translation_mode(precise=True):
    '''Manually translate the printerm tool across the :math:`XY` plane.

    Parameters
    ----------
    precise : boolean, optional
        If ``True``, perform translation in units of single low-to-high transitions
        sent to the stepper motor drivers (how much the tool is translated
        depends on the microstep format selected through the XMS1, XMS2, YMS1,
        YMS2 jumpers in mcircuit).  If ``False`` perform translation in units
        of pixels (default is True).
    '''

    if precise:
        keys2translation = {
            'h' : 'X-p',
            'l' : 'X+p',
            'j' : 'Y+p',
            'k' : 'Y-p',
            'i' : 'Z+',
            'o' : 'Z-',
        }
    else:
        keys2translation = {
            'h' : 'X-P',
            'l' : 'X+P',
            'j' : 'Y+P',
            'k' : 'Y-P',
            'i' : 'Z+',
            'o' : 'Z-',
        }

    getch = Getch()

    while True:
        ch = getch()
        if ch not in keys2translation.keys():
            break
        while mm12_script_status() == MM12_SCRIPT_RUNNING:
            pass
        translate(keys2translation[ch], confirm=False)

def print_pixel(confirm=False):
    if confirm:
        raw_input()
    translate('Z+', confirm=False)
    translate('Z-', confirm=False)

def print_image(confirm=False):
    def report_position(x, y):
        print 'At row {0}, column {1}'.format(y, x)

    def print_img_pixel(x, y, confirm=False):
        if img[y][x] == 0.0:
            print_pixel(confirm)

    try:

        msg = 'Preparing to print an image with {0} rows and {1} columns'.format(b,
                                                                                 w)
        print msg

        x = y = 0   # We are at HOME position.
        while True:

            print 'Printing across the row {0}'.format(y)
            while True:
                report_position(x, y)
                print_img_pixel(x, y, confirm)
                if x == w - 1:
                    break
                translate('X+P', confirm)
                x += 1

            print 'Returning to a_{{{0}, 0}}'.format(y)
            while True:
                if x == 0:
                    break
                translate('X-P', confirm)
                x -= 1

            if y == b - 1:
                break
            translate('Y+P', confirm)
            y += 1

        print 'Returning to a_{{0, 0}}'.format(y)
        while True:
            if y == 0:
                break
            translate('Y-P', confirm)
            y -= 1

        print 'The image has been printed'

    except KeyboardInterrupt:
        sp.flush()
        print 'Operation interrupted, flushing command port'

def print_image_better(confirm=False):
    def report_position(x, y):
        print 'At row {0}, column {1}'.format(y, x)

    def print_img_pixel(x, y, confirm=False):
        if img[y][x] == 0.0:
            print_pixel(confirm)

    def color_in_this_row(row):
        for pixel in row:
            if pixel == 0.0:
                return True
        return False

    try:

        msg = 'Preparing to print an image with {0} rows and {1} columns'.format(b,
                                                                                 w)
        print msg

        x = y = 0   # We are at HOME position.
        while True:

            print 'Printing across the row {0}'.format(y)
            while True:
                report_position(x, y)
                print_img_pixel(x, y, confirm)
                if x == w - 1:
                    break
                translate('X+P', confirm)
                x += 1

            if y == b - 1:
                break
            translate('Y+P', confirm)
            y += 1

            print 'Printing across the row {0}'.format(y)
            while True:
                report_position(x, y)
                print_img_pixel(x, y, confirm)
                if x == 0:
                    break
                translate('X-P', confirm)
                x -= 1

            if y == b - 1:
                break
            translate('Y+P', confirm)
            y += 1

        print 'Returning to a_{{0, 0}}'.format(y)
        while True:
            if y == 0:
                break
            translate('Y-P', confirm)
            y -= 1
        while True:
            if x == 0:
                break
            translate('X-P', confirm)
            x -= 1

        print 'The image has been printed'

    except KeyboardInterrupt:
        sp.flush()
        print 'Operation interrupted, flushing command port'

def print_image_better_better(confirm=False):
    '''Automatically print the input image.

    Parameters
    ----------

    confirm : boolean, optional
        Wait for confirmation before any translation (default is ``False``).

    Notes
    -----

    Let :math:`\mathbf{A}` be the matrix representation of the input image.
    :math:`a_{y,x}` as an element of :math:`\mathbf{A}`, represents a pixel.
    Comparing :math:`\mathbf{A}` with the input image from a front perspective:

    #. :math:`a_{0,0}` corresponds to the upper left corner of the input image.

    #. :math:`a_{b-1,w-1}` corresponds to the lower right corner of the input
       image.

    Starting from the HOME position the printerm tool visit every element of
    the row from :math:`a_{0,0}` to :math:`a_{0,w-1}` then moves to the next
    row (:math:`a_{1,w-1}`) and visits every element of the row from
    :math:`a_{1,w-1}` to :math:`a_{1,0}`, and so on until there are no more
    rows to visit.  In any position, if the corresponding pixel is black then
    the tool prints it.

    '''
    def report_position(x, y):
        print 'At row {0}, column {1}'.format(y, x)

    def print_img_pixel(x, y, confirm=False):
        if img[y][x] == 0.0:
            print_pixel(confirm)

    def color_in_this_row(row):
        for pixel in row:
            if pixel == 0.0:
                return True
        return False

    try:

        msg = 'Preparing to print an image with {0} rows and {1} columns'.format(b,
                                                                                 w)
        print msg

        x = y = z = 0   # We are at HOME position.
        while True:

            # To the right.
            print 'Printing across the row {0}'.format(y)
            while True:
                report_position(x, y)
                if img[y][x] == 0.0:
                    translate('Z+', confirm)
                try:
                    if img[y][x+1] != 0.0:
                        translate('Z-', confirm)
                except IndexError as e:
                    pass
                if x == w - 1:
                    translate('Z-')
                    break
                translate('X+P', confirm)
                x += 1

            if y == b - 1:
                translate('Z-')
                break
            translate('Y+P', confirm)
            y += 1

            # To the left.
            print 'Printing across the row {0}'.format(y)
            while True:
                report_position(x, y)
                if img[y][x] == 0.0:
                    translate('Z+', confirm)
                if img[y][x-1] != 0.0:
                    translate('Z-', confirm)
                if x == 0:
                    translate('Z-')
                    break
                translate('X-P', confirm)
                x -= 1

            if y == b - 1:
                translate('Z-')
                break
            translate('Y+P', confirm)
            y += 1

        print 'Returning to a_{{0, 0}}'.format(y)
        while True:
            if y == 0:
                break
            translate('Y-P', confirm)
            y -= 1
        while True:
            if x == 0:
                break
            translate('X-P', confirm)
            x -= 1

        print 'The image has been printed'

    except KeyboardInterrupt:
        sp.flush()
        print 'Operation interrupted, flushing command port'

if __name__ == "__main__":
    # program name from file name.
    PN = os.path.splitext(sys.argv[0])[0]

    logf = open(LOGF, 'w')
    print >>logf, 'START'
    atexit.register(on_exit)
    IPython.Shell.IPShellEmbed()( INTRO_MSG)
