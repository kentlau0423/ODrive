
import sys
import platform
import threading
import fibre
import odrive
import odrive.enums
from odrive.utils import calculate_thermistor_coeffs, set_motor_thermistor_coeffs, start_liveplotter, dump_errors, oscilloscope_dump, BulkCapture, step_and_plot

def print_banner():
    print("Website: https://odriverobotics.com/")
    print("Docs: https://docs.odriverobotics.com/")
    print("Forums: https://discourse.odriverobotics.com/")
    print("Discord: https://discord.gg/k3ZZ3mS")
    print("Github: https://github.com/madcowswe/ODrive/")

    print()
    print('Please connect your ODrive.')
    print('You can also type help() or quit().')

def print_help(args, have_devices):
    print('')
    if have_devices:
        print('Connect your ODrive to {} and power it up.'.format(args.path))
        print('After that, the following message should appear:')
        print('  "Connected to ODrive [serial number] as odrv0"')
        print('')
        print('Once the ODrive is connected, type "odrv0." and press <tab>')
    else:
        print('Type "odrv0." and press <tab>')
    print('This will present you with all the properties that you can reference')
    print('')
    print('For example: "odrv0.axis0.encoder.pos_estimate"')
    print('will print the current encoder position on axis 0')
    print('and "odrv0.axis0.controller.input_pos = 0.5"')
    print('will send axis 0 to 0.5 turns')
    print('')


interactive_variables = {}

discovered_devices = []

def benchmark(odrv):
    import asyncio
    import time

    async def measure_async():
        start = time.monotonic()
        futures = [odrv.vbus_voltage for i in range(1000)]
#        data = [await f for f in futures]
#        print("took " + str(time.monotonic() - start) + " seconds. Average is " + str(sum(data) / len(data)))

    fibre.libfibre.libfibre.loop.call_soon_threadsafe(lambda: asyncio.ensure_future(measure_async()))

def launch_shell(args, logger, app_shutdown_token):
    """
    Launches an interactive python or IPython command line
    interface.
    As ODrives are connected they are made available as
    "odrv0", "odrv1", ...
    """

    interactive_variables = {
        'start_liveplotter': start_liveplotter,
        'dump_errors': dump_errors,
        'benchmark': benchmark,
        'oscilloscope_dump': oscilloscope_dump,
        'BulkCapture': BulkCapture,
        'step_and_plot': step_and_plot,
        'calculate_thermistor_coeffs': calculate_thermistor_coeffs,
        'set_motor_thermistor_coeffs': set_motor_thermistor_coeffs
    }

    # Expose all enums from odrive.enums
    interactive_variables.update({k: v for (k, v) in odrive.enums.__dict__.items() if not k.startswith("_")})

    fibre.launch_shell(args,
                       interactive_variables,
                       print_banner, print_help,
                       logger, app_shutdown_token,
                       branding_short="odrv", branding_long="ODrive")
