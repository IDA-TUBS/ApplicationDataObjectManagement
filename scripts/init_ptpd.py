# PTPD initialization for ADOM evaluation
import subprocess
import argparse
import time

import caching_node as nodes

PTPD_MASTER_CMD = 'sudo ptpd -M -i '
PTPD_MASTER_CMD_SUFFIX = ' -d 1 --e2e -r=-7'

PTPD_SLAVE_CMD = 'sudo ptpd -s -i '
PTPD_SLAVE_CMD_SUFFIX = ' -d 1 --e2e'

PTPD_LOG = '-S ~/ptp_logs/ptpd_stat.log'

SETUP_TIME = 10*60 #sec

# Define launch arguments
parser = argparse.ArgumentParser(description='Configure ptpd setup', formatter_class=argparse.RawTextHelpFormatter)
parser.add_argument('-m', '--master', action='store_true', 
                    help="ptpd master flag")
parser.add_argument('-node', dest='node', default=None, choices=([system.label for system in nodes.system_dict.values()]), type=str,
                    help="""specifies on which node the setup script is executed\nAvailable systems:\n\t""" +
                            '\n\t'.join([system.label for system in nodes.system_dict.values()]))
launch_options = parser.parse_args()

# Role independent commands
ptpd_setup_commands = [
    'sudo pkill ptpd',
    'sudo rm -rf ptp_logs/*'
]

print('Selected node: ' + launch_options.node)

# Check which role was assigned to this node and append the respective command
if(launch_options.master):
    ptpd_setup_commands.append(PTPD_MASTER_CMD + nodes.system_dict[launch_options.node].ethernet_interface + PTPD_MASTER_CMD_SUFFIX + ' ' + PTPD_LOG)
else:
    ptpd_setup_commands.append(PTPD_SLAVE_CMD + nodes.system_dict[launch_options.node].ethernet_interface + PTPD_SLAVE_CMD_SUFFIX + ' ' + PTPD_LOG)

print(ptpd_setup_commands)

# Execute all ptpd commands
for command in ptpd_setup_commands:
    output = subprocess.run(command, shell=True, capture_output=True)
    print("\033[37m" + output.args + "\033[0m")
    if(output.stdout):
        print("\033[32mstdout: %s\033[0m" %output.stdout.decode('utf-8'), end="")
    if(output.stderr):
        print("\033[31mstderr: %s\033[0m" %output.stderr.decode('utf-8'), end="")

time.sleep(SETUP_TIME)

print('PTPD Setup complete')