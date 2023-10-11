import subprocess
from getpass import getpass

def check_can0_interface():
    # Set up can0 interface
    sudo_password = getpass("Enter your sudo password: ")
    command = f"echo {sudo_password} | sudo -S ip link set can0 up type can bitrate 250000"
    subprocess.run(command, shell=True, check=True)

    # Get network interfaces
    result = subprocess.run(["ifconfig"], capture_output=True, text=True)
    interfaces = result.stdout

    # Check if can0 interface is present
    if "can0" in interfaces:
        print("can0 interface is present")
    else:
        print("can0 interface is absent")

# Call the function to check the can0 interface
check_can0_interface()