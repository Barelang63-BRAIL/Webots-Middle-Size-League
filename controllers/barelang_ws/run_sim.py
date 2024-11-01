import subprocess
import yaml
from icecream import ic

def load_programs(config_file):
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    return config['programs']

def run_program(program):
    # Separate the command and arguments correctly
    command = program['command'].split() + program['args'].split()
    ic(command)  # Debugging: Print the command to ensure it's correct
    try:
        process = subprocess.Popen(command, shell=False)
        return process
    except FileNotFoundError as e:
        print(f"Error: {e}")
        return None

def main(config_file):
    programs = load_programs(config_file)
    processes = []

    for program in programs:
        print(f"Starting {program['name']}...")
        process = run_program(program)
        if process:
            processes.append(process)

    # Optionally, wait for all processes to complete
    for process in processes:
        process.wait()

if __name__ == "__main__":
    main('config/run_program.yaml')
