import os

filepath = r"C:\Users\jiho0\GitProjects\FPGA-Autonomous-Vehicle-Project\CARLA_FPGA_PROJECT\main.py"
with open(filepath, 'r', encoding='utf-8') as f:
    lines = f.readlines()

insert_idx = -1
for i, line in enumerate(lines):
    if "perception = perception_manager.update()" in line:
        insert_idx = i + 1
        break

fpga_start_idx = -1
fpga_end_idx = -1
for i, line in enumerate(lines):
    if "[FPGA AXI REGISTER PACKING - TEST]" in line:
        fpga_start_idx = i - 1
        break

for i in range(fpga_start_idx, len(lines)):
    line = lines[i]
    if "fpga_hazard_auto_out =" in line:
        fpga_end_idx = i + 1
        break

print(f"Indices: insert_idx={insert_idx}, fpga_start_idx={fpga_start_idx}, fpga_end_idx={fpga_end_idx}")

if insert_idx != -1 and fpga_start_idx != -1 and fpga_end_idx != -1:
    fpga_lines = lines[fpga_start_idx:fpga_end_idx]
    
    for j in range(len(fpga_lines)):
        fpga_lines[j] = fpga_lines[j].replace("command.throttle", "manual_command.throttle")
        fpga_lines[j] = fpga_lines[j].replace("command.brake", "manual_command.brake")
        fpga_lines[j] = fpga_lines[j].replace("command.manual_mode", "keyboard.manual_mode")
        fpga_lines[j] = fpga_lines[j].replace("command.headlight", "manual_command.headlight")
        fpga_lines[j] = fpga_lines[j].replace("command.hazard", "manual_command.hazard")
    
    prepended_lines = [
        "\n",
        "            # [FPGA Data Prep]\n",
        "            control = vehicle.get_control()\n",
        "            manual_command = VehicleCommand()\n",
        "            keyboard.update(manual_command, sensor.speed)\n",
        "\n"
    ]
    fpga_lines = prepended_lines + fpga_lines
    
    del lines[fpga_start_idx:fpga_end_idx]
    
    lines = lines[:insert_idx] + fpga_lines + lines[insert_idx:]
    
    for i in range(len(lines)):
        if "manual_command = VehicleCommand()" in lines[i] and "FPGA Data Prep" not in lines[i-1]:
            lines[i] = "            # (Moved to top for FPGA)\n"
        if "keyboard.update(manual_command, sensor.speed)" in lines[i] and "FPGA Data Prep" not in lines[i-2] and "FPGA Data Prep" not in lines[i-1]:
            lines[i] = "            # (Moved to top for FPGA)\n"
        if "control = vehicle.get_control()" in lines[i] and "FPGA Data Prep" not in lines[i-1]:
            lines[i] = "            # control = vehicle.get_control() (Moved to top)\n"

    with open(filepath, 'w', encoding='utf-8') as f:
        f.writelines(lines)
    print("Success")
else:
    print("Failed to find indices")
