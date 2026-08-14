import csv
import random

def generate_scenarios():
    filename = "scenarios/axi_test_vectors.csv"
    
    # We want to test AXI memory mapped registers.
    # AXI mapping:
    # Reg0: sensor_data_in (distance[7:0], approach_speed[15:8], accel_x[23:16], accel_y[31:24])
    # Reg1: accel_z[7:0], gyro_x[15:8], gyro_y[23:16], gyro_z[31:24]
    # Reg2: temperature[7:0], humidity[15:8], lux[23:16]
    # Reg3: sim_data_in (speed_x[13:0], speed_y[27:14])
    # Reg4: speed_z[13:0], incline_x[27:14]
    # Reg5: incline_y[13:0], incline_z[27:14]
    # Reg6: steering[7:0], accelerator[11:8], brake[15:12], gear[17:16], headlight, hazard, manual_mode, situation[23:21], speed_limit[31:24]
    # Reg7: speed_limit[12:8] (top 5 bits in lower part)
    # Reg8: sample_seq_in
    # Reg9: valid_risk_rel
    
    # We will generate raw data fields. The testbench will pack them into AXI words.
    
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Header for the TB to parse or just for reference
        # writer.writerow(["distance", "approach_speed", "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z", 
        #                  "temp", "hum", "lux", 
        #                  "speed_x", "speed_y", "speed_z", "incline_x", "incline_y", "incline_z",
        #                  "steering", "accel", "brake", "gear", "hl", "haz", "manual", "sit", "spd_lim"])
        
        # 1. Normal driving (10 cases)
        for i in range(10):
            writer.writerow([50+i*5, 10, 0, 0, 0, 0, 0, 0,  25, 40, 100,  60, 0, 0, 0, 0, 0,  0, 2, 0, 1, 0, 0, 0, 0, 80])
            
        # 2. Risk collision approach (10 cases: speed high, distance small)
        for i in range(10):
            writer.writerow([10, 80, 0, 0, 0, 0, 0, 0,  25, 40, 100,  80, 0, 0, 0, 0, 0,  0, 0, 5, 1, 0, 0, 0, 0, 80])

        # 3. Posture abnormalities (incline high) (10 cases)
        for i in range(10):
            writer.writerow([50, 10, 0, 0, 0, 0, 0, 0,  25, 40, 100,  60, 0, 0, 300, 0, 0,  0, 0, 2, 1, 0, 0, 0, 0, 80])

        # 4. Steering deviations (10 cases)
        for i in range(10):
            writer.writerow([50, 10, 0, 0, 0, 0, 0, 0,  25, 40, 100,  60, 0, 0, 0, 0, 0,  120, 0, 2, 1, 0, 0, 0, 0, 80])

        # 5. Invalid sensors (distance stuck at 0) (10 cases)
        for i in range(10):
            writer.writerow([0, 10, 0, 0, 0, 0, 0, 0,  25, 40, 100,  60, 0, 0, 0, 0, 0,  0, 0, 2, 1, 0, 0, 0, 0, 80])

        # 6. Environmental risk (low lux, high hum) (10 cases)
        for i in range(10):
            writer.writerow([50, 10, 0, 0, 0, 0, 0, 0,  25, 90, 5,  60, 0, 0, 0, 0, 0,  0, 0, 2, 1, 1, 1, 0, 0, 80])

        # 7. Speeding (speed_x > speed_limit) (10 cases)
        for i in range(10):
            writer.writerow([100, 0, 0, 0, 0, 0, 0, 0,  25, 40, 100,  120, 0, 0, 0, 0, 0,  0, 5, 0, 1, 0, 0, 0, 0, 60])

        # 8. High Acceleration / Hard braking (10 cases)
        for i in range(10):
            writer.writerow([50, 10, 10, 0, 0, 0, 0, 0,  25, 40, 100,  60, 0, 0, 0, 0, 0,  0, 0, 10, 1, 0, 0, 0, 0, 80])

        # 9. Manual mode override (10 cases)
        for i in range(10):
            writer.writerow([50, 10, 0, 0, 0, 0, 0, 0,  25, 40, 100,  60, 0, 0, 0, 0, 0,  0, 2, 0, 1, 0, 0, 1, 0, 80])

        # 10. Random extreme variations (15 cases)
        for i in range(15):
            writer.writerow([random.randint(0,255), random.randint(-128,127), random.randint(-128,127), 0, 0, 0, 0, 0, 
                             random.randint(0,100), random.randint(0,100), random.randint(0,200),
                             random.randint(0,200), 0, 0, random.randint(-1000,1000), 0, 0,
                             random.randint(-128,127), random.randint(0,15), random.randint(0,15), 1, 0, 0, 0, 0, 80])

if __name__ == "__main__":
    generate_scenarios()
    print("Generated 105 test cases in scenarios/axi_test_vectors.csv")
