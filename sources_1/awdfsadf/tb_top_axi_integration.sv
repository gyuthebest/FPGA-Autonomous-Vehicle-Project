`timescale 1ns / 1ps

module tb_top_axi_integration;

    // Parameters
    localparam integer C_S_AXI_DATA_WIDTH = 32;
    localparam integer C_S_AXI_ADDR_WIDTH = 6;
    localparam CLK_PERIOD = 10; // 100MHz

    // AXI Signals
    logic S_AXI_ACLK;
    logic S_AXI_ARESETN;
    logic [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_AWADDR;
    logic [2:0] S_AXI_AWPROT;
    logic S_AXI_AWVALID;
    logic S_AXI_AWREADY;
    logic [C_S_AXI_DATA_WIDTH-1:0] S_AXI_WDATA;
    logic [(C_S_AXI_DATA_WIDTH/8)-1:0] S_AXI_WSTRB;
    logic S_AXI_WVALID;
    logic S_AXI_WREADY;
    logic [1:0] S_AXI_BRESP;
    logic S_AXI_BVALID;
    logic S_AXI_BREADY;
    logic [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_ARADDR;
    logic [2:0] S_AXI_ARPROT;
    logic S_AXI_ARVALID;
    logic S_AXI_ARREADY;
    logic [C_S_AXI_DATA_WIDTH-1:0] S_AXI_RDATA;
    logic [1:0] S_AXI_RRESP;
    logic S_AXI_RVALID;
    logic S_AXI_RREADY;

    // DUT
    top_controller #(
        .C_S_AXI_DATA_WIDTH(C_S_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH(C_S_AXI_ADDR_WIDTH)
    ) dut (
        .S_AXI_ACLK(S_AXI_ACLK),
        .S_AXI_ARESETN(S_AXI_ARESETN),
        .S_AXI_AWADDR(S_AXI_AWADDR),
        .S_AXI_AWPROT(S_AXI_AWPROT),
        .S_AXI_AWVALID(S_AXI_AWVALID),
        .S_AXI_AWREADY(S_AXI_AWREADY),
        .S_AXI_WDATA(S_AXI_WDATA),
        .S_AXI_WSTRB(S_AXI_WSTRB),
        .S_AXI_WVALID(S_AXI_WVALID),
        .S_AXI_WREADY(S_AXI_WREADY),
        .S_AXI_BRESP(S_AXI_BRESP),
        .S_AXI_BVALID(S_AXI_BVALID),
        .S_AXI_BREADY(S_AXI_BREADY),
        .S_AXI_ARADDR(S_AXI_ARADDR),
        .S_AXI_ARPROT(S_AXI_ARPROT),
        .S_AXI_ARVALID(S_AXI_ARVALID),
        .S_AXI_ARREADY(S_AXI_ARREADY),
        .S_AXI_RDATA(S_AXI_RDATA),
        .S_AXI_RRESP(S_AXI_RRESP),
        .S_AXI_RVALID(S_AXI_RVALID),
        .S_AXI_RREADY(S_AXI_RREADY)
    );

    // Clock Generation
    initial begin
        S_AXI_ACLK = 0;
        forever #(CLK_PERIOD/2) S_AXI_ACLK = ~S_AXI_ACLK;
    end

    // AXI Lite Write Task
    task axi_write(input [C_S_AXI_ADDR_WIDTH-1:0] addr, input [C_S_AXI_DATA_WIDTH-1:0] data);
        begin
            @(posedge S_AXI_ACLK);
            S_AXI_AWADDR  = addr;
            S_AXI_AWVALID = 1;
            S_AXI_WDATA   = data;
            S_AXI_WVALID  = 1;
            S_AXI_WSTRB   = 4'hF;
            S_AXI_BREADY  = 1;
            
            while (!(S_AXI_AWREADY && S_AXI_WREADY)) @(posedge S_AXI_ACLK);
            @(posedge S_AXI_ACLK);
            S_AXI_AWVALID = 0;
            S_AXI_WVALID  = 0;
            
            while (!S_AXI_BVALID) @(posedge S_AXI_ACLK);
            @(posedge S_AXI_ACLK);
            S_AXI_BREADY = 0;
        end
    endtask

    // AXI Lite Read Task
    task axi_read(input [C_S_AXI_ADDR_WIDTH-1:0] addr, output [C_S_AXI_DATA_WIDTH-1:0] data);
        begin
            @(posedge S_AXI_ACLK);
            S_AXI_ARADDR  = addr;
            S_AXI_ARVALID = 1;
            S_AXI_RREADY  = 1;
            
            while (!S_AXI_ARREADY) @(posedge S_AXI_ACLK);
            @(posedge S_AXI_ACLK);
            S_AXI_ARVALID = 0;
            
            while (!S_AXI_RVALID) @(posedge S_AXI_ACLK);
            data = S_AXI_RDATA;
            @(posedge S_AXI_ACLK);
            S_AXI_RREADY = 0;
        end
    endtask

    int fd;
    int status;
    logic [31:0] cycle_cnt;
    
    // Test Variables
    int val_dist, ap_spd, acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;
    int tmp, hum, lux;
    int spd_x, spd_y, spd_z, inc_x, inc_y, inc_z;
    int steer, accel, brake, gear, hl, haz, man, sit, spd_lim;
    
    logic [31:0] write_val;
    logic [31:0] read_val;

    initial begin
        // Reset state
        S_AXI_ARESETN = 0;
        S_AXI_AWADDR = 0;
        S_AXI_AWPROT = 0;
        S_AXI_AWVALID = 0;
        S_AXI_WDATA = 0;
        S_AXI_WSTRB = 0;
        S_AXI_WVALID = 0;
        S_AXI_BREADY = 0;
        S_AXI_ARADDR = 0;
        S_AXI_ARPROT = 0;
        S_AXI_ARVALID = 0;
        S_AXI_RREADY = 0;
        
        #50;
        S_AXI_ARESETN = 1;
        #50;
        
        $display("-----------------------------------------");
        $display("Starting AXI Top-Level Integration Tests");
        $display("-----------------------------------------");
        
        fd = $fopen("scenarios/axi_test_vectors.csv", "r");
        if (fd == 0) begin
            $display("ERROR: Cannot open scenarios/axi_test_vectors.csv");
            $finish;
        end
        
        cycle_cnt = 1;
        
        // Loop over 100+ cases
        while (!$feof(fd)) begin
            status = $fscanf(fd, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", 
                             val_dist, ap_spd, acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, 
                             tmp, hum, lux, 
                             spd_x, spd_y, spd_z, inc_x, inc_y, inc_z,
                             steer, accel, brake, gear, hl, haz, man, sit, spd_lim);
            if (status != 26) break;
            
            // Reg0: distance[7:0], approach_speed[15:8], accel_x[23:16], accel_y[31:24]
            write_val = (val_dist & 8'hFF) | ((ap_spd & 8'hFF) << 8) | ((acc_x & 8'hFF) << 16) | ((acc_y & 8'hFF) << 24);
            axi_write(6'h00, write_val);
            
            // Reg1: accel_z, gyro_x, gyro_y, gyro_z
            write_val = (acc_z & 8'hFF) | ((gyr_x & 8'hFF) << 8) | ((gyr_y & 8'hFF) << 16) | ((gyr_z & 8'hFF) << 24);
            axi_write(6'h04, write_val);
            
            // Reg2: temp, hum, lux
            write_val = (tmp & 8'hFF) | ((hum & 8'hFF) << 8) | ((lux & 8'hFF) << 16);
            axi_write(6'h08, write_val);
            
            // Reg3: speed_x[13:0], speed_y[27:14]
            write_val = (spd_x & 14'h3FFF) | ((spd_y & 14'h3FFF) << 14);
            axi_write(6'h0C, write_val);
            
            // Reg4: speed_z, incline_x
            write_val = (spd_z & 14'h3FFF) | ((inc_x & 14'h3FFF) << 14);
            axi_write(6'h10, write_val);
            
            // Reg5: incline_y, incline_z
            write_val = (inc_y & 14'h3FFF) | ((inc_z & 14'h3FFF) << 14);
            axi_write(6'h14, write_val);
            
            // Reg6: steering, accel, brake, gear, hl, haz, manual, sit, spd_lim(lower 8)
            write_val = (steer & 8'hFF) | ((accel & 4'hF) << 8) | ((brake & 4'hF) << 12) | ((gear & 2'h3) << 16) | 
                        ((hl & 1) << 18) | ((haz & 1) << 19) | ((man & 1) << 20) | ((sit & 3'h7) << 21) | ((spd_lim & 8'hFF) << 24);
            axi_write(6'h18, write_val);
            
            // Reg7: spd_lim upper
            write_val = (spd_lim >> 8) & 5'h1F;
            axi_write(6'h1C, write_val);
            
            // Reg8: sample seq
            axi_write(6'h20, cycle_cnt);
            
            // Wait some cycles for pipeline processing
            repeat (20) @(posedge S_AXI_ACLK);
            
            // Read output Reg9 for valid signals
            axi_read(6'h24, read_val);
            $display("[%0t] Test Case %0d | Read Valid Reg: %x", $time, cycle_cnt, read_val);
            
            // Read MRM/TD Reg
            axi_read(6'h2C, read_val);
            $display("[%0t] Test Case %0d | Read HUD/MRM Reg: %x", $time, cycle_cnt, read_val);
            
            // Wait extra cycles before next test case
            repeat (30) @(posedge S_AXI_ACLK);
            cycle_cnt++;
        end
        
        $fclose(fd);
        
        // Final flush
        repeat (100) @(posedge S_AXI_ACLK);
        
        $display("-----------------------------------------");
        $display("All 105 AXI Test Cases Completed!");
        $display("-----------------------------------------");
        $finish;
    end
endmodule
