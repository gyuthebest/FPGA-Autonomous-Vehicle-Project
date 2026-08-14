`timescale 1ns / 1ps

import types_pkg::*;

// Gap-only regression for risk_control_2.  The existing full PL regression
// already proves preprocessing, all six diagnostics, risk safe/max endpoints,
// AXI commit/packing, timeout recovery, collision emergency, black ice,
// rain control, manual pass-through, collision reliability, TD and MRM.
// This bench intentionally tests only the previously uncovered intermediate
// tiers, reliability-to-risk mappings, arbitration and warning/TD groups.
module tb_risk_reliability_matrix;
    logic clk = 1'b0;
    always #5 clk = ~clk;

    integer pass_count = 0;
    integer fail_count = 0;
    logic rst_n, valid_risk, valid_rel;
    logic [31:0] seq_risk_in, seq_rel_in;
    sensor_data_t sensor_in, sensor_out;
    sim_data_t sim_in, sim_out;
    reliability_state_t rel_in, rel_out;
    risk_t risk_in, risk_out;
    logic [31:0] seq_risk_out, seq_rel_out;
    logic [1:0] valid_out;
    logic td, hud, mrm;
    logic [3:0] td_sec;

    risk_control_2 #(.CLK_FREQ(4)) dut (
        .clk(clk), .rst_n(rst_n),
        .valid_in(valid_risk), .valid_in_rel(valid_rel),
        .sample_seq_in(seq_risk_in), .sample_seq_in_rel(seq_rel_in),
        .sensor_data_in(sensor_in), .sim_data_in(sim_in),
        .rel_in(rel_in), .risk_in(risk_in),
        .sample_seq_out_risk(seq_risk_out), .sample_seq_out_rel(seq_rel_out),
        .valid_out_rel_risk(valid_out), .risk_out(risk_out), .rel_out(rel_out),
        .sensor_data_out(sensor_out), .sim_data_out(sim_out),
        .transition_demand(td), .hud_warning(hud), .mrm(mrm),
        .td_remain_sec(td_sec)
    );

    task automatic check_eq(
        input string name,
        input logic [63:0] actual,
        input logic [63:0] expected
    );
        if (actual === expected) begin
            pass_count++;
            $display("[PASS] %s actual=%0d", name, actual);
        end else begin
            fail_count++;
            $display("[FAIL] %s actual=%0d expected=%0d", name, actual, expected);
        end
    endtask

    task automatic reset_dut;
        valid_risk = 0;
        valid_rel = 0;
        seq_risk_in = 0;
        seq_rel_in = 0;
        sensor_in = '0;
        sim_in = '0;
        rel_in = '0;
        risk_in = '0;
        sim_in.accelerator = 4'd10;
        sim_in.brake = 4'd1;
        sim_in.speed_limit = 13'd3200;
        sim_in.gear = 2'd2;
        sim_in.rpm = 2'd2;
        rst_n = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(negedge clk);
    endtask

    task automatic sample;
        seq_risk_in++;
        seq_rel_in = seq_risk_in;
        valid_risk = 1;
        valid_rel = 1;
        @(posedge clk);
        #1;
        @(negedge clk);
        valid_risk = 0;
        valid_rel = 0;
    endtask

    task automatic prepare_control;
        reset_dut();
        sim_in.accelerator = 4'd10;
        sim_in.brake = 4'd1;
        sim_in.speed_x = 14'sd0;
        sim_in.speed_limit = 13'd3200;
        sim_in.gear = 2'd2;
        sim_in.rpm = 2'd2;
        sim_in.steering = 8'sd0;
    endtask

    initial begin
        int raw;
        int exp;

        //------------------------------------------------------------------
        $display("\n=== A. RELIABILITY -> EFFECTIVE RISK: ALL UNCOVERED DOMAINS ===");
        reset_dut();

        // Road surface, road impact and light visibility: four risk tiers.
        for (raw = 0; raw < 4; raw++) begin
            risk_in.Ri_road_A = raw; rel_in.temperature.state = 2'b00; #1;
            check_eq($sformatf("road surface NORMAL raw=%0d", raw), dut.eff_tier_road_A, raw);
            rel_in.temperature.state = 2'b01; #1;
            exp = (raw == 0) ? 1 : ((raw == 1) ? 2 : raw);
            check_eq($sformatf("road surface DEGRADED raw=%0d", raw), dut.eff_tier_road_A, exp);
            rel_in = '0;

            risk_in = '0; risk_in.Ri_road_B = raw; rel_in.accel_z.state = 2'b00; #1;
            check_eq($sformatf("road impact NORMAL raw=%0d", raw), dut.eff_tier_road_B, raw);
            rel_in.accel_z.state = 2'b01; #1;
            check_eq($sformatf("road impact DEGRADED raw=%0d", raw), dut.eff_tier_road_B, exp);
            rel_in = '0;

            risk_in = '0; risk_in.Ri_vision_A = raw; rel_in.lux.state = 2'b00; #1;
            check_eq($sformatf("light visibility NORMAL raw=%0d", raw), dut.eff_tier_vision_A, raw);
            rel_in.lux.state = 2'b01; #1;
            check_eq($sformatf("light visibility DEGRADED raw=%0d", raw), dut.eff_tier_vision_A, exp);
            rel_in = '0; risk_in = '0;
        end

        // 2026-08-15 정책: INVALID 는 위험도를 올리지 않는다.
        // TD/MRM 이 담당하므로 risk_control 은 원시 tier 를 그대로 통과시킨다.
        // 이전에는 바닥값(4단계면 tier 2)을 적용해, INVALID 확정 즉시 제동이
        // 걸리면서 TD 10초와 MRM 이 시작되기도 전에 차가 멈췄다.
        rel_in = '0; risk_in = '0;
        rel_in.temperature.state = 2'b10; #1;
        check_eq("road surface INVALID passes raw 0", dut.eff_tier_road_A, 0);
        risk_in.Ri_road_A = 2; #1;
        check_eq("road surface INVALID passes raw 2", dut.eff_tier_road_A, 2);
        rel_in = '0; risk_in = '0; rel_in.accel_z.state = 2'b10; #1;
        check_eq("road impact INVALID passes raw 0", dut.eff_tier_road_B, 0);
        risk_in.Ri_road_B = 3; #1;
        check_eq("road impact INVALID passes raw 3", dut.eff_tier_road_B, 3);
        rel_in = '0; risk_in = '0; rel_in.lux.state = 2'b10; #1;
        check_eq("light visibility INVALID passes raw 0", dut.eff_tier_vision_A, 0);

        // Roll has only SAFE/DANGER, so DEGRADED cannot raise SAFE to an
        // intermediate tier; INVALID must conservatively select DANGER.
        rel_in = '0; risk_in = '0;
        rel_in.gyro_x.state = 2'b01; #1;
        check_eq("roll DEGRADED safe remains safe", dut.eff_tier_posture_A, 0);
        risk_in.Ri_posture_A = 1; #1;
        check_eq("roll DEGRADED danger remains danger", dut.eff_tier_posture_A, 1);
        risk_in = '0; rel_in.gyro_x.state = 2'b10; #1;
        check_eq("roll INVALID passes raw 0", dut.eff_tier_posture_A, 0);

        // Yaw and lateral: three risk tiers.
        for (raw = 0; raw < 3; raw++) begin
            rel_in = '0; risk_in = '0; risk_in.Ri_posture_B = raw; #1;
            check_eq($sformatf("yaw NORMAL raw=%0d", raw), dut.eff_tier_posture_B, raw);
            rel_in.gyro_z.state = 2'b01; #1;
            exp = (raw == 0) ? 1 : raw;
            check_eq($sformatf("yaw DEGRADED raw=%0d", raw), dut.eff_tier_posture_B, exp);

            rel_in = '0; risk_in = '0; risk_in.Ri_posture_C = raw; #1;
            check_eq($sformatf("lateral NORMAL raw=%0d", raw), dut.eff_tier_posture_C, raw);
            rel_in.accel_y.state = 2'b01; #1;
            check_eq($sformatf("lateral DEGRADED raw=%0d", raw), dut.eff_tier_posture_C, exp);
        end
        rel_in = '0; risk_in = '0; rel_in.gyro_z.state = 2'b10; #1;
        check_eq("yaw INVALID passes raw 0", dut.eff_tier_posture_B, 0);
        rel_in = '0; rel_in.accel_y.state = 2'b10; #1;
        check_eq("lateral INVALID passes raw 0", dut.eff_tier_posture_C, 0);

        // Weather is simulation truth and intentionally has no sensor
        // reliability input: all four tiers must pass through unchanged.
        for (raw = 0; raw < 4; raw++) begin
            rel_in = '1; risk_in = '0; risk_in.Ri_vision_B = raw; #1;
            check_eq($sformatf("weather independent raw=%0d", raw), dut.eff_tier_vision_B, raw);
        end

        //------------------------------------------------------------------
        $display("\n=== B. NO RISK RETENTION DURING INVALID ===");
        prepare_control();
        risk_in.Ri_road_A = 3; sample();
        risk_in.Ri_road_A = 0; rel_in.temperature.state = 2'b10; #1;
        check_eq("road surface INVALID does not retain tier 3", dut.eff_tier_road_A, 0);

        prepare_control(); risk_in.Ri_road_B = 3; sample();
        risk_in.Ri_road_B = 0; rel_in.accel_z.state = 2'b10; #1;
        check_eq("road impact INVALID does not retain tier 3", dut.eff_tier_road_B, 0);

        prepare_control(); risk_in.Ri_vision_A = 3; sample();
        risk_in.Ri_vision_A = 0; rel_in.lux.state = 2'b10; #1;
        check_eq("visibility INVALID does not retain tier 3", dut.eff_tier_vision_A, 0);

        prepare_control(); risk_in.Ri_posture_B = 2; sample();
        risk_in.Ri_posture_B = 0; rel_in.gyro_z.state = 2'b10; #1;
        check_eq("yaw INVALID does not retain tier 2", dut.eff_tier_posture_B, 0);

        prepare_control(); risk_in.Ri_posture_C = 2; sample();
        risk_in.Ri_posture_C = 0; rel_in.accel_y.state = 2'b10; #1;
        check_eq("lateral INVALID does not retain tier 2", dut.eff_tier_posture_C, 0);

        //------------------------------------------------------------------
        $display("\n=== C. PREVIOUSLY UNCOVERED CONTROL TIERS ===");
        prepare_control(); risk_in.Ri_collision = 1; sample();
        check_eq("collision CAUTION accelerator", sim_out.accelerator, 0);
        check_eq("collision CAUTION brake unchanged", sim_out.brake, 1);

        prepare_control(); risk_in.Ri_collision = 2; sim_in.speed_x = 1000; sample();
        check_eq("collision DANGER low-speed brake", sim_out.brake, 2);
        prepare_control(); risk_in.Ri_collision = 2; sim_in.speed_x = 1500; sample();
        check_eq("collision DANGER mid-speed brake", sim_out.brake, 3);
        prepare_control(); risk_in.Ri_collision = 2; sim_in.speed_x = 3000; sample();
        check_eq("collision DANGER high-speed brake", sim_out.brake, 4);

        prepare_control(); risk_in.Ri_collision = 3; sim_in.speed_x = 1000; sample();
        check_eq("collision CRITICAL low-speed brake", sim_out.brake, 4);
        check_eq("collision CRITICAL hazard", sim_out.hazard, 1);
        prepare_control(); risk_in.Ri_collision = 3; sim_in.speed_x = 1500; sample();
        check_eq("collision CRITICAL mid-speed brake", sim_out.brake, 6);
        prepare_control(); risk_in.Ri_collision = 3; sim_in.speed_x = 3000; sample();
        check_eq("collision CRITICAL high-speed brake", sim_out.brake, 8);

        prepare_control(); risk_in.Ri_road_A = 1; sample();
        check_eq("wet accelerator cap", sim_out.accelerator, 8);
        check_eq("wet speed limit 90 percent", sim_out.speed_limit, 2881);
        prepare_control(); risk_in.Ri_road_A = 2; sample();
        check_eq("ice accelerator cap", sim_out.accelerator, 6);
        check_eq("ice speed limit 70 percent", sim_out.speed_limit, 2240);
        // 마찰 비례 블렌딩: 요청 제동 1은 ICE 상한 5보다 작으므로 그대로 통과.
        check_eq("ice brake blended not suppressed", sim_out.brake, 1);

        prepare_control(); risk_in.Ri_road_B = 1; sample();
        check_eq("rough-road accelerator cap", sim_out.accelerator, 9);
        check_eq("rough-road brake", sim_out.brake, 2);
        check_eq("rough-road speed limit 80 percent", sim_out.speed_limit, 2559);
        prepare_control(); risk_in.Ri_road_B = 2; sample();
        check_eq("severe-impact accelerator cap", sim_out.accelerator, 7);
        check_eq("severe-impact speed limit 60 percent", sim_out.speed_limit, 1918);
        prepare_control(); risk_in.Ri_road_B = 3; sample();
        check_eq("extreme-impact accelerator cap", sim_out.accelerator, 5);
        check_eq("extreme-impact speed limit 50 percent", sim_out.speed_limit, 1600);

        prepare_control(); risk_in.Ri_vision_A = 1; sample();
        check_eq("dim light headlight", sim_out.headlight, 1);
        check_eq("dim light no speed reduction", sim_out.speed_limit, 3200);
        prepare_control(); risk_in.Ri_vision_A = 2; sample();
        check_eq("dark headlight", sim_out.headlight, 1);
        prepare_control(); risk_in.Ri_vision_A = 3; sample();
        check_eq("very-dark speed limit 90 percent", sim_out.speed_limit, 2881);

        prepare_control(); risk_in.Ri_vision_B = 1; sample();
        check_eq("fog accelerator cap", sim_out.accelerator, 8);
        check_eq("fog speed limit 90 percent", sim_out.speed_limit, 2881);
        check_eq("fog hazard off", sim_out.hazard, 0);
        prepare_control(); risk_in.Ri_vision_B = 3; sample();
        check_eq("snow accelerator cap", sim_out.accelerator, 5);
        check_eq("snow speed limit 60 percent", sim_out.speed_limit, 1918);
        check_eq("snow hazard off", sim_out.hazard, 0);

        prepare_control(); risk_in.Ri_posture_A = 1; sample();
        check_eq("roll danger accelerator", sim_out.accelerator, 0);

        prepare_control(); sim_in.steering = -100; sample();
        risk_in.Ri_posture_B = 1; sim_in.steering = 100; sample();
        check_eq("yaw caution accelerator cap", sim_out.accelerator, 8);
        check_eq("yaw caution steering delta cap", $signed(sim_out.steering), 40);
        prepare_control(); sim_in.steering = -100; sample();
        risk_in.Ri_posture_B = 2; sim_in.steering = 100; sample();
        check_eq("yaw danger accelerator", sim_out.accelerator, 0);
        check_eq("yaw danger steering delta cap", $signed(sim_out.steering), 0);

        prepare_control(); sim_in.steering = -100; sample();
        risk_in.Ri_posture_C = 1; sim_in.steering = 100; sample();
        check_eq("lateral caution accelerator cap", sim_out.accelerator, 7);
        check_eq("lateral caution steering delta cap", $signed(sim_out.steering), 60);
        prepare_control(); sim_in.steering = -100; sample();
        risk_in.Ri_posture_C = 2; sim_in.steering = 100; sample();
        check_eq("lateral danger accelerator", sim_out.accelerator, 0);
        // 횡방향 DANGER 상한은 5. 요청 제동 1은 그대로 통과한다.
        check_eq("lateral danger brake blended not suppressed", sim_out.brake, 1);
        check_eq("lateral danger steering delta cap", $signed(sim_out.steering), 20);

        //------------------------------------------------------------------
        $display("\n=== D. WARNING / TD GROUPS AND MULTI-RISK ARBITRATION ===");
        reset_dut(); rel_in.temperature.state = 2'b10; #1;
        check_eq("road surface INVALID HUD", hud, 1);
        check_eq("road surface INVALID no TD", dut.td_condition, 0);
        rel_in = '0; rel_in.accel_z.state = 2'b10; #1;
        check_eq("road impact INVALID HUD", hud, 1);
        check_eq("road impact INVALID no TD", dut.td_condition, 0);
        rel_in = '0; rel_in.lux.state = 2'b10; #1;
        check_eq("visibility INVALID HUD", hud, 1);
        check_eq("visibility INVALID no TD", dut.td_condition, 0);
        rel_in = '0; rel_in.gyro_x.state = 2'b10; #1;
        check_eq("roll INVALID TD", dut.td_condition, 1);
        rel_in = '0; rel_in.gyro_z.state = 2'b10; #1;
        check_eq("yaw INVALID TD", dut.td_condition, 1);
        rel_in = '0; rel_in.accel_y.state = 2'b10; #1;
        check_eq("lateral INVALID TD", dut.td_condition, 1);
        rel_in = '0; rel_in.gyro_y.state = 2'b10; #1;
        check_eq("pitch INVALID HUD", hud, 1);
        check_eq("pitch INVALID no TD", dut.td_condition, 0);
        rel_in = '0; rel_in.accel_x.state = 2'b10; #1;
        check_eq("longitudinal INVALID HUD", hud, 1);
        check_eq("longitudinal INVALID no TD", dut.td_condition, 0);

        // 마찰 비례 블렌딩 정책. 이전에는 ICE/BLACK ICE가 EMERGENCY 충돌
        // 제동 요청까지 0으로 지웠고 이것이 안전 검토 지적사항이었다.
        // 이제 EMERGENCY 요청 10은 BLACK ICE 상한 3으로 제한되어 남는다.
        prepare_control();
        risk_in.Ri_collision = 4;
        risk_in.Ri_road_A = 3;
        sample();
        check_eq("combined emergency+black-ice accelerator", sim_out.accelerator, 0);
        check_eq("combined emergency+black-ice blended brake", sim_out.brake, 3);
        check_eq("combined emergency+black-ice hazard", sim_out.hazard, 1);
        check_eq("combined emergency+black-ice speed limit", sim_out.speed_limit, 1600);

        // --- 마찰 비례 블렌딩: 상한이 실제로 동작하는지 ---
        // 위 케이스들은 요청 제동(1)이 상한보다 작아 통과만 확인한다.
        // 여기서는 EMERGENCY 요청 10이 상한에 의해 잘리는지를 본다.
        prepare_control();
        risk_in.Ri_collision = 4;
        risk_in.Ri_road_A = 0;              // DRY
        sample();
        check_eq("dry emergency brake uncapped", sim_out.brake, 10);

        prepare_control();
        risk_in.Ri_collision = 4;
        risk_in.Ri_road_A = 1;              // WET: 상한 없음
        sample();
        check_eq("wet emergency brake uncapped", sim_out.brake, 10);

        prepare_control();
        risk_in.Ri_collision = 4;
        risk_in.Ri_road_A = 2;              // ICE: 상한 5
        sample();
        check_eq("ice emergency brake capped", sim_out.brake, 5);

        prepare_control();
        risk_in.Ri_collision = 4;
        risk_in.Ri_posture_C = 2;           // 횡방향 DANGER: 상한 5
        sample();
        check_eq("lateral danger emergency brake capped", sim_out.brake, 5);

        // 노면과 횡방향이 동시에 위험하면 더 낮은 상한이 이긴다.
        prepare_control();
        risk_in.Ri_collision = 4;
        risk_in.Ri_road_A = 3;              // BLACK ICE: 상한 3
        risk_in.Ri_posture_C = 2;           // 횡방향: 상한 5
        sample();
        check_eq("black-ice plus lateral takes lower cap", sim_out.brake, 3);

        // 충돌 위험이 없으면 저마찰에서도 제동이 완전히 사라지지 않는다.
        prepare_control();
        risk_in.Ri_road_A = 3;
        risk_in.Ri_road_B = 1;              // rough road brake 2 요청
        sample();
        check_eq("black-ice keeps small road brake", sim_out.brake, 2);

        prepare_control();
        risk_in.Ri_road_A = 1;
        risk_in.Ri_road_B = 2;
        risk_in.Ri_vision_A = 3;
        risk_in.Ri_vision_B = 3;
        sample();
        check_eq("multi-risk minimum accelerator", sim_out.accelerator, 5);
        check_eq("multi-risk maximum brake", sim_out.brake, 2);
        check_eq("multi-risk minimum speed limit", sim_out.speed_limit, 1918);
        check_eq("multi-risk lights OR", sim_out.headlight, 1);

        $display("\n============================================================");
        $display("RISK/RELIABILITY GAP MATRIX RESULT: PASS=%0d FAIL=%0d", pass_count, fail_count);
        $display("============================================================");
        $finish;
    end
endmodule
