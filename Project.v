module Project (
    input CLOCK_50,
    input [1:0] KEY, 
	 input [9:0] SW,
    inout [13:0] ARDUINO_IO,    
    output [6:0] HEX0,     
    output [6:0] HEX1,
	 output [6:0] HEX2,
	 output [6:0] HEX3,
	 output [6:0] HEX4,
	 output [6:0] HEX5,
    output [3:0] LEDR
);

    wire [7:0] temp_data_bus_in = ARDUINO_IO[7:0]; // D5–D12 (Temperature Data)
    wire data_valid_pulse_in    = ARDUINO_IO[13];  // D13 (Temperature Pulse)

	 wire reset_n = SW[0];
    wire reset = ~reset_n;

// temperature (DHT11)

    // synchronize data_valid and detect rising edge
    wire data_valid_sync;
    synchronizer sync_inst_temp (
        .clk(CLOCK_50),
        .data_in(data_valid_pulse_in),
        .data_out(data_valid_sync)
    );

    reg data_valid_d;
    always @(posedge CLOCK_50) data_valid_d <= data_valid_sync;
    wire capture_pulse = data_valid_sync & ~data_valid_d;

    // capture data on rising edge
    reg [7:0] current_temperature;
    always @(posedge CLOCK_50) begin
        if (reset)
            current_temperature <= 8'd0;
        else if (capture_pulse)
            current_temperature <= temp_data_bus_in;
    end

    // display temperature on HEX1 and HEX0
    // if temp is 0 or > 99, display 99.
    wire [7:0] display_temp = (current_temperature == 8'd0 || current_temperature > 8'd99)
                              ? 8'd99 : current_temperature;

    assign HEX0 = seg7(display_temp % 10); // ones digit
    assign HEX1 = seg7(display_temp / 10); // tens digit
	 

    // temperature validation
    assign LEDR[0] = capture_pulse; // blinks when valid pulse arrives
    assign LEDR[3:1] = current_temperature[2:0]; // lowest bits of captured data

	 
// set temperature
	 wire inc_pulse_in = ARDUINO_IO[9];
	 wire dec_pulse_in = ARDUINO_IO[10];

	 // synchronize increment pulse from arduino
	synchronizer sync_inc (
    .clk(CLOCK_50),
    .data_in(inc_pulse_in),
    .data_out(inc_sync)
	);

	// synchronize decrement pulse from arduino
	synchronizer sync_dec (
    .clk(CLOCK_50),
    .data_in(dec_pulse_in),
    .data_out(dec_sync)
	);


	reg inc_d, dec_d;

	// set increment & decrement readings on rising edge
	always @(posedge CLOCK_50) begin
		if (reset) begin
			inc_d <= 0;
			dec_d <= 0;
		end else begin
			inc_d <= inc_sync;
			dec_d <= dec_sync;
		end
	end

	wire inc_edge = inc_sync & ~inc_d;
	wire dec_edge = dec_sync & ~dec_d;
	reg [7:0] set_temp = 8'd22;
	
	// adjust set_temp on rising edge
	always @(posedge CLOCK_50) begin
		if (reset)
			set_temp <= 8'd22;
		else if (inc_edge && set_temp < 8'd99)
			set_temp <= set_temp + 1;
		else if (dec_edge && set_temp > 8'd0)
			set_temp <= set_temp - 1;
	end

	// display set_temp on HEX2 and HEX3
	assign HEX2 = seg7(set_temp % 10); // ones digit
	assign HEX3 = seg7(set_temp / 10); // tens digit
	 

// HVAC mode

	wire [1:0] letter_sel;
	// C for cooling (set_temp < actual), H for heating (set_temp > actual), O for off (if equal)
	assign letter_sel = (display_temp > set_temp) ? 2'd0 :  // C
                    (display_temp == set_temp) ? 2'd1 : // O
                    2'd2; // H

	assign HEX5 = seg_letter(letter_sel);

	function [6:0] seg_letter;
		input [1:0] sel; // 0=C, 1=O, 2=H
		case(sel)
			2'd0: seg_letter = 7'b1000110; // C
			2'd1: seg_letter = 7'b1000000; // O
			2'd2: seg_letter = 7'b0001001; // H
			default: seg_letter = 7'b1111111; // blank
		endcase
	endfunction


// flame detection

	wire flame_detected = ARDUINO_IO[11];
	// Display F if there's a flame, and blank if nothing
	assign HEX4 = flame_detected ? 7'b0001110 : 7'b1111111;


// 7 Segment
    function [6:0] seg7(input [3:0] b);
        case (b)
            4'd0: seg7 = 7'b1000000;
            4'd1: seg7 = 7'b1111001;
            4'd2: seg7 = 7'b0100100;
            4'd3: seg7 = 7'b0110000;
            4'd4: seg7 = 7'b0011001;
            4'd5: seg7 = 7'b0010010;
            4'd6: seg7 = 7'b0000010;
            4'd7: seg7 = 7'b1111000;
            4'd8: seg7 = 7'b0000000;
            4'd9: seg7 = 7'b0010000;
            default: seg7 = 7'b1111111; // E
        endcase
    endfunction

endmodule

// 2-stage synchronizer to handle asynchronous inputs
module synchronizer(
    input clk,
    input data_in,
    output data_out
);
    reg d1, d2;
    always @(posedge clk) begin
        d1 <= data_in;
        d2 <= d1;
    end
    assign data_out = d2;
endmodule
