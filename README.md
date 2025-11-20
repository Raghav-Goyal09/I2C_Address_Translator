# I2C Address Translator
FPGA-based I²C address translator that allows multiple devices with the same default address to coexist on one bus. Acts as an I²C slave upstream and master downstream, remaps addresses dynamically, and supports read/write forwarding. Includes RTL, testbench, simulations, and documentation.

I²C Address Translator 

Source code:

`timescale 1ns / 1ps

module translator(
    input wire clk,
    input wire rst_n,
    inout wire sda_up,
    input wire scl_up,
    output reg sda_down_drive,
    output reg scl_down_drive,
    inout wire sda_down,
    input wire show_debug,
    input wire [6:0] virtual_addr,
    input wire [6:0] real_addr
);

    reg drive_up_sda;
    assign sda_up = drive_up_sda ? 1'b0 : 1'bz;
    assign sda_down = sda_down_drive ? 1'b0 : 1'bz;

    reg sda_up_prev;
    reg in_frame_up;
    reg [3:0] bitcnt_up;
    reg [7:0] shift_up;
    reg addressed_up;
    reg current_rw_up;
    reg [7:0] buffer_byte;
    reg [3:0] state;

    reg ack_up_req;

    localparam IDLE = 0;
    localparam ADDR_CAPTURE = 1;
    localparam ACK_UP = 2;
    localparam FORWARD_TO_DOWN = 3;
    localparam READ_FROM_DOWN = 4;
    localparam WRITE_TO_DOWN = 5;

    initial begin
        sda_up_prev = 1;
        in_frame_up = 0;
        bitcnt_up = 0;
        shift_up = 0;
        addressed_up = 0;
        drive_up_sda = 0;
        sda_down_drive = 0;
        scl_down_drive = 0;
        buffer_byte = 0;
        state = IDLE;
        ack_up_req = 0;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            sda_up_prev <= 1;
            in_frame_up <= 0;
            addressed_up <= 0;
            state <= IDLE;
            ack_up_req <= 0;
        end else begin
            sda_up_prev <= sda_up;

            if (sda_up_prev == 1 && sda_up == 0 && scl_up == 1) begin
                in_frame_up <= 1;
                bitcnt_up <= 0;
                addressed_up <= 0;
                state <= ADDR_CAPTURE;
                ack_up_req <= 0;
                if (show_debug) $display("%0t START up", $time);
            end

            if (sda_up_prev == 0 && sda_up == 1 && scl_up == 1) begin
                in_frame_up <= 0;
                addressed_up <= 0;
                state <= IDLE;
                drive_up_sda <= 0;
                ack_up_req <= 0;
                if (show_debug) $display("%0t STOP up", $time);
            end
        end
    end

    always @(posedge scl_up) begin
        if (!rst_n) begin
            bitcnt_up <= 0;
            shift_up <= 0;
            addressed_up <= 0;
            drive_up_sda <= 0;
            state <= IDLE;
            ack_up_req <= 0;
        end else if (in_frame_up) begin
            if (state == ADDR_CAPTURE) begin
                shift_up <= {shift_up[6:0], sda_up};
                bitcnt_up <= bitcnt_up + 1;

                if (bitcnt_up == 7) begin
                    bitcnt_up <= 0;

                    if (shift_up[7:1] == virtual_addr) begin
                        addressed_up <= 1;
                        current_rw_up <= shift_up[0];
                        ack_up_req <= 1;
                        state <= ACK_UP;
                        if (show_debug) $display("%0t MATCH virt %02x RW=%b", $time, virtual_addr, shift_up[0]);
                    end else begin
                        addressed_up <= 0;
                        state <= IDLE;
                        if (show_debug) $display("%0t ADDR not ours: %02x", $time, shift_up[7:1]);
                    end
                end
            end
        end
    end

    always @(negedge scl_up or negedge rst_n) begin
        if (!rst_n) begin
            drive_up_sda <= 0;
            ack_up_req <= 0;
        end else begin
            if (ack_up_req) begin
                drive_up_sda <= 1;
                ack_up_req <= 0;
                if (show_debug) $display("%0t ACK up", $time);
            end else begin
                drive_up_sda <= 0;
            end
        end
    end

    always @(posedge scl_up) begin
        if (!rst_n) begin
        end else begin
            if (state == ACK_UP) begin
                if (current_rw_up == 1) begin
                    state <= READ_FROM_DOWN;
                    if (show_debug) $display("%0t want READ from %02x", $time, real_addr);
                end else begin
                    state <= WRITE_TO_DOWN;
                    if (show_debug) $display("%0t want WRITE to %02x", $time, real_addr);
                end
            end
        end
    end

    reg [3:0] bitcnt_data_up;
    reg [7:0] shift_data_up;
    reg awaiting_ack_from_up;

    always @(posedge scl_up) begin
        if (!rst_n) begin
            bitcnt_data_up <= 0;
            shift_data_up <= 0;
            awaiting_ack_from_up <= 0;
        end else if (in_frame_up && addressed_up && state == WRITE_TO_DOWN) begin
            shift_data_up <= {shift_data_up[6:0], sda_up};
            bitcnt_data_up <= bitcnt_data_up + 1;

            if (bitcnt_data_up == 7) begin
                bitcnt_data_up <= 0;
                buffer_byte <= shift_data_up;
                ack_up_req <= 1;
                awaiting_ack_from_up <= 1;
                if (show_debug) $display("%0t got byte %02x -> ackreq", $time, shift_data_up);
            end

        end else if (awaiting_ack_from_up) begin
            awaiting_ack_from_up <= 0;
            drive_up_sda <= 0;
            state <= FORWARD_TO_DOWN;
        end
    end

    reg [2:0] down_fsm;
    reg [7:0] down_shift;
    reg [3:0] down_bitcnt;
    reg [7:0] forwarded_byte;

    localparam D_IDLE = 0;
    localparam D_START = 1;
    localparam D_ADDR = 2;
    localparam D_DATA = 3;
    localparam D_WAIT_ACK = 4;
    localparam D_READ_DATA = 5;
    localparam D_STOP = 6;

    initial begin
        down_fsm = D_IDLE;
        scl_down_drive = 0;
        sda_down_drive = 0;
        down_bitcnt = 0;
        down_shift = 0;
        forwarded_byte = 0;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            down_fsm <= D_IDLE;
            scl_down_drive <= 0;
            sda_down_drive <= 0;
            down_bitcnt <= 0;
            down_shift <= 0;
            forwarded_byte <= 0;

        end else begin
            case (down_fsm)

                D_IDLE: begin
                    if (state == FORWARD_TO_DOWN) begin
                        down_fsm <= D_START;
                        down_shift <= {real_addr, 1'b0};
                        down_bitcnt <= 7;
                    end else if (state == READ_FROM_DOWN) begin
                        down_fsm <= D_START;
                        down_shift <= {real_addr, 1'b1};
                        down_bitcnt <= 7;
                    end
                end

                D_START: begin
                    sda_down_drive <= 1;
                    scl_down_drive <= 1;
                    scl_down_drive <= 0;
                    down_fsm <= D_ADDR;
                    down_bitcnt <= 7;
                end

                D_ADDR: begin
                    sda_down_drive <= ~down_shift[down_bitcnt];
                    scl_down_drive <= 1;
                    scl_down_drive <= 0;

                    if (down_bitcnt == 0)
                        down_fsm <= D_WAIT_ACK;
                    else
                        down_bitcnt <= down_bitcnt - 1;
                end

                D_WAIT_ACK: begin
                    sda_down_drive <= 0;

                    if (state == FORWARD_TO_DOWN) begin
                        down_shift <= buffer_byte;
                        down_bitcnt <= 7;
                        down_fsm <= D_DATA;
                    end else if (state == READ_FROM_DOWN) begin
                        down_fsm <= D_READ_DATA;
                        down_bitcnt <= 7;
                    end else begin
                        down_fsm <= D_STOP;
                    end
                end

                D_DATA: begin
                    sda_down_drive <= ~down_shift[down_bitcnt];
                    scl_down_drive <= 1;
                    scl_down_drive <= 0;

                    if (down_bitcnt == 0)
                        down_fsm <= D_WAIT_ACK;
                    else
                        down_bitcnt <= down_bitcnt - 1;
                end

                D_READ_DATA: begin
                    sda_down_drive <= 0;
                    forwarded_byte[down_bitcnt] <= sda_down;
                    scl_down_drive <= 1;
                    scl_down_drive <= 0;

                    if (down_bitcnt == 0) begin
                        down_fsm <= D_STOP;
                    end else begin
                        down_bitcnt <= down_bitcnt - 1;
                    end
                end

                D_STOP: begin
                    sda_down_drive <= 0;
                    scl_down_drive <= 1;
                    sda_down_drive <= 0;

                    if (state == FORWARD_TO_DOWN) begin
                        state <= WRITE_TO_DOWN;
                    end else if (state == READ_FROM_DOWN) begin
                        buffer_byte <= forwarded_byte;
                        state <= READ_FROM_DOWN;
                    end

                    down_fsm <= D_IDLE;
                    scl_down_drive <= 0;
                end

            endcase
        end
    end

    reg [3:0] up_read_bitptr;
    reg sending_up_read;

    initial begin
        up_read_bitptr = 7;
        sending_up_read = 0;
    end

    always @(negedge scl_up) begin
        if (!rst_n) begin
            drive_up_sda <= 0;
            sending_up_read <= 0;
            up_read_bitptr <= 7;
        end else if (state == READ_FROM_DOWN) begin

            if (!sending_up_read) begin
                sending_up_read <= 1;
                up_read_bitptr <= 7;
            end

            if (buffer_byte[up_read_bitptr] == 0)
                drive_up_sda <= 1;
            else
                drive_up_sda <= 0;

            if (up_read_bitptr == 0) begin
                up_read_bitptr <= 7;
                sending_up_read <= 0;
                drive_up_sda <= 0;
            end else begin
                up_read_bitptr <= up_read_bitptr - 1;
            end

        end else begin
            drive_up_sda <= 0;
        end
    end

endmodule


////////////////////////////////////////////////

module master(
    input wire clk,
    input wire rst_n,
    inout wire sda,
    output reg scl,
    input wire show_debug
);

    reg sda_drive;
    assign sda = sda_drive ? 1'b0 : 1'bz;

    initial begin
        scl = 1;
        sda_drive = 0;
    end

    task automatic delay_small(input integer c);
        integer j;
        begin
            for (j = 0; j < c; j = j + 1) begin
                #100;
            end
        end
    endtask

    task automatic pull_sda_low();
        begin
            sda_drive = 1;
        end
    endtask

    task automatic release_sda();
        begin
            sda_drive = 0;
        end
    endtask

    task automatic raise_scl();
        begin
            scl = 1;
        end
    endtask

    task automatic lower_scl();
        begin
            scl = 0;
        end
    endtask

    task automatic start_cond();
        begin
            release_sda();
            delay_small(1);
            raise_scl();
            delay_small(1);
            pull_sda_low();
            delay_small(1);
            if (show_debug) $display("%0t START", $time);
            lower_scl();
            delay_small(1);
        end
    endtask

    task automatic stop_cond();
        begin
            pull_sda_low();
            delay_small(1);
            raise_scl();
            delay_small(1);
            release_sda();
            delay_small(1);
            if (show_debug) $display("%0t STOP", $time);
            release_sda();
            delay_small(1);
        end
    endtask

    task automatic write_bit(input bit_value);
        begin
            lower_scl();
            delay_small(1);
            if (bit_value == 0)
                pull_sda_low();
            else
                release_sda();
            raise_scl();
            delay_small(1);
        end
    endtask

    task automatic read_bit(output reg bit_val);
        begin
            lower_scl();
            delay_small(1);
            release_sda();
            raise_scl();
            delay_small(1);
            bit_val = sda;
        end
    endtask

    task automatic write_byte(input [7:0] data, output reg ack);
        integer k;
        reg ack_bit;
        begin
            for (k = 7; k >= 0; k = k - 1) begin
                write_bit(data[k]);
            end

            lower_scl();
            delay_small(1);
            release_sda();
            raise_scl();
            delay_small(1);
            ack_bit = (sda == 1'b0);
            ack = ack_bit;
            if (show_debug)
                $display("%0t write %02x ack=%b", $time, data, ack_bit);
            lower_scl();
            delay_small(1);
        end
    endtask

    task automatic read_byte(input integer send_ack, output reg [7:0] data_out);
        integer m;
        reg temp;
        reg [7:0] b;
        begin
            b = 8'h00;

            for (m = 7; m >= 0; m = m - 1) begin
                read_bit(temp);
                b[m] = temp;
            end

            lower_scl();
            delay_small(1);

            if (send_ack == 1)
                pull_sda_low();
            else
                release_sda();

            raise_scl();
            delay_small(1);
            release_sda();

            if (show_debug)
                $display("%0t read %02x ack=%b", $time, b, send_ack);

            data_out = b;

            lower_scl();
            delay_small(1);
        end
    endtask

endmodule

////////////////////////////////////////////////

module slave(
    input wire clk,
    input wire rst_n,
    inout wire sda,
    input wire scl,
    input wire [6:0] addr,
    input wire show_debug
);

    reg drive_sda;
    assign sda = drive_sda ? 1'b0 : 1'bz;

    reg sda_prev;
    reg in_frame;
    reg [3:0] bitcnt;
    reg [7:0] byte_shift;
    reg last_rw;
    reg addressed;
    reg [7:0] mem_byte;
    reg [7:0] read_data;
    integer read_bitptr;
    reg ack_req;

    initial begin
        drive_sda = 0;
        sda_prev = 1;
        in_frame = 0;
        bitcnt = 0;
        byte_shift = 0;
        addressed = 0;
        mem_byte = 8'hA5;
        read_data = 8'h10 + addr;
        read_bitptr = 7;
        ack_req = 0;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            sda_prev <= 1;
            in_frame <= 0;
            addressed <= 0;
            drive_sda <= 0;
            ack_req <= 0;
        end else begin
            sda_prev <= sda;
            if (sda_prev == 1 && sda == 0 && scl == 1) begin
                in_frame <= 1;
                bitcnt <= 0;
                addressed <= 0;
                drive_sda <= 0;
                ack_req <= 0;
                if (show_debug) $display("%0t SLAVE %02x START", $time, addr);
            end
            if (sda_prev == 0 && sda == 1 && scl == 1) begin
                in_frame <= 0;
                addressed <= 0;
                drive_sda <= 0;
                ack_req <= 0;
                if (show_debug) $display("%0t SLAVE %02x STOP", $time, addr);
            end
        end
    end

    always @(posedge scl) begin
        if (!rst_n) begin
            bitcnt <= 0;
            byte_shift <= 0;
            drive_sda <= 0;
            ack_req <= 0;
        end else if (in_frame) begin
            if (!addressed) begin
                byte_shift <= {byte_shift[6:0], sda};
                bitcnt <= bitcnt + 1;
                if (bitcnt == 7) begin
                    if (byte_shift[7:1] == addr) begin
                        addressed <= 1;
                        last_rw <= byte_shift[0];
                        ack_req <= 1;
                        if (show_debug) $display("%0t SLAVE %02x MATCH RW=%b", $time, addr, last_rw);
                    end else begin
                        ack_req <= 0;
                        drive_sda <= 0;
                        if (show_debug) $display("%0t SLAVE %02x NO MATCH %02x", $time, addr, byte_shift[7:1]);
                    end
                    bitcnt <= 0;
                end
            end else begin
                if (last_rw == 0) begin
                    byte_shift <= {byte_shift[6:0], sda};
                    bitcnt <= bitcnt + 1;
                    if (bitcnt == 7) begin
                        mem_byte <= byte_shift;
                        ack_req <= 1;
                        bitcnt <= 0;
                        if (show_debug) $display("%0t SLAVE %02x WRITE %02x", $time, addr, byte_shift);
                    end
                end
            end
        end else begin
            ack_req <= 0;
            drive_sda <= 0;
        end
    end

    always @(negedge scl) begin
        if (!rst_n) begin
            drive_sda <= 0;
            read_bitptr <= 7;
            ack_req <= 0;
        end else if (in_frame && addressed && last_rw == 1) begin
            if (read_data[read_bitptr] == 0)
                drive_sda <= 1;
            else
                drive_sda <= 0;

            if (read_bitptr == 0)
                read_bitptr <= 7;
            else
                read_bitptr <= read_bitptr - 1;

            read_data <= read_data + 1;
        end else begin
            if (ack_req) begin
                drive_sda <= 1;
                ack_req <= 0;
                if (show_debug) $display("%0t SLAVE %02x ACK", $time, addr);
            end else begin
                drive_sda <= 0;
            end
        end
    end

endmodule

//////////////////////////////////////////////////

Testbench:

module tb_top;
    reg clk;
    reg rst_n;

    wire sda_bus;
    pullup(sda_bus);

    wire sda_down_wire;
    pullup(sda_down_wire);

    reg scl_master;

    localparam real_addr = 7'h48;
    localparam virtual_addr = 7'h49;

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    initial begin
        rst_n = 0;
        #50 rst_n = 1;
    end

    wire master_sda;
    reg master_show = 1;
    master master(
        .clk(clk),
        .rst_n(rst_n),
        .sda(master_sda),
        .scl(scl_master),
        .show_debug(master_show)
    );

    assign sda_bus = master_sda;
    wire sda_up_bus = sda_bus;

    reg slave_show = 1;
    slave slave1(
        .clk(clk),
        .rst_n(rst_n),
        .sda(sda_up_bus),
        .scl(scl_master),
        .addr(real_addr),
        .show_debug(slave_show)
    );

    wire sda_down = sda_down_wire;
    slave slave2(
        .clk(clk),
        .rst_n(rst_n),
        .sda(sda_down),
        .scl(scl_master),
        .addr(real_addr),
        .show_debug(slave_show)
    );

    wire trans_sda_down;
    wire trans_scl_down;
    translator translator(
        .clk(clk),
        .rst_n(rst_n),
        .sda_up(sda_up_bus),
        .scl_up(scl_master),
        .sda_down_drive(trans_sda_down),
        .scl_down_drive(trans_scl_down),
        .sda_down(sda_down_wire),
        .show_debug(1'b1),
        .virtual_addr(virtual_addr),
        .real_addr(real_addr)
    );

    assign sda_down_wire = trans_sda_down ? 1'b0 : 1'bz;

    initial begin
        $dumpfile("i2c_translator_tb.vcd");
        $dumpvars(0, tb_top);
    end

    always @(posedge scl_master) begin
        $display("%0t BUS sda_up=%b sda_down=%b master=%b", $time, sda_bus, sda_down_wire, master_sda);
    end

    initial begin
        @(posedge rst_n);
        #1000;

        $display("\nWRITE VIRTUAL 0x%02X\n", virtual_addr);
        master.start_cond();
        begin
            reg ack;
            master.write_byte({virtual_addr,1'b0}, ack);
            master.write_byte(8'h5A, ack);
            master.stop_cond();
        end

        #2000;

        $display("\nREAD VIRTUAL 0x%02X\n", virtual_addr);
        master.start_cond();
        begin
            reg ack;
            reg [7:0] rdata;
            master.write_byte({virtual_addr,1'b1}, ack);
            master.read_byte(0, rdata);
            master.stop_cond();
        end

        #2000;

        $display("\nWRITE DIRECT REAL 0x%02X\n", real_addr);
        master.start_cond();
        begin
            reg ack;
            master.write_byte({real_addr,1'b0}, ack);
            master.write_byte(8'hC3, ack);
            master.stop_cond();
        end

        #2000;

        $finish;
    end

endmodule

/////////////////////////////////////


Output:

WRITE VIRTUAL 0x49

1255000 SLAVE 48 START
1255000 START up
1350000 START
1550000 BUS sda_up=1 sda_down=1 master=z
1555000 SLAVE 48 STOP
1555000 STOP up
1750000 BUS sda_up=0 sda_down=1 master=0
1755000 SLAVE 48 START
1755000 START up
1950000 BUS sda_up=0 sda_down=1 master=0
2150000 BUS sda_up=1 sda_down=1 master=z
2155000 SLAVE 48 STOP
2155000 STOP up
2350000 BUS sda_up=0 sda_down=1 master=0
2355000 SLAVE 48 START
2355000 START up
2550000 BUS sda_up=0 sda_down=1 master=0
2750000 BUS sda_up=1 sda_down=1 master=z
2755000 SLAVE 48 STOP
2755000 STOP up
2950000 BUS sda_up=0 sda_down=1 master=0
2955000 SLAVE 48 START
2955000 START up
3150000 BUS sda_up=1 sda_down=1 master=z
3155000 SLAVE 48 STOP
3155000 STOP up
3250000 write 92 ack=x
3450000 BUS sda_up=0 sda_down=1 master=0
3455000 SLAVE 48 START
3455000 START up
3650000 BUS sda_up=1 sda_down=1 master=z
3655000 SLAVE 48 STOP
3655000 STOP up
3850000 BUS sda_up=0 sda_down=1 master=0
3855000 SLAVE 48 START
3855000 START up
4050000 BUS sda_up=1 sda_down=1 master=z
4055000 SLAVE 48 STOP
4055000 STOP up
4250000 BUS sda_up=1 sda_down=1 master=z
4450000 BUS sda_up=0 sda_down=1 master=0
4455000 SLAVE 48 START
4455000 START up
4650000 BUS sda_up=1 sda_down=1 master=z
4655000 SLAVE 48 STOP
4655000 STOP up
4850000 BUS sda_up=0 sda_down=1 master=0
4855000 SLAVE 48 START
4855000 START up
5050000 BUS sda_up=1 sda_down=1 master=z
5055000 SLAVE 48 STOP
5055000 STOP up
5150000 write 5a ack=x
5350000 BUS sda_up=0 sda_down=1 master=0
5455000 SLAVE 48 STOP
5455000 STOP up
5550000 STOP

READ VIRTUAL 0x49

7855000 SLAVE 48 START
7855000 START up
7950000 START
8150000 BUS sda_up=1 sda_down=1 master=z
8155000 SLAVE 48 STOP
8155000 STOP up
8350000 BUS sda_up=0 sda_down=1 master=0
8355000 SLAVE 48 START
8355000 START up
8550000 BUS sda_up=0 sda_down=1 master=0
8750000 BUS sda_up=1 sda_down=1 master=z
8755000 SLAVE 48 STOP
8755000 STOP up
8950000 BUS sda_up=0 sda_down=1 master=0
8955000 SLAVE 48 START
8955000 START up
9150000 BUS sda_up=0 sda_down=1 master=0
9350000 BUS sda_up=1 sda_down=1 master=z
9355000 SLAVE 48 STOP
9355000 STOP up
9550000 BUS sda_up=1 sda_down=1 master=z
9750000 BUS sda_up=1 sda_down=1 master=z
9850000 write 93 ack=x
10050000 BUS sda_up=1 sda_down=1 master=z
10250000 BUS sda_up=1 sda_down=1 master=z
10450000 BUS sda_up=1 sda_down=1 master=z
10650000 BUS sda_up=1 sda_down=1 master=z
10850000 BUS sda_up=1 sda_down=1 master=z
11050000 BUS sda_up=1 sda_down=1 master=z
11250000 BUS sda_up=1 sda_down=1 master=z
11450000 BUS sda_up=1 sda_down=1 master=z
11650000 BUS sda_up=1 sda_down=1 master=z
11750000 read zz ack=00000000000000000000000000000000
11950000 BUS sda_up=0 sda_down=1 master=0
12055000 SLAVE 48 STOP
12055000 STOP up
12150000 STOP

WRITE DIRECT REAL 0x48

14455000 SLAVE 48 START
14455000 START up
14550000 START
14750000 BUS sda_up=1 sda_down=1 master=z
14755000 SLAVE 48 STOP
14755000 STOP up
14950000 BUS sda_up=0 sda_down=1 master=0
14955000 SLAVE 48 START
14955000 START up
15150000 BUS sda_up=0 sda_down=1 master=0
15350000 BUS sda_up=1 sda_down=1 master=z
15355000 SLAVE 48 STOP
15355000 STOP up
15550000 BUS sda_up=0 sda_down=1 master=0
15555000 SLAVE 48 START
15555000 START up
15750000 BUS sda_up=0 sda_down=1 master=0
15950000 BUS sda_up=0 sda_down=1 master=0
16150000 BUS sda_up=0 sda_down=1 master=0
16350000 BUS sda_up=1 sda_down=1 master=z
16355000 SLAVE 48 STOP
16355000 STOP up
16450000 write 90 ack=x
16650000 BUS sda_up=1 sda_down=1 master=z
16850000 BUS sda_up=1 sda_down=1 master=z
17050000 BUS sda_up=0 sda_down=1 master=0
17055000 SLAVE 48 START
17055000 START up
17250000 BUS sda_up=0 sda_down=1 master=0
17450000 BUS sda_up=0 sda_down=1 master=0
17650000 BUS sda_up=0 sda_down=1 master=0
17850000 BUS sda_up=1 sda_down=1 master=z
17855000 SLAVE 48 STOP
17855000 STOP up
18050000 BUS sda_up=1 sda_down=1 master=z
18250000 BUS sda_up=1 sda_down=1 master=z
18350000 write c3 ack=x
18550000 BUS sda_up=0 sda_down=1 master=0
18655000 SLAVE 48 STOP
18655000 STOP up
18750000 STOP

////////////////////////////////////////////////////////


EDAPlayground link: https://edaplayground.com/x/FzTK
