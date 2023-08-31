//-----------------------------------------------------------------
//              Lightweight DDR3 Memory Controller
//                            V0.5
//                     Ultra-Embedded.com
//                     Copyright 2020-21
//
//                   admin@ultra-embedded.com
//
//                     License: Apache 2.0
//-----------------------------------------------------------------
// Copyright 2020-21 Ultra-Embedded.com
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//-----------------------------------------------------------------
/*description：执行core模块传来的指令，完成指令译码和执行（主要让发送的命令满足ddr3的时序要求，时间延迟等），并将数据准换为dfi接口
延时定时器模块
写数据缓存至fifo ：采用两个计数器，一个用来判断，另一个用来计数

cmd addr 控制信号驱动输出

写数据和写使能驱动输出，寄存器打一拍后同步。

读数据和读使能驱动输出，寄存器打一拍后同步

*/ 



module ddr3_dfi_seq
//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
#(
     parameter DDR_MHZ           =  100                 //系统频率
    ,parameter DDR_WRITE_LATENCY =  4
    ,parameter DDR_READ_LATENCY  =  4
    ,parameter DDR_BURST_LEN     =  4
    ,parameter DDR_COL_W         =  9
    ,parameter DDR_BANK_W        =  3
    ,parameter DDR_ROW_W         =  15
    ,parameter DDR_DATA_W        =  32                  // 数据宽度
    ,parameter DDR_DQM_W         =  4
)
//-----------------------------------------------------------------
// Ports
//-----------------------------------------------------------------
(
    // Inputs
     input           clk_i
    ,input           rst_i


    // input from core model 
    ,input  [ 14:0]  address_i                  //  地址线
    ,input  [  2:0]  bank_i                     //  bank线
    ,input  [  3:0]  command_i                  //  命令
    ,input           cke_i                      //  时钟使能
    ,input  [127:0]  wrdata_i                   //  数据线
    ,input  [ 15:0]  wrdata_mask_i              //  写数据掩码


    // from phy 读数据接口 
    ,input  [ 31:0]  dfi_rddata_i
    ,input           dfi_rddata_valid_i
    ,input  [  1:0]  dfi_rddata_dnv_i

    // Outputs 从DDR3 中读出的数据
    ,output          accept_o
    ,output [127:0]  rddata_o
    ,output          rddata_valid_o



    // DFI interface with DDR3 
    ,output [ 14:0]  dfi_address_o
    ,output [  2:0]  dfi_bank_o
    ,output          dfi_cas_n_o
    ,output          dfi_cke_o
    ,output          dfi_cs_n_o
    ,output          dfi_odt_o
    ,output          dfi_ras_n_o
    ,output          dfi_reset_n_o
    ,output          dfi_we_n_o
    ,output [ 31:0]  dfi_wrdata_o
    ,output          dfi_wrdata_en_o
    ,output [  3:0]  dfi_wrdata_mask_o
    ,output          dfi_rddata_en_o
);



localparam CYCLE_TIME_NS     = 1000 / DDR_MHZ;      

// DDR timing
localparam DDR_TRCD_CYCLES   = ( 15 + (CYCLE_TIME_NS-1 )) / CYCLE_TIME_NS;                // 行列地址命令之间的延迟
localparam DDR_TRP_CYCLES    = ( 15 + (CYCLE_TIME_NS-1 )) / CYCLE_TIME_NS;                // 预充电延迟，充电后不能立即发出激活命令，需要等待延迟后才可发出命令。
localparam DDR_TRFC_CYCLES   = ( 260 + (CYCLE_TIME_NS-1)) / CYCLE_TIME_NS;                // SDRAM 行刷新周期时间
localparam DDR_TWTR_CYCLES   = 5 + 1;

// Standard R/W -> W->R (non-sequential)
localparam DDR_RW_NONSEQ_CYCLES = DDR_WRITE_LATENCY + DDR_BURST_LEN + DDR_TWTR_CYCLES;      // 4 + 4 + 6 = 14
localparam DDR_RW_SEQ_CYCLES    = DDR_RW_NONSEQ_CYCLES + 1 - DDR_BURST_LEN;                 // 11 上一个读命令的突发传输结束了，可以进行下一次命令的突发传输了

// CMD 参数 
localparam CMD_W             = 4;
localparam CMD_NOP           = 4'b0111;
localparam CMD_ACTIVE        = 4'b0011;
localparam CMD_READ          = 4'b0101;         //  5
localparam CMD_WRITE         = 4'b0100;
localparam CMD_ZQCL          = 4'b0110;
localparam CMD_PRECHARGE     = 4'b0010;
localparam CMD_REFRESH       = 4'b0001;
localparam CMD_LOAD_MODE     = 4'b0000;

localparam tPHY_WRLAT        =  DDR_WRITE_LATENCY - 1;          // 3 
localparam tPHY_RDLAT        =  DDR_READ_LATENCY  - 1;

localparam DELAY_W = 6;

reg [DELAY_W-1:0] delay_q;                  // 延迟计数器
reg [DELAY_W-1:0] delay_r;

//-----------------------------------------------------------------
// Write data FIFO 缓冲 数据
//-----------------------------------------------------------------
wire [127:0] wrdata_w;              // 写入的数据（fifo输出端口）
wire [15:0]  wrdata_mask_w;         // 写入数据的掩码
wire         write_pop_w;           // 缓存写数据fifo的输出使能信号

ddr3_dfi_fifo #(
     .WIDTH(144)
    ,.DEPTH(4)
    ,.ADDR_W(2)
)
u_write_fifo
(
     .clk_i(clk_i)
    ,.rst_i(rst_i)

    ,.push_i(command_i == CMD_WRITE && accept_o)        
    ,.data_in_i({wrdata_mask_i, wrdata_i})         // 写数据和掩码 
    ,.accept_o()

    ,.valid_o()
    ,.data_out_o({wrdata_mask_w, wrdata_w})
    ,.pop_i(write_pop_w)
);

//-----------------------------------------------------------------
// Last command 寄存前一个指令
//-----------------------------------------------------------------
reg [CMD_W-1:0] last_cmd_q; 

always @ (posedge clk_i )
if (rst_i)
    last_cmd_q       <= CMD_NOP; 
else if (accept_o && command_i != CMD_NOP)
    last_cmd_q       <= command_i;

// //-----------------------------------------------------------------
// // Write accept 好像没用上
// //-----------------------------------------------------------------
// localparam CMD_ACCEPT_W = 9;
// reg [CMD_ACCEPT_W-1:0] wr_accept_q;
// always @ (posedge clk_i )
// if (rst_i)
//     wr_accept_q       <= {(CMD_ACCEPT_W){1'b0}};
// else if (command_i == CMD_WRITE && delay_q  == {DELAY_W{1'b0}})
//     wr_accept_q       <= {1'b1, wr_accept_q[CMD_ACCEPT_W-1:1]};
// else
//     wr_accept_q       <= {1'b0, wr_accept_q[CMD_ACCEPT_W-1:1]};

// 表示连续的读写
wire read_early_accept_w  = (last_cmd_q == CMD_READ  && command_i == CMD_READ  && delay_q == DDR_RW_SEQ_CYCLES);            //  刷新延迟计数器
wire write_early_accept_w = (last_cmd_q == CMD_WRITE && command_i == CMD_WRITE && delay_q == DDR_RW_SEQ_CYCLES);        

assign accept_o  = (delay_q == {DELAY_W{1'b0}}) || read_early_accept_w || write_early_accept_w || (command_i == CMD_NOP);   // 是否可以接受新的命令
// （有延迟计数器，必须等上一个命令执行完毕后，才能接受新的命令）

//-----------------------------------------------------------------
// Write Enable （写使能信号，用移位寄存器实现，如果接收写命令，最高DDR_BURST_LEN位置1；clk到来时寄存器右移，最低位作为使能输出，实现延迟写使能功能
//-----------------------------------------------------------------
localparam WR_SHIFT_W = tPHY_WRLAT + DDR_BURST_LEN;
reg [WR_SHIFT_W-1:0] wr_en_q;
always @ (posedge clk_i )
if (rst_i)
    wr_en_q       <= {(WR_SHIFT_W){1'b0}};
else if (command_i == CMD_WRITE && accept_o)
    wr_en_q       <= {{(DDR_BURST_LEN){1'b1}}, wr_en_q[tPHY_WRLAT:1]};
else
    wr_en_q       <= {1'b0, wr_en_q[WR_SHIFT_W-1:1]};

wire wr_en_w = wr_en_q[0];

//-----------------------------------------------------------------
// Read Enable
//-----------------------------------------------------------------
localparam RD_SHIFT_W = tPHY_RDLAT+DDR_BURST_LEN;
reg [RD_SHIFT_W-1:0] rd_en_q;
always @ (posedge clk_i )
if (rst_i)
    rd_en_q       <= {(RD_SHIFT_W){1'b0}};
else if (command_i == CMD_READ && accept_o)
    rd_en_q       <= {{(DDR_BURST_LEN){1'b1}}, rd_en_q[tPHY_RDLAT:1]};
else
    rd_en_q       <= {1'b0, rd_en_q[RD_SHIFT_W-1:1]};

wire rd_en_w = rd_en_q[0];

//-----------------------------------------------------------------
// Delays delay_q判断条件用, delay_r计数用
//-----------------------------------------------------------------
/* verilator lint_off WIDTH */
always @ *
begin
    delay_r = delay_q;

    if (delay_q == {DELAY_W{1'b0}})         // 计数完毕后，根据命令载入新的计数值
    begin
        //-----------------------------------------
        // ACTIVATE 激活时发送行地址 延时计数器置位： T_RCD,行列地址 之间的最大延时
        //-----------------------------------------
        if (command_i == CMD_ACTIVE)            
        begin
            // tRCD (ACTIVATE -> READ / WRITE)
            delay_r = DDR_TRCD_CYCLES;        
        end
        //-----------------------------------------
        // READ / WRITE 
        //-----------------------------------------
        else if (command_i == CMD_READ || command_i == CMD_WRITE)
        begin
            delay_r = DDR_RW_NONSEQ_CYCLES;
        end
        //-----------------------------------------
        // PRECHARGE   TRP预充电完成的延时
        //-------------TRP----------------------------
        else if (command_i == CMD_PRECHARGE)
        begin
            // tRP (PRECHARGE -> ACTIVATE)
            delay_r = DDR_TRP_CYCLES;
        end
        //-----------------------------------------
        // REFRESH 自刷新延迟
        //-----------------------------------------
        else if (command_i == CMD_REFRESH)
        begin
            // tRFC
            delay_r = DDR_TRFC_CYCLES;
        end
        //-----------------------------------------
        // Others
        //-----------------------------------------
        else
            delay_r = {DELAY_W{1'b0}};
    end
    else if (delay_r != {DELAY_W{1'b0}})    // 当前命令计数器未清0，计数器递减，如果传入连续读写命令，则刷新计数器
    begin
        delay_r = delay_q - 4'd1;

        // Read to Read, Write to Write
        if (read_early_accept_w || write_early_accept_w)
        begin
            delay_r = DDR_RW_NONSEQ_CYCLES;
        end
    end
end
/* verilator lint_on WIDTH */

always @ (posedge clk_i )begin
    if (rst_i)
        delay_q <= {DELAY_W{1'b0}};
    else
        delay_q <= delay_r;
end

//-----------------------------------------------------------------
// Drive Flops，将控制信号和地址信号传递到输出端口
//-----------------------------------------------------------------
reg [CMD_W-1:0]      command_q;
reg [DDR_ROW_W-1:0]  addr_q;
reg [DDR_BANK_W-1:0] bank_q;
reg                  cke_q;

always @ (posedge clk_i )
if (rst_i)                  // 复位
begin
    command_q       <= CMD_NOP;
    addr_q          <= {DDR_ROW_W{1'b0}};
    bank_q          <= {DDR_BANK_W{1'b0}};
end
else if (accept_o)          // 接受新的命令，驱动至输出端口
begin
    command_q       <= command_i;
    addr_q          <= address_i;
    bank_q          <= bank_i;
end
else                        // 不接受新的命令，用nop命令代替
begin
    command_q       <= CMD_NOP;
    addr_q          <= {DDR_ROW_W{1'b0}};
    bank_q          <= {DDR_BANK_W{1'b0}};
end

 // 驱动时钟使能信号
always @ (posedge clk_i )
if (rst_i)
    cke_q           <= 1'b0; 
else
    cke_q           <= cke_i;

//输出命令，地址，时钟等到phy model
assign dfi_address_o     = addr_q;
assign dfi_bank_o        = bank_q;
assign dfi_cs_n_o        = command_q[3];
assign dfi_ras_n_o       = command_q[2];
assign dfi_cas_n_o       = command_q[1];
assign dfi_we_n_o        = command_q[0];
assign dfi_cke_o         = cke_q;
assign dfi_odt_o         = 1'b0;
assign dfi_reset_n_o     = 1'b1;

//-----------------------------------------------------------------
// Write Data 根据写使能和idx掩码 写数据
//-----------------------------------------------------------------
reg [DDR_DATA_W-1:0] dfi_wrdata_q;
reg [DDR_DQM_W-1:0]  dfi_wrdata_mask_q;
reg [1:0] dfi_wr_idx_q;

always @ (posedge clk_i )
if (rst_i)
begin
    dfi_wrdata_q        <= {DDR_DATA_W{1'b0}};
    dfi_wrdata_mask_q   <= {DDR_DQM_W{1'b0}};
    dfi_wr_idx_q        <= 2'b0;
end
else if (wr_en_w)               // 写使能有效，将fifo中的128bit数据驱动输出至io口
begin
    case (dfi_wr_idx_q)
    default: dfi_wrdata_q  <= wrdata_w[31:0];
    2'd1:    dfi_wrdata_q  <= wrdata_w[63:32];
    2'd2:    dfi_wrdata_q  <= wrdata_w[95:64];
    2'd3:    dfi_wrdata_q  <= wrdata_w[127:96];
    endcase

    case (dfi_wr_idx_q)
    default: dfi_wrdata_mask_q  <= wrdata_mask_w[3:0];
    2'd1:    dfi_wrdata_mask_q  <= wrdata_mask_w[7:4];
    2'd2:    dfi_wrdata_mask_q  <= wrdata_mask_w[11:8];
    2'd3:    dfi_wrdata_mask_q  <= wrdata_mask_w[15:12];
    endcase

    dfi_wr_idx_q <= dfi_wr_idx_q + 2'd1;

end
else
begin
    dfi_wrdata_q        <= {DDR_DATA_W{1'b0}};
    dfi_wrdata_mask_q   <= {DDR_DQM_W{1'b0}};
end

assign write_pop_w       = wr_en_w && (dfi_wr_idx_q == 2'd3);       //128bits数据的写数据传递完毕后，popfifo 中的数据，开始进行下一个128bits数据的写操作
assign dfi_wrdata_o      = dfi_wrdata_q;                            // wire连接的，无延迟，输出数据
assign dfi_wrdata_mask_o = dfi_wrdata_mask_q;

// --------------------------------------------------------------------------------------------
// 读写使能信号延时，使其和读写数据同步
// Make sure dfi_wrdata_en is synchronous，写使能  延迟一个时钟周期，是的输出 写数据使能和写数据同步
// --------------------------------------------------------------------------------------------
reg dfi_wrdata_en_q;

always @ (posedge clk_i )
if (rst_i)
    dfi_wrdata_en_q <= 1'b0;
else
    dfi_wrdata_en_q <= wr_en_w;

assign dfi_wrdata_en_o   = dfi_wrdata_en_q;

// Make sure dfi_rddata_en is synchronous 读使能延迟一个时钟周期，是的输出 写数据使能和写数据同步
reg dfi_rddata_en_q;

always @ (posedge clk_i )
if (rst_i)
    dfi_rddata_en_q <= 1'b0;
else
    dfi_rddata_en_q <= rd_en_w;

assign dfi_rddata_en_o   = dfi_rddata_en_q;




//-----------------------------------------------------------------
// Read Data
//-----------------------------------------------------------------
reg [1:0] dfi_rd_idx_q;
// 位宽转换计数器，用于产生读完毕脉冲信号
always @ (posedge clk_i )
if (rst_i)
    dfi_rd_idx_q <= 2'b0;
else if (dfi_rddata_valid_i)
    dfi_rd_idx_q <= dfi_rd_idx_q + 2'd1;

// 读数据32bits —> 128bits，
reg [127:0]  rd_data_q;      
always @ (posedge clk_i )
if (rst_i)
    rd_data_q <= 128'b0;
else if (dfi_rddata_valid_i)
    rd_data_q <= {dfi_rddata_i, rd_data_q[127:32]};

// 读有效信号脉冲，当读操作完成时产生读脉冲信号
reg rd_valid_q;
always @ (posedge clk_i )
if (rst_i)
    rd_valid_q <= 1'b0;
else if (dfi_rddata_valid_i && dfi_rd_idx_q == 2'd3)
    rd_valid_q <= 1'b1;
else
    rd_valid_q <= 1'b0;

assign rddata_valid_o    = rd_valid_q;      //  输出数据使能
assign rddata_o          = rd_data_q;       //  输出数据

//-----------------------------------------------------------------
// Simulation only
//-----------------------------------------------------------------
`ifdef verilator
    reg [79:0] dbg_cmd;
    always @ *
    begin
        case (command_q)
        CMD_NOP:        dbg_cmd = "NOP";
        CMD_ACTIVE:     dbg_cmd = "ACTIVE";
        CMD_READ:       dbg_cmd = "READ";
        CMD_WRITE:      dbg_cmd = "WRITE";
        CMD_ZQCL:       dbg_cmd = "ZQCL";
        CMD_PRECHARGE:  dbg_cmd = "PRECHRG";
        CMD_REFRESH:    dbg_cmd = "REFRESH";
        CMD_LOAD_MODE:  dbg_cmd = "LOADMODE";
        default:        dbg_cmd = "UNKNOWN";
        endcase
    end
`endif

endmodule

//-----------------------------------------------------------------
// FIFO
//-----------------------------------------------------------------
module ddr3_dfi_fifo
    //-----------------------------------------------------------------
    // Params
    //-----------------------------------------------------------------
    #(
        parameter WIDTH   = 144,
        parameter DEPTH   = 2,
        parameter ADDR_W  = 1
    )
    //-----------------------------------------------------------------
    // Ports
    //-----------------------------------------------------------------
    (
        // Inputs
         input               clk_i
        ,input               rst_i

        ,input  [WIDTH-1:0]  data_in_i
        ,input               push_i
        ,input               pop_i

        // Outputs
        ,output [WIDTH-1:0]  data_out_o
        
        ,output              accept_o
        ,output              valid_o
    );

    //-----------------------------------------------------------------
    // Local Params
    //-----------------------------------------------------------------
    localparam COUNT_W = ADDR_W + 1;

    //-----------------------------------------------------------------
    // Registers
    //-----------------------------------------------------------------
    reg [WIDTH-1:0]         ram [DEPTH-1:0];
    reg [ADDR_W-1:0]        rd_ptr;
    reg [ADDR_W-1:0]        wr_ptr;
    reg [COUNT_W-1:0]       count;

    //-----------------------------------------------------------------
    // Sequential
    //-----------------------------------------------------------------
    always @ (posedge clk_i )
    if (rst_i)
    begin
        count   <= {(COUNT_W) {1'b0}};
        rd_ptr  <= {(ADDR_W) {1'b0}};
        wr_ptr  <= {(ADDR_W) {1'b0}};
    end
    else
    begin
        // Push
        if (push_i & accept_o)
        begin
            ram[wr_ptr] <= data_in_i;
            wr_ptr      <= wr_ptr + 1;
        end

        // Pop
        if (pop_i & valid_o)
            rd_ptr      <= rd_ptr + 1;

        // Count up
        if ((push_i & accept_o) & ~(pop_i & valid_o))
            count <= count + 1;
        // Count down
        else if (~(push_i & accept_o) & (pop_i & valid_o))
            count <= count - 1;
    end

    //-------------------------------------------------------------------
    // Combinatorial
    //-------------------------------------------------------------------
    /* verilator lint_off WIDTH */
    assign accept_o   = (count != DEPTH);
    assign valid_o    = (count != 0);
    /* verilator lint_on WIDTH */

    assign data_out_o = ram[rd_ptr];

endmodule
