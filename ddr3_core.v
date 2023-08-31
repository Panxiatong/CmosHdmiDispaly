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

// description：接收读写命令，完成ddr3初始化，完成读写命令转换和解析，转化成符合ddr3时序标准的地址信号，控制信号，转换成dfi接口信号 
// 完成DDR3初始化，并根据输入端口，读写指令，完成ddr3状态切换，解析命令，将命令对应的地址和数据传递到到ddr3引脚
/*
1, 根据读写请求，译码为cmd命令（三段式状态机实现）
2. 地址译码
3. bank管理模块
4。响应模块（fifO ）
*/ 

module ddr3_core
//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
#(
     parameter DDR_MHZ           = 100          
    ,parameter DDR_WRITE_LATENCY = 4    
    ,parameter DDR_READ_LATENCY  = 4
    ,parameter DDR_COL_W         = 10           // 地址宽度
    ,parameter DDR_BANK_W        = 3
    ,parameter DDR_ROW_W         = 15
    ,parameter DDR_BRC_MODE      = 0
)
//-----------------------------------------------------------------
// Ports
//-----------------------------------------------------------------
(
    // Inputs
     input           clk_i
    ,input           rst_i
    // 配置寄存器输出（一般不使用
    //--------------------------------
    ,input           cfg_enable_i
    ,input           cfg_stb_i
    ,input  [ 31:0]  cfg_data_i
    ,input  [ 31:0]  dfi_rddata_i
    ,input           dfi_rddata_valid_i
    ,input  [  1:0]  dfi_rddata_dnv_i
    ,output          cfg_stall_o

    // from axi ,读写命令，地址，数据，响应信息
    //-------------------------------------
    ,input  [ 15:0]  inport_wr_i            // 写命令
    ,input           inport_rd_i            // 读命令
    ,input  [ 31:0]  inport_addr_i          // 读写地址
    ,input  [127:0]  inport_write_data_i    // 写数据
    ,input  [ 15:0]  inport_req_id_i        // 事务id

    //-------------------------------------
    // to  axi，响应信息，传递给axi 模块
    ,output          inport_accept_o
    ,output          inport_ack_o
    ,output          inport_error_o
    ,output [ 15:0]  inport_resp_id_o       // 事务id 
    ,output [127:0]  inport_read_data_o     // 读数据

    // to dfi phy interface 
    ,output [ 14:0]  dfi_address_o
    ,output [  2:0]  dfi_bank_o
    ,output          dfi_cas_n_o
    ,output          dfi_cke_o
    ,output          dfi_cs_n_o
    ,output          dfi_odt_o
    ,output          dfi_ras_n_o
    ,output          dfi_reset_n_o
    ,output          dfi_we_n_o
    ,output [ 31:0]  dfi_wrdata_o           // 传给phy芯片的位宽是32bits
    ,output          dfi_wrdata_en_o
    ,output [  3:0]  dfi_wrdata_mask_o
    ,output          dfi_rddata_en_o
);



//-----------------------------------------------------------------
// Defines / Local params
//-----------------------------------------------------------------
localparam DDR_BANKS         = 2 ** DDR_BANK_W;     // bank 数量
// `ifdef XILINX_SIMULATOR
localparam DDR_START_DELAY   = 60000 / (1000 / DDR_MHZ); // 60uS            
// `else
// localparam DDR_START_DELAY   = 600000 / (1000 / DDR_MHZ); // 600uS
// `endif
localparam DDR_REFRESH_CYCLES= (64000*DDR_MHZ) / 8192;       // 自刷新时间3.9(85-95度) 7.8（0-85） us 

//cmd  paramters,传输给 dfi_seq 模块 
localparam CMD_W             = 4;               // 命令宽度
localparam CMD_NOP           = 4'b0111;         //7
localparam CMD_ACTIVE        = 4'b0011;         //3
localparam CMD_READ          = 4'b0101;         //5
localparam CMD_WRITE         = 4'b0100;         //4
localparam CMD_PRECHARGE     = 4'b0010;         //2
localparam CMD_REFRESH       = 4'b0001;         //1
localparam CMD_LOAD_MODE     = 4'b0000;         //0
localparam CMD_ZQCL          = 4'b0110;         //6 

// Mode Configuration 
// - DLL disabled (low speed only)
// - CL=6
// - AL=0
// - CWL=6 
// MR 地址
localparam MR0_REG           = 15'h0120;
localparam MR1_REG           = 15'h0001;
localparam MR2_REG           = 15'h0008;
localparam MR3_REG           = 15'h0000;

// SM states（）
localparam STATE_W           = 4;
localparam STATE_INIT        = 4'd0;
localparam STATE_DELAY       = 4'd1;
localparam STATE_IDLE        = 4'd2;
localparam STATE_ACTIVATE    = 4'd3;
localparam STATE_READ        = 4'd4;
localparam STATE_WRITE       = 4'd5;
localparam STATE_PRECHARGE   = 4'd6;
localparam STATE_REFRESH     = 4'd7;

localparam AUTO_PRECHARGE    = 10;  // 读写完毕后自动预充电
localparam ALL_BANKS         = 10;

//-----------------------------------------------------------------
// External Interface，
// description：定义一些信号，完成输入端口的信号接入，和输出端口的信号输出。
//-----------------------------------------------------------------
wire [ 31:0]  ram_addr_w       =    inport_addr_i;               // 地址
wire [ 15:0]  ram_wr_w         =    inport_wr_i;                 // 写信号
wire          ram_rd_w         =    inport_rd_i;                 // 读信号
wire [127:0]  ram_write_data_w =    inport_write_data_i;         // 写数据

wire          ram_accept_w;
wire [127:0]  ram_read_data_w;                                   // 读数据
wire          ram_ack_w;                                         // 响应

wire          id_fifo_space_w;
wire          ram_req_w     = ((ram_wr_w != 16'b0) | ram_rd_w) && id_fifo_space_w;  // 可接收读写命令，如果读写有效且缓存fifo 非满

assign inport_ack_o       = ram_ack_w;                                              // 数据赋值给输出接口信号
assign inport_read_data_o = ram_read_data_w;
assign inport_error_o     = 1'b0;
assign inport_accept_o    = ram_accept_w;

//-----------------------------------------------------------------
// Registers / Wires
//-----------------------------------------------------------------
wire                   cmd_accept_w;                            // ddr3 接收命令的有效反馈信号

wire                   sdram_rd_valid_w;
wire [127:0]           sdram_data_in_w;

reg                    refresh_q;                               // 计数完毕（）自刷新

reg [DDR_BANKS-1:0]    row_open_q;
reg [DDR_ROW_W-1:0]    active_row_q[0:DDR_BANKS-1];

// 状态寄存器
reg  [STATE_W-1:0]     state_q;
reg  [STATE_W-1:0]     next_state_r;
reg  [STATE_W-1:0]     target_state_r;
reg  [STATE_W-1:0]     target_state_q;

// Address bits (RBC mode) 地址映射
wire [DDR_ROW_W-1:0]  addr_col_w  = {{(DDR_ROW_W-DDR_COL_W){1'b0}}, ram_addr_w[DDR_COL_W:2], 1'b0};
wire [DDR_ROW_W-1:0]  addr_row_w  = DDR_BRC_MODE ? ram_addr_w[DDR_ROW_W+DDR_COL_W:DDR_COL_W+1] :            // BRC
                                                   ram_addr_w[DDR_ROW_W+DDR_COL_W+3:DDR_COL_W+3+1];         // RBC
wire [DDR_BANK_W-1:0] addr_bank_w = DDR_BRC_MODE ? ram_addr_w[DDR_ROW_W+DDR_COL_W+3:DDR_ROW_W+DDR_COL_W+1]: // BRC
                                                   ram_addr_w[DDR_COL_W+1+3-1:DDR_COL_W+1];                 // RBC

//-----------------------------------------------------------------
// SDRAM State Machine 三段式状态机
// 组合逻辑实现，状态跳转
//-----------------------------------------------------------------
always @ (*)begin
    next_state_r   = state_q;
    target_state_r = target_state_q;

    case (state_q)
    //-----------------------------------------
    // STATE_INIT
    //-----------------------------------------
    STATE_INIT :
    begin
        if (refresh_q)
            next_state_r = STATE_IDLE;
    end
    //-----------------------------------------
    // STATE_IDLE
    //-----------------------------------------
    STATE_IDLE :
    begin
        // Disabled
        if (!cfg_enable_i)
            next_state_r = STATE_IDLE;
        // Pending refresh
        // Note: tRAS (open row time) cannot be exceeded due to periodic
        //        auto refreshes.
        else if (refresh_q)         //自刷新信号有效
        begin
            // Close open rows, then refresh
            if (|row_open_q)                        
                next_state_r = STATE_PRECHARGE;         // 有打开的行，调转预充电状态
            else
                next_state_r = STATE_REFRESH;           // 没有打开的行，跳转到自刷新状态

            target_state_r = STATE_REFRESH;             // 目的状态数自刷新状态
        end
        // Access request
        else if (ram_req_w)         // 接收读写请求
        begin
            // Open row hit
            if ( row_open_q[addr_bank_w] && addr_row_w == active_row_q[addr_bank_w] )     // 请求的地址单元刚好是激活状态
            begin
                if (!ram_rd_w)
                    next_state_r = STATE_WRITE;         // 进入写状态
                else
                    next_state_r = STATE_READ;          // 进入读状态
            end
            // Row miss, close row, open new row 当前有打开的非目的地的行，先关闭，在读写
            else if (row_open_q[addr_bank_w])
            begin
                next_state_r   = STATE_PRECHARGE;   // 预充电 关闭当前打开的行

                if (!ram_rd_w)
                    target_state_r = STATE_WRITE;   // 写状态
                else
                    target_state_r = STATE_READ;    // 读状态
            end
            // No open row, open row                // 当前没有打开的行，先激活在读写
            else
            begin
                next_state_r   = STATE_ACTIVATE;

                if (!ram_rd_w)
                    target_state_r = STATE_WRITE;
                else
                    target_state_r = STATE_READ;
            end
        end
    end
    //-----------------------------------------
    // STATE_ACTIVATE
    //-----------------------------------------
    STATE_ACTIVATE :
    begin
        // Proceed to read or write state
        next_state_r = target_state_q;
    end
    //-----------------------------------------
    // STATE_READ
    //-----------------------------------------
    STATE_READ :
    begin
        next_state_r = STATE_IDLE;
    end
    //-----------------------------------------
    // STATE_WRITE
    //-----------------------------------------
    STATE_WRITE :
    begin
        next_state_r = STATE_IDLE;
    end
    //-----------------------------------------
    // STATE_PRECHARGE
    //-----------------------------------------
    STATE_PRECHARGE :
    begin
        // Closing row to perform refresh
        if (target_state_q == STATE_REFRESH)    // 如果目的状态是刷新，则下一个状态跳转为刷新（刷新状态后的预充电）
            next_state_r = STATE_REFRESH;
        // Must be closing row to open another
        else                                // 预充电后进入激活状态（读写状态的预充电）
            next_state_r = STATE_ACTIVATE;
    end
    //-----------------------------------------
    // STATE_REFRESH
    //-----------------------------------------
    STATE_REFRESH :
    begin
        next_state_r = STATE_IDLE;
    end
    default:
        ;
   endcase
end


//-----------------------------------------------------------------
// Record target state
// 状态更新
always @ (posedge clk_i )
if (rst_i)
    target_state_q   <= STATE_IDLE;
else if (cmd_accept_w)
    target_state_q   <= target_state_r;

// Update state
always @ (posedge clk_i )
if (rst_i)
    state_q   <= STATE_INIT;
else if (cmd_accept_w)
    state_q   <= next_state_r;

//-----------------------------------------------------------------
// Refresh counter 自刷新计数器周期性发出自刷新信号
//-----------------------------------------------------------------
localparam REFRESH_CNT_W = 20;

reg [REFRESH_CNT_W-1:0] refresh_timer_q;

always @ (posedge clk_i )
if (rst_i)
    refresh_timer_q <= DDR_START_DELAY;
else if (refresh_timer_q == {REFRESH_CNT_W{1'b0}})      // 自刷新计数器
    refresh_timer_q <= DDR_REFRESH_CYCLES;
else
    refresh_timer_q <= refresh_timer_q - 1;

always @ (posedge clk_i )
if (rst_i)
    refresh_q <= 1'b0;
else if (refresh_timer_q == {REFRESH_CNT_W{1'b0}})      // 计数完毕，发出刷新信号
    refresh_q <= 1'b1;
else if (state_q == STATE_REFRESH)
    refresh_q <= 1'b0;

//-----------------------------------------------------------------
// Bank Logic bank管理逻辑
//-----------------------------------------------------------------
integer idx;

always @ (posedge clk_i )
if (rst_i)
begin
    for (idx=0;idx<DDR_BANKS;idx=idx+1)
        active_row_q[idx] <= {DDR_ROW_W{1'b0}}; // 初始化logic bank

    row_open_q <= {DDR_BANKS{1'b0}};            
end
else
begin
    case (state_q)
    //-----------------------------------------
    // STATE_IDLE / Default (delays)
    //-----------------------------------------
    default:
    begin
        if (!cfg_enable_i)
            row_open_q <= {DDR_BANKS{1'b0}};
    end
    //-----------------------------------------
    // STATE_ACTIVATE 行地址 和激活命令一起发出
    //-----------------------------------------
    STATE_ACTIVATE :
    begin
        active_row_q[addr_bank_w]  <= addr_row_w;       //  which  bank ，which row ，记录当前行地址是属于which  bank
        row_open_q[addr_bank_w]    <= 1'b1;             //  记录哪一个bank处于打开状态 
    end 
    //-----------------------------------------
    // STATE_PRECHARGE
    //-----------------------------------------
    STATE_PRECHARGE :
    begin
        // Precharge due to refresh, close all banks
        if (target_state_q == STATE_REFRESH)
        begin
            // Precharge all banks
            row_open_q          <= {DDR_BANKS{1'b0}};       // 自刷新命令关闭所有bank
        end
        else
        begin
            // Precharge specific banks
            row_open_q[addr_bank_w] <= 1'b0;                 // 读写指令的刷新关闭指定的bank
        end
    end
    endcase
end

//-----------------------------------------------------------------
// Command
// description: 根据当前何种状态，发出读写命令给dfi_seq
//-----------------------------------------------------------------
reg [CMD_W-1:0]      command_r;         // 命令
reg [DDR_ROW_W-1:0]  addr_r;            // 地址
reg                  cke_r;             // 时钟使能
reg [DDR_BANK_W-1:0] bank_r;            // bank 指示器 which bank 

always @ *
begin
    command_r = CMD_NOP;                    // 初始化命令和bank addr 地址
    addr_r    = {DDR_ROW_W{1'b0}};
    bank_r    = {DDR_BANK_W{1'b0}};
    cke_r     = 1'b1;                      // 时钟使能，高电平有效，有效时数据才可以被正确送入ddr 

    case (state_q)
    //-----------------------------------------
    // STATE_INIT完成出初始化步骤  完成MR寄存器的配置和zq校准，输出 cmd、bank、addr 地址
    //-----------------------------------------
    STATE_INIT:
    begin
        // Assert CKE after 500uS 25000ns = 25 us
        if (refresh_timer_q > 2500)
            cke_r = 1'b0;

        if (refresh_timer_q == 2400)
        begin
            command_r = CMD_LOAD_MODE;
            bank_r    = 3'd2;               // 根据bank地址线确定 which MRS
            addr_r    = MR2_REG;            // 配置寄存器内容
        end

        if (refresh_timer_q == 2300)
        begin
            command_r = CMD_LOAD_MODE;
            bank_r    = 3'd3;
            addr_r    = MR3_REG;
        end

        if (refresh_timer_q == 2200)
        begin
            command_r = CMD_LOAD_MODE;
            bank_r    = 3'd1;
            addr_r    = MR1_REG;
        end

        if (refresh_timer_q == 2100)
        begin
            command_r = CMD_LOAD_MODE;
            bank_r    = 3'd0;
            addr_r    = MR0_REG;
        end

        // Long ZQ calibration
        if (refresh_timer_q == 2000)
        begin
            command_r  = CMD_ZQCL;
            addr_r[10] = 1;
        end

        // PRECHARGE
        if (refresh_timer_q == 10)
        begin
            // Precharge all banks
            command_r           = CMD_PRECHARGE;
            addr_r[ALL_BANKS]   = 1'b1;
        end
    end
    //-----------------------------------------
    // STATE_IDLE 空闲状态接收外部配置信号的配置内容
    //-----------------------------------------
    STATE_IDLE :
    begin
        if (!cfg_enable_i && cfg_stb_i)
            {cke_r, addr_r, bank_r, command_r} = cfg_data_i[CMD_W + DDR_ROW_W + DDR_BANK_W:0];
    end
    //-----------------------------------------
    // STATE_ACTIVATE
    //-----------------------------------------
    STATE_ACTIVATE :
    begin
        // Select a row and activate it
        command_r     =     CMD_ACTIVE;
        addr_r        =     addr_row_w;
        bank_r        =     addr_bank_w;
    end
    //-----------------------------------------
    // STATE_PRECHARGE
    //-----------------------------------------
    STATE_PRECHARGE :
    begin
        // Precharge due to refresh, close all banks
        if (target_state_r == STATE_REFRESH)
        begin
            // Precharge all banks
            command_r           = CMD_PRECHARGE;
            addr_r[ALL_BANKS]   = 1'b1;             // 预充电后最高位为什么置位1？
        end
        else
        begin
            // Precharge specific banks
            command_r           = CMD_PRECHARGE;
            addr_r[ALL_BANKS]   = 1'b0;
            bank_r              = addr_bank_w;
        end
    end
    //-----------------------------------------
    // STATE_REFRESH
    //-----------------------------------------
    STATE_REFRESH :
    begin
        // Auto refresh
        command_r    = CMD_REFRESH;
        addr_r       = {DDR_ROW_W{1'b0}};
        bank_r       = {DDR_BANK_W{1'b0}};        
    end
    //-----------------------------------------
    // STATE_READ
    //-----------------------------------------
    STATE_READ :
    begin
        command_r    = CMD_READ;
        addr_r       = {addr_col_w[DDR_ROW_W-1:3], 3'b0};
        bank_r       = addr_bank_w;

        // Disable auto precharge (auto close of row)
        addr_r[AUTO_PRECHARGE]  = 1'b0;
    end
    //-----------------------------------------
    // STATE_WRITE
    //-----------------------------------------
    STATE_WRITE :
    begin
        command_r        = CMD_WRITE;
        addr_r           = {addr_col_w[DDR_ROW_W-1:3], 3'b0};
        bank_r           = addr_bank_w;

        // Disable auto precharge (auto close of row)
        addr_r[AUTO_PRECHARGE] = 1'b0;
    end
    default:
        ;
    endcase
end

//-----------------------------------------------------------------
// ACK
//-----------------------------------------------------------------
reg write_ack_q;

always @ (posedge clk_i )
if (rst_i)
    write_ack_q <= 1'b0;
else
    write_ack_q <= (state_q == STATE_WRITE) && cmd_accept_w;   // dfi_seq model 可接收命令+有写请求，发出写响应有效信号


// fifo  读写有问题 （写入第一个数据直接被放进输出端口，输出端口可以直接读出）
ddr3_fifo #(
     .WIDTH(16)
    ,.DEPTH(8)
    ,.ADDR_W(3)
)
u_id_fifo
(
     .clk_i(clk_i)
    ,.rst_i(rst_i)

    ,.push_i(ram_req_w & ram_accept_w)
    ,.data_in_i(inport_req_id_i)            // 事务id写入同步fifo      
    ,.accept_o(id_fifo_space_w)

    ,.valid_o()
    ,.data_out_o(inport_resp_id_o)
    ,.pop_i(ram_ack_w)
);

assign ram_ack_w = sdram_rd_valid_w || write_ack_q;

// Accept command in READ or WRITE0 states
assign ram_accept_w = (state_q == STATE_READ || state_q == STATE_WRITE) && cmd_accept_w;

// Config stall
assign cfg_stall_o = ~(state_q == STATE_IDLE && cmd_accept_w);

//-----------------------------------------------------------------
// DDR3 DFI Interface
//-----------------------------------------------------------------
ddr3_dfi_seq #(
     .DDR_MHZ(DDR_MHZ)
    ,.DDR_WRITE_LATENCY(DDR_WRITE_LATENCY)
    ,.DDR_READ_LATENCY(DDR_READ_LATENCY)
)
u_seq
(
     .clk_i(clk_i)
    ,.rst_i(rst_i)

    ,.address_i(addr_r)
    ,.bank_i(bank_r)
    ,.command_i(command_r)
    ,.cke_i(cke_r)
    ,.accept_o(cmd_accept_w)

    ,.wrdata_i(ram_write_data_w)
    ,.wrdata_mask_i(~ram_wr_w)

    ,.rddata_valid_o(sdram_rd_valid_w)
    ,.rddata_o(sdram_data_in_w)

    ,.dfi_address_o(dfi_address_o)
    ,.dfi_bank_o(dfi_bank_o)
    ,.dfi_cas_n_o(dfi_cas_n_o)
    ,.dfi_cke_o(dfi_cke_o)
    ,.dfi_cs_n_o(dfi_cs_n_o)
    ,.dfi_odt_o(dfi_odt_o)
    ,.dfi_ras_n_o(dfi_ras_n_o)
    ,.dfi_reset_n_o(dfi_reset_n_o)
    ,.dfi_we_n_o(dfi_we_n_o)
    ,.dfi_wrdata_o(dfi_wrdata_o)
    ,.dfi_wrdata_en_o(dfi_wrdata_en_o)
    ,.dfi_wrdata_mask_o(dfi_wrdata_mask_o)
    ,.dfi_rddata_en_o(dfi_rddata_en_o)
    ,.dfi_rddata_i(dfi_rddata_i)
    ,.dfi_rddata_valid_i(dfi_rddata_valid_i)
    ,.dfi_rddata_dnv_i(dfi_rddata_dnv_i)
);

// Read data output
assign ram_read_data_w = sdram_data_in_w;

//-----------------------------------------------------------------
// Simulation only
//-----------------------------------------------------------------
`ifdef verilator
reg [79:0] dbg_state;

always @ *
begin
    case (state_q)
    STATE_INIT        : dbg_state = "INIT";
    STATE_DELAY       : dbg_state = "DELAY";
    STATE_IDLE        : dbg_state = "IDLE";
    STATE_ACTIVATE    : dbg_state = "ACTIVATE";
    STATE_READ        : dbg_state = "READ";
    STATE_WRITE       : dbg_state = "WRITE";
    STATE_PRECHARGE   : dbg_state = "PRECHARGE";
    STATE_REFRESH     : dbg_state = "REFRESH";
    default           : dbg_state = "UNKNOWN";
    endcase
end
`endif


endmodule

//-----------------------------------------------------------------
// FIFO
//-----------------------------------------------------------------
module ddr3_fifo

//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
#(
    parameter WIDTH   = 8,
    parameter DEPTH   = 4,
    parameter ADDR_W  = 2
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
