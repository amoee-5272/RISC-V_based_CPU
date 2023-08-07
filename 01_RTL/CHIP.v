//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//
module CHIP #(                                                                                  //
    parameter BIT_W = 32                                                                        //
)(                                                                                              //
    // clock                                                                                    //
        input               i_clk,                                                              //
        input               i_rst_n,                                                            //
    // instruction memory                                                                       //
        input  [BIT_W-1:0]  i_IMEM_data,    //instruction binary code                           //
        output [BIT_W-1:0]  o_IMEM_addr,    //pc address                                        //
        output              o_IMEM_cen,     //set hogh to "load"                                //
    // data memory                                                                              //
        input               i_DMEM_stall,                                                       //
        input  [BIT_W-1:0]  i_DMEM_rdata,                                                       //
        output              o_DMEM_cen,                                                         //
        output              o_DMEM_wen,                                                         //
        output [BIT_W-1:0]  o_DMEM_addr,                                                   //
        output [BIT_W-1:0]  o_DMEM_wdata                                                    //
);                                                                                              //
//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameters
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any declaration

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
    reg [BIT_W-1:0] PC, next_PC;
    //wire mem_cen, mem_wen;
    //wire [BIT_W-1:0] mem_addr, mem_wdata, mem_rdata;
    wire stall,mem_stall,mul_stall;
    wire Branch;
    wire MemRead;
    wire MemtoReg;
    wire [4:0] ALUOp;
    wire MemWrite;
    wire ALUSrc;
    wire RegWrite;
    wire [2:0]ALU_inst;
    wire wen; //0:reg read 1:reg write
    wire [31:0]wdata; //the data going to write into reg
    reg [31:0]r_wdata;
    wire [31:0]rdata1,rdata2; //data read from reg
    wire [31:0]data1,data2; // data input of alu
    reg [31:0]r_data1,r_data2;
    wire [31:0]alu_result; // data output of alu
    wire select;//branch select
    wire [31:0] imm;
    //reg  [3:0]state,next_state;
    wire do_mul;
    wire [31:0]mul_result;
    //cache
    wire proc_cen;
    wire proc_wen;
    wire [31:0] proc_addr;
    reg [31:0] r_proc_addr;
    wire [31:0] proc_wdata;
    reg [31:0] r_proc_wdata;
    wire [31:0] proc_rdata;
    wire proc_stall;
    wire DMEM_cen;
    wire DMEM_wen;
    wire [31:0] DMEM_addr;
    wire [31:0] DMEM_wdata;
    wire [31:0] DMEM_rdata;
    wire DMEM_stall;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    assign wdata = r_wdata;
    assign data1 = r_data1;
    assign data2 = r_data2;
    assign proc_cen = (MemWrite||MemRead)? 1:0;
    assign proc_wen = (MemWrite)? 1:0;
    assign mem_stall = proc_stall;
    assign stall = ((mem_stall||mul_stall)==1)? 1:0;
    assign do_mul = (ALUOp == 5'd13)?1:0;

    assign proc_addr = r_proc_addr;
    assign proc_wdata = r_proc_wdata;
    assign o_DMEM_addr = DMEM_addr;
    assign o_DMEM_cen = DMEM_cen;
    assign o_DMEM_wen = DMEM_wen;
    assign o_DMEM_wdata = DMEM_wdata;
    //assign i_DMEM_stall = DMEM_stall;
    //assign i_mem_rdata = DMEM_rdata;


    // TODO: any wire assignment

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    assign wen = (RegWrite)? 1:0; 
    Reg_file reg0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (wen),          
        .rs1    (i_IMEM_data[19:15]),                
        .rs2    (i_IMEM_data[24:20]),                
        .rd     (i_IMEM_data[11:7]),                 
        .wdata  (wdata),             
        .rdata1 (rdata1),           
        .rdata2 (rdata2)
    );
    //control block
    Control ctrl(
        .instruction(i_IMEM_data),
        .Stall(stall),
        .Branch(Branch),
        .MemRead(MemRead),
        .MemtoReg(MemtoReg), 
        .ALUOp(ALUOp),
        .MemWrite(MemWrite),
        .ALUSrc(ALUSrc),
        .RegWrite(RegWrite)
    );
    //alu_control block
    alu_control ALU_CONTROL(
        .ALUOp(ALUOp),
        .ALU_inst(ALU_inst)
    );
    ALU alu(
        .i_A(data1),
        .i_B(data2),
        .ALU_inst(ALU_inst),
        .o_data(alu_result)
    );
    BranchDecide branchdecide(
        .i_Branch(Branch), 
        .i_ALUOp(ALUOp), 
        .i_ALUresult(alu_result), 
        .o_select(select)
    );
    ImmGen immgen(
        .i_ALUOp(ALUOp),
        .i_inst(i_IMEM_data),
        .o_imm(imm)
    );
    MUL mul(
        .i_clk(i_clk),
        .i_rst_n(i_rst_n), 
        .i_valid(do_mul), 
        .i_A(data1),
        .i_B(data2),
        .o_data(mul_result),
        .o_stall(mul_stall)
    );
    Cache cache(
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        // processor interface
        .i_proc_cen(proc_cen),
        .i_proc_wen(proc_wen),
        .i_proc_addr(proc_addr),
        .i_proc_wdata(proc_wdata),
        .o_proc_rdata(proc_rdata),
        .o_proc_stall(proc_stall),
        // memory interface
        .o_mem_cen(DMEM_cen),
        .o_mem_wen(DMEM_wen),
        .o_mem_addr(DMEM_addr),
        .o_mem_wdata(DMEM_wdata),
        .i_mem_rdata(i_DMEM_rdata),
        .i_mem_stall(i_DMEM_stall)
    );

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Always Blocks
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    always @(*)begin
        r_proc_wdata = 32'b0;
        r_wdata = 32'b0;
        r_data1 = 32'b0;
        r_data2 = (ALUSrc)? imm:rdata2;
        r_proc_addr = 32'b0;
        
        case(ALUOp) 
            5'd0:begin//auipc
                next_PC = PC + 4;
                r_wdata = PC + imm;
            end
            5'd1:begin //jal
                r_wdata = PC + 4;
                next_PC = PC +imm;
            end
            5'd2:begin //jalr
                next_PC = rdata1 +imm;
            end
            5'd3:begin //add
                next_PC = PC + 4;
                r_data1 = rdata1;
                //r_data2 = rdata2;
                r_wdata = alu_result;
            end
            5'd4:begin //sub
                next_PC = PC + 4;
                r_data1 = rdata1;
                //r_data2 = rdata2;
                r_wdata = alu_result;
            end
            5'd5:begin //and
                next_PC = PC + 4;
                r_data1 = rdata1;
                //r_data2 = rdata2;
                r_wdata = alu_result;
            end
            5'd6:begin //xor
                next_PC = PC + 4;
                r_data1 = rdata1;
                //r_data2 = rdata2;
                r_wdata = alu_result;
            end
            5'd7:begin //addi
                next_PC = PC + 4;
                r_data1 = rdata1;
                //r_data2 = imm;
                r_wdata = alu_result;
            end
            5'd8:begin //slli
                next_PC = PC + 4;
                r_data1 = rdata1;
                //r_data2 = imm;
                r_wdata = alu_result;
            end
            5'd9:begin //slti
                next_PC = PC + 4;
                r_data1 = rdata1;
                //r_data2 = imm;
                r_wdata = alu_result;
            end
            5'd10:begin //srai
                next_PC = PC + 4;
                r_data1 = rdata1;
                //r_data2 = imm;
                r_wdata = alu_result;
                
            end
            5'd11:begin //lw
                next_PC = PC + 4;
                r_proc_addr = rdata1 + imm;
                r_wdata = proc_rdata;
            end
            5'd12:begin //sw
                next_PC = PC + 4;
                r_proc_addr = rdata1 + imm;
                r_proc_wdata = rdata2;
            end
            5'd13:begin //mul
                next_PC = PC + 4;
                r_data1 = rdata1;
                //r_data2 = rdata2;
                r_wdata = mul_result;
            end
            5'd14:begin //beq
                r_data1 = rdata1;
                //r_data2 = rdata2;
                next_PC = (select)? PC+imm:PC+4;
            end
            5'd15:begin //bge
                r_data1 = rdata1;
                //r_data2 = rdata2;
                next_PC = (select)? PC+imm:PC+4;
            end
            5'd16:begin //blt
                r_data1 = rdata1;
                //r_data2 = rdata2;
                next_PC = (select)? PC+imm:PC+4;
            end
            5'd17:begin //bne
                r_data1 = rdata1;
                //r_data2 = rdata2;
                next_PC = (select)? PC+imm:PC+4;
            end
            default: next_PC = PC;
        endcase
        
    end
    assign o_IMEM_addr = PC;
    assign o_IMEM_cen = 1;
    // Todo: any combinational/sequential circuit

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
        end
        else begin
            PC <= (stall)? PC:next_PC;
        end
    end
endmodule

module BranchDecide(i_Branch, i_ALUOp, i_ALUresult, o_select);
    parameter BITS = 32;
    input i_Branch;
    input [4:0] i_ALUOp;
    input signed [BITS-1:0] i_ALUresult;
    output o_select;

    reg o_select_r;
    assign o_select=o_select_r;

    always @(*) begin
        case (i_ALUOp)
            5'd14: begin //beq
                o_select_r=(i_Branch&&(i_ALUresult==0)) ?1:0;
            end
            5'd15: begin //bge
                o_select_r=(i_Branch&&(i_ALUresult>=0)) ?1:0;
            end
            5'd16: begin //blt
                o_select_r=(i_Branch&&(i_ALUresult<0)) ?1:0;
            end
            5'd17: begin //bne
                o_select_r=(i_Branch&&(i_ALUresult!=0)) ?1:0;
            end
            default: o_select_r=0;
        endcase
    end
endmodule

module ImmGen(i_ALUOp, i_inst, o_imm);
    parameter BITS = 32;
    input [4:0] i_ALUOp;
    input [BITS-1:0] i_inst;
    output [BITS-1:0] o_imm;
    reg [BITS-1:0]o_imm;

    always @(*) begin
        case (i_ALUOp)
            5'd0: o_imm = {i_inst[31:12],12'h000};
            5'd1: o_imm = {{11{i_inst[31]}},i_inst[31],i_inst[19:12],i_inst[20],i_inst[30:21],1'b0};
            5'd2: o_imm = {{21{i_inst[31]}},i_inst[30:20]};
            5'd7: o_imm = {{21{i_inst[31]}},i_inst[30:20]};
            5'd8: o_imm = {{27{1'b0}},i_inst[24:20]};
            5'd9: o_imm = {{21{i_inst[31]}},i_inst[30:20]};
            5'd10: o_imm = {{27{1'b0}},i_inst[24:20]};
            5'd11: o_imm = {{21{i_inst[31]}},i_inst[30:20]};
            5'd12: o_imm = {{21{i_inst[31]}},i_inst[30:25],i_inst[11:7]};
            5'd14: o_imm = {{20{i_inst[31]}},i_inst[30],i_inst[7],i_inst[30:25],i_inst[11:8],1'b0};
            5'd15: o_imm = {{20{i_inst[31]}},i_inst[30],i_inst[7],i_inst[30:25],i_inst[11:8],1'b0};
            5'd16: o_imm = {{20{i_inst[31]}},i_inst[30],i_inst[7],i_inst[30:25],i_inst[11:8],1'b0};
            5'd17: o_imm = {{20{i_inst[31]}},i_inst[30],i_inst[7],i_inst[30:25],i_inst[11:8],1'b0};
            default: o_imm=32'd0;
        endcase
    end
endmodule

module Reg_file(i_clk, i_rst_n, wen, rs1, rs2, rd, wdata, rdata1, rdata2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input i_clk, i_rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] wdata;
    input [addr_width-1:0] rs1, rs2, rd;

    output [BITS-1:0] rdata1, rdata2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign rdata1 = mem[rs1];
    assign rdata2 = mem[rs2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (rd == i)) ? wdata : mem[i]; //write data 
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module Cache#(
        parameter BIT_W = 32,
        parameter ADDR_W = 32,
        parameter OS = 32'h0001_0000
    )(
        input i_clk,
        input i_rst_n,
        // processor interface
            input i_proc_cen,
            input i_proc_wen,
            input [ADDR_W-1:0] i_proc_addr,
            input [BIT_W-1:0]  i_proc_wdata,
            output [BIT_W-1:0] o_proc_rdata,
            output o_proc_stall,
        // memory interface
            output o_mem_cen,
            output o_mem_wen,
            output [ADDR_W-1:0] o_mem_addr,
            output [BIT_W-1:0]  o_mem_wdata,
            input [BIT_W-1:0] i_mem_rdata,
            input i_mem_stall
    );
        reg [31:0] cache_mem_data[15:0],next_cache_mem_data[15:0];
        reg [31:0] cache_mem_addr[15:0],next_cache_mem_addr[15:0];
        reg valid[15:0],next_valid[15:0];
        reg [1:0] state,next_state;
        integer i;
        reg [31:0] r_proc_rdata;
        assign o_proc_rdata = (state==2'b00||state==2'b01)? r_proc_rdata:o_proc_rdata;
        always @(*) begin
            for(i=0;i<16;i=i+1)begin
                    next_valid[i] = valid[i];
                    next_cache_mem_addr[i] = cache_mem_addr[i];
                    next_cache_mem_data[i] = cache_mem_data[i];
            end
            next_state = state;
            r_proc_rdata = 32'h0;
            case(state)
                2'b00:begin  //IDLE

                    if(i_proc_cen && !i_proc_wen)begin //read
                        i=i_proc_addr[5:2];
                        if(valid[i]==1 && cache_mem_addr[i]==i_proc_addr)begin
                            r_proc_rdata = cache_mem_data[i];
                            next_state = 2'b11;
                        end
                        else next_state = 2'b01;
                    end
                    else if(i_proc_cen && i_proc_wen)next_state=2'b10;//write
                    else next_state=state;
                end
                2'b01:begin //access mem read
                    if(i_mem_stall==1)next_state = 2'b01;
                    else begin
                        i = o_mem_addr[5:2];
                        next_valid[i] = 1;
                        next_cache_mem_data[i] = i_mem_rdata;
                        next_cache_mem_addr[i] = i_proc_addr;
                        next_state = 2'b11;
                        r_proc_rdata = i_mem_rdata;
                    end
                end
                2'b10:begin //write through
                    if(i_mem_stall==1)next_state = 2'b10;
                    else begin
                        i = o_mem_addr[5:2];
                        next_valid[i] = 1;
                        next_cache_mem_data[i] = i_proc_wdata;
                        next_cache_mem_addr[i] = i_proc_addr;
                        next_state = 2'b11;
                    end 
                end
                2'b11: begin
                    next_state = 2'b00;//end
                end
            endcase
        end
        always @(posedge i_clk or negedge i_rst_n) begin
            if (!i_rst_n) begin
                state <= 2'b00;
                for(i=0;i<16;i=i+1)begin
                    valid[i] <= 0;
                    cache_mem_addr[i] <= 0;
                    cache_mem_data[i] <= 0;
                end
            end
            else begin
                state <= next_state;
                for(i=0;i<16;i=i+1)begin
                    valid[i] <= next_valid[i];
                    cache_mem_addr[i] <= next_cache_mem_addr[i];
                    cache_mem_data[i] <= next_cache_mem_data[i];
                end
            end       
        end
    //---------------------------------------//
    //          default connection           //
    assign o_mem_cen = (state ==2'b01||state==2'b10)? i_proc_cen:0;        //
    assign o_mem_wen = (state ==2'b01||state==2'b10)? i_proc_wen:0;        //
    assign o_mem_addr = i_proc_addr;      //
    assign o_mem_wdata = i_proc_wdata;    //
    assign o_proc_stall = (i_proc_cen && (state!=2'b11))? 1:0;    //
    //---------------------------------------//

    // Todo: BONUS
endmodule
module Control(instruction,Stall,Branch, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite);
    // I/O INTERFACE 
    input [31:0] instruction;
    input Stall;
    output Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite;
    output [4:0] ALUOp;
    //
    reg [6:0] opcode;
    reg [2:0] funct3;
    reg [6:0] funct7;
    reg [4:0] ALUOp;
    always @(*) begin
        ALUOp = 5'd18;
        opcode = instruction[6:0];
        funct3 = instruction[14:12];
        funct7 = instruction[31:25];
        case(opcode)
            7'b0010111: begin //auipc
                ALUOp = 5'd0;
            end
            7'b1101111: begin //jal
                ALUOp = 5'd1;
            end
            7'b1100111: begin //jalr
                ALUOp = 5'd2;
                
            end
            7'b0110011: begin
                if(funct7[5])ALUOp = 5'd4;//sub
                else if(funct3[0]) ALUOp = 5'd5;//and
                else if(funct3[2]) ALUOp = 5'd6;//xor
                else if(funct7[0]) ALUOp = 5'd13;//mul
                else ALUOp=5'd3; //add
            end
            7'b0010011: begin
                case(funct3) 
                    3'b000: ALUOp = 5'd7;//addi
                    3'b001: ALUOp = 5'd8;//slli
                    3'b010: ALUOp = 5'd9;//slti
                    3'b101: ALUOp = 5'd10;//srai
                endcase
            end
            7'b0000011:begin
                ALUOp = 5'd11;//lw
            end
            7'b0100011:begin
                ALUOp = 5'd12;//sw
            end
            7'b1100011:begin
                case(funct3) 
                    3'b000: ALUOp = 5'd14;//beq
                    3'b101: ALUOp = 5'd15;//bge
                    3'b100: ALUOp = 5'd16;//blt
                    3'b001: ALUOp = 5'd17;//bne
                endcase
            end
            default: begin
                ALUOp = 5'd18; //something wrong
            end
            
        endcase
    end
    assign Branch = (ALUOp==5'd1||ALUOp==5'd2||ALUOp==5'd14||ALUOp==5'd15||ALUOp==5'd16||ALUOp==5'd17)? 1:0;
    assign MemRead = (ALUOp==5'd11)? 1:0;
    assign MemtoReg = (ALUOp==5'd11)? 1:0;
    assign MemWrite = (ALUOp==5'd12)? 1:0;
    assign  ALUSrc = (ALUOp==5'd7||ALUOp==5'd8||ALUOp==5'd9||ALUOp==5'd10)?1:0;
    assign RegWrite = (!Stall&&(ALUOp==5'd0||ALUOp==5'd1||ALUOp==5'd3||ALUOp==5'd4||ALUOp==5'd5||ALUOp==5'd6||ALUOp==5'd7||ALUOp==5'd8||ALUOp==5'd9||ALUOp==5'd10||ALUOp==5'd11||ALUOp==5'd13))? 1:0;

endmodule
module alu_control(ALUOp, ALU_inst);
    input [4:0] ALUOp;
    output [2:0] ALU_inst;
    reg [2:0] ALU_inst;
    always @(*) begin
        case(ALUOp)
            5'd0: ALU_inst=3'd7;
            5'd1: ALU_inst=3'd7;
            5'd2: ALU_inst=3'd7;
            5'd3: ALU_inst=3'd0;
            5'd4: ALU_inst=3'd1;
            5'd5: ALU_inst=3'd2;
            5'd6: ALU_inst=3'd3;
            5'd7: ALU_inst=3'd0;
            5'd8: ALU_inst=3'd7;
            5'd9: ALU_inst=3'd4;
            5'd10: ALU_inst=3'd5;
            5'd11: ALU_inst=3'd0;
            5'd12: ALU_inst=3'd0;
            5'd13: ALU_inst=3'd6;
            5'd14: ALU_inst=3'd1;
            5'd15: ALU_inst=3'd1;
            5'd16: ALU_inst=3'd1;
            5'd17: ALU_inst=3'd1;
            default: ALU_inst=3'd0;
        endcase
    end
endmodule
module ALU(i_A,i_B, ALU_inst, o_data);
    input  [31:0] i_A, i_B;
    input  [2:0]  ALU_inst;
    output [31:0] o_data;
    reg [2:0]inst;  
    reg [31:0]o_data;  
    always @(*) begin
        inst = ALU_inst;
        case(inst)
            3'd0: o_data = i_A + i_B;  // add
            3'd1: o_data = i_A - i_B;  // sub
            3'd2: o_data = i_A & i_B;  // and
            3'd3: o_data = i_A ^ i_B;  // and
            3'd4: o_data = ($signed(i_A) < $signed(i_B)) ? 1:0; //set less than
            3'd5: o_data = i_A >>> i_B;  // shift right arithmetic
            3'd7: o_data = i_A <<< i_B;  // shift left arithmetic
            default: o_data = 32'b0;
        endcase
    end
    
endmodule

module MUL #(
    parameter DATA_W = 32
)
(
    input                       i_clk,
    input                       i_rst_n,
    input                       i_valid,
    input [DATA_W - 1 : 0]      i_A,
    input [DATA_W - 1 : 0]      i_B,
    output [DATA_W - 1 : 0]     o_data,
    output                      o_stall
);
// Parameters
    // Definition of states
    parameter S_IDLE = 2'b00;
    parameter S_MUL = 2'b01;
    parameter S_OUT = 2'b10;
// Wires & Regs
    reg  [1:0] state, next_state;
    reg  [2*DATA_W - 1 : 0] nex_a,nex_b,a,b;
    reg  [2*DATA_W - 1 : 0] result,nex_result;
    reg  [6:0] count,nex_count;
    reg  c_out;   
// Always Combination
        always @(*) begin
            nex_count = count;
            nex_result = result;
            nex_a = a;
            nex_b = b;
            case(state)
                S_IDLE  : begin
                    nex_count = 0;
                    nex_result = 64'b0;
                    if(i_valid)begin
                        nex_a=i_A;
                        nex_b=i_B;
                        next_state = S_MUL;
                    end
                    else next_state=S_IDLE;
                end 
                S_MUL   :begin
                    next_state = (count==31)? S_OUT : S_MUL;
                    nex_result = (count==0)? nex_a : result;
                    c_out=0;
                    if(nex_result[0]==1)begin
                        nex_result = {{nex_result[63:32]+nex_b[31:0]},nex_result[31:0]};
                        c_out = (result[63:32]+nex_b[31:0]<nex_b[31:0])? 1'b1:1'b0; //add overflow
                    end
                    else nex_result = nex_result;
                    
                    nex_result = nex_result>>1;
                    nex_result[63]=c_out;
                    nex_count = count + 1;
                end
                S_OUT   : next_state = S_IDLE;
                default : next_state = state;
            endcase
        end
        assign o_data = (state == S_OUT) ? result[31:0] : 0;
        assign o_stall = (i_valid && state != S_OUT) ? 1 : 0;  
    // Todo: Sequential always block
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state <= S_IDLE;
        end
        else begin
            state <= next_state;
            count <= nex_count;
            result <= nex_result;
            a <= nex_a;
            b <= nex_b;
        end
    end

endmodule
