module riscvmulti (
    input 	      clk,
    input 	      reset,
    output [31:0] Address, 
    output [31:0] WriteData,
    output 	      MemWrite,
    input  [31:0] ReadData,
    output  [3:0] WriteMask, 
    output logic  halt = 0); 

    logic [31:0] instr, PC = 0;
    
    // guardar dados lidos da memoria
    logic [31:0] mem_data_out; 

    // EXECUTE -> intrucoes com resultado WAIT_DATA -> lw, dado vindo da memoria
    // write-back de LOAD agora acontece no estado WB_LOAD
    wire writeBackEn = (state == EXECUTE && (isALUreg || isALUimm || isLUI || isAUIPC || isJAL || isJALR)) || (state == WB_LOAD && isLoad);

    // lw - LOAD_data_extracted jal/jalr - PCplus4 lui - Uimm ou ALUResult
    wire [31:0] writeBackData = isLoad ? LOAD_data_extracted : 
                                 isJAL || isJALR ? PCplus4 :
                                 isLUI ? Uimm :
                                 isAUIPC ? (PC - 4) + Uimm :
                                 ALUResult;

    // rs1 + deslocamento
    wire [31:0] LoadStoreAddress = rs1 + (isStore ? Simm : Iimm);

    // pode ser PC para instrucoes ou LoadStoreAddress para dados
    assign Address = (state == FETCH_INSTR || state == WAIT_INSTR) ? PC : LoadStoreAddress;
    
    // apenas em estado STORE_WRITE
    assign MemWrite = (state == STORE_WRITE);

    // escreve a palavra toda modificada
    assign WriteData = STORE_data_final;

    // The 10 RISC-V instructions
    wire isALUreg  =  (instr[6:0] == 7'b0110011); // rd <- rs1 OP rs2   
    wire isALUimm  =  (instr[6:0] == 7'b0010011); // rd <- rs1 OP Iimm
    wire isBranch  =  (instr[6:0] == 7'b1100011); // if(rs1 OP rs2) PC<-PC+Bimm
    wire isJALR    =  (instr[6:0] == 7'b1100111); // rd <- PC+4; PC<-rs1+Iimm
    wire isJAL     =  (instr[6:0] == 7'b1101111); // rd <- PC+4; PC<-PC+Jimm
    wire isAUIPC   =  (instr[6:0] == 7'b0010111); // rd <- PC + Uimm
    wire isLUI     =  (instr[6:0] == 7'b0110111); // rd <- Uimm   
    wire isLoad    =  (instr[6:0] == 7'b0000011); // rd <- mem[rs1+Iimm]
    wire isStore   =  (instr[6:0] == 7'b0100011); // mem[rs1+Simm] <- rs2
    wire isSYSTEM  =  (instr[6:0] == 7'b1110011); // special

    // The 5 immediate formats
    wire [31:0] Uimm={    instr[31],   instr[30:12], {12{1'b0}}};
    wire [31:0] Iimm={{21{instr[31]}}, instr[30:20]};
    wire [31:0] Simm={{21{instr[31]}}, instr[30:25],instr[11:7]};
    wire [31:0] Bimm={{20{instr[31]}}, instr[7],instr[30:25],instr[11:8],1'b0};
    wire [31:0] Jimm={{12{instr[31]}}, instr[19:12],instr[20],instr[30:21],1'b0};

    // Source and destination registers
    wire [4:0] rs1Id_A1 = instr[19:15];
    wire [4:0] rs2Id_A2 = instr[24:20];
    wire [4:0] rdId_A3  = instr[11:7];

    // function codes
    wire [2:0] funct3 = instr[14:12];
    wire [6:0] funct7 = instr[31:25];

    // https://github.com/BrunoLevy/learn-fpga/blob/master/FemtoRV/TUTORIALS/FROM_BLINKER_TO_RISCV/step20.v

    wire mem_byteAccess     = funct3[1:0] == 2'b00;
    wire mem_halfwordAccess = funct3[1:0] == 2'b01;

    wire [15:0] LOAD_halfword =
        LoadStoreAddress[1] ? mem_data_out[31:16] : mem_data_out[15:0];
    wire  [7:0] LOAD_byte =
        LoadStoreAddress[0] ? LOAD_halfword[15:8] : LOAD_halfword[7:0];
    wire LOAD_sign =
     !funct3[2] & (mem_byteAccess ? LOAD_byte[7] : LOAD_halfword[15]);
    wire [31:0] LOAD_data_extracted =
           mem_byteAccess ? {{24{LOAD_sign}},      LOAD_byte} :
       mem_halfwordAccess ? {{16{LOAD_sign}}, LOAD_halfword} :
                           mem_data_out ;

    wire [3:0] STORE_wmask =
           mem_byteAccess     ?
             (LoadStoreAddress[1] ?
               (LoadStoreAddress[0] ? 4'b1000 : 4'b0100) :
               (LoadStoreAddress[0] ? 4'b0010 : 4'b0001)
               ) :
           mem_halfwordAccess ?
             (LoadStoreAddress[1] ? 4'b1100 : 4'b0011) :
             4'b1111;
             
    reg [31:0] STORE_data_final;
    always @(*) begin
      STORE_data_final = mem_data_out;
      if (STORE_wmask[0]) STORE_data_final[ 7: 0] = rs2[7:0];
      if (STORE_wmask[1]) STORE_data_final[15: 8] = rs2[7:0];
      if (STORE_wmask[2]) STORE_data_final[23:16] = rs2[7:0];
      if (STORE_wmask[3]) STORE_data_final[31:24] = rs2[7:0];

      if (STORE_wmask == 4'b0011) STORE_data_final[15:0] = rs2[15:0];
      if (STORE_wmask == 4'b1100) STORE_data_final[31:16] = rs2[15:0];

      if (STORE_wmask == 4'b1111) STORE_data_final = rs2;
    end

    // https://github.com/BrunoLevy/learn-fpga/blob/master/FemtoRV/TUTORIALS/FROM_BLINKER_TO_RISCV/step20.v

    // The registers bank
    reg [31:0] RegisterBank [0:31];
    reg [31:0] rs1; // value of source
    reg [31:0] rs2; //  registers.

    // The ALU
    wire [31:0] SrcA = (isAUIPC || isLUI) ? (PC - 4) : rs1;
    wire [31:0] SrcB = (isALUreg || isBranch) ? rs2 : 
                       (isStore) ? Simm : 
                       (isLUI) ? Uimm : Iimm;
    wire [ 4:0] shamt  = isALUreg ? rs2[4:0] : instr[24:20]; // shift amount

    // The adder is used by both arithmetic instructions and JALR.
    wire [31:0] aluPlus = SrcA + SrcB;

    // Use a single 33 bits subtract to do subtraction and all comparisons
    // (trick borrowed from swapforth/J1)
    wire [32:0] aluMinus = {1'b1, ~SrcB} + {1'b0,SrcA} + 33'b1;
    wire        LT  = (SrcA[31] ^ SrcB[31]) ? SrcA[31] : aluMinus[32];
    wire        LTU = aluMinus[32];
    wire        EQ  = (aluMinus[31:0] == 0);

    // Flip a 32 bit word. Used by the shifter (a single shifter for
    // left and right shifts, saves silicium !)
    function [31:0] flip32;
        input [31:0] x;
        flip32 = {x[ 0], x[ 1], x[ 2], x[ 3], x[ 4], x[ 5], x[ 6], x[ 7], 
        x[ 8], x[ 9], x[10], x[11], x[12], x[13], x[14], x[15], 
        x[16], x[17], x[18], x[19], x[20], x[21], x[22], x[23],
        x[24], x[25], x[26], x[27], x[28], x[29], x[30], x[31]};
    endfunction

    wire [31:0] shifter_in = (funct3 == 3'b001) ? flip32(SrcA) : SrcA;
    wire [31:0] shifter = $signed({instr[30] & SrcA[31], shifter_in}) >>> SrcB[4:0];
    wire [31:0] leftshift = flip32(shifter);

    // ADD/SUB/ADDI: 
    // funct7[5] is 1 for SUB and 0 for ADD. We need also to test instr[5]
    // to make the difference with ADDI
    //
    // SRLI/SRAI/SRL/SRA: 
    // funct7[5] is 1 for arithmetic shift (SRA/SRAI) and 
    // 0 for logical shift (SRL/SRLI)
    reg [31:0]  ALUResult;
    always @(*) begin
        case(funct3)
            3'b000: ALUResult = (funct7[5] & isALUreg) ? aluMinus[31:0] : aluPlus;
            3'b001: ALUResult = leftshift;
            3'b010: ALUResult = {31'b0, LT};
            3'b011: ALUResult = {31'b0, LTU};
            3'b100: ALUResult = (SrcA ^ SrcB);
            3'b101: ALUResult = shifter;
            3'b110: ALUResult = (SrcA | SrcB);
            3'b111: ALUResult = (SrcA & SrcB);   
        endcase
    end

    // The predicate for branch instructions
    reg takeBranch;
    always @(*) begin
        case(funct3)
            3'b000: takeBranch = EQ;
            3'b001: takeBranch = !EQ;
            3'b100: takeBranch = LT;
            3'b101: takeBranch = !LT;
            3'b110: takeBranch = LTU;
            3'b111: takeBranch = !LTU;
            default: takeBranch = 1'b0;
        endcase
    end

    // Address computation
    // An adder used to compute branch address, JAL address and AUIPC.
    // branch->PC+Bimm    AUIPC->PC+Uimm    JAL->PC+Jimm
    wire [31:0] PCplus4  = PC + 4;
    wire [31:0] PCTarget = (PC - 4) + (isJAL ? Jimm : isAUIPC ? Uimm : Bimm); 
    wire [31:0] PCNext = ((isBranch && takeBranch) || isJAL) ? PCTarget :
                                                 isJALR ? {aluPlus[31:1],1'b0} :
                                                          PCplus4;


    // The state machine
    localparam FETCH_INSTR = 0;
    localparam WAIT_INSTR  = 1;
    localparam FETCH_REGS  = 2;
    localparam EXECUTE     = 3;

    localparam LOAD_READ     = 4; // LOAD
    localparam WB_LOAD       = 5; // WAIT_DATA
    localparam STORE_READ    = 6; // STORE
    localparam STORE_WRITE   = 7; // novo estado
    
    reg [2:0] state = FETCH_INSTR;
    reg [2:0] nextstate; // proximo estado separado

    always @(*) begin
      case(state)
        FETCH_INSTR: nextstate = WAIT_INSTR;
        WAIT_INSTR:  nextstate = FETCH_REGS;
        FETCH_REGS:  nextstate = EXECUTE;
        EXECUTE:     if (isLoad)  nextstate = LOAD_READ;
                     else if (isStore) nextstate = STORE_READ;
                     else nextstate = FETCH_INSTR;
        LOAD_READ:   nextstate = WB_LOAD;
        WB_LOAD:     nextstate = FETCH_INSTR;
        STORE_READ:  nextstate = STORE_WRITE;
        STORE_WRITE: nextstate = FETCH_INSTR;
        default:     nextstate = FETCH_INSTR;
      endcase
    end

    always @(posedge clk)
        if (reset) begin
            PC    <= 0;
            state <= FETCH_INSTR;
            for (integer j = 0; j < 32; j = j + 1) begin
                RegisterBank[j] <= 32'b0;
            end
        end else begin
            if (writeBackEn && rdId_A3 != 0) begin
                RegisterBank[rdId_A3] <= writeBackData;
                //$display("r%0d <= %b (%d) (%d)",rdId_A3,writeBackData,writeBackData,$signed(writeBackData));
            end
            
            state <= nextstate;

            case(state)
                WAIT_INSTR: begin
                    instr <= ReadData;
                end
                FETCH_REGS: begin
                    rs1 <= rs1Id_A1 ? RegisterBank[rs1Id_A1] : 32'b0;
                    rs2 <= rs2Id_A2 ? RegisterBank[rs2Id_A2] : 32'b0;
                end
                EXECUTE: begin
                    if (!isSYSTEM) begin
                        PC <= PCNext;
                    end
                end
                LOAD_READ: begin
                    mem_data_out <= ReadData; 
                end
                STORE_READ: begin
                    mem_data_out <= ReadData;
                end
            endcase 
        end
endmodule