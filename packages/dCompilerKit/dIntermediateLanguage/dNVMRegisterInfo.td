/* Copyright (c) <2009> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


include "../dIntermediateLanguage/Target.td"

let Namespace = "Newton" in 
{
  class NVMRegister<string n> : Register<n> 
  {
    field bits<5> Num;
  }

  class NewtonDoubleReg<string n, list<Register> subregs> : RegisterWithSubRegs<n, subregs> 
  {
    field bits<5> Num;
  }

  // Registers are identified with 5-bit ID numbers.
  // Ri - 32-bit integer registers.
  class Ri<bits<5> num, string n> : NVMRegister<n> 
  {
    let Num = num;
  }

  // Rf - 32-bit floating-point registers.
  class Rf<bits<5> num, string n> : NVMRegister<n> 
  {
    let Num = num;
  }


  // Rp - predicate registers
  class Rp<bits<5> num, string n> : NVMRegister<n> 
  {
    let Num = num;
  }

  // Rc - control registers
  class Rc<bits<5> num, string n> : NVMRegister<n> 
  {
    let Num = num;
  }

  // Rj - aliased integer registers
  class Rj<string n, Ri R>: NVMRegister<n> 
  {
    let Num = R.Num;
    let Aliases = [R];
  }

  def subreg_loreg  : SubRegIndex<32>;
  def subreg_hireg  : SubRegIndex<32, 32>;

  // Integer registers.
  def R0 : Ri< 0, "r0">, DwarfRegNum<[0]>;
  def R1 : Ri< 1, "r1">, DwarfRegNum<[1]>;
  def R2 : Ri< 2, "r2">, DwarfRegNum<[2]>;
  def R3 : Ri< 3, "r3">, DwarfRegNum<[3]>;
  def R4 : Ri< 4, "r4">, DwarfRegNum<[4]>;
  def R5 : Ri< 5, "r5">, DwarfRegNum<[5]>;
  def R6 : Ri< 6, "r6">, DwarfRegNum<[6]>;
  def R7 : Ri< 7, "r7">, DwarfRegNum<[7]>;
  def R8 : Ri< 8, "r8">, DwarfRegNum<[8]>;
  def R9 : Ri< 9, "r9">, DwarfRegNum<[9]>;
  def R10 : Ri<10, "r10">, DwarfRegNum<[10]>;
  def R11 : Ri<11, "r11">, DwarfRegNum<[11]>;
  def R12 : Ri<12, "r12">, DwarfRegNum<[12]>;
  def R13 : Ri<13, "r13">, DwarfRegNum<[13]>;
  def R14 : Ri<14, "r14">, DwarfRegNum<[14]>;
  def R15 : Ri<15, "r15">, DwarfRegNum<[15]>;
  def R16 : Ri<16, "r16">, DwarfRegNum<[16]>;
  def R17 : Ri<17, "r17">, DwarfRegNum<[17]>;
  def R18 : Ri<18, "r18">, DwarfRegNum<[18]>;
  def R19 : Ri<19, "r19">, DwarfRegNum<[19]>;
  def R20 : Ri<20, "r20">, DwarfRegNum<[20]>;
  def R21 : Ri<21, "r21">, DwarfRegNum<[21]>;
  def R22 : Ri<22, "r22">, DwarfRegNum<[22]>;
  def R23 : Ri<23, "r23">, DwarfRegNum<[23]>;
  def R24 : Ri<24, "r24">, DwarfRegNum<[24]>;
  def R25 : Ri<25, "r25">, DwarfRegNum<[25]>;
  def R26 : Ri<26, "r26">, DwarfRegNum<[26]>;
  def R27 : Ri<27, "r27">, DwarfRegNum<[27]>;
  def R28 : Ri<28, "r28">, DwarfRegNum<[28]>;
  def R29 : Ri<29, "r29">, DwarfRegNum<[29]>;
  def R30 : Ri<30, "r30">, DwarfRegNum<[30]>;
  def R31 : Ri<31, "r31">, DwarfRegNum<[31]>;

  def SP : Rj<"sp", R29>, DwarfRegNum<[29]>;
  def FP : Rj<"fp", R30>, DwarfRegNum<[30]>;
  def LR : Rj<"lr", R31>, DwarfRegNum<[31]>;

  
  // Predicate registers.
  def P0 : Rp<0, "p0">, DwarfRegNum<[63]>;
  def P1 : Rp<1, "p1">, DwarfRegNum<[64]>;
  def P2 : Rp<2, "p2">, DwarfRegNum<[65]>;
  def P3 : Rp<3, "p3">, DwarfRegNum<[66]>;

  // Control registers.
  def SA0 : Rc<0, "sa0">, DwarfRegNum<[67]>;
  def LC0 : Rc<1, "lc0">, DwarfRegNum<[68]>;

  def SA1 : Rc<2, "sa1">, DwarfRegNum<[69]>;
  def LC1 : Rc<3, "lc1">, DwarfRegNum<[70]>;

  def M0 : Rc<6, "m0">, DwarfRegNum<[71]>;
  def M1 : Rc<7, "m1">, DwarfRegNum<[72]>;

  def PC : Rc<9,  "pc">, DwarfRegNum<[32]>; // is the Dwarf number correct?
  def GP : Rc<11, "gp">, DwarfRegNum<[33]>; // is the Dwarf number correct?
}

// Register classes.
//
// FIXME: the register order should be defined in terms of the preferred
// allocation order...
//
def IntRegs : RegisterClass<"Newton", [i32,f32], 32, (add (sequence "R%u", 0, 9), (sequence "R%u", 12, 28), R10, R11, R29, R30, R31)> 
{
}


def PredRegs : RegisterClass<"Newton", [i1], 32, (add (sequence "P%u", 0, 3))>
{
  let Size = 32;
}

def CRRegs : RegisterClass<"Newton", [i32], 32, (add (sequence "LC%u", 0, 1), (sequence "SA%u", 0, 1), (sequence "M%u", 0, 1), PC, GP)> 
{
  let Size = 32;
}
