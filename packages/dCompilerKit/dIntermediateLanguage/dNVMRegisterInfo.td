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

  // alias stack pointer
  def SP : Rj<"sp", R31>, DwarfRegNum<[31]>;

  def F0	: Rf< 0, "f0">, DwarfRegNum<[32]>;
  def F1	: Rf< 1, "f1">, DwarfRegNum<[33]>;
  def F2	: Rf< 2, "f2">, DwarfRegNum<[34]>;
  def F3	: Rf< 3, "f3">, DwarfRegNum<[35]>;
  def F4	: Rf< 4, "f4">, DwarfRegNum<[36]>;
  def F5	: Rf< 5, "f5">, DwarfRegNum<[37]>;
  def F6	: Rf< 6, "f6">, DwarfRegNum<[38]>;
  def F7	: Rf< 7, "f7">, DwarfRegNum<[39]>;
  def F8	: Rf< 8, "f8">, DwarfRegNum<[40]>;
  def F9	: Rf< 9, "f9">, DwarfRegNum<[41]>;
  def F10	: Rf<10, "f10">, DwarfRegNum<[42]>;
  def F11	: Rf<11, "f11">, DwarfRegNum<[43]>;
  def F12	: Rf<12, "f12">, DwarfRegNum<[44]>;
  def F13	: Rf<13, "f13">, DwarfRegNum<[45]>;
  def F14	: Rf<14, "f14">, DwarfRegNum<[46]>;
  def F15	: Rf<15, "f15">, DwarfRegNum<[47]>;
  def F16	: Rf<16, "f16">, DwarfRegNum<[48]>;
  def F17	: Rf<17, "f17">, DwarfRegNum<[49]>;
  def F18	: Rf<18, "f18">, DwarfRegNum<[50]>;
  def F19	: Rf<19, "f19">, DwarfRegNum<[51]>;
  def F20	: Rf<20, "f20">, DwarfRegNum<[52]>;
  def F21	: Rf<21, "f21">, DwarfRegNum<[53]>;
  def F22	: Rf<22, "f22">, DwarfRegNum<[54]>;
  def F23	: Rf<23, "f23">, DwarfRegNum<[55]>;
  def F24	: Rf<24, "f24">, DwarfRegNum<[56]>;
  def F25	: Rf<25, "f25">, DwarfRegNum<[57]>;
  def F26	: Rf<26, "f26">, DwarfRegNum<[58]>;
  def F27	: Rf<27, "f27">, DwarfRegNum<[59]>;
  def F28	: Rf<28, "f28">, DwarfRegNum<[60]>;
  def F29	: Rf<29, "f29">, DwarfRegNum<[61]>;
  def F30	: Rf<30, "f30">, DwarfRegNum<[62]>;
  def F31	: Rf<31, "f31">, DwarfRegNum<[63]>;

  // Control registers.
  def PC   : Rc<9,  "pc">, DwarfRegNum<[64]>; 
  def CTRL : Rc<1, "ctrl">, DwarfRegNum<[65]>;

}

// Register classes.
def IntegerRegs : RegisterClass<"Newton", [i32], 32, (add (sequence "r%u", 0, 31))> 
{
}

def floatRegs : RegisterClass<"Newton", [f64], 32, (add (sequence "f%u", 0, 31))> 
{
}


def ControlRegs : RegisterClass<"Newton", [i32], 32, (add CTRL, PC)> 
{
  let Size = 32;
}
