/* Copyright (c) <2009> <nvm Game Dynamics>
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

let Namespace = "nvm" in 
{
	  class nvmRegister<string n> : Register<n> 
	  {
		field bits<5> Num;
	  }

	  // Ri - 32-bit integer registers.
	  class Ri<bits<5> num, string n> : nvmRegister<n> 
	  {
		let Num = num;
	  }

	  // Rf - 64-bit floating-point register.
	  class Rf<bits<5> num, string n> : nvmRegister<n> 
	  {
		let Num = num;
	  }

	  // Rc - control registers
	  class Rc<bits<5> num, string n> : nvmRegister<n> 
	  {
		let Num = num;
	  }

	  // Rj - aliased integer registers
	  class Rj<string n, Ri R>: nvmRegister<n> 
	  {
		let Num = R.Num;
		let Aliases = [R];
	  }

	  // define integer register set
	  foreach I = 0-31 in
	  def R#I : Ri<I, "r"#I>, DwarfRegNum<[#I]>;

	  // define floating point register set
	  foreach I = 32-63 in
	  def F#I : Rf<I, "f"#I>, DwarfRegNum<[#I]>;
  
	  // alias stack pointer
	  def SP : Rj<"sp", R31>, DwarfRegNum<[31]>;

	  // Control registers.
	  def PC   : Rc<0, "pc">, DwarfRegNum<[64]>; 
	  def CTRL : Rc<1, "ctrl">, DwarfRegNum<[65]>;
}

// Register classes.
def IntegerRegs : RegisterClass<"nvm", [i32], 32, (add (sequence "r%u", 0, 31))> 
{
	let Size = 64;
}

def floatRegs : RegisterClass<"nvm", [f64], 64, (add (sequence "f%u", 0, 31))> 
{
	let Size = 64;
}

def ControlRegs : RegisterClass<"nvm", [i32], 32, (add CTRL, PC)> 
{
	let Size = 32;
}
