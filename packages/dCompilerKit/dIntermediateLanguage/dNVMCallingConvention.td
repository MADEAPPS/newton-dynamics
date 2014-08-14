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


//
// This describes the calling conventions for NVM architecture.


/// CCIfSubtarget - Match if the current subtarget has a feature F.
class CCIfSubtarget<string F, CCAction A>: CCIf<!strconcat("State.getTarget().getSubtarget<dNVMSubTarget>().", F), A>;

def CSR_O32 : CalleeSavedRegs<(add (sequence "R%u", 2, 3, 4, 5))>;
