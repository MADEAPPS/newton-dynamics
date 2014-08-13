------------- Classes -----------------
class AsmOperandClass {
  string Name = ?;
  list<AsmOperandClass> SuperClasses = [];
  string PredicateMethod = ?;
  string RenderMethod = ?;
  string ParserMethod = ?;
  string DiagnosticType = "";
  string NAME = ?;
}
class AsmParser {
  string AsmParserClassName = "AsmParser";
  string AsmParserInstCleanup = "";
  bit ShouldEmitMatchRegisterName = 1;
  bit MnemonicContainsDot = 0;
  string NAME = ?;
}
class AsmParserVariant {
  int Variant = 0;
  string Name = "";
  string CommentDelimiter = "";
  string RegisterPrefix = "";
  string NAME = ?;
}
class AsmWriter {
  string AsmWriterClassName = "InstPrinter";
  int Variant = 0;
  int OperandSpacing = -1;
  string NAME = ?;
}
class AssemblerPredicate<string AssemblerPredicate:cond = ?, string AssemblerPredicate:name = ""> {
  bit AssemblerMatcherPredicate = 1;
  string AssemblerCondString = AssemblerPredicate:cond;
  string PredicateName = AssemblerPredicate:name;
  string NAME = ?;
}
class Bypass {
  string NAME = ?;
}
class CCAction {
  string NAME = ?;
}
class CCAssignToReg<list<Register> CCAssignToReg:regList = ?> {	// CCAction
  list<Register> RegList = CCAssignToReg:regList;
  string NAME = ?;
}
class CCAssignToRegWithShadow<list<Register> CCAssignToRegWithShadow:regList = ?, list<Register> CCAssignToRegWithShadow:shadowList = ?> {	// CCAction
  list<Register> RegList = CCAssignToRegWithShadow:regList;
  list<Register> ShadowRegList = CCAssignToRegWithShadow:shadowList;
  string NAME = ?;
}
class CCAssignToStack<int CCAssignToStack:size = ?, int CCAssignToStack:align = ?> {	// CCAction
  int Size = CCAssignToStack:size;
  int Align = CCAssignToStack:align;
  string NAME = ?;
}
class CCAssignToStackWithShadow<int CCAssignToStackWithShadow:size = ?, int CCAssignToStackWithShadow:align = ?, list<Register> CCAssignToStackWithShadow:shadowList = ?> {	// CCAction
  int Size = CCAssignToStackWithShadow:size;
  int Align = CCAssignToStackWithShadow:align;
  list<Register> ShadowRegList = CCAssignToStackWithShadow:shadowList;
  string NAME = ?;
}
class CCBitConvertToType<ValueType CCBitConvertToType:destTy = ?> {	// CCAction
  ValueType DestTy = CCBitConvertToType:destTy;
  string NAME = ?;
}
class CCCustom<string CCCustom:fn = ?> {	// CCAction
  string FuncName = CCCustom:fn;
  string NAME = ?;
}
class CCDelegateTo<CallingConv CCDelegateTo:cc = ?> {	// CCAction
  CallingConv CC = CCDelegateTo:cc;
  string NAME = ?;
}
class CCIf<string CCIf:predicate = ?, CCAction CCIf:A = ?> {	// CCAction CCPredicateAction
  CCAction SubAction = CCIf:A;
  string Predicate = CCIf:predicate;
  string NAME = ?;
}
class CCIfByVal<CCAction CCIfByVal:A = ?> {	// CCAction CCPredicateAction CCIf
  CCAction SubAction = CCIfByVal:A;
  string Predicate = "ArgFlags.isByVal()";
  string NAME = ?;
}
class CCIfCC<string CCIfCC:CC = ?, CCAction CCIfCC:A = ?> {	// CCAction CCPredicateAction CCIf
  CCAction SubAction = CCIfCC:A;
  string Predicate = !strconcat("State.getCallingConv() == ", CCIfCC:CC);
  string NAME = ?;
}
class CCIfConsecutiveRegs<CCAction CCIfConsecutiveRegs:A = ?> {	// CCAction CCPredicateAction CCIf
  CCAction SubAction = CCIfConsecutiveRegs:A;
  string Predicate = "ArgFlags.isInConsecutiveRegs()";
  string NAME = ?;
}
class CCIfInReg<CCAction CCIfInReg:A = ?> {	// CCAction CCPredicateAction CCIf
  CCAction SubAction = CCIfInReg:A;
  string Predicate = "ArgFlags.isInReg()";
  string NAME = ?;
}
class CCIfNest<CCAction CCIfNest:A = ?> {	// CCAction CCPredicateAction CCIf
  CCAction SubAction = CCIfNest:A;
  string Predicate = "ArgFlags.isNest()";
  string NAME = ?;
}
class CCIfNotVarArg<CCAction CCIfNotVarArg:A = ?> {	// CCAction CCPredicateAction CCIf
  CCAction SubAction = CCIfNotVarArg:A;
  string Predicate = "!State.isVarArg()";
  string NAME = ?;
}
class CCIfSRet<CCAction CCIfSRet:A = ?> {	// CCAction CCPredicateAction CCIf
  CCAction SubAction = CCIfSRet:A;
  string Predicate = "ArgFlags.isSRet()";
  string NAME = ?;
}
class CCIfSplit<CCAction CCIfSplit:A = ?> {	// CCAction CCPredicateAction CCIf
  CCAction SubAction = CCIfSplit:A;
  string Predicate = "ArgFlags.isSplit()";
  string NAME = ?;
}
class CCIfType<list<ValueType> CCIfType:vts = ?, CCAction CCIfType:A = ?> {	// CCAction CCPredicateAction
  CCAction SubAction = CCIfType:A;
  list<ValueType> VTs = CCIfType:vts;
  string NAME = ?;
}
class CCPassByVal<int CCPassByVal:size = ?, int CCPassByVal:align = ?> {	// CCAction
  int Size = CCPassByVal:size;
  int Align = CCPassByVal:align;
  string NAME = ?;
}
class CCPassIndirect<ValueType CCPassIndirect:destTy = ?> {	// CCAction
  ValueType DestTy = CCPassIndirect:destTy;
  string NAME = ?;
}
class CCPredicateAction<CCAction CCPredicateAction:A = ?> {	// CCAction
  CCAction SubAction = CCPredicateAction:A;
  string NAME = ?;
}
class CCPromoteToType<ValueType CCPromoteToType:destTy = ?> {	// CCAction
  ValueType DestTy = CCPromoteToType:destTy;
  string NAME = ?;
}
class CalleeSavedRegs<dag CalleeSavedRegs:saves = ?> {
  dag SaveList = CalleeSavedRegs:saves;
  dag OtherPreserved = ?;
  string NAME = ?;
}
class CallingConv<list<CCAction> CallingConv:actions = ?> {
  list<CCAction> Actions = CallingConv:actions;
  string NAME = ?;
}
class CodePatPred<string CodePatPred:predicate = ?> {	// PatPred
  string PredicateCode = CodePatPred:predicate;
  string NAME = ?;
}
class ComplexDeprecationPredicate<string ComplexDeprecationPredicate:dep = ?> {
  string ComplexDeprecationPredicate = ComplexDeprecationPredicate:dep;
  string NAME = ?;
}
class ComplexPattern<ValueType ComplexPattern:ty = ?, int ComplexPattern:numops = ?, string ComplexPattern:fn = ?, list<SDNode> ComplexPattern:roots = [], list<SDNodeProperty> ComplexPattern:props = []> {
  ValueType Ty = ComplexPattern:ty;
  int NumOperands = ComplexPattern:numops;
  string SelectFunc = ComplexPattern:fn;
  list<SDNode> RootNodes = ComplexPattern:roots;
  list<SDNodeProperty> Properties = ComplexPattern:props;
  string NAME = ?;
}
class ComposedSubRegIndex<SubRegIndex ComposedSubRegIndex:A = ?, SubRegIndex ComposedSubRegIndex:B = ?> {	// SubRegIndex
  string Namespace = "";
  int Size = ComposedSubRegIndex:B.Size;
  int Offset = !if(!eq(ComposedSubRegIndex:A.Offset, -1), -1, !if(!eq(ComposedSubRegIndex:B.Offset, -1), -1, !add(ComposedSubRegIndex:A.Offset, ComposedSubRegIndex:B.Offset)));
  list<SubRegIndex> ComposedOf = [ComposedSubRegIndex:A, ComposedSubRegIndex:B];
  list<SubRegIndex> CoveringSubRegIndices = [];
  string NAME = ?;
}
class CondCode {
  string NAME = ?;
}
class DAGOperand {
  string NAME = ?;
}
class Deprecated<SubtargetFeature Deprecated:dep = ?> {
  SubtargetFeature DeprecatedFeatureMask = Deprecated:dep;
  string NAME = ?;
}
class DwarfRegAlias<Register DwarfRegAlias:reg = ?> {
  Register DwarfAlias = DwarfRegAlias:reg;
  string NAME = ?;
}
class DwarfRegNum<list<int> DwarfRegNum:Numbers = ?> {
  list<int> DwarfNumbers = DwarfRegNum:Numbers;
  string NAME = ?;
}
class FuncUnit {
  string NAME = ?;
}
class GCCBuiltin<string GCCBuiltin:name = ?> {
  string GCCBuiltinName = GCCBuiltin:name;
  string NAME = ?;
}
class ImmLeaf<ValueType ImmLeaf:vt = ?, string ImmLeaf:pred = ?, SDNodeXForm ImmLeaf:xform = NOOP_SDNodeXForm> {	// SDPatternOperator PatFrag
  dag Operands = (ops);
  dag Fragment = (ImmLeaf:vt imm);
  string PredicateCode = "";
  string ImmediateCode = ImmLeaf:pred;
  SDNodeXForm OperandTransform = ImmLeaf:xform;
  bit FastIselShouldIgnore = 0;
  string NAME = ?;
}
class InstAlias<string InstAlias:Asm = ?, dag InstAlias:Result = ?, int InstAlias:Emit = 1> {
  string AsmString = InstAlias:Asm;
  dag ResultInst = InstAlias:Result;
  int EmitPriority = InstAlias:Emit;
  list<Predicate> Predicates = [];
  string NAME = ?;
}
class InstRW<list<SchedReadWrite> InstRW:rw = ?, dag InstRW:instrlist = ?> {
  list<SchedReadWrite> OperandReadWrites = InstRW:rw;
  dag Instrs = InstRW:instrlist;
  SchedMachineModel SchedModel = ?;
  string NAME = ?;
}
class InstrInfo {
  bit isLittleEndianEncoding = 0;
  bit guessInstructionProperties = 1;
  bit decodePositionallyEncodedOperands = 0;
  bit noNamedPositionallyEncodedOperands = 0;
  string NAME = ?;
}
class InstrItinClass {
  string NAME = ?;
}
class InstrItinData<InstrItinClass InstrItinData:Class = ?, list<InstrStage> InstrItinData:stages = ?, list<int> InstrItinData:operandcycles = [], list<Bypass> InstrItinData:bypasses = [], int InstrItinData:uops = 1> {
  InstrItinClass TheClass = InstrItinData:Class;
  int NumMicroOps = InstrItinData:uops;
  list<InstrStage> Stages = InstrItinData:stages;
  list<int> OperandCycles = InstrItinData:operandcycles;
  list<Bypass> Bypasses = InstrItinData:bypasses;
  string NAME = ?;
}
class InstrMapping {
  string FilterClass = ?;
  list<string> RowFields = [];
  list<string> ColFields = [];
  list<string> KeyCol = [];
  list<list<string>> ValueCols = [];
  string NAME = ?;
}
class InstrStage<int InstrStage:cycles = ?, list<FuncUnit> InstrStage:units = ?, int InstrStage:timeinc = -1, ReservationKind InstrStage:kind = Required> {
  int Cycles = InstrStage:cycles;
  list<FuncUnit> Units = InstrStage:units;
  int TimeInc = InstrStage:timeinc;
  int Kind = InstrStage:kind.Value;
  string NAME = ?;
}
class Instruction {
  string Namespace = "";
  dag OutOperandList = ?;
  dag InOperandList = ?;
  string AsmString = "";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 0;
  bit canFoldAsLoad = 0;
  bit mayLoad = ?;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 0;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 0;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 0;
  bit isNotDuplicable = 0;
  bit isAsCheapAsAMove = 0;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 0;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 0;
  bit isCodeGenOnly = 0;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
class Intrinsic<list<LLVMType> Intrinsic:ret_types = ?, list<LLVMType> Intrinsic:param_types = [], list<IntrinsicProperty> Intrinsic:properties = [], string Intrinsic:name = ""> {	// SDPatternOperator
  string LLVMName = Intrinsic:name;
  string TargetPrefix = "";
  list<LLVMType> RetTypes = Intrinsic:ret_types;
  list<LLVMType> ParamTypes = Intrinsic:param_types;
  list<IntrinsicProperty> Properties = Intrinsic:properties;
  bit isTarget = 0;
  string NAME = ?;
}
class IntrinsicProperty {
  string NAME = ?;
}
class ItinRW<list<SchedReadWrite> ItinRW:rw = ?, list<InstrItinClass> ItinRW:iic = ?> {
  list<InstrItinClass> MatchedItinClasses = ItinRW:iic;
  list<SchedReadWrite> OperandReadWrites = ItinRW:rw;
  SchedMachineModel SchedModel = ?;
  string NAME = ?;
}
class LLVMAnyPointerType<LLVMType LLVMAnyPointerType:elty = ?> {	// LLVMType
  ValueType VT = iPTRAny;
  LLVMType ElTy = LLVMAnyPointerType:elty;
  string NAME = ?;
}
class LLVMExtendedType<int LLVMExtendedType:num = ?> {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = LLVMExtendedType:num;
  string NAME = ?;
}
class LLVMHalfElementsVectorType<int LLVMHalfElementsVectorType:num = ?> {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = LLVMHalfElementsVectorType:num;
  string NAME = ?;
}
class LLVMMatchType<int LLVMMatchType:num = ?> {	// LLVMType
  ValueType VT = OtherVT;
  int Number = LLVMMatchType:num;
  string NAME = ?;
}
class LLVMPointerType<LLVMType LLVMPointerType:elty = ?> {	// LLVMType LLVMQualPointerType
  ValueType VT = iPTR;
  LLVMType ElTy = LLVMPointerType:elty;
  int AddrSpace = 0;
  string NAME = ?;
}
class LLVMQualPointerType<LLVMType LLVMQualPointerType:elty = ?, int LLVMQualPointerType:addrspace = ?> {	// LLVMType
  ValueType VT = iPTR;
  LLVMType ElTy = LLVMQualPointerType:elty;
  int AddrSpace = LLVMQualPointerType:addrspace;
  string NAME = ?;
}
class LLVMTruncatedType<int LLVMTruncatedType:num = ?> {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = LLVMTruncatedType:num;
  string NAME = ?;
}
class LLVMType<ValueType LLVMType:vt = ?> {
  ValueType VT = LLVMType:vt;
  string NAME = ?;
}
class MSBuiltin<string MSBuiltin:name = ?> {
  string MSBuiltinName = MSBuiltin:name;
  string NAME = ?;
}
class MnemonicAlias<string MnemonicAlias:From = ?, string MnemonicAlias:To = ?, string MnemonicAlias:VariantName = ""> {
  string FromMnemonic = MnemonicAlias:From;
  string ToMnemonic = MnemonicAlias:To;
  string AsmVariantName = MnemonicAlias:VariantName;
  list<Predicate> Predicates = [];
  string NAME = ?;
}
class NVMRegister<string NVMRegister:n = ?> {	// Register
  field bits<5> Num = { ?, ?, ?, ?, ? };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = NVMRegister:n;
  list<string> AltNames = Register:altNames;
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
class NoCapture<int NoCapture:argNo = ?> {	// IntrinsicProperty
  int ArgNo = NoCapture:argNo;
  string NAME = ?;
}
class Operand<ValueType Operand:ty = ?> {	// DAGOperand
  ValueType Type = Operand:ty;
  string PrintMethod = "printOperand";
  string EncoderMethod = "";
  string DecoderMethod = "";
  string AsmOperandLowerMethod = ?;
  string OperandType = "OPERAND_UNKNOWN";
  dag MIOperandInfo = (ops);
  string MCOperandPredicate = ?;
  AsmOperandClass ParserMatchClass = ImmAsmOperand;
  string NAME = ?;
}
class OperandWithDefaultOps<ValueType OperandWithDefaultOps:ty = ?, dag OperandWithDefaultOps:defaultops = ?> {	// DAGOperand Operand
  ValueType Type = OperandWithDefaultOps:ty;
  string PrintMethod = "printOperand";
  string EncoderMethod = "";
  string DecoderMethod = "";
  string AsmOperandLowerMethod = ?;
  string OperandType = "OPERAND_UNKNOWN";
  dag MIOperandInfo = (ops);
  string MCOperandPredicate = ?;
  AsmOperandClass ParserMatchClass = ImmAsmOperand;
  dag DefaultOps = OperandWithDefaultOps:defaultops;
  string NAME = ?;
}
class OptionalDefOperand<ValueType OptionalDefOperand:ty = ?, dag OptionalDefOperand:OpTypes = ?, dag OptionalDefOperand:defaultops = ?> {	// DAGOperand Operand OperandWithDefaultOps
  ValueType Type = OptionalDefOperand:ty;
  string PrintMethod = "printOperand";
  string EncoderMethod = "";
  string DecoderMethod = "";
  string AsmOperandLowerMethod = ?;
  string OperandType = "OPERAND_UNKNOWN";
  dag MIOperandInfo = OptionalDefOperand:OpTypes;
  string MCOperandPredicate = ?;
  AsmOperandClass ParserMatchClass = ImmAsmOperand;
  dag DefaultOps = OptionalDefOperand:defaultops;
  string NAME = ?;
}
class OutPatFrag<dag OutPatFrag:ops = ?, dag OutPatFrag:frag = ?> {	// SDPatternOperator PatFrag
  dag Operands = OutPatFrag:ops;
  dag Fragment = OutPatFrag:frag;
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
class Pat<dag Pat:pattern = ?, dag Pat:result = ?> {	// Pattern
  dag PatternToMatch = Pat:pattern;
  list<dag> ResultInstrs = [Pat:result];
  list<Predicate> Predicates = [];
  int AddedComplexity = 0;
  string NAME = ?;
}
class PatFrag<dag PatFrag:ops = ?, dag PatFrag:frag = ?, string PatFrag:pred = "", SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm> {	// SDPatternOperator
  dag Operands = PatFrag:ops;
  dag Fragment = PatFrag:frag;
  string PredicateCode = PatFrag:pred;
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = PatFrag:xform;
  string NAME = ?;
}
class PatLeaf<dag PatLeaf:frag = ?, string PatLeaf:pred = "", SDNodeXForm PatLeaf:xform = NOOP_SDNodeXForm> {	// SDPatternOperator PatFrag
  dag Operands = (ops);
  dag Fragment = PatLeaf:frag;
  string PredicateCode = PatLeaf:pred;
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = PatLeaf:xform;
  string NAME = ?;
}
class PatPred {
  string NAME = ?;
}
class Pattern<dag Pattern:patternToMatch = ?, list<dag> Pattern:resultInstrs = ?> {
  dag PatternToMatch = Pattern:patternToMatch;
  list<dag> ResultInstrs = Pattern:resultInstrs;
  list<Predicate> Predicates = [];
  int AddedComplexity = 0;
  string NAME = ?;
}
class PointerLikeRegClass<int PointerLikeRegClass:Kind = ?> {
  int RegClassKind = PointerLikeRegClass:Kind;
  string NAME = ?;
}
class Predicate<string Predicate:cond = ?> {
  string CondString = Predicate:cond;
  bit AssemblerMatcherPredicate = 0;
  string AssemblerCondString = "";
  string PredicateName = "";
  string NAME = ?;
}
class PredicateOp {
  string NAME = ?;
}
class PredicateOperand<ValueType PredicateOperand:ty = ?, dag PredicateOperand:OpTypes = ?, dag PredicateOperand:AlwaysVal = ?> {	// DAGOperand Operand OperandWithDefaultOps PredicateOp
  ValueType Type = PredicateOperand:ty;
  string PrintMethod = "printOperand";
  string EncoderMethod = "";
  string DecoderMethod = "";
  string AsmOperandLowerMethod = ?;
  string OperandType = "OPERAND_UNKNOWN";
  dag MIOperandInfo = PredicateOperand:OpTypes;
  string MCOperandPredicate = ?;
  AsmOperandClass ParserMatchClass = ImmAsmOperand;
  dag DefaultOps = PredicateOperand:AlwaysVal;
  string NAME = ?;
}
class PredicateProlog<string PredicateProlog:c = ?> {
  string Code = PredicateProlog:c;
  string NAME = ?;
}
class ProcReadAdvance<int ProcReadAdvance:cycles = ?, list<SchedWrite> ProcReadAdvance:writes = []> {
  int Cycles = ProcReadAdvance:cycles;
  list<SchedWrite> ValidWrites = ProcReadAdvance:writes;
  bit Unsupported = 0;
  SchedMachineModel SchedModel = ?;
  string NAME = ?;
}
class ProcResGroup<list<ProcResource> ProcResGroup:resources = ?> {	// ProcResourceKind
  list<ProcResource> Resources = ProcResGroup:resources;
  SchedMachineModel SchedModel = ?;
  int BufferSize = -1;
  string NAME = ?;
}
class ProcResource<int ProcResource:num = ?> {	// ProcResourceKind ProcResourceUnits
  ProcResourceKind Kind = EponymousProcResourceKind;
  int NumUnits = ProcResource:num;
  ProcResourceKind Super = ?;
  int BufferSize = -1;
  SchedMachineModel SchedModel = ?;
  string NAME = ?;
}
class ProcResourceKind {
  string NAME = ?;
}
class ProcResourceUnits<ProcResourceKind ProcResourceUnits:kind = ?, int ProcResourceUnits:num = ?> {
  ProcResourceKind Kind = ProcResourceUnits:kind;
  int NumUnits = ProcResourceUnits:num;
  ProcResourceKind Super = ?;
  int BufferSize = -1;
  SchedMachineModel SchedModel = ?;
  string NAME = ?;
}
class ProcWriteResources<list<ProcResourceKind> ProcWriteResources:resources = ?> {
  list<ProcResourceKind> ProcResources = ProcWriteResources:resources;
  list<int> ResourceCycles = [];
  int Latency = 1;
  int NumMicroOps = 1;
  bit BeginGroup = 0;
  bit EndGroup = 0;
  bit Unsupported = 0;
  SchedMachineModel SchedModel = ?;
  string NAME = ?;
}
class Processor<string Processor:n = ?, ProcessorItineraries Processor:pi = ?, list<SubtargetFeature> Processor:f = ?> {
  string Name = Processor:n;
  SchedMachineModel SchedModel = NoSchedModel;
  ProcessorItineraries ProcItin = Processor:pi;
  list<SubtargetFeature> Features = Processor:f;
  string NAME = ?;
}
class ProcessorItineraries<list<FuncUnit> ProcessorItineraries:fu = ?, list<Bypass> ProcessorItineraries:bp = ?, list<InstrItinData> ProcessorItineraries:iid = ?> {
  list<FuncUnit> FU = ProcessorItineraries:fu;
  list<Bypass> BP = ProcessorItineraries:bp;
  list<InstrItinData> IID = ProcessorItineraries:iid;
  string NAME = ?;
}
class ProcessorModel<string ProcessorModel:n = ?, SchedMachineModel ProcessorModel:m = ?, list<SubtargetFeature> ProcessorModel:f = ?> {	// Processor
  string Name = ProcessorModel:n;
  SchedMachineModel SchedModel = ProcessorModel:m;
  ProcessorItineraries ProcItin = NoItineraries;
  list<SubtargetFeature> Features = ProcessorModel:f;
  string NAME = ?;
}
class PseudoInstExpansion<dag PseudoInstExpansion:Result = ?> {
  dag ResultInst = PseudoInstExpansion:Result;
  bit isPseudo = 1;
  string NAME = ?;
}
class Rc<bits<5> Rc:num = { ?, ?, ?, ?, ? }, string Rc:n = ?> {	// Register NVMRegister
  field bits<5> Num = { Rc:num{4}, Rc:num{3}, Rc:num{2}, Rc:num{1}, Rc:num{0} };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = Rc:n;
  list<string> AltNames = Register:altNames;
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
class ReadAdvance<SchedRead ReadAdvance:read = ?, int ReadAdvance:cycles = ?, list<SchedWrite> ReadAdvance:writes = []> {	// ProcReadAdvance
  int Cycles = ReadAdvance:cycles;
  list<SchedWrite> ValidWrites = ReadAdvance:writes;
  bit Unsupported = 0;
  SchedMachineModel SchedModel = ?;
  SchedRead ReadType = ReadAdvance:read;
  string NAME = ?;
}
class ReadNone<int ReadNone:argNo = ?> {	// IntrinsicProperty
  int ArgNo = ReadNone:argNo;
  string NAME = ?;
}
class ReadOnly<int ReadOnly:argNo = ?> {	// IntrinsicProperty
  int ArgNo = ReadOnly:argNo;
  string NAME = ?;
}
class RegAltNameIndex {
  string Namespace = "";
  string NAME = ?;
}
class Register<string Register:n = ?, list<string> Register:altNames = []> {
  string Namespace = "";
  string AsmName = Register:n;
  list<string> AltNames = Register:altNames;
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
class RegisterClass<string RegisterClass:namespace = ?, list<ValueType> RegisterClass:regTypes = ?, int RegisterClass:alignment = ?, dag RegisterClass:regList = ?, RegAltNameIndex RegisterClass:idx = NoRegAltName> {	// DAGOperand
  string Namespace = RegisterClass:namespace;
  list<ValueType> RegTypes = RegisterClass:regTypes;
  int Size = 0;
  int Alignment = RegisterClass:alignment;
  int CopyCost = 1;
  dag MemberList = RegisterClass:regList;
  RegAltNameIndex altNameIndex = RegisterClass:idx;
  bit isAllocatable = 1;
  list<dag> AltOrders = [];
  string AltOrderSelect = "";
  string NAME = ?;
}
class RegisterOperand<RegisterClass RegisterOperand:regclass = ?, string RegisterOperand:pm = "printOperand"> {	// DAGOperand
  RegisterClass RegClass = RegisterOperand:regclass;
  string PrintMethod = RegisterOperand:pm;
  AsmOperandClass ParserMatchClass = ?;
  string NAME = ?;
}
class RegisterTuples<list<SubRegIndex> RegisterTuples:Indices = ?, list<dag> RegisterTuples:Regs = ?> {
  list<dag> SubRegs = RegisterTuples:Regs;
  list<SubRegIndex> SubRegIndices = RegisterTuples:Indices;
  string NAME = ?;
}
class RegisterWithSubRegs<string RegisterWithSubRegs:n = ?, list<Register> RegisterWithSubRegs:subregs = ?> {	// Register
  list<string> Register:altNames = [];
  string Namespace = "";
  string AsmName = RegisterWithSubRegs:n;
  list<string> AltNames = Register:altNames;
  list<Register> Aliases = [];
  list<Register> SubRegs = RegisterWithSubRegs:subregs;
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
class Requires<list<Predicate> Requires:preds = ?> {
  list<Predicate> Predicates = Requires:preds;
  string NAME = ?;
}
class ReservationKind<bits<1> ReservationKind:val = { ? }> {
  int Value = ReservationKind:val;
  string NAME = ?;
}
class Rf<bits<5> Rf:num = { ?, ?, ?, ?, ? }, string Rf:n = ?> {	// Register NVMRegister
  field bits<5> Num = { Rf:num{4}, Rf:num{3}, Rf:num{2}, Rf:num{1}, Rf:num{0} };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = Rf:n;
  list<string> AltNames = Register:altNames;
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
class Ri<bits<5> Ri:num = { ?, ?, ?, ?, ? }, string Ri:n = ?> {	// Register NVMRegister
  field bits<5> Num = { Ri:num{4}, Ri:num{3}, Ri:num{2}, Ri:num{1}, Ri:num{0} };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = Ri:n;
  list<string> AltNames = Register:altNames;
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
class Rj<string Rj:n = ?, Ri Rj:R = ?> {	// Register NVMRegister
  field bits<5> Num = { Rj:R.Num{4}, Rj:R.Num{3}, Rj:R.Num{2}, Rj:R.Num{1}, Rj:R.Num{0} };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = Rj:n;
  list<string> AltNames = Register:altNames;
  list<Register> Aliases = [Rj:R];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
class SDCallSeqEnd<list<SDTypeConstraint> SDCallSeqEnd:constraints = ?> {	// SDTypeProfile
  int NumResults = 0;
  int NumOperands = 2;
  list<SDTypeConstraint> Constraints = SDCallSeqEnd:constraints;
  string NAME = ?;
}
class SDCallSeqStart<list<SDTypeConstraint> SDCallSeqStart:constraints = ?> {	// SDTypeProfile
  int NumResults = 0;
  int NumOperands = 1;
  list<SDTypeConstraint> Constraints = SDCallSeqStart:constraints;
  string NAME = ?;
}
class SDNode<string SDNode:opcode = ?, SDTypeProfile SDNode:typeprof = ?, list<SDNodeProperty> SDNode:props = [], string SDNode:sdclass = "SDNode"> {	// SDPatternOperator
  string Opcode = SDNode:opcode;
  string SDClass = SDNode:sdclass;
  list<SDNodeProperty> Properties = SDNode:props;
  SDTypeProfile TypeProfile = SDNode:typeprof;
  string NAME = ?;
}
class SDNodeProperty {
  string NAME = ?;
}
class SDNodeXForm<SDNode SDNodeXForm:opc = ?, string SDNodeXForm:xformFunction = ?> {
  SDNode Opcode = SDNodeXForm:opc;
  string XFormFunction = SDNodeXForm:xformFunction;
  string NAME = ?;
}
class SDPatternOperator {
  string NAME = ?;
}
class SDTCisEltOfVec<int SDTCisEltOfVec:ThisOp = ?, int SDTCisEltOfVec:OtherOp = ?> {	// SDTypeConstraint
  int OperandNum = SDTCisEltOfVec:ThisOp;
  int OtherOpNum = SDTCisEltOfVec:OtherOp;
  string NAME = ?;
}
class SDTCisFP<int SDTCisFP:OpNum = ?> {	// SDTypeConstraint
  int OperandNum = SDTCisFP:OpNum;
  string NAME = ?;
}
class SDTCisInt<int SDTCisInt:OpNum = ?> {	// SDTypeConstraint
  int OperandNum = SDTCisInt:OpNum;
  string NAME = ?;
}
class SDTCisOpSmallerThanOp<int SDTCisOpSmallerThanOp:SmallOp = ?, int SDTCisOpSmallerThanOp:BigOp = ?> {	// SDTypeConstraint
  int OperandNum = SDTCisOpSmallerThanOp:SmallOp;
  int BigOperandNum = SDTCisOpSmallerThanOp:BigOp;
  string NAME = ?;
}
class SDTCisPtrTy<int SDTCisPtrTy:OpNum = ?> {	// SDTypeConstraint
  int OperandNum = SDTCisPtrTy:OpNum;
  string NAME = ?;
}
class SDTCisSameAs<int SDTCisSameAs:OpNum = ?, int SDTCisSameAs:OtherOp = ?> {	// SDTypeConstraint
  int OperandNum = SDTCisSameAs:OpNum;
  int OtherOperandNum = SDTCisSameAs:OtherOp;
  string NAME = ?;
}
class SDTCisSubVecOfVec<int SDTCisSubVecOfVec:ThisOp = ?, int SDTCisSubVecOfVec:OtherOp = ?> {	// SDTypeConstraint
  int OperandNum = SDTCisSubVecOfVec:ThisOp;
  int OtherOpNum = SDTCisSubVecOfVec:OtherOp;
  string NAME = ?;
}
class SDTCisVT<int SDTCisVT:OpNum = ?, ValueType SDTCisVT:vt = ?> {	// SDTypeConstraint
  int OperandNum = SDTCisVT:OpNum;
  ValueType VT = SDTCisVT:vt;
  string NAME = ?;
}
class SDTCisVTSmallerThanOp<int SDTCisVTSmallerThanOp:OpNum = ?, int SDTCisVTSmallerThanOp:OtherOp = ?> {	// SDTypeConstraint
  int OperandNum = SDTCisVTSmallerThanOp:OpNum;
  int OtherOperandNum = SDTCisVTSmallerThanOp:OtherOp;
  string NAME = ?;
}
class SDTCisVec<int SDTCisVec:OpNum = ?> {	// SDTypeConstraint
  int OperandNum = SDTCisVec:OpNum;
  string NAME = ?;
}
class SDTypeConstraint<int SDTypeConstraint:opnum = ?> {
  int OperandNum = SDTypeConstraint:opnum;
  string NAME = ?;
}
class SDTypeProfile<int SDTypeProfile:numresults = ?, int SDTypeProfile:numoperands = ?, list<SDTypeConstraint> SDTypeProfile:constraints = ?> {
  int NumResults = SDTypeProfile:numresults;
  int NumOperands = SDTypeProfile:numoperands;
  list<SDTypeConstraint> Constraints = SDTypeProfile:constraints;
  string NAME = ?;
}
class Sched<list<SchedReadWrite> Sched:schedrw = ?> {
  list<SchedReadWrite> SchedRW = Sched:schedrw;
  string NAME = ?;
}
class SchedAlias<SchedReadWrite SchedAlias:match = ?, SchedReadWrite SchedAlias:alias = ?> {
  SchedReadWrite MatchRW = SchedAlias:match;
  SchedReadWrite AliasRW = SchedAlias:alias;
  SchedMachineModel SchedModel = ?;
  string NAME = ?;
}
class SchedMachineModel {
  int IssueWidth = -1;
  int MinLatency = -1;
  int MicroOpBufferSize = -1;
  int LoopMicroOpBufferSize = -1;
  int LoadLatency = -1;
  int HighLatency = -1;
  int MispredictPenalty = -1;
  ProcessorItineraries Itineraries = NoItineraries;
  bit PostRAScheduler = 0;
  bit CompleteModel = 1;
  bit NoModel = 0;
  string NAME = ?;
}
class SchedPredicate<string SchedPredicate:pred = ?> {
  SchedMachineModel SchedModel = ?;
  string Predicate = SchedPredicate:pred;
  string NAME = ?;
}
class SchedRead {	// SchedReadWrite
  string NAME = ?;
}
class SchedReadAdvance<int SchedReadAdvance:cycles = ?, list<SchedWrite> SchedReadAdvance:writes = []> {	// SchedReadWrite SchedRead ProcReadAdvance
  int Cycles = SchedReadAdvance:cycles;
  list<SchedWrite> ValidWrites = SchedReadAdvance:writes;
  bit Unsupported = 0;
  SchedMachineModel SchedModel = ?;
  string NAME = ?;
}
class SchedReadVariant<list<SchedVar> SchedReadVariant:variants = ?> {	// SchedReadWrite SchedRead SchedVariant
  list<SchedVar> Variants = SchedReadVariant:variants;
  bit Variadic = 0;
  SchedMachineModel SchedModel = ?;
  string NAME = ?;
}
class SchedReadWrite {
  string NAME = ?;
}
class SchedVar<SchedPredicate SchedVar:pred = ?, list<SchedReadWrite> SchedVar:selected = ?> {
  SchedPredicate Predicate = SchedVar:pred;
  list<SchedReadWrite> Selected = SchedVar:selected;
  string NAME = ?;
}
class SchedVariant<list<SchedVar> SchedVariant:variants = ?> {
  list<SchedVar> Variants = SchedVariant:variants;
  bit Variadic = 0;
  SchedMachineModel SchedModel = ?;
  string NAME = ?;
}
class SchedWrite {	// SchedReadWrite
  string NAME = ?;
}
class SchedWriteRes<list<ProcResourceKind> SchedWriteRes:resources = ?> {	// SchedReadWrite SchedWrite ProcWriteResources
  list<ProcResourceKind> ProcResources = SchedWriteRes:resources;
  list<int> ResourceCycles = [];
  int Latency = 1;
  int NumMicroOps = 1;
  bit BeginGroup = 0;
  bit EndGroup = 0;
  bit Unsupported = 0;
  SchedMachineModel SchedModel = ?;
  string NAME = ?;
}
class SchedWriteVariant<list<SchedVar> SchedWriteVariant:variants = ?> {	// SchedReadWrite SchedWrite SchedVariant
  list<SchedVar> Variants = SchedWriteVariant:variants;
  bit Variadic = 0;
  SchedMachineModel SchedModel = ?;
  string NAME = ?;
}
class SubRegIndex<int SubRegIndex:size = ?, int SubRegIndex:offset = 0> {
  string Namespace = "";
  int Size = SubRegIndex:size;
  int Offset = SubRegIndex:offset;
  list<SubRegIndex> ComposedOf = [];
  list<SubRegIndex> CoveringSubRegIndices = [];
  string NAME = ?;
}
class SubtargetFeature<string SubtargetFeature:n = ?, string SubtargetFeature:a = ?, string SubtargetFeature:v = ?, string SubtargetFeature:d = ?, list<SubtargetFeature> SubtargetFeature:i = []> {
  string Name = SubtargetFeature:n;
  string Attribute = SubtargetFeature:a;
  string Value = SubtargetFeature:v;
  string Desc = SubtargetFeature:d;
  list<SubtargetFeature> Implies = SubtargetFeature:i;
  string NAME = ?;
}
class Target {
  InstrInfo InstructionSet = ?;
  list<AsmParser> AssemblyParsers = [DefaultAsmParser];
  list<AsmParserVariant> AssemblyParserVariants = [DefaultAsmParserVariant];
  list<AsmWriter> AssemblyWriters = [DefaultAsmWriter];
  string NAME = ?;
}
class TokenAlias<string TokenAlias:From = ?, string TokenAlias:To = ?> {
  string FromToken = TokenAlias:From;
  string ToToken = TokenAlias:To;
  string NAME = ?;
}
class ValueType<int ValueType:size = ?, int ValueType:value = ?> {
  string Namespace = "MVT";
  int Size = ValueType:size;
  int Value = ValueType:value;
  string NAME = ?;
}
class WriteRes<SchedWrite WriteRes:write = ?, list<ProcResourceKind> WriteRes:resources = ?> {	// ProcWriteResources
  list<ProcResourceKind> ProcResources = WriteRes:resources;
  list<int> ResourceCycles = [];
  int Latency = 1;
  int NumMicroOps = 1;
  bit BeginGroup = 0;
  bit EndGroup = 0;
  bit Unsupported = 0;
  SchedMachineModel SchedModel = ?;
  SchedWrite WriteType = WriteRes:write;
  string NAME = ?;
}
class WriteSequence<list<SchedWrite> WriteSequence:writes = ?, int WriteSequence:rep = 1> {	// SchedReadWrite SchedWrite
  list<SchedWrite> Writes = WriteSequence:writes;
  int Repeat = WriteSequence:rep;
  SchedMachineModel SchedModel = ?;
  string NAME = ?;
}
class unknown_class {
  string NAME = ?;
}
------------- Defs -----------------
def BUNDLE {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs);
  dag InOperandList = (ins variable_ops);
  string AsmString = "BUNDLE";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 0;
  bit canFoldAsLoad = 0;
  bit mayLoad = ?;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 0;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 0;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 0;
  bit isNotDuplicable = 0;
  bit isAsCheapAsAMove = 0;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 0;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def CFI_INSTRUCTION {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs);
  dag InOperandList = (ins i32imm:$id);
  string AsmString = "";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 0;
  bit canFoldAsLoad = 0;
  bit mayLoad = ?;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 0;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 0;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 1;
  bit isNotDuplicable = 1;
  bit isAsCheapAsAMove = 0;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 0;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def COPY {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs unknown:$dst);
  dag InOperandList = (ins unknown:$src);
  string AsmString = "";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 0;
  bit canFoldAsLoad = 0;
  bit mayLoad = ?;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 0;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 0;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 0;
  bit isNotDuplicable = 0;
  bit isAsCheapAsAMove = 1;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 1;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def COPY_TO_REGCLASS {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs unknown:$dst);
  dag InOperandList = (ins unknown:$src, i32imm:$regclass);
  string AsmString = "";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 0;
  bit canFoldAsLoad = 0;
  bit mayLoad = ?;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 0;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 0;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 0;
  bit isNotDuplicable = 0;
  bit isAsCheapAsAMove = 1;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 1;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def CTRL {	// Register NVMRegister Rc DwarfRegNum
  field bits<5> Num = { 0, 0, 0, 0, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "ctrl";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [65];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def Commutative {	// IntrinsicProperty
  string NAME = ?;
}
def ControlRegs {	// DAGOperand RegisterClass
  RegAltNameIndex RegisterClass:idx = NoRegAltName;
  string Namespace = "Newton";
  list<ValueType> RegTypes = [i32];
  int Size = 32;
  int Alignment = 32;
  int CopyCost = 1;
  dag MemberList = (add CTRL, PC);
  RegAltNameIndex altNameIndex = NoRegAltName;
  bit isAllocatable = 1;
  list<dag> AltOrders = [];
  string AltOrderSelect = "";
  string NAME = ?;
}
def DBG_VALUE {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs);
  dag InOperandList = (ins variable_ops);
  string AsmString = "DBG_VALUE";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 0;
  bit canFoldAsLoad = 0;
  bit mayLoad = ?;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 0;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 0;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 0;
  bit isNotDuplicable = 0;
  bit isAsCheapAsAMove = 0;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 1;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def DefaultAsmParser {	// AsmParser
  string AsmParserClassName = "AsmParser";
  string AsmParserInstCleanup = "";
  bit ShouldEmitMatchRegisterName = 1;
  bit MnemonicContainsDot = 0;
  string NAME = ?;
}
def DefaultAsmParserVariant {	// AsmParserVariant
  int Variant = 0;
  string Name = "";
  string CommentDelimiter = "";
  string RegisterPrefix = "";
  string NAME = ?;
}
def DefaultAsmWriter {	// AsmWriter
  string AsmWriterClassName = "InstPrinter";
  int Variant = 0;
  int OperandSpacing = -1;
  string NAME = ?;
}
def EH_LABEL {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs);
  dag InOperandList = (ins i32imm:$id);
  string AsmString = "";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 0;
  bit canFoldAsLoad = 0;
  bit mayLoad = ?;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 0;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 0;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 1;
  bit isNotDuplicable = 1;
  bit isAsCheapAsAMove = 0;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 0;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def EXTRACT_SUBREG {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs unknown:$dst);
  dag InOperandList = (ins unknown:$supersrc, i32imm:$subidx);
  string AsmString = "";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 0;
  bit canFoldAsLoad = 0;
  bit mayLoad = ?;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 0;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 0;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 0;
  bit isNotDuplicable = 0;
  bit isAsCheapAsAMove = 0;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 1;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def EponymousProcResourceKind {	// ProcResourceKind
  string NAME = ?;
}
def F32 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 0, 0, 0, 0, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f32";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [32];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F33 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 0, 0, 0, 0, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f33";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [33];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F34 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 0, 0, 0, 1, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f34";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [34];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F35 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 0, 0, 0, 1, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f35";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [35];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F36 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 0, 0, 1, 0, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f36";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [36];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F37 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 0, 0, 1, 0, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f37";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [37];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F38 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 0, 0, 1, 1, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f38";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [38];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F39 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 0, 0, 1, 1, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f39";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [39];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F40 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 0, 1, 0, 0, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f40";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [40];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F41 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 0, 1, 0, 0, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f41";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [41];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F42 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 0, 1, 0, 1, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f42";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [42];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F43 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 0, 1, 0, 1, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f43";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [43];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F44 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 0, 1, 1, 0, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f44";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [44];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F45 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 0, 1, 1, 0, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f45";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [45];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F46 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 0, 1, 1, 1, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f46";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [46];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F47 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 0, 1, 1, 1, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f47";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [47];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F48 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 1, 0, 0, 0, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f48";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [48];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F49 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 1, 0, 0, 0, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f49";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [49];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F50 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 1, 0, 0, 1, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f50";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [50];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F51 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 1, 0, 0, 1, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f51";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [51];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F52 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 1, 0, 1, 0, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f52";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [52];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F53 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 1, 0, 1, 0, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f53";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [53];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F54 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 1, 0, 1, 1, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f54";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [54];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F55 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 1, 0, 1, 1, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f55";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [55];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F56 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 1, 1, 0, 0, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f56";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [56];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F57 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 1, 1, 0, 0, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f57";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [57];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F58 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 1, 1, 0, 1, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f58";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [58];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F59 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 1, 1, 0, 1, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f59";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [59];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F60 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 1, 1, 1, 0, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f60";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [60];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F61 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 1, 1, 1, 0, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f61";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [61];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F62 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 1, 1, 1, 1, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f62";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [62];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def F63 {	// Register NVMRegister Rf DwarfRegNum
  field bits<5> Num = { 1, 1, 1, 1, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "f63";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [63];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def FlagVT {	// ValueType
  string Namespace = "MVT";
  int Size = 0;
  int Value = 55;
  string NAME = ?;
}
def GC_LABEL {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs);
  dag InOperandList = (ins i32imm:$id);
  string AsmString = "";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 0;
  bit canFoldAsLoad = 0;
  bit mayLoad = ?;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 0;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 0;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 1;
  bit isNotDuplicable = 1;
  bit isAsCheapAsAMove = 0;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 0;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def IMPLICIT_DEF {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs unknown:$dst);
  dag InOperandList = (ins);
  string AsmString = "";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 0;
  bit canFoldAsLoad = 0;
  bit mayLoad = ?;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 1;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 0;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 0;
  bit isNotDuplicable = 0;
  bit isAsCheapAsAMove = 1;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 1;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def INLINEASM {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs);
  dag InOperandList = (ins variable_ops);
  string AsmString = "";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 0;
  bit canFoldAsLoad = 0;
  bit mayLoad = ?;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 0;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 0;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 0;
  bit isNotDuplicable = 0;
  bit isAsCheapAsAMove = 0;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 1;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def INSERT_SUBREG {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs unknown:$dst);
  dag InOperandList = (ins unknown:$supersrc, unknown:$subsrc, i32imm:$subidx);
  string AsmString = "";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 0;
  bit canFoldAsLoad = 0;
  bit mayLoad = ?;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 0;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 0;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 0;
  bit isNotDuplicable = 0;
  bit isAsCheapAsAMove = 0;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 1;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "$supersrc = $dst";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def ImmAsmOperand {	// AsmOperandClass
  string Name = "Imm";
  list<AsmOperandClass> SuperClasses = [];
  string PredicateMethod = ?;
  string RenderMethod = ?;
  string ParserMethod = ?;
  string DiagnosticType = "";
  string NAME = ?;
}
def IntegerRegs {	// DAGOperand RegisterClass
  RegAltNameIndex RegisterClass:idx = NoRegAltName;
  string Namespace = "Newton";
  list<ValueType> RegTypes = [i32];
  int Size = 64;
  int Alignment = 32;
  int CopyCost = 1;
  dag MemberList = (add (sequence "r%u", 0, 31));
  RegAltNameIndex altNameIndex = NoRegAltName;
  bit isAllocatable = 1;
  list<dag> AltOrders = [];
  string AltOrderSelect = "";
  string NAME = ?;
}
def IntrNoDuplicate {	// IntrinsicProperty
  string NAME = ?;
}
def IntrNoMem {	// IntrinsicProperty
  string NAME = ?;
}
def IntrNoReturn {	// IntrinsicProperty
  string NAME = ?;
}
def IntrReadArgMem {	// IntrinsicProperty
  string NAME = ?;
}
def IntrReadMem {	// IntrinsicProperty
  string NAME = ?;
}
def IntrReadWriteArgMem {	// IntrinsicProperty
  string NAME = ?;
}
def KILL {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs);
  dag InOperandList = (ins variable_ops);
  string AsmString = "";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 0;
  bit canFoldAsLoad = 0;
  bit mayLoad = ?;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 0;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 0;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 0;
  bit isNotDuplicable = 0;
  bit isAsCheapAsAMove = 0;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 1;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def LIFETIME_END {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs);
  dag InOperandList = (ins i32imm:$id);
  string AsmString = "LIFETIME_END";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 0;
  bit canFoldAsLoad = 0;
  bit mayLoad = ?;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 0;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 0;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 0;
  bit isNotDuplicable = 0;
  bit isAsCheapAsAMove = 0;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 1;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def LIFETIME_START {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs);
  dag InOperandList = (ins i32imm:$id);
  string AsmString = "LIFETIME_START";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 0;
  bit canFoldAsLoad = 0;
  bit mayLoad = ?;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 0;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 0;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 0;
  bit isNotDuplicable = 0;
  bit isAsCheapAsAMove = 0;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 1;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def LOAD_STACK_GUARD {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs ptr_rc:$dst);
  dag InOperandList = (ins);
  string AsmString = "";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 0;
  bit canFoldAsLoad = 0;
  bit mayLoad = 1;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 1;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 0;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 0;
  bit isNotDuplicable = 0;
  bit isAsCheapAsAMove = 0;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = 0;
  bit neverHasSideEffects = 0;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def MetadataVT {	// ValueType
  string Namespace = "MVT";
  int Size = 0;
  int Value = 250;
  string NAME = ?;
}
def NOOP_SDNodeXForm {	// SDNodeXForm
  SDNode Opcode = imm;
  string XFormFunction = "";
  string NAME = ?;
}
def NoBypass {	// Bypass
  string NAME = ?;
}
def NoHonorSignDependentRounding {	// Predicate
  string CondString = "!TM.Options.HonorSignDependentRoundingFPMath()";
  bit AssemblerMatcherPredicate = 0;
  string AssemblerCondString = "";
  string PredicateName = "";
  string NAME = ?;
}
def NoItineraries {	// ProcessorItineraries
  list<FuncUnit> FU = [];
  list<Bypass> BP = [];
  list<InstrItinData> IID = [];
  string NAME = ?;
}
def NoItinerary {	// InstrItinClass
  string NAME = ?;
}
def NoReadAdvance {	// SchedReadWrite SchedRead ProcReadAdvance SchedReadAdvance
  list<SchedWrite> SchedReadAdvance:writes = [];
  int Cycles = 0;
  list<SchedWrite> ValidWrites = [];
  bit Unsupported = 0;
  SchedMachineModel SchedModel = ?;
  string NAME = ?;
}
def NoRegAltName {	// RegAltNameIndex
  string Namespace = "";
  string NAME = ?;
}
def NoSchedModel {	// SchedMachineModel
  int IssueWidth = -1;
  int MinLatency = -1;
  int MicroOpBufferSize = -1;
  int LoopMicroOpBufferSize = -1;
  int LoadLatency = -1;
  int HighLatency = -1;
  int MispredictPenalty = -1;
  ProcessorItineraries Itineraries = NoItineraries;
  bit PostRAScheduler = 0;
  bit CompleteModel = 1;
  bit NoModel = 1;
  string NAME = ?;
}
def NoSchedPred {	// SchedPredicate
  SchedMachineModel SchedModel = ?;
  string Predicate = "true";
  string NAME = ?;
}
def NoWrite {	// SchedReadWrite SchedWrite
  string NAME = ?;
}
def OtherVT {	// ValueType
  string Namespace = "MVT";
  int Size = 0;
  int Value = 0;
  string NAME = ?;
}
def PATCHPOINT {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs unknown:$dst);
  dag InOperandList = (ins i64imm:$id, i32imm:$nbytes, unknown:$callee, i32imm:$nargs, i32imm:$cc, variable_ops);
  string AsmString = "";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 1;
  bit canFoldAsLoad = 0;
  bit mayLoad = 1;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 0;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 1;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 0;
  bit isNotDuplicable = 0;
  bit isAsCheapAsAMove = 0;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 0;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def PC {	// Register NVMRegister Rc DwarfRegNum
  field bits<5> Num = { 0, 0, 0, 0, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "pc";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [64];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def PHI {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs);
  dag InOperandList = (ins variable_ops);
  string AsmString = "PHINODE";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 0;
  bit canFoldAsLoad = 0;
  bit mayLoad = ?;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 0;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 0;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 0;
  bit isNotDuplicable = 0;
  bit isAsCheapAsAMove = 0;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 0;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def R0 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 0, 0, 0, 0, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r0";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [0];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R1 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 0, 0, 0, 0, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r1";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [1];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R10 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 0, 1, 0, 1, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r10";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [10];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R11 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 0, 1, 0, 1, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r11";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [11];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R12 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 0, 1, 1, 0, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r12";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [12];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R13 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 0, 1, 1, 0, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r13";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [13];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R14 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 0, 1, 1, 1, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r14";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [14];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R15 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 0, 1, 1, 1, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r15";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [15];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R16 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 1, 0, 0, 0, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r16";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [16];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R17 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 1, 0, 0, 0, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r17";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [17];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R18 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 1, 0, 0, 1, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r18";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [18];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R19 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 1, 0, 0, 1, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r19";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [19];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R2 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 0, 0, 0, 1, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r2";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [2];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R20 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 1, 0, 1, 0, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r20";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [20];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R21 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 1, 0, 1, 0, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r21";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [21];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R22 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 1, 0, 1, 1, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r22";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [22];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R23 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 1, 0, 1, 1, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r23";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [23];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R24 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 1, 1, 0, 0, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r24";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [24];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R25 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 1, 1, 0, 0, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r25";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [25];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R26 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 1, 1, 0, 1, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r26";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [26];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R27 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 1, 1, 0, 1, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r27";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [27];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R28 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 1, 1, 1, 0, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r28";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [28];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R29 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 1, 1, 1, 0, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r29";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [29];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R3 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 0, 0, 0, 1, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r3";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [3];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R30 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 1, 1, 1, 1, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r30";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [30];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R31 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 1, 1, 1, 1, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r31";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [31];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R4 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 0, 0, 1, 0, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r4";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [4];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R5 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 0, 0, 1, 0, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r5";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [5];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R6 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 0, 0, 1, 1, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r6";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [6];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R7 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 0, 0, 1, 1, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r7";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [7];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R8 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 0, 1, 0, 0, 0 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r8";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [8];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def R9 {	// Register NVMRegister Ri DwarfRegNum
  field bits<5> Num = { 0, 1, 0, 0, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "r9";
  list<string> AltNames = [];
  list<Register> Aliases = [];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [9];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def REG_SEQUENCE {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs unknown:$dst);
  dag InOperandList = (ins variable_ops);
  string AsmString = "";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 0;
  bit canFoldAsLoad = 0;
  bit mayLoad = ?;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 0;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 0;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 0;
  bit isNotDuplicable = 0;
  bit isAsCheapAsAMove = 1;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 1;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def ReadDefault {	// SchedReadWrite SchedRead
  string NAME = ?;
}
def Required {	// ReservationKind
  int Value = 0;
  string NAME = ?;
}
def Reserved {	// ReservationKind
  int Value = 1;
  string NAME = ?;
}
def SDNPAssociative {	// SDNodeProperty
  string NAME = ?;
}
def SDNPCommutative {	// SDNodeProperty
  string NAME = ?;
}
def SDNPHasChain {	// SDNodeProperty
  string NAME = ?;
}
def SDNPInGlue {	// SDNodeProperty
  string NAME = ?;
}
def SDNPMayLoad {	// SDNodeProperty
  string NAME = ?;
}
def SDNPMayStore {	// SDNodeProperty
  string NAME = ?;
}
def SDNPMemOperand {	// SDNodeProperty
  string NAME = ?;
}
def SDNPOptInGlue {	// SDNodeProperty
  string NAME = ?;
}
def SDNPOutGlue {	// SDNodeProperty
  string NAME = ?;
}
def SDNPSideEffect {	// SDNodeProperty
  string NAME = ?;
}
def SDNPVariadic {	// SDNodeProperty
  string NAME = ?;
}
def SDNPWantParent {	// SDNodeProperty
  string NAME = ?;
}
def SDNPWantRoot {	// SDNodeProperty
  string NAME = ?;
}
def SDTAtomic2 {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 2;
  list<SDTypeConstraint> Constraints = [anonymous_159, anonymous_160, anonymous_161];
  string NAME = ?;
}
def SDTAtomic3 {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 3;
  list<SDTypeConstraint> Constraints = [anonymous_155, anonymous_156, anonymous_157, anonymous_158];
  string NAME = ?;
}
def SDTAtomicFence {	// SDTypeProfile
  int NumResults = 0;
  int NumOperands = 2;
  list<SDTypeConstraint> Constraints = [anonymous_153, anonymous_154];
  string NAME = ?;
}
def SDTAtomicLoad {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 1;
  list<SDTypeConstraint> Constraints = [anonymous_164, anonymous_165];
  string NAME = ?;
}
def SDTAtomicStore {	// SDTypeProfile
  int NumResults = 0;
  int NumOperands = 2;
  list<SDTypeConstraint> Constraints = [anonymous_162, anonymous_163];
  string NAME = ?;
}
def SDTBr {	// SDTypeProfile
  int NumResults = 0;
  int NumOperands = 1;
  list<SDTypeConstraint> Constraints = [anonymous_123];
  string NAME = ?;
}
def SDTBrcond {	// SDTypeProfile
  int NumResults = 0;
  int NumOperands = 2;
  list<SDTypeConstraint> Constraints = [anonymous_124, anonymous_125];
  string NAME = ?;
}
def SDTBrind {	// SDTypeProfile
  int NumResults = 0;
  int NumOperands = 1;
  list<SDTypeConstraint> Constraints = [anonymous_126];
  string NAME = ?;
}
def SDTConvertOp {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 5;
  list<SDTypeConstraint> Constraints = [anonymous_166, anonymous_167, anonymous_168, anonymous_169];
  string NAME = ?;
}
def SDTExtInreg {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 2;
  list<SDTypeConstraint> Constraints = [anonymous_106, anonymous_107, anonymous_108, anonymous_109];
  string NAME = ?;
}
def SDTFPBinOp {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 2;
  list<SDTypeConstraint> Constraints = [anonymous_76, anonymous_77, anonymous_78];
  string NAME = ?;
}
def SDTFPExtendOp {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 1;
  list<SDTypeConstraint> Constraints = [anonymous_99, anonymous_100, anonymous_101];
  string NAME = ?;
}
def SDTFPLeaf {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 0;
  list<SDTypeConstraint> Constraints = [anonymous_63];
  string NAME = ?;
}
def SDTFPRoundOp {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 1;
  list<SDTypeConstraint> Constraints = [anonymous_96, anonymous_97, anonymous_98];
  string NAME = ?;
}
def SDTFPSignOp {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 2;
  list<SDTypeConstraint> Constraints = [anonymous_79, anonymous_80, anonymous_81];
  string NAME = ?;
}
def SDTFPTernaryOp {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 3;
  list<SDTypeConstraint> Constraints = [anonymous_82, anonymous_83, anonymous_84, anonymous_85];
  string NAME = ?;
}
def SDTFPToIntOp {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 1;
  list<SDTypeConstraint> Constraints = [anonymous_104, anonymous_105];
  string NAME = ?;
}
def SDTFPUnaryOp {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 1;
  list<SDTypeConstraint> Constraints = [anonymous_94, anonymous_95];
  string NAME = ?;
}
def SDTIStore {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 3;
  list<SDTypeConstraint> Constraints = [anonymous_129, anonymous_130, anonymous_131];
  string NAME = ?;
}
def SDTIntBinHiLoOp {	// SDTypeProfile
  int NumResults = 2;
  int NumOperands = 2;
  list<SDTypeConstraint> Constraints = [anonymous_72, anonymous_73, anonymous_74, anonymous_75];
  string NAME = ?;
}
def SDTIntBinOp {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 2;
  list<SDTypeConstraint> Constraints = [anonymous_66, anonymous_67, anonymous_68];
  string NAME = ?;
}
def SDTIntExtendOp {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 1;
  list<SDTypeConstraint> Constraints = [anonymous_88, anonymous_89, anonymous_90];
  string NAME = ?;
}
def SDTIntLeaf {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 0;
  list<SDTypeConstraint> Constraints = [anonymous_62];
  string NAME = ?;
}
def SDTIntShiftOp {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 2;
  list<SDTypeConstraint> Constraints = [anonymous_69, anonymous_70, anonymous_71];
  string NAME = ?;
}
def SDTIntToFPOp {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 1;
  list<SDTypeConstraint> Constraints = [anonymous_102, anonymous_103];
  string NAME = ?;
}
def SDTIntTruncOp {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 1;
  list<SDTypeConstraint> Constraints = [anonymous_91, anonymous_92, anonymous_93];
  string NAME = ?;
}
def SDTIntUnaryOp {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 1;
  list<SDTypeConstraint> Constraints = [anonymous_86, anonymous_87];
  string NAME = ?;
}
def SDTLoad {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 1;
  list<SDTypeConstraint> Constraints = [anonymous_127];
  string NAME = ?;
}
def SDTMemBarrier {	// SDTypeProfile
  int NumResults = 0;
  int NumOperands = 5;
  list<SDTypeConstraint> Constraints = [anonymous_148, anonymous_149, anonymous_150, anonymous_151, anonymous_152];
  string NAME = ?;
}
def SDTNone {	// SDTypeProfile
  int NumResults = 0;
  int NumOperands = 0;
  list<SDTypeConstraint> Constraints = [];
  string NAME = ?;
}
def SDTOther {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 0;
  list<SDTypeConstraint> Constraints = [anonymous_65];
  string NAME = ?;
}
def SDTPrefetch {	// SDTypeProfile
  int NumResults = 0;
  int NumOperands = 4;
  list<SDTypeConstraint> Constraints = [anonymous_144, anonymous_145, anonymous_146, anonymous_147];
  string NAME = ?;
}
def SDTPtrLeaf {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 0;
  list<SDTypeConstraint> Constraints = [anonymous_64];
  string NAME = ?;
}
def SDTSelect {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 3;
  list<SDTypeConstraint> Constraints = [anonymous_113, anonymous_114, anonymous_115];
  string NAME = ?;
}
def SDTSelectCC {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 5;
  list<SDTypeConstraint> Constraints = [anonymous_119, anonymous_120, anonymous_121, anonymous_122];
  string NAME = ?;
}
def SDTSetCC {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 3;
  list<SDTypeConstraint> Constraints = [anonymous_110, anonymous_111, anonymous_112];
  string NAME = ?;
}
def SDTStore {	// SDTypeProfile
  int NumResults = 0;
  int NumOperands = 2;
  list<SDTypeConstraint> Constraints = [anonymous_128];
  string NAME = ?;
}
def SDTSubVecExtract {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 2;
  list<SDTypeConstraint> Constraints = [anonymous_139, anonymous_140];
  string NAME = ?;
}
def SDTSubVecInsert {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 3;
  list<SDTypeConstraint> Constraints = [anonymous_141, anonymous_142, anonymous_143];
  string NAME = ?;
}
def SDTUNDEF {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 0;
  list<SDTypeConstraint> Constraints = [];
  string NAME = ?;
}
def SDTUnaryOp {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 1;
  list<SDTypeConstraint> Constraints = [];
  string NAME = ?;
}
def SDTVSelect {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 3;
  list<SDTypeConstraint> Constraints = [anonymous_116, anonymous_117, anonymous_118];
  string NAME = ?;
}
def SDTVecExtract {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 2;
  list<SDTypeConstraint> Constraints = [anonymous_134, anonymous_135];
  string NAME = ?;
}
def SDTVecInsert {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 3;
  list<SDTypeConstraint> Constraints = [anonymous_136, anonymous_137, anonymous_138];
  string NAME = ?;
}
def SDTVecShuffle {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 2;
  list<SDTypeConstraint> Constraints = [anonymous_132, anonymous_133];
  string NAME = ?;
}
def SDT_assertext {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 1;
  list<SDTypeConstraint> Constraints = [anonymous_190, anonymous_191, anonymous_192];
  string NAME = ?;
}
def SETEQ {	// CondCode
  string NAME = ?;
}
def SETGE {	// CondCode
  string NAME = ?;
}
def SETGT {	// CondCode
  string NAME = ?;
}
def SETLE {	// CondCode
  string NAME = ?;
}
def SETLT {	// CondCode
  string NAME = ?;
}
def SETNE {	// CondCode
  string NAME = ?;
}
def SETO {	// CondCode
  string NAME = ?;
}
def SETOEQ {	// CondCode
  string NAME = ?;
}
def SETOGE {	// CondCode
  string NAME = ?;
}
def SETOGT {	// CondCode
  string NAME = ?;
}
def SETOLE {	// CondCode
  string NAME = ?;
}
def SETOLT {	// CondCode
  string NAME = ?;
}
def SETONE {	// CondCode
  string NAME = ?;
}
def SETUEQ {	// CondCode
  string NAME = ?;
}
def SETUGE {	// CondCode
  string NAME = ?;
}
def SETUGT {	// CondCode
  string NAME = ?;
}
def SETULE {	// CondCode
  string NAME = ?;
}
def SETULT {	// CondCode
  string NAME = ?;
}
def SETUNE {	// CondCode
  string NAME = ?;
}
def SETUO {	// CondCode
  string NAME = ?;
}
def SP {	// Register NVMRegister Rj DwarfRegNum
  field bits<5> Num = { 1, 1, 1, 1, 1 };
  list<string> Register:altNames = [];
  string Namespace = "Newton";
  string AsmName = "sp";
  list<string> AltNames = [];
  list<Register> Aliases = [R31];
  list<Register> SubRegs = [];
  list<SubRegIndex> SubRegIndices = [];
  list<RegAltNameIndex> RegAltNameIndices = [];
  list<int> DwarfNumbers = [31];
  int CostPerUse = 0;
  bit CoveredBySubRegs = 0;
  bits<16> HWEncoding = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string NAME = ?;
}
def STACKMAP {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs);
  dag InOperandList = (ins i64imm:$id, i32imm:$nbytes, variable_ops);
  string AsmString = "";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 1;
  bit canFoldAsLoad = 0;
  bit mayLoad = 1;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 0;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 1;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 0;
  bit isNotDuplicable = 0;
  bit isAsCheapAsAMove = 0;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 0;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def SUBREG_TO_REG {	// Instruction
  string Namespace = "TargetOpcode";
  dag OutOperandList = (outs unknown:$dst);
  dag InOperandList = (ins unknown:$implsrc, unknown:$subsrc, i32imm:$subidx);
  string AsmString = "";
  list<dag> Pattern = ?;
  list<Register> Uses = [];
  list<Register> Defs = [];
  list<Predicate> Predicates = [];
  int Size = 0;
  string DecoderNamespace = "";
  int CodeSize = 0;
  int AddedComplexity = 0;
  bit isReturn = 0;
  bit isBranch = 0;
  bit isIndirectBranch = 0;
  bit isCompare = 0;
  bit isMoveImm = 0;
  bit isBitcast = 0;
  bit isSelect = 0;
  bit isBarrier = 0;
  bit isCall = 0;
  bit canFoldAsLoad = 0;
  bit mayLoad = ?;
  bit mayStore = ?;
  bit isConvertibleToThreeAddress = 0;
  bit isCommutable = 0;
  bit isTerminator = 0;
  bit isReMaterializable = 0;
  bit isPredicable = 0;
  bit hasDelaySlot = 0;
  bit usesCustomInserter = 0;
  bit hasPostISelHook = 0;
  bit hasCtrlDep = 0;
  bit isNotDuplicable = 0;
  bit isAsCheapAsAMove = 0;
  bit hasExtraSrcRegAllocReq = 0;
  bit hasExtraDefRegAllocReq = 0;
  bit isPseudo = 1;
  bit hasSideEffects = ?;
  bit neverHasSideEffects = 1;
  bit isCodeGenOnly = 1;
  bit isAsmParserOnly = 0;
  InstrItinClass Itinerary = NoItinerary;
  list<SchedReadWrite> SchedRW = ?;
  string Constraints = "";
  string DisableEncoding = "";
  string PostEncoderMethod = "";
  string DecoderMethod = "";
  bits<64> TSFlags = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  string AsmMatchConverter = "";
  string TwoOperandAliasConstraint = "";
  bit UseNamedOperandTable = 0;
  string NAME = ?;
}
def Throws {	// IntrinsicProperty
  string NAME = ?;
}
def add {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ADD";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPCommutative, SDNPAssociative];
  SDTypeProfile TypeProfile = SDTIntBinOp;
  string NAME = ?;
}
def addc {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ADDC";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPCommutative, SDNPOutGlue];
  SDTypeProfile TypeProfile = SDTIntBinOp;
  string NAME = ?;
}
def adde {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ADDE";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPCommutative, SDNPOutGlue, SDNPInGlue];
  SDTypeProfile TypeProfile = SDTIntBinOp;
  string NAME = ?;
}
def and {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::AND";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPCommutative, SDNPAssociative];
  SDTypeProfile TypeProfile = SDTIntBinOp;
  string NAME = ?;
}
def anonymous_0 {	// IntrinsicProperty NoCapture
  int ArgNo = 1;
  string NAME = ?;
}
def anonymous_1 {	// IntrinsicProperty NoCapture
  int ArgNo = 2;
  string NAME = ?;
}
def anonymous_10 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_100 {	// SDTypeConstraint SDTCisFP
  int OperandNum = 1;
  string NAME = ?;
}
def anonymous_101 {	// SDTypeConstraint SDTCisOpSmallerThanOp
  int OperandNum = 1;
  int BigOperandNum = 0;
  string NAME = ?;
}
def anonymous_102 {	// SDTypeConstraint SDTCisFP
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_103 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 1;
  string NAME = ?;
}
def anonymous_104 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_105 {	// SDTypeConstraint SDTCisFP
  int OperandNum = 1;
  string NAME = ?;
}
def anonymous_106 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 1;
  string NAME = ?;
}
def anonymous_107 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_108 {	// SDTypeConstraint SDTCisVT
  int OperandNum = 2;
  ValueType VT = OtherVT;
  string NAME = ?;
}
def anonymous_109 {	// SDTypeConstraint SDTCisVTSmallerThanOp
  int OperandNum = 2;
  int OtherOperandNum = 1;
  string NAME = ?;
}
def anonymous_11 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_110 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_111 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 1;
  int OtherOperandNum = 2;
  string NAME = ?;
}
def anonymous_112 {	// SDTypeConstraint SDTCisVT
  int OperandNum = 3;
  ValueType VT = OtherVT;
  string NAME = ?;
}
def anonymous_113 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 1;
  string NAME = ?;
}
def anonymous_114 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 2;
  string NAME = ?;
}
def anonymous_115 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 2;
  int OtherOperandNum = 3;
  string NAME = ?;
}
def anonymous_116 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 1;
  string NAME = ?;
}
def anonymous_117 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 2;
  string NAME = ?;
}
def anonymous_118 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 2;
  int OtherOperandNum = 3;
  string NAME = ?;
}
def anonymous_119 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 1;
  int OtherOperandNum = 2;
  string NAME = ?;
}
def anonymous_12 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_120 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 3;
  int OtherOperandNum = 4;
  string NAME = ?;
}
def anonymous_121 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 3;
  string NAME = ?;
}
def anonymous_122 {	// SDTypeConstraint SDTCisVT
  int OperandNum = 5;
  ValueType VT = OtherVT;
  string NAME = ?;
}
def anonymous_123 {	// SDTypeConstraint SDTCisVT
  int OperandNum = 0;
  ValueType VT = OtherVT;
  string NAME = ?;
}
def anonymous_124 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_125 {	// SDTypeConstraint SDTCisVT
  int OperandNum = 1;
  ValueType VT = OtherVT;
  string NAME = ?;
}
def anonymous_126 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_127 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 1;
  string NAME = ?;
}
def anonymous_128 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 1;
  string NAME = ?;
}
def anonymous_129 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 2;
  string NAME = ?;
}
def anonymous_13 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_130 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_131 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 3;
  string NAME = ?;
}
def anonymous_132 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 1;
  string NAME = ?;
}
def anonymous_133 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 1;
  int OtherOperandNum = 2;
  string NAME = ?;
}
def anonymous_134 {	// SDTypeConstraint SDTCisEltOfVec
  int OperandNum = 0;
  int OtherOpNum = 1;
  string NAME = ?;
}
def anonymous_135 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 2;
  string NAME = ?;
}
def anonymous_136 {	// SDTypeConstraint SDTCisEltOfVec
  int OperandNum = 2;
  int OtherOpNum = 1;
  string NAME = ?;
}
def anonymous_137 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 1;
  string NAME = ?;
}
def anonymous_138 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 3;
  string NAME = ?;
}
def anonymous_139 {	// SDTypeConstraint SDTCisSubVecOfVec
  int OperandNum = 0;
  int OtherOpNum = 1;
  string NAME = ?;
}
def anonymous_14 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_140 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 2;
  string NAME = ?;
}
def anonymous_141 {	// SDTypeConstraint SDTCisSubVecOfVec
  int OperandNum = 2;
  int OtherOpNum = 1;
  string NAME = ?;
}
def anonymous_142 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 1;
  string NAME = ?;
}
def anonymous_143 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 3;
  string NAME = ?;
}
def anonymous_144 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_145 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 1;
  int OtherOperandNum = 2;
  string NAME = ?;
}
def anonymous_146 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 1;
  int OtherOperandNum = 3;
  string NAME = ?;
}
def anonymous_147 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 1;
  string NAME = ?;
}
def anonymous_148 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 1;
  string NAME = ?;
}
def anonymous_149 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 2;
  string NAME = ?;
}
def anonymous_15 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_150 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 3;
  string NAME = ?;
}
def anonymous_151 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 4;
  string NAME = ?;
}
def anonymous_152 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_153 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 1;
  string NAME = ?;
}
def anonymous_154 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_155 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 2;
  string NAME = ?;
}
def anonymous_156 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 3;
  string NAME = ?;
}
def anonymous_157 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_158 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 1;
  string NAME = ?;
}
def anonymous_159 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 2;
  string NAME = ?;
}
def anonymous_16 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_160 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_161 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 1;
  string NAME = ?;
}
def anonymous_162 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_163 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 1;
  string NAME = ?;
}
def anonymous_164 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_165 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 1;
  string NAME = ?;
}
def anonymous_166 {	// SDTypeConstraint SDTCisVT
  int OperandNum = 2;
  ValueType VT = OtherVT;
  string NAME = ?;
}
def anonymous_167 {	// SDTypeConstraint SDTCisVT
  int OperandNum = 3;
  ValueType VT = OtherVT;
  string NAME = ?;
}
def anonymous_168 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 4;
  string NAME = ?;
}
def anonymous_169 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 5;
  string NAME = ?;
}
def anonymous_17 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_170 {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = -1;
  list<SDTypeConstraint> Constraints = [];
  string NAME = ?;
}
def anonymous_171 {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 1;
  list<SDTypeConstraint> Constraints = [];
  string NAME = ?;
}
def anonymous_172 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 2;
  string NAME = ?;
}
def anonymous_173 {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 2;
  list<SDTypeConstraint> Constraints = [anonymous_172];
  string NAME = ?;
}
def anonymous_174 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 1;
  string NAME = ?;
}
def anonymous_175 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 3;
  string NAME = ?;
}
def anonymous_176 {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 3;
  list<SDTypeConstraint> Constraints = [anonymous_174, anonymous_175];
  string NAME = ?;
}
def anonymous_177 {	// SDTypeConstraint SDTCisSubVecOfVec
  int OperandNum = 1;
  int OtherOpNum = 0;
  string NAME = ?;
}
def anonymous_178 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 1;
  int OtherOperandNum = 2;
  string NAME = ?;
}
def anonymous_179 {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 2;
  list<SDTypeConstraint> Constraints = [anonymous_177, anonymous_178];
  string NAME = ?;
}
def anonymous_18 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_180 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 2;
  string NAME = ?;
}
def anonymous_181 {	// SDTypeConstraint SDTCisVec
  int OperandNum = 1;
  string NAME = ?;
}
def anonymous_182 {	// SDTypeConstraint SDTCisVec
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_183 {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = 2;
  list<SDTypeConstraint> Constraints = [anonymous_180, anonymous_181, anonymous_182];
  string NAME = ?;
}
def anonymous_184 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_185 {	// SDTypeProfile
  int NumResults = 0;
  int NumOperands = -1;
  list<SDTypeConstraint> Constraints = [anonymous_184];
  string NAME = ?;
}
def anonymous_186 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 1;
  string NAME = ?;
}
def anonymous_187 {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = -1;
  list<SDTypeConstraint> Constraints = [anonymous_186];
  string NAME = ?;
}
def anonymous_188 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 1;
  string NAME = ?;
}
def anonymous_189 {	// SDTypeProfile
  int NumResults = 1;
  int NumOperands = -1;
  list<SDTypeConstraint> Constraints = [anonymous_188];
  string NAME = ?;
}
def anonymous_19 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_190 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_191 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 1;
  string NAME = ?;
}
def anonymous_192 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 1;
  int OtherOperandNum = 0;
  string NAME = ?;
}
def anonymous_2 {	// IntrinsicProperty NoCapture
  int ArgNo = 0;
  string NAME = ?;
}
def anonymous_20 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_21 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_22 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_23 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_24 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_25 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_26 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_27 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_28 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_29 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_3 {	// IntrinsicProperty NoCapture
  int ArgNo = 0;
  string NAME = ?;
}
def anonymous_30 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_31 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_32 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_33 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_34 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_35 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_36 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_37 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_38 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_39 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_4 {	// IntrinsicProperty NoCapture
  int ArgNo = 1;
  string NAME = ?;
}
def anonymous_40 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_41 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_42 {	// LLVMType LLVMAnyPointerType
  ValueType VT = iPTRAny;
  LLVMType ElTy = llvm_anyint_ty;
  string NAME = ?;
}
def anonymous_43 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_44 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_45 {	// IntrinsicProperty NoCapture
  int ArgNo = 0;
  string NAME = ?;
}
def anonymous_46 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_47 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_48 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_49 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_5 {	// IntrinsicProperty ReadOnly
  int ArgNo = 1;
  string NAME = ?;
}
def anonymous_50 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_51 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_52 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_53 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_54 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_55 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_56 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_57 {	// LLVMType LLVMMatchType
  ValueType VT = OtherVT;
  int Number = 0;
  string NAME = ?;
}
def anonymous_58 {	// IntrinsicProperty NoCapture
  int ArgNo = 1;
  string NAME = ?;
}
def anonymous_59 {	// IntrinsicProperty NoCapture
  int ArgNo = 1;
  string NAME = ?;
}
def anonymous_6 {	// IntrinsicProperty NoCapture
  int ArgNo = 0;
  string NAME = ?;
}
def anonymous_60 {	// IntrinsicProperty NoCapture
  int ArgNo = 1;
  string NAME = ?;
}
def anonymous_61 {	// IntrinsicProperty NoCapture
  int ArgNo = 2;
  string NAME = ?;
}
def anonymous_62 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_63 {	// SDTypeConstraint SDTCisFP
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_64 {	// SDTypeConstraint SDTCisPtrTy
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_65 {	// SDTypeConstraint SDTCisVT
  int OperandNum = 0;
  ValueType VT = OtherVT;
  string NAME = ?;
}
def anonymous_66 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 1;
  string NAME = ?;
}
def anonymous_67 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 2;
  string NAME = ?;
}
def anonymous_68 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_69 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 1;
  string NAME = ?;
}
def anonymous_7 {	// IntrinsicProperty NoCapture
  int ArgNo = 1;
  string NAME = ?;
}
def anonymous_70 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_71 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 2;
  string NAME = ?;
}
def anonymous_72 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 1;
  string NAME = ?;
}
def anonymous_73 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 2;
  string NAME = ?;
}
def anonymous_74 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 3;
  string NAME = ?;
}
def anonymous_75 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_76 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 1;
  string NAME = ?;
}
def anonymous_77 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 2;
  string NAME = ?;
}
def anonymous_78 {	// SDTypeConstraint SDTCisFP
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_79 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 1;
  string NAME = ?;
}
def anonymous_8 {	// IntrinsicProperty ReadOnly
  int ArgNo = 1;
  string NAME = ?;
}
def anonymous_80 {	// SDTypeConstraint SDTCisFP
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_81 {	// SDTypeConstraint SDTCisFP
  int OperandNum = 2;
  string NAME = ?;
}
def anonymous_82 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 1;
  string NAME = ?;
}
def anonymous_83 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 2;
  string NAME = ?;
}
def anonymous_84 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 3;
  string NAME = ?;
}
def anonymous_85 {	// SDTypeConstraint SDTCisFP
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_86 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 1;
  string NAME = ?;
}
def anonymous_87 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_88 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_89 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 1;
  string NAME = ?;
}
def anonymous_9 {	// IntrinsicProperty NoCapture
  int ArgNo = 0;
  string NAME = ?;
}
def anonymous_90 {	// SDTypeConstraint SDTCisOpSmallerThanOp
  int OperandNum = 1;
  int BigOperandNum = 0;
  string NAME = ?;
}
def anonymous_91 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_92 {	// SDTypeConstraint SDTCisInt
  int OperandNum = 1;
  string NAME = ?;
}
def anonymous_93 {	// SDTypeConstraint SDTCisOpSmallerThanOp
  int OperandNum = 0;
  int BigOperandNum = 1;
  string NAME = ?;
}
def anonymous_94 {	// SDTypeConstraint SDTCisSameAs
  int OperandNum = 0;
  int OtherOperandNum = 1;
  string NAME = ?;
}
def anonymous_95 {	// SDTypeConstraint SDTCisFP
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_96 {	// SDTypeConstraint SDTCisFP
  int OperandNum = 0;
  string NAME = ?;
}
def anonymous_97 {	// SDTypeConstraint SDTCisFP
  int OperandNum = 1;
  string NAME = ?;
}
def anonymous_98 {	// SDTypeConstraint SDTCisOpSmallerThanOp
  int OperandNum = 0;
  int BigOperandNum = 1;
  string NAME = ?;
}
def anonymous_99 {	// SDTypeConstraint SDTCisFP
  int OperandNum = 0;
  string NAME = ?;
}
def anyext {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ANY_EXTEND";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntExtendOp;
  string NAME = ?;
}
def assertsext {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::AssertSext";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDT_assertext;
  string NAME = ?;
}
def assertzext {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::AssertZext";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDT_assertext;
  string NAME = ?;
}
def atomic_cmp_swap {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ATOMIC_CMP_SWAP";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPMayStore, SDNPMayLoad, SDNPMemOperand];
  SDTypeProfile TypeProfile = SDTAtomic3;
  string NAME = ?;
}
def atomic_cmp_swap_16 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$cmp, node:$swap);
  dag Fragment = (atomic_cmp_swap node:$ptr, node:$cmp, node:$swap);
  string PredicateCode = "
  return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i16;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def atomic_cmp_swap_32 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$cmp, node:$swap);
  dag Fragment = (atomic_cmp_swap node:$ptr, node:$cmp, node:$swap);
  string PredicateCode = "
  return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i32;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def atomic_cmp_swap_64 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$cmp, node:$swap);
  dag Fragment = (atomic_cmp_swap node:$ptr, node:$cmp, node:$swap);
  string PredicateCode = "
  return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i64;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def atomic_cmp_swap_8 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$cmp, node:$swap);
  dag Fragment = (atomic_cmp_swap node:$ptr, node:$cmp, node:$swap);
  string PredicateCode = "
  return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i8;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def atomic_fence {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ATOMIC_FENCE";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPSideEffect];
  SDTypeProfile TypeProfile = SDTAtomicFence;
  string NAME = ?;
}
def atomic_load {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ATOMIC_LOAD";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPMayLoad, SDNPMemOperand];
  SDTypeProfile TypeProfile = SDTAtomicLoad;
  string NAME = ?;
}
def atomic_load_16 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (atomic_load node:$ptr);
  string PredicateCode = "
  return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i16;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def atomic_load_32 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (atomic_load node:$ptr);
  string PredicateCode = "
  return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i32;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def atomic_load_64 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (atomic_load node:$ptr);
  string PredicateCode = "
  return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i64;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def atomic_load_8 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (atomic_load node:$ptr);
  string PredicateCode = "
  return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i8;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def atomic_load_add {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ATOMIC_LOAD_ADD";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPMayStore, SDNPMayLoad, SDNPMemOperand];
  SDTypeProfile TypeProfile = SDTAtomic2;
  string NAME = ?;
}
def atomic_load_add_16 {	// SDPatternOperator PatFrag _16
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_add node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i16;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_add";
}
def atomic_load_add_32 {	// SDPatternOperator PatFrag _32
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_add node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i32;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_add";
}
def atomic_load_add_64 {	// SDPatternOperator PatFrag _64
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_add node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i64;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_add";
}
def atomic_load_add_8 {	// SDPatternOperator PatFrag _8
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_add node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i8;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_add";
}
def atomic_load_and {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ATOMIC_LOAD_AND";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPMayStore, SDNPMayLoad, SDNPMemOperand];
  SDTypeProfile TypeProfile = SDTAtomic2;
  string NAME = ?;
}
def atomic_load_and_16 {	// SDPatternOperator PatFrag _16
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_and node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i16;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_and";
}
def atomic_load_and_32 {	// SDPatternOperator PatFrag _32
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_and node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i32;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_and";
}
def atomic_load_and_64 {	// SDPatternOperator PatFrag _64
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_and node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i64;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_and";
}
def atomic_load_and_8 {	// SDPatternOperator PatFrag _8
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_and node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i8;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_and";
}
def atomic_load_max {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ATOMIC_LOAD_MAX";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPMayStore, SDNPMayLoad, SDNPMemOperand];
  SDTypeProfile TypeProfile = SDTAtomic2;
  string NAME = ?;
}
def atomic_load_max_16 {	// SDPatternOperator PatFrag _16
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_max node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i16;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_max";
}
def atomic_load_max_32 {	// SDPatternOperator PatFrag _32
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_max node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i32;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_max";
}
def atomic_load_max_64 {	// SDPatternOperator PatFrag _64
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_max node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i64;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_max";
}
def atomic_load_max_8 {	// SDPatternOperator PatFrag _8
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_max node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i8;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_max";
}
def atomic_load_min {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ATOMIC_LOAD_MIN";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPMayStore, SDNPMayLoad, SDNPMemOperand];
  SDTypeProfile TypeProfile = SDTAtomic2;
  string NAME = ?;
}
def atomic_load_min_16 {	// SDPatternOperator PatFrag _16
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_min node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i16;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_min";
}
def atomic_load_min_32 {	// SDPatternOperator PatFrag _32
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_min node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i32;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_min";
}
def atomic_load_min_64 {	// SDPatternOperator PatFrag _64
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_min node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i64;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_min";
}
def atomic_load_min_8 {	// SDPatternOperator PatFrag _8
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_min node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i8;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_min";
}
def atomic_load_nand {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ATOMIC_LOAD_NAND";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPMayStore, SDNPMayLoad, SDNPMemOperand];
  SDTypeProfile TypeProfile = SDTAtomic2;
  string NAME = ?;
}
def atomic_load_nand_16 {	// SDPatternOperator PatFrag _16
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_nand node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i16;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_nand";
}
def atomic_load_nand_32 {	// SDPatternOperator PatFrag _32
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_nand node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i32;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_nand";
}
def atomic_load_nand_64 {	// SDPatternOperator PatFrag _64
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_nand node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i64;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_nand";
}
def atomic_load_nand_8 {	// SDPatternOperator PatFrag _8
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_nand node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i8;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_nand";
}
def atomic_load_or {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ATOMIC_LOAD_OR";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPMayStore, SDNPMayLoad, SDNPMemOperand];
  SDTypeProfile TypeProfile = SDTAtomic2;
  string NAME = ?;
}
def atomic_load_or_16 {	// SDPatternOperator PatFrag _16
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_or node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i16;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_or";
}
def atomic_load_or_32 {	// SDPatternOperator PatFrag _32
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_or node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i32;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_or";
}
def atomic_load_or_64 {	// SDPatternOperator PatFrag _64
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_or node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i64;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_or";
}
def atomic_load_or_8 {	// SDPatternOperator PatFrag _8
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_or node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i8;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_or";
}
def atomic_load_sub {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ATOMIC_LOAD_SUB";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPMayStore, SDNPMayLoad, SDNPMemOperand];
  SDTypeProfile TypeProfile = SDTAtomic2;
  string NAME = ?;
}
def atomic_load_sub_16 {	// SDPatternOperator PatFrag _16
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_sub node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i16;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_sub";
}
def atomic_load_sub_32 {	// SDPatternOperator PatFrag _32
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_sub node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i32;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_sub";
}
def atomic_load_sub_64 {	// SDPatternOperator PatFrag _64
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_sub node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i64;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_sub";
}
def atomic_load_sub_8 {	// SDPatternOperator PatFrag _8
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_sub node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i8;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_sub";
}
def atomic_load_umax {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ATOMIC_LOAD_UMAX";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPMayStore, SDNPMayLoad, SDNPMemOperand];
  SDTypeProfile TypeProfile = SDTAtomic2;
  string NAME = ?;
}
def atomic_load_umax_16 {	// SDPatternOperator PatFrag _16
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_umax node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i16;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_umax";
}
def atomic_load_umax_32 {	// SDPatternOperator PatFrag _32
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_umax node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i32;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_umax";
}
def atomic_load_umax_64 {	// SDPatternOperator PatFrag _64
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_umax node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i64;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_umax";
}
def atomic_load_umax_8 {	// SDPatternOperator PatFrag _8
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_umax node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i8;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_umax";
}
def atomic_load_umin {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ATOMIC_LOAD_UMIN";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPMayStore, SDNPMayLoad, SDNPMemOperand];
  SDTypeProfile TypeProfile = SDTAtomic2;
  string NAME = ?;
}
def atomic_load_umin_16 {	// SDPatternOperator PatFrag _16
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_umin node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i16;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_umin";
}
def atomic_load_umin_32 {	// SDPatternOperator PatFrag _32
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_umin node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i32;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_umin";
}
def atomic_load_umin_64 {	// SDPatternOperator PatFrag _64
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_umin node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i64;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_umin";
}
def atomic_load_umin_8 {	// SDPatternOperator PatFrag _8
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_umin node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i8;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_umin";
}
def atomic_load_xor {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ATOMIC_LOAD_XOR";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPMayStore, SDNPMayLoad, SDNPMemOperand];
  SDTypeProfile TypeProfile = SDTAtomic2;
  string NAME = ?;
}
def atomic_load_xor_16 {	// SDPatternOperator PatFrag _16
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_xor node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i16;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_xor";
}
def atomic_load_xor_32 {	// SDPatternOperator PatFrag _32
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_xor node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i32;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_xor";
}
def atomic_load_xor_64 {	// SDPatternOperator PatFrag _64
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_xor node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i64;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_xor";
}
def atomic_load_xor_8 {	// SDPatternOperator PatFrag _8
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_load_xor node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i8;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_load_xor";
}
def atomic_store {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ATOMIC_STORE";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPMayStore, SDNPMemOperand];
  SDTypeProfile TypeProfile = SDTAtomicStore;
  string NAME = ?;
}
def atomic_store_16 {	// SDPatternOperator PatFrag _16
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_store node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i16;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_store";
}
def atomic_store_32 {	// SDPatternOperator PatFrag _32
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_store node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i32;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_store";
}
def atomic_store_64 {	// SDPatternOperator PatFrag _64
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_store node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i64;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_store";
}
def atomic_store_8 {	// SDPatternOperator PatFrag _8
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_store node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i8;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_store";
}
def atomic_swap {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ATOMIC_SWAP";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPMayStore, SDNPMayLoad, SDNPMemOperand];
  SDTypeProfile TypeProfile = SDTAtomic2;
  string NAME = ?;
}
def atomic_swap_16 {	// SDPatternOperator PatFrag _16
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_swap node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i16;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_swap";
}
def atomic_swap_32 {	// SDPatternOperator PatFrag _32
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_swap node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i32;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_swap";
}
def atomic_swap_64 {	// SDPatternOperator PatFrag _64
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_swap node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i64;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_swap";
}
def atomic_swap_8 {	// SDPatternOperator PatFrag _8
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr, node:$val);
  dag Fragment = (atomic_swap node:$ptr, node:$val);
  string PredicateCode = "
    return cast<AtomicSDNode>(N)->getMemoryVT() == MVT::i8;
  ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = "atomic_swap";
}
def bb {	// SDPatternOperator SDNode
  string Opcode = "ISD::BasicBlock";
  string SDClass = "BasicBlockSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTOther;
  string NAME = ?;
}
def bitconvert {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::BITCAST";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTUnaryOp;
  string NAME = ?;
}
def blockaddress {	// SDPatternOperator SDNode
  string Opcode = "ISD::BlockAddress";
  string SDClass = "BlockAddressSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTPtrLeaf;
  string NAME = ?;
}
def br {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::BR";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain];
  SDTypeProfile TypeProfile = SDTBr;
  string NAME = ?;
}
def brcond {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::BRCOND";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain];
  SDTypeProfile TypeProfile = SDTBrcond;
  string NAME = ?;
}
def brind {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::BRIND";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain];
  SDTypeProfile TypeProfile = SDTBrind;
  string NAME = ?;
}
def bswap {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::BSWAP";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntUnaryOp;
  string NAME = ?;
}
def build_vector {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::BUILD_VECTOR";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = anonymous_170;
  string NAME = ?;
}
def concat_vectors {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::CONCAT_VECTORS";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = anonymous_179;
  string NAME = ?;
}
def cond {	// SDPatternOperator SDNode
  string Opcode = "ISD::CONDCODE";
  string SDClass = "CondCodeSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTOther;
  string NAME = ?;
}
def constpool {	// SDPatternOperator SDNode
  string Opcode = "ISD::ConstantPool";
  string SDClass = "ConstantPoolSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTPtrLeaf;
  string NAME = ?;
}
def ctlz {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::CTLZ";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntUnaryOp;
  string NAME = ?;
}
def ctlz_zero_undef {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::CTLZ_ZERO_UNDEF";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntUnaryOp;
  string NAME = ?;
}
def ctpop {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::CTPOP";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntUnaryOp;
  string NAME = ?;
}
def cttz {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::CTTZ";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntUnaryOp;
  string NAME = ?;
}
def cttz_zero_undef {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::CTTZ_ZERO_UNDEF";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntUnaryOp;
  string NAME = ?;
}
def cvt {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::CONVERT_RNDSAT";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTConvertOp;
  string NAME = ?;
}
def cvtff {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$dty, node:$sty, node:$rd, node:$sat);
  dag Fragment = (cvt node:$val, node:$dty, node:$sty, node:$rd, node:$sat);
  string PredicateCode = "
       return cast<CvtRndSatSDNode>(N)->getCvtCode() == ISD::CVT_FF;
    ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def cvtfs {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$dty, node:$sty, node:$rd, node:$sat);
  dag Fragment = (cvt node:$val, node:$dty, node:$sty, node:$rd, node:$sat);
  string PredicateCode = "
       return cast<CvtRndSatSDNode>(N)->getCvtCode() == ISD::CVT_FS;
    ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def cvtfu {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$dty, node:$sty, node:$rd, node:$sat);
  dag Fragment = (cvt node:$val, node:$dty, node:$sty, node:$rd, node:$sat);
  string PredicateCode = "
       return cast<CvtRndSatSDNode>(N)->getCvtCode() == ISD::CVT_FU;
    ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def cvtsf {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$dty, node:$sty, node:$rd, node:$sat);
  dag Fragment = (cvt node:$val, node:$dty, node:$sty, node:$rd, node:$sat);
  string PredicateCode = "
       return cast<CvtRndSatSDNode>(N)->getCvtCode() == ISD::CVT_SF;
    ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def cvtss {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$dty, node:$sty, node:$rd, node:$sat);
  dag Fragment = (cvt node:$val, node:$dty, node:$sty, node:$rd, node:$sat);
  string PredicateCode = "
       return cast<CvtRndSatSDNode>(N)->getCvtCode() == ISD::CVT_SS;
    ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def cvtsu {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$dty, node:$sty, node:$rd, node:$sat);
  dag Fragment = (cvt node:$val, node:$dty, node:$sty, node:$rd, node:$sat);
  string PredicateCode = "
       return cast<CvtRndSatSDNode>(N)->getCvtCode() == ISD::CVT_SU;
    ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def cvtuf {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$dty, node:$sty, node:$rd, node:$sat);
  dag Fragment = (cvt node:$val, node:$dty, node:$sty, node:$rd, node:$sat);
  string PredicateCode = "
       return cast<CvtRndSatSDNode>(N)->getCvtCode() == ISD::CVT_UF;
    ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def cvtus {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$dty, node:$sty, node:$rd, node:$sat);
  dag Fragment = (cvt node:$val, node:$dty, node:$sty, node:$rd, node:$sat);
  string PredicateCode = "
       return cast<CvtRndSatSDNode>(N)->getCvtCode() == ISD::CVT_US;
    ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def cvtuu {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$dty, node:$sty, node:$rd, node:$sat);
  dag Fragment = (cvt node:$val, node:$dty, node:$sty, node:$rd, node:$sat);
  string PredicateCode = "
       return cast<CvtRndSatSDNode>(N)->getCvtCode() == ISD::CVT_UU;
    ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def debugtrap {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::DEBUGTRAP";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPSideEffect];
  SDTypeProfile TypeProfile = SDTNone;
  string NAME = ?;
}
def decimate {
  string NAME = ?;
}
def externalsym {	// SDPatternOperator SDNode
  string Opcode = "ISD::ExternalSymbol";
  string SDClass = "ExternalSymbolSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTPtrLeaf;
  string NAME = ?;
}
def extload {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (unindexedload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getExtensionType() == ISD::EXTLOAD;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def extloadf32 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (extload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT() == MVT::f32;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def extloadf64 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (extload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT() == MVT::f64;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def extloadi1 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (extload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT() == MVT::i1;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def extloadi16 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (extload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT() == MVT::i16;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def extloadi32 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (extload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT() == MVT::i32;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def extloadi8 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (extload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT() == MVT::i8;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def extloadvf32 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (extload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT().getScalarType() == MVT::f32;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def extloadvf64 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (extload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT().getScalarType() == MVT::f64;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def extloadvi1 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (extload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT().getScalarType() == MVT::i1;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def extloadvi16 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (extload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT().getScalarType() == MVT::i16;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def extloadvi32 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (extload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT().getScalarType() == MVT::i32;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def extloadvi8 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (extload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT().getScalarType() == MVT::i8;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def extract_subvector {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::EXTRACT_SUBVECTOR";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTSubVecExtract;
  string NAME = ?;
}
def extractelt {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::EXTRACT_VECTOR_ELT";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTVecExtract;
  string NAME = ?;
}
def f128 {	// ValueType
  string Namespace = "MVT";
  int Size = 128;
  int Value = 11;
  string NAME = ?;
}
def f16 {	// ValueType
  string Namespace = "MVT";
  int Size = 16;
  int Value = 7;
  string NAME = ?;
}
def f16_to_fp {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FP16_TO_FP";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntToFPOp;
  string NAME = ?;
}
def f32 {	// ValueType
  string Namespace = "MVT";
  int Size = 32;
  int Value = 8;
  string NAME = ?;
}
def f32imm {	// DAGOperand Operand
  ValueType Type = f32;
  string PrintMethod = "printOperand";
  string EncoderMethod = "";
  string DecoderMethod = "";
  string AsmOperandLowerMethod = ?;
  string OperandType = "OPERAND_IMMEDIATE";
  dag MIOperandInfo = (ops);
  string MCOperandPredicate = ?;
  AsmOperandClass ParserMatchClass = ImmAsmOperand;
  string NAME = ?;
}
def f64 {	// ValueType
  string Namespace = "MVT";
  int Size = 64;
  int Value = 9;
  string NAME = ?;
}
def f64imm {	// DAGOperand Operand
  ValueType Type = f64;
  string PrintMethod = "printOperand";
  string EncoderMethod = "";
  string DecoderMethod = "";
  string AsmOperandLowerMethod = ?;
  string OperandType = "OPERAND_IMMEDIATE";
  dag MIOperandInfo = (ops);
  string MCOperandPredicate = ?;
  AsmOperandClass ParserMatchClass = ImmAsmOperand;
  string NAME = ?;
}
def f80 {	// ValueType
  string Namespace = "MVT";
  int Size = 80;
  int Value = 10;
  string NAME = ?;
}
def fAny {	// ValueType
  string Namespace = "MVT";
  int Size = 0;
  int Value = 253;
  string NAME = ?;
}
def fabs {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FABS";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPUnaryOp;
  string NAME = ?;
}
def fadd {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FADD";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPCommutative];
  SDTypeProfile TypeProfile = SDTFPBinOp;
  string NAME = ?;
}
def fceil {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FCEIL";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPUnaryOp;
  string NAME = ?;
}
def fcopysign {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FCOPYSIGN";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPSignOp;
  string NAME = ?;
}
def fcos {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FCOS";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPUnaryOp;
  string NAME = ?;
}
def fdiv {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FDIV";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPBinOp;
  string NAME = ?;
}
def fexp2 {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FEXP2";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPUnaryOp;
  string NAME = ?;
}
def fextend {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FP_EXTEND";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPExtendOp;
  string NAME = ?;
}
def ffloor {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FFLOOR";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPUnaryOp;
  string NAME = ?;
}
def fgetsign {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FGETSIGN";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPToIntOp;
  string NAME = ?;
}
def floatRegs {	// DAGOperand RegisterClass
  RegAltNameIndex RegisterClass:idx = NoRegAltName;
  string Namespace = "Newton";
  list<ValueType> RegTypes = [f64];
  int Size = 64;
  int Alignment = 64;
  int CopyCost = 1;
  dag MemberList = (add (sequence "f%u", 0, 31));
  RegAltNameIndex altNameIndex = NoRegAltName;
  bit isAllocatable = 1;
  list<dag> AltOrders = [];
  string AltOrderSelect = "";
  string NAME = ?;
}
def flog2 {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FLOG2";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPUnaryOp;
  string NAME = ?;
}
def fma {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FMA";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPTernaryOp;
  string NAME = ?;
}
def fmul {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FMUL";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPCommutative];
  SDTypeProfile TypeProfile = SDTFPBinOp;
  string NAME = ?;
}
def fnearbyint {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FNEARBYINT";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPUnaryOp;
  string NAME = ?;
}
def fneg {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FNEG";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPUnaryOp;
  string NAME = ?;
}
def fp_to_f16 {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FP_TO_FP16";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPToIntOp;
  string NAME = ?;
}
def fp_to_sint {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FP_TO_SINT";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPToIntOp;
  string NAME = ?;
}
def fp_to_uint {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FP_TO_UINT";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPToIntOp;
  string NAME = ?;
}
def fpimm {	// SDPatternOperator SDNode
  string Opcode = "ISD::ConstantFP";
  string SDClass = "ConstantFPSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPLeaf;
  string NAME = ?;
}
def fpow {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FPOW";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPBinOp;
  string NAME = ?;
}
def frameindex {	// SDPatternOperator SDNode
  string Opcode = "ISD::FrameIndex";
  string SDClass = "FrameIndexSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTPtrLeaf;
  string NAME = ?;
}
def frem {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FREM";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPBinOp;
  string NAME = ?;
}
def frint {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FRINT";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPUnaryOp;
  string NAME = ?;
}
def frnd {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FROUND";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPUnaryOp;
  string NAME = ?;
}
def fround {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FP_ROUND";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPRoundOp;
  string NAME = ?;
}
def fsin {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FSIN";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPUnaryOp;
  string NAME = ?;
}
def fsqrt {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FSQRT";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPUnaryOp;
  string NAME = ?;
}
def fsub {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FSUB";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPBinOp;
  string NAME = ?;
}
def ftrunc {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::FTRUNC";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTFPUnaryOp;
  string NAME = ?;
}
def globaladdr {	// SDPatternOperator SDNode
  string Opcode = "ISD::GlobalAddress";
  string SDClass = "GlobalAddressSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTPtrLeaf;
  string NAME = ?;
}
def globaltlsaddr {	// SDPatternOperator SDNode
  string Opcode = "ISD::GlobalTLSAddress";
  string SDClass = "GlobalAddressSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTPtrLeaf;
  string NAME = ?;
}
def i1 {	// ValueType
  string Namespace = "MVT";
  int Size = 1;
  int Value = 1;
  string NAME = ?;
}
def i128 {	// ValueType
  string Namespace = "MVT";
  int Size = 128;
  int Value = 6;
  string NAME = ?;
}
def i16 {	// ValueType
  string Namespace = "MVT";
  int Size = 16;
  int Value = 3;
  string NAME = ?;
}
def i16imm {	// DAGOperand Operand
  ValueType Type = i16;
  string PrintMethod = "printOperand";
  string EncoderMethod = "";
  string DecoderMethod = "";
  string AsmOperandLowerMethod = ?;
  string OperandType = "OPERAND_IMMEDIATE";
  dag MIOperandInfo = (ops);
  string MCOperandPredicate = ?;
  AsmOperandClass ParserMatchClass = ImmAsmOperand;
  string NAME = ?;
}
def i1imm {	// DAGOperand Operand
  ValueType Type = i1;
  string PrintMethod = "printOperand";
  string EncoderMethod = "";
  string DecoderMethod = "";
  string AsmOperandLowerMethod = ?;
  string OperandType = "OPERAND_IMMEDIATE";
  dag MIOperandInfo = (ops);
  string MCOperandPredicate = ?;
  AsmOperandClass ParserMatchClass = ImmAsmOperand;
  string NAME = ?;
}
def i32 {	// ValueType
  string Namespace = "MVT";
  int Size = 32;
  int Value = 4;
  string NAME = ?;
}
def i32imm {	// DAGOperand Operand
  ValueType Type = i32;
  string PrintMethod = "printOperand";
  string EncoderMethod = "";
  string DecoderMethod = "";
  string AsmOperandLowerMethod = ?;
  string OperandType = "OPERAND_IMMEDIATE";
  dag MIOperandInfo = (ops);
  string MCOperandPredicate = ?;
  AsmOperandClass ParserMatchClass = ImmAsmOperand;
  string NAME = ?;
}
def i64 {	// ValueType
  string Namespace = "MVT";
  int Size = 64;
  int Value = 5;
  string NAME = ?;
}
def i64imm {	// DAGOperand Operand
  ValueType Type = i64;
  string PrintMethod = "printOperand";
  string EncoderMethod = "";
  string DecoderMethod = "";
  string AsmOperandLowerMethod = ?;
  string OperandType = "OPERAND_IMMEDIATE";
  dag MIOperandInfo = (ops);
  string MCOperandPredicate = ?;
  AsmOperandClass ParserMatchClass = ImmAsmOperand;
  string NAME = ?;
}
def i8 {	// ValueType
  string Namespace = "MVT";
  int Size = 8;
  int Value = 2;
  string NAME = ?;
}
def i8imm {	// DAGOperand Operand
  ValueType Type = i8;
  string PrintMethod = "printOperand";
  string EncoderMethod = "";
  string DecoderMethod = "";
  string AsmOperandLowerMethod = ?;
  string OperandType = "OPERAND_IMMEDIATE";
  dag MIOperandInfo = (ops);
  string MCOperandPredicate = ?;
  AsmOperandClass ParserMatchClass = ImmAsmOperand;
  string NAME = ?;
}
def iAny {	// ValueType
  string Namespace = "MVT";
  int Size = 0;
  int Value = 254;
  string NAME = ?;
}
def iPTR {	// ValueType
  string Namespace = "MVT";
  int Size = 0;
  int Value = 255;
  string NAME = ?;
}
def iPTRAny {	// ValueType
  string Namespace = "MVT";
  int Size = 0;
  int Value = 251;
  string NAME = ?;
}
def imm {	// SDPatternOperator SDNode
  string Opcode = "ISD::Constant";
  string SDClass = "ConstantSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntLeaf;
  string NAME = ?;
}
def immAllOnesV {	// SDPatternOperator PatFrag PatLeaf
  SDNodeXForm PatLeaf:xform = NOOP_SDNodeXForm;
  dag Operands = (ops);
  dag Fragment = (build_vector);
  string PredicateCode = "
  return ISD::isBuildVectorAllOnes(N);
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def immAllZerosV {	// SDPatternOperator PatFrag PatLeaf
  SDNodeXForm PatLeaf:xform = NOOP_SDNodeXForm;
  dag Operands = (ops);
  dag Fragment = (build_vector);
  string PredicateCode = "
  return ISD::isBuildVectorAllZeros(N);
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def implicit {
  string NAME = ?;
}
def ineg {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$in);
  dag Fragment = (sub 0, node:$in);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def ins {
  string NAME = ?;
}
def insert_subvector {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::INSERT_SUBVECTOR";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTSubVecInsert;
  string NAME = ?;
}
def insertelt {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::INSERT_VECTOR_ELT";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTVecInsert;
  string NAME = ?;
}
def instregex {
  string NAME = ?;
}
def instrs {
  string NAME = ?;
}
def int_adjust_trampoline {	// SDPatternOperator Intrinsic GCCBuiltin
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_ptr_ty];
  list<LLVMType> ParamTypes = [llvm_ptr_ty];
  list<IntrinsicProperty> Properties = [IntrReadArgMem];
  bit isTarget = 0;
  string GCCBuiltinName = "__builtin_adjust_trampoline";
  string NAME = ?;
}
def int_annotation {	// SDPatternOperator Intrinsic
  string LLVMName = "llvm.annotation";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty];
  list<LLVMType> ParamTypes = [anonymous_44, llvm_ptr_ty, llvm_ptr_ty, llvm_i32_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_assume {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_i1_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_bswap {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty];
  list<LLVMType> ParamTypes = [anonymous_38];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_ceil {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_31];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_clear_cache {	// SDPatternOperator Intrinsic
  string LLVMName = "llvm.clear_cache";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_ptr_ty, llvm_ptr_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_convert_from_fp16 {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [llvm_i16_ty];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_convert_to_fp16 {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_i16_ty];
  list<LLVMType> ParamTypes = [llvm_anyfloat_ty];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_convertff {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [llvm_anyfloat_ty, llvm_i32_ty, llvm_i32_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_convertfsi {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [llvm_anyint_ty, llvm_i32_ty, llvm_i32_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_convertfui {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [llvm_anyint_ty, llvm_i32_ty, llvm_i32_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_convertsif {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty];
  list<LLVMType> ParamTypes = [llvm_anyfloat_ty, llvm_i32_ty, llvm_i32_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_convertss {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty];
  list<LLVMType> ParamTypes = [llvm_anyint_ty, llvm_i32_ty, llvm_i32_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_convertsu {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty];
  list<LLVMType> ParamTypes = [llvm_anyint_ty, llvm_i32_ty, llvm_i32_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_convertuif {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty];
  list<LLVMType> ParamTypes = [llvm_anyfloat_ty, llvm_i32_ty, llvm_i32_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_convertus {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty];
  list<LLVMType> ParamTypes = [llvm_anyint_ty, llvm_i32_ty, llvm_i32_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_convertuu {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty];
  list<LLVMType> ParamTypes = [llvm_anyint_ty, llvm_i32_ty, llvm_i32_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_copysign {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_28, anonymous_29];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_cos {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_19];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_ctlz {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty];
  list<LLVMType> ParamTypes = [anonymous_40, llvm_i1_ty];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_ctpop {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty];
  list<LLVMType> ParamTypes = [anonymous_39];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_cttz {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty];
  list<LLVMType> ParamTypes = [anonymous_41, llvm_i1_ty];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_dbg_declare {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_metadata_ty, llvm_metadata_ty];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_dbg_value {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_metadata_ty, llvm_i64_ty, llvm_metadata_ty];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_debugtrap {	// SDPatternOperator Intrinsic GCCBuiltin
  list<LLVMType> Intrinsic:param_types = [];
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string GCCBuiltinName = "__builtin_debugtrap";
  string NAME = ?;
}
def int_donothing {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_eh_dwarf_cfa {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_ptr_ty];
  list<LLVMType> ParamTypes = [llvm_i32_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_eh_return_i32 {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_i32_ty, llvm_ptr_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_eh_return_i64 {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_i64_ty, llvm_ptr_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_eh_sjlj_callsite {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_i32_ty];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_eh_sjlj_functioncontext {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_ptr_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_eh_sjlj_longjmp {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_ptr_ty];
  list<IntrinsicProperty> Properties = [IntrNoReturn];
  bit isTarget = 0;
  string NAME = ?;
}
def int_eh_sjlj_lsda {	// SDPatternOperator Intrinsic
  list<LLVMType> Intrinsic:param_types = [];
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_ptr_ty];
  list<LLVMType> ParamTypes = [];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_eh_sjlj_setjmp {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_i32_ty];
  list<LLVMType> ParamTypes = [llvm_ptr_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_eh_typeid_for {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_i32_ty];
  list<LLVMType> ParamTypes = [llvm_ptr_ty];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_eh_unwind_init {	// SDPatternOperator Intrinsic GCCBuiltin
  list<LLVMType> Intrinsic:param_types = [];
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string GCCBuiltinName = "__builtin_unwind_init";
  string NAME = ?;
}
def int_exp {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_25];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_exp2 {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_26];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_expect {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty];
  list<LLVMType> ParamTypes = [anonymous_36, anonymous_37];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_experimental_patchpoint_i64 {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_i64_ty];
  list<LLVMType> ParamTypes = [llvm_i64_ty, llvm_i32_ty, llvm_ptr_ty, llvm_i32_ty, llvm_vararg_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_experimental_patchpoint_void {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_i64_ty, llvm_i32_ty, llvm_ptr_ty, llvm_i32_ty, llvm_vararg_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_experimental_stackmap {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_i64_ty, llvm_i32_ty, llvm_vararg_ty];
  list<IntrinsicProperty> Properties = [Throws];
  bit isTarget = 0;
  string NAME = ?;
}
def int_fabs {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_27];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_floor {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_30];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_flt_rounds {	// SDPatternOperator Intrinsic GCCBuiltin
  list<LLVMType> Intrinsic:param_types = [];
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_i32_ty];
  list<LLVMType> ParamTypes = [];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string GCCBuiltinName = "__builtin_flt_rounds";
  string NAME = ?;
}
def int_fma {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_10, anonymous_11, anonymous_12];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_fmuladd {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_13, anonymous_14, anonymous_15];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_frameaddress {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_ptr_ty];
  list<LLVMType> ParamTypes = [llvm_i32_ty];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_gcread {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_ptr_ty];
  list<LLVMType> ParamTypes = [llvm_ptr_ty, llvm_ptrptr_ty];
  list<IntrinsicProperty> Properties = [IntrReadArgMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_gcroot {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_ptrptr_ty, llvm_ptr_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_gcwrite {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_ptr_ty, llvm_ptr_ty, llvm_ptrptr_ty];
  list<IntrinsicProperty> Properties = [IntrReadWriteArgMem, anonymous_0, anonymous_1];
  bit isTarget = 0;
  string NAME = ?;
}
def int_init_trampoline {	// SDPatternOperator Intrinsic GCCBuiltin
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_ptr_ty, llvm_ptr_ty, llvm_ptr_ty];
  list<IntrinsicProperty> Properties = [IntrReadWriteArgMem, anonymous_45];
  bit isTarget = 0;
  string GCCBuiltinName = "__builtin_init_trampoline";
  string NAME = ?;
}
def int_invariant_end {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_descriptor_ty, llvm_i64_ty, llvm_ptr_ty];
  list<IntrinsicProperty> Properties = [IntrReadWriteArgMem, anonymous_61];
  bit isTarget = 0;
  string NAME = ?;
}
def int_invariant_start {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_descriptor_ty];
  list<LLVMType> ParamTypes = [llvm_i64_ty, llvm_ptr_ty];
  list<IntrinsicProperty> Properties = [IntrReadWriteArgMem, anonymous_60];
  bit isTarget = 0;
  string NAME = ?;
}
def int_lifetime_end {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_i64_ty, llvm_ptr_ty];
  list<IntrinsicProperty> Properties = [IntrReadWriteArgMem, anonymous_59];
  bit isTarget = 0;
  string NAME = ?;
}
def int_lifetime_start {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_i64_ty, llvm_ptr_ty];
  list<IntrinsicProperty> Properties = [IntrReadWriteArgMem, anonymous_58];
  bit isTarget = 0;
  string NAME = ?;
}
def int_log {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_22];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_log10 {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_23];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_log2 {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_24];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_longjmp {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_ptr_ty, llvm_i32_ty];
  list<IntrinsicProperty> Properties = [IntrNoReturn];
  bit isTarget = 0;
  string NAME = ?;
}
def int_memcpy {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_anyptr_ty, llvm_anyptr_ty, llvm_anyint_ty, llvm_i32_ty, llvm_i1_ty];
  list<IntrinsicProperty> Properties = [IntrReadWriteArgMem, anonymous_3, anonymous_4, anonymous_5];
  bit isTarget = 0;
  string NAME = ?;
}
def int_memmove {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_anyptr_ty, llvm_anyptr_ty, llvm_anyint_ty, llvm_i32_ty, llvm_i1_ty];
  list<IntrinsicProperty> Properties = [IntrReadWriteArgMem, anonymous_6, anonymous_7, anonymous_8];
  bit isTarget = 0;
  string NAME = ?;
}
def int_memset {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_anyptr_ty, llvm_i8_ty, llvm_anyint_ty, llvm_i32_ty, llvm_i1_ty];
  list<IntrinsicProperty> Properties = [IntrReadWriteArgMem, anonymous_9];
  bit isTarget = 0;
  string NAME = ?;
}
def int_nearbyint {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_34];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_objectsize {	// SDPatternOperator Intrinsic GCCBuiltin
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty];
  list<LLVMType> ParamTypes = [llvm_anyptr_ty, llvm_i1_ty];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string GCCBuiltinName = "__builtin_object_size";
  string NAME = ?;
}
def int_pcmarker {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_i32_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_pow {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_20, anonymous_21];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_powi {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_17, llvm_i32_ty];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_prefetch {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_ptr_ty, llvm_i32_ty, llvm_i32_ty, llvm_i32_ty];
  list<IntrinsicProperty> Properties = [IntrReadWriteArgMem, anonymous_2];
  bit isTarget = 0;
  string NAME = ?;
}
def int_ptr_annotation {	// SDPatternOperator Intrinsic
  string LLVMName = "llvm.ptr.annotation";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [anonymous_42];
  list<LLVMType> ParamTypes = [anonymous_43, llvm_ptr_ty, llvm_ptr_ty, llvm_i32_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_read_register {	// SDPatternOperator Intrinsic
  string LLVMName = "llvm.read_register";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty];
  list<LLVMType> ParamTypes = [llvm_metadata_ty];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_readcyclecounter {	// SDPatternOperator Intrinsic
  list<LLVMType> Intrinsic:param_types = [];
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_i64_ty];
  list<LLVMType> ParamTypes = [];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_returnaddress {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_ptr_ty];
  list<LLVMType> ParamTypes = [llvm_i32_ty];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_rint {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_33];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_round {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_35];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_sadd_with_overflow {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty, llvm_i1_ty];
  list<LLVMType> ParamTypes = [anonymous_46, anonymous_47];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_setjmp {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_i32_ty];
  list<LLVMType> ParamTypes = [llvm_ptr_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_siglongjmp {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_ptr_ty, llvm_i32_ty];
  list<IntrinsicProperty> Properties = [IntrNoReturn];
  bit isTarget = 0;
  string NAME = ?;
}
def int_sigsetjmp {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_i32_ty];
  list<LLVMType> ParamTypes = [llvm_ptr_ty, llvm_i32_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_sin {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_18];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_smul_with_overflow {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty, llvm_i1_ty];
  list<LLVMType> ParamTypes = [anonymous_54, anonymous_55];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_sqrt {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_16];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_ssub_with_overflow {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty, llvm_i1_ty];
  list<LLVMType> ParamTypes = [anonymous_50, anonymous_51];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_stackprotector {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_ptr_ty, llvm_ptrptr_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_stackprotectorcheck {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_ptrptr_ty];
  list<IntrinsicProperty> Properties = [IntrReadWriteArgMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_stackrestore {	// SDPatternOperator Intrinsic GCCBuiltin
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_ptr_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string GCCBuiltinName = "__builtin_stack_restore";
  string NAME = ?;
}
def int_stacksave {	// SDPatternOperator Intrinsic GCCBuiltin
  list<LLVMType> Intrinsic:param_types = [];
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_ptr_ty];
  list<LLVMType> ParamTypes = [];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string GCCBuiltinName = "__builtin_stack_save";
  string NAME = ?;
}
def int_trap {	// SDPatternOperator Intrinsic GCCBuiltin
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [];
  list<IntrinsicProperty> Properties = [IntrNoReturn];
  bit isTarget = 0;
  string GCCBuiltinName = "__builtin_trap";
  string NAME = ?;
}
def int_trunc {	// SDPatternOperator Intrinsic
  list<IntrinsicProperty> Intrinsic:properties = [];
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyfloat_ty];
  list<LLVMType> ParamTypes = [anonymous_32];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_uadd_with_overflow {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty, llvm_i1_ty];
  list<LLVMType> ParamTypes = [anonymous_48, anonymous_49];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_umul_with_overflow {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty, llvm_i1_ty];
  list<LLVMType> ParamTypes = [anonymous_56, anonymous_57];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_usub_with_overflow {	// SDPatternOperator Intrinsic
  string Intrinsic:name = "";
  string LLVMName = "";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [llvm_anyint_ty, llvm_i1_ty];
  list<LLVMType> ParamTypes = [anonymous_52, anonymous_53];
  list<IntrinsicProperty> Properties = [IntrNoMem];
  bit isTarget = 0;
  string NAME = ?;
}
def int_vacopy {	// SDPatternOperator Intrinsic
  string LLVMName = "llvm.va_copy";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_ptr_ty, llvm_ptr_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_vaend {	// SDPatternOperator Intrinsic
  string LLVMName = "llvm.va_end";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_ptr_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_var_annotation {	// SDPatternOperator Intrinsic
  string LLVMName = "llvm.var.annotation";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_ptr_ty, llvm_ptr_ty, llvm_ptr_ty, llvm_i32_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_vastart {	// SDPatternOperator Intrinsic
  string LLVMName = "llvm.va_start";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_ptr_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def int_write_register {	// SDPatternOperator Intrinsic
  string LLVMName = "llvm.write_register";
  string TargetPrefix = "";
  list<LLVMType> RetTypes = [];
  list<LLVMType> ParamTypes = [llvm_metadata_ty, llvm_anyint_ty];
  list<IntrinsicProperty> Properties = [];
  bit isTarget = 0;
  string NAME = ?;
}
def interleave {
  string NAME = ?;
}
def intrinsic_void {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::INTRINSIC_VOID";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain];
  SDTypeProfile TypeProfile = anonymous_185;
  string NAME = ?;
}
def intrinsic_w_chain {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::INTRINSIC_W_CHAIN";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain];
  SDTypeProfile TypeProfile = anonymous_187;
  string NAME = ?;
}
def intrinsic_wo_chain {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::INTRINSIC_WO_CHAIN";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = anonymous_189;
  string NAME = ?;
}
def isVoid {	// ValueType
  string Namespace = "MVT";
  int Size = 0;
  int Value = 56;
  string NAME = ?;
}
def ist {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::STORE";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPMayStore, SDNPMemOperand];
  SDTypeProfile TypeProfile = SDTIStore;
  string NAME = ?;
}
def istore {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$base, node:$offset);
  dag Fragment = (ist node:$val, node:$base, node:$offset);
  string PredicateCode = "
  return !cast<StoreSDNode>(N)->isTruncatingStore();
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def itruncstore {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$base, node:$offset);
  dag Fragment = (ist node:$val, node:$base, node:$offset);
  string PredicateCode = "
  return cast<StoreSDNode>(N)->isTruncatingStore();
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def jumptable {	// SDPatternOperator SDNode
  string Opcode = "ISD::JumpTable";
  string SDClass = "JumpTableSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTPtrLeaf;
  string NAME = ?;
}
def ld {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::LOAD";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPMayLoad, SDNPMemOperand];
  SDTypeProfile TypeProfile = SDTLoad;
  string NAME = ?;
}
def llvm_anyfloat_ty {	// LLVMType
  ValueType VT = fAny;
  string NAME = ?;
}
def llvm_anyint_ty {	// LLVMType
  ValueType VT = iAny;
  string NAME = ?;
}
def llvm_anyptr_ty {	// LLVMType LLVMAnyPointerType
  ValueType VT = iPTRAny;
  LLVMType ElTy = llvm_i8_ty;
  string NAME = ?;
}
def llvm_anyvector_ty {	// LLVMType
  ValueType VT = vAny;
  string NAME = ?;
}
def llvm_descriptor_ty {	// LLVMType LLVMQualPointerType LLVMPointerType
  ValueType VT = iPTR;
  LLVMType ElTy = llvm_empty_ty;
  int AddrSpace = 0;
  string NAME = ?;
}
def llvm_double_ty {	// LLVMType
  ValueType VT = f64;
  string NAME = ?;
}
def llvm_empty_ty {	// LLVMType
  ValueType VT = OtherVT;
  string NAME = ?;
}
def llvm_f128_ty {	// LLVMType
  ValueType VT = f128;
  string NAME = ?;
}
def llvm_f80_ty {	// LLVMType
  ValueType VT = f80;
  string NAME = ?;
}
def llvm_float_ty {	// LLVMType
  ValueType VT = f32;
  string NAME = ?;
}
def llvm_half_ty {	// LLVMType
  ValueType VT = f16;
  string NAME = ?;
}
def llvm_i16_ty {	// LLVMType
  ValueType VT = i16;
  string NAME = ?;
}
def llvm_i1_ty {	// LLVMType
  ValueType VT = i1;
  string NAME = ?;
}
def llvm_i32_ty {	// LLVMType
  ValueType VT = i32;
  string NAME = ?;
}
def llvm_i64_ty {	// LLVMType
  ValueType VT = i64;
  string NAME = ?;
}
def llvm_i8_ty {	// LLVMType
  ValueType VT = i8;
  string NAME = ?;
}
def llvm_metadata_ty {	// LLVMType
  ValueType VT = MetadataVT;
  string NAME = ?;
}
def llvm_ppcf128_ty {	// LLVMType
  ValueType VT = ppcf128;
  string NAME = ?;
}
def llvm_ptr_ty {	// LLVMType LLVMQualPointerType LLVMPointerType
  ValueType VT = iPTR;
  LLVMType ElTy = llvm_i8_ty;
  int AddrSpace = 0;
  string NAME = ?;
}
def llvm_ptrptr_ty {	// LLVMType LLVMQualPointerType LLVMPointerType
  ValueType VT = iPTR;
  LLVMType ElTy = llvm_ptr_ty;
  int AddrSpace = 0;
  string NAME = ?;
}
def llvm_ptrx86mmx_ty {	// LLVMType LLVMQualPointerType LLVMPointerType
  ValueType VT = iPTR;
  LLVMType ElTy = llvm_x86mmx_ty;
  int AddrSpace = 0;
  string NAME = ?;
}
def llvm_v16f32_ty {	// LLVMType
  ValueType VT = v16f32;
  string NAME = ?;
}
def llvm_v16i16_ty {	// LLVMType
  ValueType VT = v16i16;
  string NAME = ?;
}
def llvm_v16i1_ty {	// LLVMType
  ValueType VT = v16i1;
  string NAME = ?;
}
def llvm_v16i32_ty {	// LLVMType
  ValueType VT = v16i32;
  string NAME = ?;
}
def llvm_v16i64_ty {	// LLVMType
  ValueType VT = v16i64;
  string NAME = ?;
}
def llvm_v16i8_ty {	// LLVMType
  ValueType VT = v16i8;
  string NAME = ?;
}
def llvm_v1f32_ty {	// LLVMType
  ValueType VT = v1f32;
  string NAME = ?;
}
def llvm_v1f64_ty {	// LLVMType
  ValueType VT = v1f64;
  string NAME = ?;
}
def llvm_v1i16_ty {	// LLVMType
  ValueType VT = v1i16;
  string NAME = ?;
}
def llvm_v1i32_ty {	// LLVMType
  ValueType VT = v1i32;
  string NAME = ?;
}
def llvm_v1i64_ty {	// LLVMType
  ValueType VT = v1i64;
  string NAME = ?;
}
def llvm_v1i8_ty {	// LLVMType
  ValueType VT = v1i8;
  string NAME = ?;
}
def llvm_v2f16_ty {	// LLVMType
  ValueType VT = v2f16;
  string NAME = ?;
}
def llvm_v2f32_ty {	// LLVMType
  ValueType VT = v2f32;
  string NAME = ?;
}
def llvm_v2f64_ty {	// LLVMType
  ValueType VT = v2f64;
  string NAME = ?;
}
def llvm_v2i16_ty {	// LLVMType
  ValueType VT = v2i16;
  string NAME = ?;
}
def llvm_v2i1_ty {	// LLVMType
  ValueType VT = v2i1;
  string NAME = ?;
}
def llvm_v2i32_ty {	// LLVMType
  ValueType VT = v2i32;
  string NAME = ?;
}
def llvm_v2i64_ty {	// LLVMType
  ValueType VT = v2i64;
  string NAME = ?;
}
def llvm_v2i8_ty {	// LLVMType
  ValueType VT = v2i8;
  string NAME = ?;
}
def llvm_v32i16_ty {	// LLVMType
  ValueType VT = v32i16;
  string NAME = ?;
}
def llvm_v32i1_ty {	// LLVMType
  ValueType VT = v32i1;
  string NAME = ?;
}
def llvm_v32i8_ty {	// LLVMType
  ValueType VT = v32i8;
  string NAME = ?;
}
def llvm_v4f16_ty {	// LLVMType
  ValueType VT = v4f16;
  string NAME = ?;
}
def llvm_v4f32_ty {	// LLVMType
  ValueType VT = v4f32;
  string NAME = ?;
}
def llvm_v4f64_ty {	// LLVMType
  ValueType VT = v4f64;
  string NAME = ?;
}
def llvm_v4i16_ty {	// LLVMType
  ValueType VT = v4i16;
  string NAME = ?;
}
def llvm_v4i1_ty {	// LLVMType
  ValueType VT = v4i1;
  string NAME = ?;
}
def llvm_v4i32_ty {	// LLVMType
  ValueType VT = v4i32;
  string NAME = ?;
}
def llvm_v4i64_ty {	// LLVMType
  ValueType VT = v4i64;
  string NAME = ?;
}
def llvm_v4i8_ty {	// LLVMType
  ValueType VT = v4i8;
  string NAME = ?;
}
def llvm_v64i1_ty {	// LLVMType
  ValueType VT = v64i1;
  string NAME = ?;
}
def llvm_v64i8_ty {	// LLVMType
  ValueType VT = v64i8;
  string NAME = ?;
}
def llvm_v8f16_ty {	// LLVMType
  ValueType VT = v8f16;
  string NAME = ?;
}
def llvm_v8f32_ty {	// LLVMType
  ValueType VT = v8f32;
  string NAME = ?;
}
def llvm_v8f64_ty {	// LLVMType
  ValueType VT = v8f64;
  string NAME = ?;
}
def llvm_v8i16_ty {	// LLVMType
  ValueType VT = v8i16;
  string NAME = ?;
}
def llvm_v8i1_ty {	// LLVMType
  ValueType VT = v8i1;
  string NAME = ?;
}
def llvm_v8i32_ty {	// LLVMType
  ValueType VT = v8i32;
  string NAME = ?;
}
def llvm_v8i64_ty {	// LLVMType
  ValueType VT = v8i64;
  string NAME = ?;
}
def llvm_v8i8_ty {	// LLVMType
  ValueType VT = v8i8;
  string NAME = ?;
}
def llvm_vararg_ty {	// LLVMType
  ValueType VT = isVoid;
  string NAME = ?;
}
def llvm_void_ty {	// LLVMType
  ValueType VT = isVoid;
  string NAME = ?;
}
def llvm_x86mmx_ty {	// LLVMType
  ValueType VT = x86mmx;
  string NAME = ?;
}
def load {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (unindexedload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getExtensionType() == ISD::NON_EXTLOAD;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def mul {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::MUL";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPCommutative, SDNPAssociative];
  SDTypeProfile TypeProfile = SDTIntBinOp;
  string NAME = ?;
}
def mulhs {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::MULHS";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPCommutative];
  SDTypeProfile TypeProfile = SDTIntBinOp;
  string NAME = ?;
}
def mulhu {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::MULHU";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPCommutative];
  SDTypeProfile TypeProfile = SDTIntBinOp;
  string NAME = ?;
}
def node {
  string NAME = ?;
}
def not {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$in);
  dag Fragment = (xor node:$in, -1);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def null_frag {	// SDPatternOperator
  string NAME = ?;
}
def ops {
  string NAME = ?;
}
def or {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::OR";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPCommutative, SDNPAssociative];
  SDTypeProfile TypeProfile = SDTIntBinOp;
  string NAME = ?;
}
def outs {
  string NAME = ?;
}
def post_store {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$ptr, node:$offset);
  dag Fragment = (istore node:$val, node:$ptr, node:$offset);
  string PredicateCode = "
  ISD::MemIndexedMode AM = cast<StoreSDNode>(N)->getAddressingMode();
  return AM == ISD::POST_INC || AM == ISD::POST_DEC;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def post_truncst {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$base, node:$offset);
  dag Fragment = (itruncstore node:$val, node:$base, node:$offset);
  string PredicateCode = "
  ISD::MemIndexedMode AM = cast<StoreSDNode>(N)->getAddressingMode();
  return AM == ISD::POST_INC || AM == ISD::POST_DEC;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def post_truncstf32 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$base, node:$offset);
  dag Fragment = (post_truncst node:$val, node:$base, node:$offset);
  string PredicateCode = "
  return cast<StoreSDNode>(N)->getMemoryVT() == MVT::f32;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def post_truncsti1 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$base, node:$offset);
  dag Fragment = (post_truncst node:$val, node:$base, node:$offset);
  string PredicateCode = "
  return cast<StoreSDNode>(N)->getMemoryVT() == MVT::i1;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def post_truncsti16 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$base, node:$offset);
  dag Fragment = (post_truncst node:$val, node:$base, node:$offset);
  string PredicateCode = "
  return cast<StoreSDNode>(N)->getMemoryVT() == MVT::i16;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def post_truncsti32 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$base, node:$offset);
  dag Fragment = (post_truncst node:$val, node:$base, node:$offset);
  string PredicateCode = "
  return cast<StoreSDNode>(N)->getMemoryVT() == MVT::i32;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def post_truncsti8 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$base, node:$offset);
  dag Fragment = (post_truncst node:$val, node:$base, node:$offset);
  string PredicateCode = "
  return cast<StoreSDNode>(N)->getMemoryVT() == MVT::i8;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def ppcf128 {	// ValueType
  string Namespace = "MVT";
  int Size = 128;
  int Value = 12;
  string NAME = ?;
}
def pre_store {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$base, node:$offset);
  dag Fragment = (istore node:$val, node:$base, node:$offset);
  string PredicateCode = "
  ISD::MemIndexedMode AM = cast<StoreSDNode>(N)->getAddressingMode();
  return AM == ISD::PRE_INC || AM == ISD::PRE_DEC;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def pre_truncst {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$base, node:$offset);
  dag Fragment = (itruncstore node:$val, node:$base, node:$offset);
  string PredicateCode = "
  ISD::MemIndexedMode AM = cast<StoreSDNode>(N)->getAddressingMode();
  return AM == ISD::PRE_INC || AM == ISD::PRE_DEC;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def pre_truncstf32 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$base, node:$offset);
  dag Fragment = (pre_truncst node:$val, node:$base, node:$offset);
  string PredicateCode = "
  return cast<StoreSDNode>(N)->getMemoryVT() == MVT::f32;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def pre_truncsti1 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$base, node:$offset);
  dag Fragment = (pre_truncst node:$val, node:$base, node:$offset);
  string PredicateCode = "
  return cast<StoreSDNode>(N)->getMemoryVT() == MVT::i1;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def pre_truncsti16 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$base, node:$offset);
  dag Fragment = (pre_truncst node:$val, node:$base, node:$offset);
  string PredicateCode = "
  return cast<StoreSDNode>(N)->getMemoryVT() == MVT::i16;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def pre_truncsti32 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$base, node:$offset);
  dag Fragment = (pre_truncst node:$val, node:$base, node:$offset);
  string PredicateCode = "
  return cast<StoreSDNode>(N)->getMemoryVT() == MVT::i32;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def pre_truncsti8 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$base, node:$offset);
  dag Fragment = (pre_truncst node:$val, node:$base, node:$offset);
  string PredicateCode = "
  return cast<StoreSDNode>(N)->getMemoryVT() == MVT::i8;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def prefetch {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::PREFETCH";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPMayLoad, SDNPMayStore, SDNPMemOperand];
  SDTypeProfile TypeProfile = SDTPrefetch;
  string NAME = ?;
}
def ptr_rc {	// PointerLikeRegClass
  int RegClassKind = 0;
  string NAME = ?;
}
def readcyclecounter {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::READCYCLECOUNTER";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPSideEffect];
  SDTypeProfile TypeProfile = SDTIntLeaf;
  string NAME = ?;
}
def rotl {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ROTL";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntShiftOp;
  string NAME = ?;
}
def rotr {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ROTR";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntShiftOp;
  string NAME = ?;
}
def scalar_to_vector {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::SCALAR_TO_VECTOR";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = anonymous_171;
  string NAME = ?;
}
def sdiv {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::SDIV";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntBinOp;
  string NAME = ?;
}
def sdivrem {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::SDIVREM";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntBinHiLoOp;
  string NAME = ?;
}
def select {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::SELECT";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTSelect;
  string NAME = ?;
}
def selectcc {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::SELECT_CC";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTSelectCC;
  string NAME = ?;
}
def sequence {
  string NAME = ?;
}
def set {
  string NAME = ?;
}
def setcc {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::SETCC";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTSetCC;
  string NAME = ?;
}
def seteq {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETEQ);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def setge {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETGE);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def setgt {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETGT);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def setle {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETLE);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def setlt {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETLT);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def setne {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETNE);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def seto {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETO);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def setoeq {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETOEQ);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def setoge {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETOGE);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def setogt {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETOGT);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def setole {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETOLE);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def setolt {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETOLT);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def setone {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETONE);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def setueq {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETUEQ);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def setuge {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETUGE);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def setugt {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETUGT);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def setule {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETULE);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def setult {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETULT);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def setune {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETUNE);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def setuo {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$lhs, node:$rhs);
  dag Fragment = (setcc node:$lhs, node:$rhs, SETUO);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def sext {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::SIGN_EXTEND";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntExtendOp;
  string NAME = ?;
}
def sext_inreg {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::SIGN_EXTEND_INREG";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTExtInreg;
  string NAME = ?;
}
def sextload {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (unindexedload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getExtensionType() == ISD::SEXTLOAD;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def sextloadi1 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (sextload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT() == MVT::i1;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def sextloadi16 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (sextload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT() == MVT::i16;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def sextloadi32 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (sextload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT() == MVT::i32;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def sextloadi8 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (sextload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT() == MVT::i8;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def sextloadvi1 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (sextload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT().getScalarType() == MVT::i1;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def sextloadvi16 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (sextload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT().getScalarType() == MVT::i16;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def sextloadvi32 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (sextload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT().getScalarType() == MVT::i32;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def sextloadvi8 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (sextload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT().getScalarType() == MVT::i8;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def shl {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::SHL";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntShiftOp;
  string NAME = ?;
}
def sint_to_fp {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::SINT_TO_FP";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntToFPOp;
  string NAME = ?;
}
def smullohi {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::SMUL_LOHI";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPCommutative];
  SDTypeProfile TypeProfile = SDTIntBinHiLoOp;
  string NAME = ?;
}
def sra {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::SRA";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntShiftOp;
  string NAME = ?;
}
def srcvalue {
  string NAME = ?;
}
def srem {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::SREM";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntBinOp;
  string NAME = ?;
}
def srl {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::SRL";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntShiftOp;
  string NAME = ?;
}
def st {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::STORE";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPMayStore, SDNPMemOperand];
  SDTypeProfile TypeProfile = SDTStore;
  string NAME = ?;
}
def store {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$ptr);
  dag Fragment = (unindexedstore node:$val, node:$ptr);
  string PredicateCode = "
  return !cast<StoreSDNode>(N)->isTruncatingStore();
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def sub {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::SUB";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntBinOp;
  string NAME = ?;
}
def subc {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::SUBC";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPOutGlue];
  SDTypeProfile TypeProfile = SDTIntBinOp;
  string NAME = ?;
}
def sube {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::SUBE";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPOutGlue, SDNPInGlue];
  SDTypeProfile TypeProfile = SDTIntBinOp;
  string NAME = ?;
}
def tblockaddress {	// SDPatternOperator SDNode
  string Opcode = "ISD::TargetBlockAddress";
  string SDClass = "BlockAddressSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTPtrLeaf;
  string NAME = ?;
}
def tconstpool {	// SDPatternOperator SDNode
  string Opcode = "ISD::TargetConstantPool";
  string SDClass = "ConstantPoolSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTPtrLeaf;
  string NAME = ?;
}
def texternalsym {	// SDPatternOperator SDNode
  string Opcode = "ISD::TargetExternalSymbol";
  string SDClass = "ExternalSymbolSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTPtrLeaf;
  string NAME = ?;
}
def tframeindex {	// SDPatternOperator SDNode
  string Opcode = "ISD::TargetFrameIndex";
  string SDClass = "FrameIndexSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTPtrLeaf;
  string NAME = ?;
}
def tglobaladdr {	// SDPatternOperator SDNode
  string Opcode = "ISD::TargetGlobalAddress";
  string SDClass = "GlobalAddressSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTPtrLeaf;
  string NAME = ?;
}
def tglobaltlsaddr {	// SDPatternOperator SDNode
  string Opcode = "ISD::TargetGlobalTLSAddress";
  string SDClass = "GlobalAddressSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTPtrLeaf;
  string NAME = ?;
}
def timm {	// SDPatternOperator SDNode
  string Opcode = "ISD::TargetConstant";
  string SDClass = "ConstantSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntLeaf;
  string NAME = ?;
}
def tjumptable {	// SDPatternOperator SDNode
  string Opcode = "ISD::TargetJumpTable";
  string SDClass = "JumpTableSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTPtrLeaf;
  string NAME = ?;
}
def trap {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::TRAP";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPHasChain, SDNPSideEffect];
  SDTypeProfile TypeProfile = SDTNone;
  string NAME = ?;
}
def trunc {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::TRUNCATE";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntTruncOp;
  string NAME = ?;
}
def truncstore {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$ptr);
  dag Fragment = (unindexedstore node:$val, node:$ptr);
  string PredicateCode = "
  return cast<StoreSDNode>(N)->isTruncatingStore();
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def truncstoref32 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$ptr);
  dag Fragment = (truncstore node:$val, node:$ptr);
  string PredicateCode = "
  return cast<StoreSDNode>(N)->getMemoryVT() == MVT::f32;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def truncstoref64 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$ptr);
  dag Fragment = (truncstore node:$val, node:$ptr);
  string PredicateCode = "
  return cast<StoreSDNode>(N)->getMemoryVT() == MVT::f64;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def truncstorei16 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$ptr);
  dag Fragment = (truncstore node:$val, node:$ptr);
  string PredicateCode = "
  return cast<StoreSDNode>(N)->getMemoryVT() == MVT::i16;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def truncstorei32 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$ptr);
  dag Fragment = (truncstore node:$val, node:$ptr);
  string PredicateCode = "
  return cast<StoreSDNode>(N)->getMemoryVT() == MVT::i32;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def truncstorei8 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$ptr);
  dag Fragment = (truncstore node:$val, node:$ptr);
  string PredicateCode = "
  return cast<StoreSDNode>(N)->getMemoryVT() == MVT::i8;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def udiv {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::UDIV";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntBinOp;
  string NAME = ?;
}
def udivrem {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::UDIVREM";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntBinHiLoOp;
  string NAME = ?;
}
def uint_to_fp {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::UINT_TO_FP";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntToFPOp;
  string NAME = ?;
}
def umullohi {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::UMUL_LOHI";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPCommutative];
  SDTypeProfile TypeProfile = SDTIntBinHiLoOp;
  string NAME = ?;
}
def undef {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::UNDEF";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTUNDEF;
  string NAME = ?;
}
def unindexedload {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (ld node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getAddressingMode() == ISD::UNINDEXED;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def unindexedstore {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$val, node:$ptr);
  dag Fragment = (st node:$val, node:$ptr);
  string PredicateCode = "
  return cast<StoreSDNode>(N)->getAddressingMode() == ISD::UNINDEXED;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def unknown {	// unknown_class
  string NAME = ?;
}
def untyped {	// ValueType
  string Namespace = "MVT";
  int Size = 8;
  int Value = 57;
  string NAME = ?;
}
def urem {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::UREM";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntBinOp;
  string NAME = ?;
}
def v16f32 {	// ValueType
  string Namespace = "MVT";
  int Size = 512;
  int Value = 49;
  string NAME = ?;
}
def v16i1 {	// ValueType
  string Namespace = "MVT";
  int Size = 16;
  int Value = 16;
  string NAME = ?;
}
def v16i16 {	// ValueType
  string Namespace = "MVT";
  int Size = 256;
  int Value = 30;
  string NAME = ?;
}
def v16i32 {	// ValueType
  string Namespace = "MVT";
  int Size = 512;
  int Value = 36;
  string NAME = ?;
}
def v16i64 {	// ValueType
  string Namespace = "MVT";
  int Size = 1024;
  int Value = 41;
  string NAME = ?;
}
def v16i8 {	// ValueType
  string Namespace = "MVT";
  int Size = 128;
  int Value = 23;
  string NAME = ?;
}
def v1f32 {	// ValueType
  string Namespace = "MVT";
  int Size = 32;
  int Value = 45;
  string NAME = ?;
}
def v1f64 {	// ValueType
  string Namespace = "MVT";
  int Size = 64;
  int Value = 50;
  string NAME = ?;
}
def v1i16 {	// ValueType
  string Namespace = "MVT";
  int Size = 16;
  int Value = 26;
  string NAME = ?;
}
def v1i32 {	// ValueType
  string Namespace = "MVT";
  int Size = 32;
  int Value = 32;
  string NAME = ?;
}
def v1i64 {	// ValueType
  string Namespace = "MVT";
  int Size = 64;
  int Value = 37;
  string NAME = ?;
}
def v1i8 {	// ValueType
  string Namespace = "MVT";
  int Size = 16;
  int Value = 19;
  string NAME = ?;
}
def v2f16 {	// ValueType
  string Namespace = "MVT";
  int Size = 32;
  int Value = 42;
  string NAME = ?;
}
def v2f32 {	// ValueType
  string Namespace = "MVT";
  int Size = 64;
  int Value = 46;
  string NAME = ?;
}
def v2f64 {	// ValueType
  string Namespace = "MVT";
  int Size = 128;
  int Value = 51;
  string NAME = ?;
}
def v2i1 {	// ValueType
  string Namespace = "MVT";
  int Size = 2;
  int Value = 13;
  string NAME = ?;
}
def v2i16 {	// ValueType
  string Namespace = "MVT";
  int Size = 32;
  int Value = 27;
  string NAME = ?;
}
def v2i32 {	// ValueType
  string Namespace = "MVT";
  int Size = 64;
  int Value = 33;
  string NAME = ?;
}
def v2i64 {	// ValueType
  string Namespace = "MVT";
  int Size = 128;
  int Value = 38;
  string NAME = ?;
}
def v2i8 {	// ValueType
  string Namespace = "MVT";
  int Size = 16;
  int Value = 20;
  string NAME = ?;
}
def v32i1 {	// ValueType
  string Namespace = "MVT";
  int Size = 32;
  int Value = 17;
  string NAME = ?;
}
def v32i16 {	// ValueType
  string Namespace = "MVT";
  int Size = 512;
  int Value = 31;
  string NAME = ?;
}
def v32i8 {	// ValueType
  string Namespace = "MVT";
  int Size = 256;
  int Value = 24;
  string NAME = ?;
}
def v4f16 {	// ValueType
  string Namespace = "MVT";
  int Size = 64;
  int Value = 43;
  string NAME = ?;
}
def v4f32 {	// ValueType
  string Namespace = "MVT";
  int Size = 128;
  int Value = 47;
  string NAME = ?;
}
def v4f64 {	// ValueType
  string Namespace = "MVT";
  int Size = 256;
  int Value = 52;
  string NAME = ?;
}
def v4i1 {	// ValueType
  string Namespace = "MVT";
  int Size = 4;
  int Value = 14;
  string NAME = ?;
}
def v4i16 {	// ValueType
  string Namespace = "MVT";
  int Size = 64;
  int Value = 28;
  string NAME = ?;
}
def v4i32 {	// ValueType
  string Namespace = "MVT";
  int Size = 128;
  int Value = 34;
  string NAME = ?;
}
def v4i64 {	// ValueType
  string Namespace = "MVT";
  int Size = 256;
  int Value = 39;
  string NAME = ?;
}
def v4i8 {	// ValueType
  string Namespace = "MVT";
  int Size = 32;
  int Value = 21;
  string NAME = ?;
}
def v64i1 {	// ValueType
  string Namespace = "MVT";
  int Size = 64;
  int Value = 18;
  string NAME = ?;
}
def v64i8 {	// ValueType
  string Namespace = "MVT";
  int Size = 512;
  int Value = 25;
  string NAME = ?;
}
def v8f16 {	// ValueType
  string Namespace = "MVT";
  int Size = 128;
  int Value = 44;
  string NAME = ?;
}
def v8f32 {	// ValueType
  string Namespace = "MVT";
  int Size = 256;
  int Value = 48;
  string NAME = ?;
}
def v8f64 {	// ValueType
  string Namespace = "MVT";
  int Size = 512;
  int Value = 53;
  string NAME = ?;
}
def v8i1 {	// ValueType
  string Namespace = "MVT";
  int Size = 8;
  int Value = 15;
  string NAME = ?;
}
def v8i16 {	// ValueType
  string Namespace = "MVT";
  int Size = 128;
  int Value = 29;
  string NAME = ?;
}
def v8i32 {	// ValueType
  string Namespace = "MVT";
  int Size = 256;
  int Value = 35;
  string NAME = ?;
}
def v8i64 {	// ValueType
  string Namespace = "MVT";
  int Size = 512;
  int Value = 40;
  string NAME = ?;
}
def v8i8 {	// ValueType
  string Namespace = "MVT";
  int Size = 64;
  int Value = 22;
  string NAME = ?;
}
def vAny {	// ValueType
  string Namespace = "MVT";
  int Size = 0;
  int Value = 252;
  string NAME = ?;
}
def variable_ops {
  string NAME = ?;
}
def vector_extract {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::EXTRACT_VECTOR_ELT";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = anonymous_173;
  string NAME = ?;
}
def vector_extract_subvec {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::EXTRACT_SUBVECTOR";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = anonymous_183;
  string NAME = ?;
}
def vector_insert {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::INSERT_VECTOR_ELT";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = anonymous_176;
  string NAME = ?;
}
def vector_shuffle {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::VECTOR_SHUFFLE";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTVecShuffle;
  string NAME = ?;
}
def vnot {	// SDPatternOperator PatFrag
  string PatFrag:pred = "";
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$in);
  dag Fragment = (xor node:$in, immAllOnesV);
  string PredicateCode = "";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def vselect {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::VSELECT";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTVSelect;
  string NAME = ?;
}
def vt {	// SDPatternOperator SDNode
  string Opcode = "ISD::VALUETYPE";
  string SDClass = "VTSDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTOther;
  string NAME = ?;
}
def vtFP {	// SDPatternOperator PatFrag PatLeaf
  SDNodeXForm PatLeaf:xform = NOOP_SDNodeXForm;
  dag Operands = (ops);
  dag Fragment = (vt);
  string PredicateCode = " return N->getVT().isFloatingPoint(); ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def vtInt {	// SDPatternOperator PatFrag PatLeaf
  SDNodeXForm PatLeaf:xform = NOOP_SDNodeXForm;
  dag Operands = (ops);
  dag Fragment = (vt);
  string PredicateCode = " return N->getVT().isInteger(); ";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def x86mmx {	// ValueType
  string Namespace = "MVT";
  int Size = 64;
  int Value = 54;
  string NAME = ?;
}
def xor {	// SDPatternOperator SDNode
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::XOR";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [SDNPCommutative, SDNPAssociative];
  SDTypeProfile TypeProfile = SDTIntBinOp;
  string NAME = ?;
}
def zero_reg {
  string NAME = ?;
}
def zext {	// SDPatternOperator SDNode
  list<SDNodeProperty> SDNode:props = [];
  string SDNode:sdclass = "SDNode";
  string Opcode = "ISD::ZERO_EXTEND";
  string SDClass = "SDNode";
  list<SDNodeProperty> Properties = [];
  SDTypeProfile TypeProfile = SDTIntExtendOp;
  string NAME = ?;
}
def zextload {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (unindexedload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getExtensionType() == ISD::ZEXTLOAD;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def zextloadi1 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (zextload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT() == MVT::i1;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def zextloadi16 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (zextload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT() == MVT::i16;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def zextloadi32 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (zextload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT() == MVT::i32;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def zextloadi8 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (zextload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT() == MVT::i8;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def zextloadvi1 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (zextload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT().getScalarType() == MVT::i1;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def zextloadvi16 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (zextload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT().getScalarType() == MVT::i16;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def zextloadvi32 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (zextload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT().getScalarType() == MVT::i32;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
def zextloadvi8 {	// SDPatternOperator PatFrag
  SDNodeXForm PatFrag:xform = NOOP_SDNodeXForm;
  dag Operands = (ops node:$ptr);
  dag Fragment = (zextload node:$ptr);
  string PredicateCode = "
  return cast<LoadSDNode>(N)->getMemoryVT().getScalarType() == MVT::i8;
";
  string ImmediateCode = "";
  SDNodeXForm OperandTransform = NOOP_SDNodeXForm;
  string NAME = ?;
}
