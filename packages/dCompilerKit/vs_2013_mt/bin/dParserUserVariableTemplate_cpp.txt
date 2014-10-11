	class dUserVariable: public dDefualtUserVariable
	{
		public:
		dUserVariable () 
			:dDefualtUserVariable ()
		{
		}

		dUserVariable (dToken token, const char* const data, int scannerLine, int scannerIndex)
			:dDefualtUserVariable  (token, data, scannerLine, scannerIndex)
		{
		}
	};

