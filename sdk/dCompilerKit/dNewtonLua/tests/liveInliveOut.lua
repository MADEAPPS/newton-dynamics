	-- defines a factorial function
    function RegisterAllocation (a, b, c, d)
		a = b + c	

		local e = d + a
		local f = b + c
		f = f + b
		if f == 0 then
			d = e + f
		else
			d = e - f
		end
		return d;
    end
    
