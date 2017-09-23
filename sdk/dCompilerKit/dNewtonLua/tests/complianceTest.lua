---[[	
	local a, b, c, d	
	local e = d + a
	local f = b + c
	f = f + b
	if f == 0 then
		d = e + f
	else
		d = e - f
	end
--]]
--[[	
	local d	
	if 1 == 0 then
		d = 3
	else
		d = 4
	end		 
--]]
	return d;