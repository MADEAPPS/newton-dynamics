--[[
	local a, b, c, d	
	local e = d + a
	local f = b + c
	f = f + b
	if f == 0 then
		d = e + f
	else
		d = e - f
	end
	return d;
--]]


---[[
	-- defines a factorial function
    function fact (n)
      if n <= 1 then
        return 1
      else
        return n * fact(n-1)
      end
    end
--]]

--[[
	function fibonacciNaive(n)
		if m < 2 then
		  return m
		end
		return fibonacciNaive(m-1) + fibonacciNaive(m-2)
	end
--]]